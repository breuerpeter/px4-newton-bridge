import time

import numpy as np
import warp as wp

import newton

from .builders import BuilderBase
from .logging import logger
from .mavlink_interface import MAVLinkInterface
from .propeller_basic import (
    MotorModel,
    step_motor_model,
    update_body_f,
    update_rotor_states,
)

STABILIZE_VEL_THRESHOLD = 0.01  # m/s
STABILIZE_MIN_STEPS = 10
STABILIZE_MAX_STEPS = 10000


class Simulator:
    def __init__(
        self,
        cfg: dict,
        vehicle_builder: BuilderBase,
    ):
        logger.debug(f"Default device: {wp.get_device()}")

        self.vehicle_builder = vehicle_builder

        self.sim_dt = cfg["physics"]["dt"]
        self.sim_time = 0.0
        self.physics_enabled = cfg["physics"]["enabled"]
        if not self.physics_enabled:
            logger.info("Sensor-only mode: physics disabled")
        self.grpc_server = None
        if cfg["api"]["enabled"]:
            from .grpc_server import GRPCServer

            self.grpc_server = GRPCServer(cfg["api"]["port"])

        self.motor_params = MotorModel()
        self.motor_params.rpm_max = 3800
        self.motor_params.tau = 0.033
        self.motor_params.ct = 0.000003463
        self.motor_params.cd = 0.05
        self.motor_params.dt = 0.004

        self.rpms = wp.zeros(4)
        self.actuator_controls = wp.zeros(4)

        builder = newton.ModelBuilder()
        builder.add_ground_plane()
        self.vehicle_builder.build(builder)
        self.model = builder.finalize()
        self.vehicle_builder.model_debug_print(self.model)

        self.solver = newton.solvers.SolverMuJoCo(self.model, njmax=224)

        self.state0 = self.model.state()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state0)
        self.state1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.collide(self.state0)

        logger.debug(f"control dim {self.model.joint_dof_count}")

        # Buffer to store previous body velocities for acceleration computation
        self._body_qd_prev = wp.zeros_like(self.state0.body_qd)

        self.capture()

        self.rtf = cfg["physics"].get("rtf", 0)
        logger.info(f"Real time factor: {self.rtf}")
        self._step_start_time = time.time()

    def _get_sensor_data(self):
        """TODO: warp kernel to compute sensor data from sim state."""
        pass

    def capture(self):
        """Capture CUDA graph."""
        self.graph = None
        if wp.get_device().is_cuda and wp.is_mempool_enabled(wp.get_device()):
            logger.info("Using CUDA graph")
            if self.physics_enabled:
                with wp.ScopedCapture() as capture:
                    self._simulate_physics()
                    self._get_sensor_data()
            else:
                with wp.ScopedCapture() as capture:
                    self._get_sensor_data()
            self.graph = capture.graph
        else:
            logger.info("CUDA graph not available, using standard simulation")

    def _simulate_physics(self):
        """Run a single physics step - suitable for CUDA graph capture.

        Steps state0 into state1, then copies state1 back to state0 so the
        CUDA graph always operates on the same buffers across replays.
        """
        wp.launch(
            step_motor_model,
            dim=4,
            inputs=(self.actuator_controls, self.motor_params),
            outputs=(self.rpms,),
        )
        wp.launch(
            update_rotor_states,
            dim=4,
            inputs=(self.motor_params, self.rpms),
            outputs=(self.state0.joint_q, self.state0.joint_qd),
        )
        # newton.eval_fk(
        # self.model, self.state0.joint_q, self.state0.joint_qd, self.state0
        # )

        self._body_qd_prev.assign(self.state0.body_qd)
        self.state0.clear_forces()
        wp.launch(
            update_body_f,
            dim=4,
            inputs=(self.motor_params, self.state0.body_q, self.rpms),
            outputs=(self.state0.body_f,),
        )
        self.contacts = self.model.collide(self.state0)
        self.solver.step(self.state0, self.state1, self.control, self.contacts, self.sim_dt)
        self.state0.assign(self.state1)

    def simulate(self, mav: MAVLinkInterface):
        """Full simulation step including MAVLink I/O and physics.

        Implements lockstep synchronization with PX4:
        1. Send current sensor data to PX4
        2. Wait (blocking) for actuator controls from PX4
        3. Run physics with received controls (or apply pose in sensor-only mode)
        """
        if not self.physics_enabled and self.grpc_server is not None:
            mav.gps_fix_type = self.grpc_server.gps_fix_type

            pos = self.grpc_server.pos
            quat = self.grpc_server.quat_xyzw
            omega = self.grpc_server.omega

            # Integrate angular velocity into orientation
            speed = wp.length(omega)
            if speed > 0.0:
                axis = wp.normalize(omega)
                angle = speed * self.sim_dt
                rotation = wp.quat_from_axis_angle(axis, angle)
                quat = wp.mul(rotation, quat)
                self.grpc_server.quat_xyzw = quat

            # Tick ChangeAttTo transition
            transition = self.grpc_server.att_transition
            if transition is not None:
                transition["remaining_time"] -= self.sim_dt
                if transition["remaining_time"] <= 0.0:
                    self.grpc_server.quat_xyzw = transition["target"]
                    quat = transition["target"]
                    self.grpc_server.omega = wp.vec3(0.0, 0.0, 0.0)
                    self.grpc_server.att_transition = None

            joint_q = self.state0.joint_q.numpy()
            joint_q[0:7] = wp.transform(pos, quat)
            self.state0.joint_q.assign(joint_q)

            joint_qd = self.state0.joint_qd.numpy()
            joint_qd[3:6] = omega
            self.state0.joint_qd.assign(joint_qd)

            newton.eval_fk(self.model, self.state0.joint_q, self.state0.joint_qd, self.state0)
            self._body_qd_prev.assign(self.state0.body_qd)

        # Step PX4
        mav._send_sensor_data(self.state0, self._body_qd_prev.numpy(), self.sim_time)

        # Block waiting for actuator controls (lockstep synchronization)
        if not mav.receive_actuator_controls(timeout=2.0):
            raise ConnectionError("PX4 disconnected (no actuator controls received)")

        # Copy actuator controls to GPU (H2D transfer, cannot be in CUDA graph)
        self.actuator_controls.assign(mav.actuator_controls[:4])

        if self.physics_enabled:
            # Run physics (uses CUDA graph if available)
            if self.graph:
                wp.capture_launch(self.graph)
            else:
                self._simulate_physics()

    def stabilize(self):
        """Step physics with zero actuator inputs until the body settles.

        This should be called before wait_for_px4 so PX4 starts receiving sensor data
        at the resting position rather than at the spawn height.
        """
        if not self.physics_enabled:
            return
        for i in range(STABILIZE_MAX_STEPS):
            self._simulate_physics()
            wp.synchronize()
            body_qd = self.state0.body_qd.numpy()
            linear_vel = np.linalg.norm(body_qd[0, :3])

            if i > STABILIZE_MIN_STEPS and linear_vel < STABILIZE_VEL_THRESHOLD:
                logger.info(f"Stabilized after {i + 1} steps (vel={linear_vel:.4f} m/s)")
                return

        logger.warning(f"Stabilization did not converge after {STABILIZE_MAX_STEPS} steps (vel={linear_vel:.4f} m/s)")

    def step(self, mav: MAVLinkInterface):
        # Increment sim_time BEFORE simulate() so sensor data has non-zero timestamps
        self.sim_time += self.sim_dt

        # simulate() handles I/O and physics (with CUDA graph if available)
        self.simulate(mav)

        if self.rtf > 0:
            step_time = time.time() - self._step_start_time
            target_step_time = self.sim_dt / self.rtf
            sleep_time = target_step_time - step_time
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._step_start_time = time.time()
