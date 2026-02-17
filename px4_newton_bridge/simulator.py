import os
import time

import newton
import warp as wp

from .builders import BuilderBase
from .logging import logger
from .mavlink_interface import MAVLinkInterface
from .propeller_basic import (
    MotorModel,
    step_motor_model,
    update_body_f,
    update_rotor_states,
)


class Simulator:
    def __init__(
        self,
        cfg: dict,
        mavlink_interface: MAVLinkInterface,
        vehicle_builder: BuilderBase,
        # vehicle_actuator: ActuatorBase,
    ):
        logger.debug(f"Default device: { wp.get_device() }")

        self.vehicle_builder = vehicle_builder
        # self.vehicle_actuator = vehicle_actuator

        self.sim_dt = cfg["sim_dt"]
        self.sim_time = 0.0

        self.motor_params = MotorModel()
        self.motor_params.rpm_max = 3800
        self.motor_params.tau = 0.033
        self.motor_params.ct = 0.000003463
        self.motor_params.cd = 0.05
        self.motor_params.dt = 0.004

        self.rpms = wp.zeros(4)
        self.actuator_controls = wp.zeros(4)

        self.mav = mavlink_interface

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

        self.speed_factor = float(os.environ.get("PX4_REAL_TIME_FACTOR", 0))
        logger.info(f"Real time factor: {self.speed_factor}")
        self._step_start_time = time.time()

    def capture(self):
        """Capture CUDA graph for physics substeps only (no I/O)."""
        self.graph = None
        if wp.get_device().is_cuda and wp.is_mempool_enabled(wp.get_device()):
            logger.info("Using CUDA graph for physics simulation")
            with wp.ScopedCapture() as capture:
                self._simulate_physics()
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
        self.solver.step(
            self.state0, self.state1, self.control, self.contacts, self.sim_dt
        )
        self.state0.assign(self.state1)

    def simulate(self):
        """Full simulation step including MAVLink I/O and physics.

        Implements lockstep synchronization with PX4:
        1. Send current sensor data to PX4
        2. Wait (blocking) for actuator controls from PX4
        3. Run physics with received controls
        """
        # Step PX4
        self.mav._send_sensor_data(
            self.state0, self._body_qd_prev.numpy(), self.sim_time
        )

        # Block waiting for actuator controls (lockstep synchronization)
        self.mav.receive_actuator_controls(timeout=None)

        # Copy actuator controls to GPU (H2D transfer, cannot be in CUDA graph)
        self.actuator_controls.assign(self.mav.actuator_controls[:4])

        # Run physics (uses CUDA graph if available)
        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self._simulate_physics()

    def step(self):
        # Increment sim_time BEFORE simulate() so sensor data has non-zero timestamps
        self.sim_time += self.sim_dt

        # simulate() handles I/O and physics (with CUDA graph if available)
        self.simulate()

        if self.speed_factor > 0:
            step_time = time.time() - self._step_start_time
            target_step_time = self.sim_dt / self.speed_factor
            sleep_time = target_step_time - step_time
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._step_start_time = time.time()
