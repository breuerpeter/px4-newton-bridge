import os
import time

import newton
import warp as wp

from .actuators import ActuatorBase
from .builders import BuilderBase
from .logging import logger
from .mavlink_interface import MAVLinkInterface


class Simulator:
    def __init__(
        self,
        cfg: dict,
        mavlink_interface: MAVLinkInterface,
        vehicle_builder: BuilderBase,
        vehicle_actuator: ActuatorBase,
    ):
        self.vehicle_builder = vehicle_builder
        self.vehicle_actuator = vehicle_actuator

        self.sim_dt = cfg["sim_dt"]
        self.sim_time = 0.0

        self.mav = mavlink_interface

        builder = newton.ModelBuilder()
        self.vehicle_builder.build(builder)
        builder.add_ground_plane()
        self.model = builder.finalize()

        logger.debug(f"body_q={self.model.body_q}")
        logger.debug(f"body_qd={self.model.body_qd}")
        logger.debug(f"joint_q={self.model.joint_q}")
        logger.debug(f"joint_qd={self.model.joint_qd}")

        self.solver = newton.solvers.SolverMuJoCo(self.model, njmax=224)

        self.state0 = self.model.state()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state0)
        self.state1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.collide(self.state0)

        logger.debug(f"control dim {self.model.joint_dof_count}")

        # Pre-allocate Warp array for joint forces (used by CUDA graph)
        # First 6 elements: [fx, fy, fz, tx, ty, tz] wrench for floating base
        # Remaining elements (if any): zero (no torque on rotor joints)
        self._joint_f_buffer = wp.zeros(
            self.model.joint_dof_count, dtype=wp.float32, device=wp.get_device()
        )

        self._body_f_buffer = wp.zeros(
            6 * self.model.body_count, dtype=wp.float32, device=wp.get_device()
        )

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
        self._body_qd_prev.assign(self.state0.body_qd)
        self.state0.clear_forces()
        self.control.joint_f.assign(self._joint_f_buffer)
        self.state0.body_f.assign(self._body_f_buffer)
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

        # Compute wrench from actuator commands (CPU, before graph launch)
        self.vehicle_actuator.apply_forces_and_torques(
            self.mav.actuator_controls,
            self.model,
            self.state0,
            self._joint_f_buffer,
            self._body_f_buffer,
        )

        # Run physics (uses CUDA graph if available)
        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self._simulate_physics()

        # Update rotor joint angles/velocities for visualization
        self.vehicle_actuator.update_rotor_visuals(self.state0, self.model, self.sim_dt)

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
