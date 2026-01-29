import math
import os
import random
import time

import newton
import warp as wp

from .logging import logger
from .mavlink_interface import MAVLinkInterface


class Drone:
    def __init__(self, platform: str = "astro"):
        self.platform = platform

        self.sim_dt = 0.004  # [s] (250 Hz, matches Gazebo default)
        self.sim_time = 0.0

        self.mavlink = MAVLinkInterface()

        # Motor configuration for quadrotor (max thrust per motor in Newtons)
        self.max_motor_thrust = 50.0  # Adjust based on drone mass
        # Torque coefficient: ratio of reaction torque to thrust (N·m per N)
        self.motor_torque_coeff = 0.05
        # Motor spin directions for yaw torque (1 = CCW, -1 = CW when viewed from above)
        # Standard X-quad: alternating spin directions
        self.motor_spin_dirs = [1, -1, 1, -1]



        # Drone body (rigid, symmetric in xz and xy plane)
        self.carbon_fiber_density = 1750  # [kg/m^3]

        # Astro (default)
        self.body_density = 800  # [kg/m^3]
        # Half lengths
        self.body_hx_m = 0.125
        self.body_hy_m = self.body_hx_m
        self.body_hz_m = 0.05

        self.body_boom_diam_m = 0.05
        self.body_boom_len_m = 0.20

        self.motor_diam_m = 0.08
        self.motor_height_m = 0.05

        self.lnd_gear_angle_rad = wp.pi / 6
        self.lnd_gear_length_m = 0.2
        self.lnd_gear_diam_m = 0.02

        # Alta X Gen2
        if self.platform == "altaxgen2":
            self.body_hx_m *= 2
            self.body_hy_m *= 2
            self.body_hz_m *= 2

            self.body_boom_diam_m *= 2
            self.body_boom_len_m *= 2

            self.motor_diam_m *= 2
            self.motor_height_m *= 2

        builder = newton.ModelBuilder()
        builder.default_shape_cfg.ke = 1e4  # Contact stiffness
        builder.default_shape_cfg.kd = 1000.0  # Contact damping
        builder.default_shape_cfg.mu = 0.5  # Friction coefficient
        builder.rigid_contact_margin = 0.01  # Contact margin

        init_pos_body = wp.vec3(0.0, 0.0, 1.0)
        init_att_body = wp.quat_identity()
        init_tf_body = wp.transform(init_pos_body, init_att_body)

        body = builder.add_body(xform=init_tf_body)
        builder.add_shape_box(
            body,
            hx=self.body_hx_m,
            hy=self.body_hy_m,
            hz=self.body_hz_m,
            cfg=newton.ModelBuilder.ShapeConfig(density=self.body_density),
            key="fuselage",
        )

        boom_rot_y = wp.quat_from_axis_angle(
            wp.vec3(0, 1, 0), wp.half_pi
        )  # rotate cylinder to point along x

        boom_half_length = self.body_boom_len_m / 2
        body_diagonal_xy = math.sqrt(self.body_hx_m**2 + self.body_hy_m**2)
        boom_radius = self.body_boom_diam_m / 2
        diagonal_boom = body_diagonal_xy + boom_half_length - boom_radius
        diagonal_motor = diagonal_boom + boom_half_length

        # Store motor geometry for torque calculations
        self.motor_arm_length = diagonal_motor
        self.motor_angles = [-(2 * i + 1) * math.pi / 4 for i in range(4)]

        lnd_gear_rot_y = wp.quat_from_axis_angle(
            wp.vec3(0, 1, 0), -self.lnd_gear_angle_rad
        )

        for id in range(4):
            # add booms
            boom_angle = (2 * id + 1) * wp.pi / 4
            boom_rot_z = wp.quat_from_axis_angle(wp.vec3(0, 0, 1), boom_angle)
            boom_rot = (
                boom_rot_z * boom_rot_y
            )  # rotate around world y-axis first, then world z-axis

            boom_shift = wp.vec3(
                diagonal_boom * math.cos(boom_angle),
                diagonal_boom * math.sin(boom_angle),
                0.0,
            )

            builder.add_shape_cylinder(
                body,
                xform=wp.transform(
                    boom_shift,
                    boom_rot,
                ),
                radius=boom_radius,
                half_height=boom_half_length,
                cfg=newton.ModelBuilder.ShapeConfig(density=self.carbon_fiber_density),
            )

            # add motors
            motor_shift = wp.vec3(
                diagonal_motor * math.cos(boom_angle),
                diagonal_motor * math.sin(boom_angle),
                0.0,
            )

            builder.add_shape_cylinder(
                body,
                xform=wp.transform(motor_shift, wp.quat_identity()),
                radius=self.motor_diam_m / 2,
                half_height=self.motor_height_m / 2,
                cfg=newton.ModelBuilder.ShapeConfig(density=1000),
            )

            # add landing gear
            lnd_gear_rot = (
                boom_rot_z * lnd_gear_rot_y
            )  # rotate around world y-axis first, then world z-axis

            top_local = wp.vec3(0.0, 0.0, self.lnd_gear_length_m / 2)
            top_offset = wp.quat_rotate(lnd_gear_rot, top_local)

            attachment_point = wp.vec3(
                body_diagonal_xy * math.cos(boom_angle),
                body_diagonal_xy * math.sin(boom_angle),
                -self.body_hz_m,
            )

            lnd_gear_shift = attachment_point - top_offset

            builder.add_shape_cylinder(
                body,
                xform=wp.transform(lnd_gear_shift, lnd_gear_rot),
                radius=self.lnd_gear_diam_m / 2,
                half_height=self.lnd_gear_length_m / 2,
                cfg=newton.ModelBuilder.ShapeConfig(density=self.carbon_fiber_density),
            )

            # add GPS antenna
            # TODO: change to site

            gps_ant_radius_m = 0.02
            gps_ant_half_height_m = 0.05
            gps_antenna_shift = wp.vec3(
                self.body_hx_m - gps_ant_radius_m,
                0.0,
                self.body_hz_m + gps_ant_half_height_m,
            )

            builder.add_shape_cylinder(
                body,
                xform=wp.transform(gps_antenna_shift, wp.quat_identity()),
                radius=gps_ant_radius_m,
                half_height=gps_ant_half_height_m,
                cfg=newton.ModelBuilder.ShapeConfig(density=0.0),
            )
        builder.add_ground_plane()

        self.model = builder.finalize()

        logger.debug(f"body_q={self.model.body_q}")
        logger.debug(f"body_qd={self.model.body_qd}")
        logger.debug(f"joint_q={self.model.joint_q}")
        logger.debug(f"joint_qd={self.model.joint_qd}")

        self.solver = newton.solvers.SolverMuJoCo(self.model, njmax=224)

        self.state0 = self.model.state()
        self.state1 = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.collide(self.state0)

        logger.debug(f"control dim {self.model.joint_dof_count}")

        # Pre-allocate Warp array for joint forces (used by CUDA graph)
        # joint_f format: [fx, fy, fz, tx, ty, tz] in world frame
        self._joint_f_buffer = wp.zeros(6, dtype=wp.float32, device=wp.get_device())

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
        self.contacts = self.model.collide(self.state0)
        self.solver.step(
            self.state0, self.state1, self.control, self.contacts, self.sim_dt
        )
        self.state0.assign(self.state1)

    def _compute_forces_from_actuators(self):
        """Convert actuator commands to thrust forces and torques in world frame."""
        total_thrust = 0.0
        torque_x = 0.0  # Roll
        torque_y = 0.0  # Pitch
        torque_z = 0.0  # Yaw

        for i in range(4):
            motor_cmd = max(0.0, min(1.0, self.mavlink.actuator_controls[i]))
            thrust = motor_cmd * self.max_motor_thrust
            total_thrust += thrust

            # Motor position in body frame
            motor_x = self.motor_arm_length * math.cos(self.motor_angles[i])
            motor_y = self.motor_arm_length * math.sin(self.motor_angles[i])

            # Torque from thrust at motor position: τ = r × F
            # F = (0, 0, thrust), r = (motor_x, motor_y, 0)
            torque_x += motor_y * thrust  # Roll: r_y * F_z
            torque_y += -motor_x * thrust  # Pitch: -r_x * F_z

            # Yaw torque from motor reaction (opposite to spin direction)
            torque_z += -self.motor_spin_dirs[i] * self.motor_torque_coeff * thrust

        # Extract body rotation quaternion (XYZW format from body_q transform)
        body_rot = wp.quat(self.state0.body_q.numpy()[0, 3:7])

        # Forces in body frame [fx, fy, fz, tx, ty, tz]
        joint_f_b = [0.0, 0.0, total_thrust, torque_x, torque_y, torque_z]

        # Rotate linear force from body to world frame
        force_world = wp.quat_rotate(body_rot, wp.vec3(joint_f_b[:3]))

        # Rotate angular torque from body to world frame
        torque_world = wp.quat_rotate(body_rot, wp.vec3(joint_f_b[3:]))

        return [
            force_world[0],
            force_world[1],
            force_world[2],
            torque_world[0],
            torque_world[1],
            torque_world[2],
        ]

    def wait_for_px4(self):
        """Send sensor data until PX4 starts responding with actuator controls."""
        logger.info("Waiting for PX4 to start lockstep...")
        while True:
            self.sim_time += self.sim_dt
            self.mavlink.send_heartbeat()
            self._send_sensor_data()
            if self.mavlink.receive_actuator_controls(timeout=1.0):
                logger.info("PX4 lockstep established")
                return

    def simulate(self):
        """Full simulation step including MAVLink I/O and physics.

        Implements lockstep synchronization with PX4:
        1. Send current sensor data to PX4
        2. Wait (blocking) for actuator controls from PX4
        3. Run physics with received controls
        """
        self.mavlink.send_heartbeat()

        # Send sensor data first - this triggers PX4 to compute actuator controls
        self._send_sensor_data()

        # Block waiting for actuator controls (lockstep synchronization)
        self.mavlink.receive_actuator_controls(timeout=None)

        # Compute forces from actuator commands (CPU, before graph launch)
        joint_f_world = self._compute_forces_from_actuators()

        # Update the force buffer used by physics simulation
        self._joint_f_buffer.assign(joint_f_world)

        # Run physics (uses CUDA graph if available)
        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self._simulate_physics()

    def _send_sensor_data(self):
        """Send simulated sensor data to PX4."""
        time_usec = int(self.sim_time * 1e6)

        # Get current state from simulation
        # body_q contains position and quaternion for each body
        # body_qd contains linear and angular velocity for each body
        body_q = self.state0.body_q.numpy()
        body_qd = self.state0.body_qd.numpy()
        body_qd_prev = self._body_qd_prev.numpy()

        # Extract drone state (body index 0)
        # Position: [x, y, z]
        pos = body_q[0, :3]
        # Quaternion: [x, y, z, w] (warp convention)
        quat_xyzw = body_q[0, 3:7]
        # Convert to [w, x, y, z] for MAVLink
        quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]

        # Velocity: [vx, vy, vz, wx, wy, wz] (linear, angular)
        vel_linear = body_qd[0, :3]
        vel_linear_prev = body_qd_prev[0, :3]
        vel_angular = body_qd[0, 3:6]

        # Acceleration: [x, y, z]
        acc_linear = (vel_linear - vel_linear_prev) / self.sim_dt

        # Generic/placeholder sensor data
        # In a real implementation, these would be computed from simulation state

        # IMU data (accelerometer measures specific force in body frame)
        # Specific force = linear_acceleration - gravity (both in body frame)
        # Transform world frame vectors to body frame using inverse quaternion rotation
        quat = wp.quat(
            float(quat_xyzw[0]),
            float(quat_xyzw[1]),
            float(quat_xyzw[2]),
            float(quat_xyzw[3]),
        )

        # Gravity in world frame (pointing down)
        gravity_world = wp.vec3(0.0, 0.0, -9.81)
        # Transform gravity to body frame
        gravity_body = wp.quat_rotate_inv(quat, gravity_world)

        # Linear acceleration in world frame, transform to body frame
        acc_world = wp.vec3(
            float(acc_linear[0]), float(acc_linear[1]), float(acc_linear[2])
        )
        acc_body = wp.quat_rotate_inv(quat, acc_world)

        # Accelerometer reading = specific force = acceleration - gravity
        # Convert from Newton FLU (Forward-Left-Up) to PX4 FRD (Forward-Right-Down)
        # 180° rotation about X axis: x_frd = x_flu, y_frd = -y_flu, z_frd = -z_flu
        xacc = acc_body[0] - gravity_body[0] + random.gauss(0, 0.02)
        yacc = -(acc_body[1] - gravity_body[1]) + random.gauss(0, 0.02)
        zacc = -(acc_body[2] - gravity_body[2]) + random.gauss(0, 0.02)

        # Gyroscope (angular velocity in body frame)
        vel_angular_body = wp.quat_rotate_inv(quat, wp.vec3(vel_angular))
        # Convert from FLU to FRD
        xgyro = vel_angular_body[0] + random.gauss(0, 0.02)
        ygyro = -vel_angular_body[1] + random.gauss(0, 0.02)
        zgyro = -vel_angular_body[2] + random.gauss(0, 0.02)

        # Magnetometer - World Magnetic Model for Zurich (lat: 47.4°, lon: 8.5°)
        # Hardcoded WMM values for Zurich
        declination_rad = math.radians(3.0)  # ~3° East (magnetic north vs true north)
        inclination_rad = math.radians(64.0)  # ~64° dip angle (field points into earth)
        field_strength_gauss = 0.48

        # Construct magnetic field in NED frame
        # mag_ned = Dcm(Euler(0, -inclination, declination)) * [H, 0, 0]
        mag_n = (
            field_strength_gauss * math.cos(declination_rad) * math.cos(inclination_rad)
        )
        mag_e = field_strength_gauss * math.sin(declination_rad)
        mag_d = (
            field_strength_gauss * math.cos(declination_rad) * math.sin(inclination_rad)
        )

        # Convert NED to Newton world frame (X=North, Y=East, Z=Up)
        mag_world = wp.vec3(mag_n, mag_e, -mag_d)

        # Rotate to body frame (FLU)
        mag_body = wp.quat_rotate_inv(quat, mag_world)

        # Convert from FLU to FRD and add noise
        xmag = mag_body[0] + random.gauss(0, 0.02)
        ymag = -mag_body[1] + random.gauss(0, 0.02)
        zmag = -mag_body[2] + random.gauss(0, 0.03)

        # Barometer
        # Approximate pressure from altitude (simplified model)
        altitude = pos[2]  # z is up in simulation
        sea_level_pressure = 1013.25  # hPa
        # Barometric formula approximation
        abs_pressure = sea_level_pressure * (1 - 2.25577e-5 * altitude) ** 5.25588
        pressure_alt = altitude

        # Send HIL_SENSOR at simulation rate
        self.mavlink.send_hil_sensor(
            time_usec=time_usec,
            xacc=xacc,
            yacc=yacc,
            zacc=zacc,
            xgyro=xgyro,
            ygyro=ygyro,
            zgyro=zgyro,
            xmag=xmag,
            ymag=ymag,
            zmag=zmag,
            abs_pressure=abs_pressure,
            pressure_alt=pressure_alt,
        )

        # Send GPS at lower rate (10 Hz)
        if (
            self.sim_time - self.mavlink.last_hil_gps_time
            >= self.mavlink.hil_gps_interval
        ):
            self.mavlink.last_hil_gps_time = self.sim_time

            # Convert simulation position to GPS coordinates
            # Using a reference point (Zurich) and adding local offsets
            # 1 degree latitude ~= 111km, 1 degree longitude ~= 111km * cos(lat)
            ref_lat = 47.3977418  # Reference latitude
            ref_lon = 8.5455939  # Reference longitude
            ref_alt = 488.0  # Reference altitude MSL [m]

            # Position offset in meters (x=East, y=North in typical GPS convention)
            # Simulation uses x=forward, y=left, z=up
            lat_offset = pos[0] / 111000.0  # Approximate
            lon_offset = pos[1] / (111000.0 * math.cos(math.radians(ref_lat)))

            lat = int((ref_lat + lat_offset) * 1e7)
            lon = int((ref_lon + lon_offset) * 1e7)
            alt = int((ref_alt + altitude) * 1000)  # mm

            # Velocity in cm/s (NED frame)
            vn = int(vel_linear[0] * 100)
            ve = int(-vel_linear[1] * 100)  # y_frd (px4)= -y_flu(newton)
            vd = int(-vel_linear[2] * 100)  # Down is negative z

            vel = int(math.sqrt(vel_linear[0] ** 2 + vel_linear[1] ** 2) * 100)

            self.mavlink.send_hil_gps(
                time_usec=time_usec,
                lat=lat,
                lon=lon,
                alt=alt,
                vn=vn,
                ve=ve,
                vd=vd,
                vel=vel,
            )

            # Also send full state for visualization/logging
            self.mavlink.send_hil_state_quaternion(
                time_usec=time_usec,
                attitude_quaternion=quat_wxyz,
                rollspeed=xgyro,
                pitchspeed=ygyro,
                yawspeed=zgyro,
                lat=lat,
                lon=lon,
                alt=alt,
                vx=vn,
                vy=ve,
                vz=vd,
                xacc=int(xacc * 1000 / 9.81),  # Convert to mG
                yacc=int(yacc * 1000 / 9.81),
                zacc=int(zacc * 1000 / 9.81),
            )

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
