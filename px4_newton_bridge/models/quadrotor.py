import math

import newton
import warp as wp

from .base_model import VehicleModel


class QuadrotorModel(VehicleModel):
    """Quadrotor vehicle model built from YAML config parameters."""

    def __init__(self, cfg: dict):
        body = cfg["body"]
        boom = cfg["boom"]
        motor = cfg["motor"]
        landing_gear = cfg["landing_gear"]

        self.body_density = body["density"]
        self.body_hx = body["hx"]
        self.body_hy = body["hy"]
        self.body_hz = body["hz"]

        self.boom_diam = boom["diameter"]
        self.boom_len = boom["length"]

        self.motor_diam = motor["diameter"]
        self.motor_height = motor["height"]
        self.max_motor_thrust = motor["max_thrust"]
        self.motor_torque_coeff = motor["torque_coeff"]
        self.motor_spin_dirs = motor["spin_dirs"]

        self.lnd_gear_angle_rad = landing_gear["angle_rad"]
        self.lnd_gear_length = landing_gear["length"]
        self.lnd_gear_diam = landing_gear["diameter"]

        self.carbon_fiber_density = cfg["carbon_fiber_density"]

        # Derived geometry
        boom_half_length = self.boom_len / 2
        body_diagonal_xy = math.sqrt(self.body_hx**2 + self.body_hy**2)
        boom_radius = self.boom_diam / 2
        diagonal_boom = body_diagonal_xy + boom_half_length - boom_radius
        self.diagonal_motor = diagonal_boom + boom_half_length
        self.body_diagonal_xy = body_diagonal_xy

        self.motor_arm_length = self.diagonal_motor
        self.motor_angles = [-(2 * i + 1) * math.pi / 4 for i in range(4)]

    def build(self, builder: newton.ModelBuilder, body: int) -> None:
        boom_rot_y = wp.quat_from_axis_angle(wp.vec3(0, 1, 0), wp.half_pi)

        boom_half_length = self.boom_len / 2
        boom_radius = self.boom_diam / 2
        diagonal_boom = self.body_diagonal_xy + boom_half_length - boom_radius

        lnd_gear_rot_y = wp.quat_from_axis_angle(
            wp.vec3(0, 1, 0), -self.lnd_gear_angle_rad
        )

        # Central body
        builder.add_shape_box(
            body,
            hx=self.body_hx,
            hy=self.body_hy,
            hz=self.body_hz,
            cfg=newton.ModelBuilder.ShapeConfig(density=self.body_density),
            key="fuselage",
        )

        for i in range(4):
            boom_angle = (2 * i + 1) * wp.pi / 4
            boom_rot_z = wp.quat_from_axis_angle(wp.vec3(0, 0, 1), boom_angle)
            boom_rot = boom_rot_z * boom_rot_y

            # Boom
            boom_shift = wp.vec3(
                diagonal_boom * math.cos(boom_angle),
                diagonal_boom * math.sin(boom_angle),
                0.0,
            )
            builder.add_shape_cylinder(
                body,
                xform=wp.transform(boom_shift, boom_rot),
                radius=boom_radius,
                half_height=boom_half_length,
                cfg=newton.ModelBuilder.ShapeConfig(density=self.carbon_fiber_density),
            )

            # Motor
            motor_shift = wp.vec3(
                self.diagonal_motor * math.cos(boom_angle),
                self.diagonal_motor * math.sin(boom_angle),
                0.0,
            )
            builder.add_shape_cylinder(
                body,
                xform=wp.transform(motor_shift, wp.quat_identity()),
                radius=self.motor_diam / 2,
                half_height=self.motor_height / 2,
                cfg=newton.ModelBuilder.ShapeConfig(density=1000),
            )

            # Landing gear
            lnd_gear_rot = boom_rot_z * lnd_gear_rot_y
            top_local = wp.vec3(0.0, 0.0, self.lnd_gear_length / 2)
            top_offset = wp.quat_rotate(lnd_gear_rot, top_local)
            attachment_point = wp.vec3(
                self.body_diagonal_xy * math.cos(boom_angle),
                self.body_diagonal_xy * math.sin(boom_angle),
                -self.body_hz,
            )
            lnd_gear_shift = attachment_point - top_offset
            builder.add_shape_cylinder(
                body,
                xform=wp.transform(lnd_gear_shift, lnd_gear_rot),
                radius=self.lnd_gear_diam / 2,
                half_height=self.lnd_gear_length / 2,
                cfg=newton.ModelBuilder.ShapeConfig(density=self.carbon_fiber_density),
            )

        # GPS antenna
        gps_ant_radius = 0.02
        gps_ant_half_height = 0.05
        gps_antenna_shift = wp.vec3(
            self.body_hx - gps_ant_radius,
            0.0,
            self.body_hz + gps_ant_half_height,
        )
        builder.add_shape_cylinder(
            body,
            xform=wp.transform(gps_antenna_shift, wp.quat_identity()),
            radius=gps_ant_radius,
            half_height=gps_ant_half_height,
            cfg=newton.ModelBuilder.ShapeConfig(density=0.0),
        )

    def compute_control_wrench(
        self, actuator_controls: list[float], body_q
    ) -> list[float]:

        body_rot = wp.quatf(body_q[0, 3:7])

        total_thrust = 0.0
        torque_x = 0.0
        torque_y = 0.0
        torque_z = 0.0

        for i in range(4):
            motor_cmd = max(0.0, min(1.0, actuator_controls[i]))
            thrust = motor_cmd * self.max_motor_thrust
            total_thrust += thrust

            motor_x = self.motor_arm_length * math.cos(self.motor_angles[i])
            motor_y = self.motor_arm_length * math.sin(self.motor_angles[i])

            torque_x += motor_y * thrust
            torque_y += -motor_x * thrust
            torque_z += -self.motor_spin_dirs[i] * self.motor_torque_coeff * thrust

        force_world = wp.quat_rotate(body_rot, wp.vec3(0.0, 0.0, total_thrust))
        torque_world = wp.quat_rotate(body_rot, wp.vec3(torque_x, torque_y, torque_z))

        return [
            force_world[0],
            force_world[1],
            force_world[2],
            torque_world[0],
            torque_world[1],
            torque_world[2],
        ]
