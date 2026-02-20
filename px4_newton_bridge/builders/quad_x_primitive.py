import math

import newton
import warp as wp

from .builder_base import BuilderBase


class QuadXPrimitive(BuilderBase):
    """Multi-body quadrotor X configuration model built from geometric primitives."""

    def __init__(self, cfg, vehicle_dir):
        super().__init__(cfg, vehicle_dir)

        body = self.cfg["body"]
        boom = self.cfg["boom"]
        motor = self.cfg["motor"]
        landing_gear = self.cfg["landing_gear"]

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
        # Motor 0 = front-right, then clockwise: BR, BL, FL
        self.motor_angles = [math.pi / 4 + i * math.pi / 2 for i in range(4)]

    def build(self, builder: newton.ModelBuilder) -> None:
        # Body-local frame is FRD (Forward-Right-Down), matching PX4 body convention.
        # The 180° X rotation maps FRD body axes to Newton's z-up world.
        init_pos_body = wp.vec3(0.0, 0.0, 0.3)
        init_att_body = wp.quat_from_axis_angle(wp.vec3(1, 0, 0), wp.pi)

        body = builder.add_link(key="body_frd")

        # Floating base joint (connects body_frd to world)
        joint_free = builder.add_joint_free(child=body, key="floating_base")

        # Set initial pose for floating base
        start = builder.joint_q_start[joint_free]
        builder.joint_q[start + 0] = init_pos_body[0]
        builder.joint_q[start + 1] = init_pos_body[1]
        builder.joint_q[start + 2] = init_pos_body[2]
        builder.joint_q[start + 3] = init_att_body[0]
        builder.joint_q[start + 4] = init_att_body[1]
        builder.joint_q[start + 5] = init_att_body[2]
        builder.joint_q[start + 6] = init_att_body[3]

        joint_indices = [joint_free]

        # Rotate cylinder (axis along shape Z) to lay horizontal (axis along body X)
        boom_rot_y = wp.quat_from_axis_angle(wp.vec3(0, 1, 0), wp.half_pi)

        boom_half_length = self.boom_len / 2
        boom_radius = self.boom_diam / 2
        diagonal_boom = self.body_diagonal_xy + boom_half_length - boom_radius

        # Landing gear tilt: splay outward in +Z (downward in FRD)
        lnd_gear_rot_y = wp.quat_from_axis_angle(
            wp.vec3(0, 1, 0), self.lnd_gear_angle_rad
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

        # All shape positions below are in FRD body-local frame:
        # X = forward, Y = right, Z = down
        for i in range(4):
            boom_angle = self.motor_angles[i]
            boom_rot_z = wp.quat_from_axis_angle(wp.vec3(0, 0, 1), boom_angle)
            boom_rot = boom_rot_z * boom_rot_y

            # Boom (attached to base body)
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

            # Rotor body (separate body connected via revolute joint)
            rotor = builder.add_link(key=f"rotor_{i + 1}")

            # Motor shape (on rotor body, at its local origin)
            builder.add_shape_cylinder(
                rotor,
                xform=wp.transform(wp.vec3(0, 0, 0), wp.quat_identity()),
                radius=self.motor_diam / 2,
                half_height=self.motor_height / 2,
                cfg=newton.ModelBuilder.ShapeConfig(density=1000),
            )

            # Joint orientation: CCW rotors (spin_dir=1) have Z flipped via
            # 180° X rotation, CW rotors (spin_dir=-1) keep Z aligned with
            # parent. This matches the astro_max URDF joint orientations.
            spin_dir = self.motor_spin_dirs[i]
            if spin_dir == 1:  # CCW
                joint_rot = wp.quat_from_axis_angle(wp.vec3(1, 0, 0), wp.pi)
            else:  # CW
                joint_rot = wp.quat_identity()

            motor_pos = wp.vec3(
                self.diagonal_motor * math.cos(boom_angle),
                self.diagonal_motor * math.sin(boom_angle),
                0.0,
            )

            joint_indices.append(
                builder.add_joint_revolute(
                    parent=body,
                    child=rotor,
                    parent_xform=wp.transform(motor_pos, joint_rot),
                    axis=wp.vec3(0, 0, 1),
                    key=f"rotor_{i + 1}_joint",
                )
            )

            # Landing gear (attached to base body)
            lnd_gear_rot = boom_rot_z * lnd_gear_rot_y
            top_local = wp.vec3(0.0, 0.0, -self.lnd_gear_length / 2)
            top_offset = wp.quat_rotate(lnd_gear_rot, top_local)
            attachment_point = wp.vec3(
                self.body_diagonal_xy * math.cos(boom_angle),
                self.body_diagonal_xy * math.sin(boom_angle),
                self.body_hz,  # below body = +Z in FRD
            )
            lnd_gear_shift = attachment_point - top_offset
            builder.add_shape_cylinder(
                body,
                xform=wp.transform(lnd_gear_shift, lnd_gear_rot),
                radius=self.lnd_gear_diam / 2,
                half_height=self.lnd_gear_length / 2,
                cfg=newton.ModelBuilder.ShapeConfig(density=self.carbon_fiber_density),
            )

        # GPS antenna (above body = -Z in FRD)
        gps_ant_radius = 0.02
        gps_ant_half_height = 0.05
        gps_antenna_shift = wp.vec3(
            self.body_hx - gps_ant_radius,
            0.0,
            -(self.body_hz + gps_ant_half_height),
        )
        builder.add_shape_cylinder(
            body,
            xform=wp.transform(gps_antenna_shift, wp.quat_identity()),
            radius=gps_ant_radius,
            half_height=gps_ant_half_height,
            cfg=newton.ModelBuilder.ShapeConfig(density=0.0),
        )

        # Create single articulation from all joints
        builder.add_articulation(joint_indices, key="quad_x")
