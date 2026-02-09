import math

import warp as wp

from .actuator_base import ActuatorBase


class PropellerBasic(ActuatorBase):

    def __init__(self, cfg: dict):
        super().__init__(cfg)

        # Derived geometry
        boom_half_length = 0.1
        body_diagonal_xy = math.sqrt(0.125**2 + 0.125**2)
        boom_radius = 0.025
        diagonal_boom = body_diagonal_xy + boom_half_length - boom_radius
        self.diagonal_motor = diagonal_boom + boom_half_length

        self.motor_arm_length = self.diagonal_motor
        self.motor_angles = [-(2 * i + 1) * math.pi / 4 for i in range(4)]
        self.max_motor_thrust = 50
        self.motor_torque_coeff = 0.05
        self.motor_spin_dirs = [1, -1, 1, -1]

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
