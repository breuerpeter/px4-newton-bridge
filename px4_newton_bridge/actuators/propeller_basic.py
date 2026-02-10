import math

import warp as wp

from .actuator_base import ActuatorBase

# todo: get rid of class structure and just have wp kernel files


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

    def step_motor_model(self, motor_idx: int, motor_cmd: float):
        """First order model"""
        rpm_desired = motor_cmd * 3800  # todo max rpm
        alpha = 1.0 - math.exp(-0.004 / 0.033)  # todo sim_dt and motor_tau
        self.rpms[motor_idx] += alpha * (rpm_desired - self.rpms[motor_idx])
        # for rotating prop viz:
        # todo: 1. set joint angles and speeds (joint_q, joint_qd) from this
        # todo: 2. update body poses with eval_fk() (updates body_q and body_qd) (doesnt change dynamics, only kinematic)

    def compute_control_wrench(
        self, actuator_controls: list[float], body_q
    ) -> list[float]:

        # todo: set body wrenches with state.body_f (switch from control.joint_f)
        body_rot = wp.quatf(body_q[0, 3:7])

        total_thrust = 0.0
        torque_x = 0.0
        torque_y = 0.0
        torque_z = 0.0

        for i in range(4):
            motor_cmd = max(0.0, min(1.0, actuator_controls[i]))
            self.step_motor_model(i, motor_cmd)
            thrust = (
                0.000003463 * self.rpms[i] ** 2
            )  # todo: self.motor_max_thrust / max_rpm*2
            total_thrust += thrust

            motor_x = self.motor_arm_length * math.cos(self.motor_angles[i])
            motor_y = self.motor_arm_length * math.sin(self.motor_angles[i])

            torque_x += motor_y * thrust
            torque_y += -motor_x * thrust
            torque_z += -self.motor_spin_dirs[i] * self.motor_torque_coeff * thrust

        force_world = wp.quat_rotate(body_rot, wp.vec3(0, 0, total_thrust))
        torque_world = wp.quat_rotate(body_rot, wp.vec3(torque_x, torque_y, torque_z))

        return [
            force_world[0],
            force_world[1],
            force_world[2],
            torque_world[0],
            torque_world[1],
            torque_world[2],
        ]


"""
@wp.kernel  
def apply_motor_thrust_kernel(  
    actuator_controls: wp.array(dtype=float),  # Shape: (num_drones, 4)  
    body_q: wp.array(dtype=wp.transform),  
    body_f: wp.array(dtype=wp.spatial_vector),  
    motor_arm_length: float,  
    motor_angles: wp.array(dtype=float),  # Pre-computed angles  
    motor_spin_dirs: wp.array(dtype=int),  # [1, -1, 1, -1]  
    max_motor_thrust: float,  
    motor_torque_coeff: float,  
):  
    drone_id = wp.tid()  
      
    # Get body rotation  
    body_rot = wp.transform_get_rotation(body_q[drone_id])  
      
    total_thrust = 0.0  
    torque_body = wp.vec3(0.0, 0.0, 0.0)  
      
    for i in range(4):  
        # Clamp motor command  
        motor_cmd = wp.clamp(actuator_controls[drone_id * 4 + i], 0.0, 1.0)  
        thrust = motor_cmd * max_motor_thrust  
        total_thrust += thrust  
          
        # Motor position  
        motor_x = motor_arm_length * wp.cos(motor_angles[i])  
        motor_y = motor_arm_length * wp.sin(motor_angles[i])  
          
        # Accumulate torques  
        torque_body += wp.vec3(  
            motor_y * thrust,  
            -motor_x * thrust,  
            -float(motor_spin_dirs[i]) * motor_torque_coeff * thrust  
        )  
      
    # Transform to world frame  
    force_body = wp.vec3(0.0, 0.0, total_thrust)  
    force_world = wp.quat_rotate(body_rot, force_body)  
    torque_world = wp.quat_rotate(body_rot, torque_body)  
      
    # Apply wrench atomically  
    wp.atomic_add(body_f, drone_id, wp.spatial_vector(force_world, torque_world))
"""
