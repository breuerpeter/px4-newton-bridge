import math

import newton
import warp as wp

from .actuator_base import ActuatorBase

# todo: get rid of class structure and just have wp kernel files


class PropellerBasic(ActuatorBase):

    def __init__(self, cfg: dict):
        super().__init__(cfg)
        self.motor_torque_coeff = cfg["motor"]["torque_coeff"]

    def step_motor_model(self, motor_idx: int, motor_cmd: float):
        """First order model"""
        rpm_desired = motor_cmd * 3800  # todo max rpm 3800
        alpha = 1.0 - math.exp(-0.004 / 0.033)  # todo sim_dt and motor_tau
        self.rpms[motor_idx] += alpha * (rpm_desired - self.rpms[motor_idx])

    def apply_forces_and_torques(
        self, actuator_controls: list[float], model, current_state, body_f
    ):

        # Forces:
        # - Thrust (acts at propeller, body_f)
        # - (NEGLECTED FOR NOW) Aerodynamic drag
        # - (NEGLECTED FOR NOW) Dynamic lift
        # - (NEGLECTED FOR NOW) Induced drag
        #
        # Note: gravitational force applied by Newton
        #
        # Torques:
        # - Drag torque (acts at propeller, joint_f)
        # - (NEGLECTED FOR NOW) Aerodynamic torque

        body_f_list = [0.0] * 6  # start with base body
        drag_world_total_list = [0.0] * 3

        q_base_frd = wp.quat(current_state.body_q.numpy()[0, 3:7])

        for i in range(4):
            motor_cmd = max(0.0, min(1.0, actuator_controls[i]))
            self.step_motor_model(i, motor_cmd)
            thrust = (
                0.000003463 * self.rpms[i] ** 2
            )  # todo: self.motor_max_thrust / max_rpm*2

            q_prop_i = wp.quat(
                current_state.body_q.numpy()[i + 1, 3:7]
            )  # index 0 is body_frd

            # TODO: precompute
            prop_i_pos_z_world = wp.quat_rotate(q_prop_i, wp.vec3(0, 0, 1))
            body_frd_pos_z_world = wp.quat_rotate(q_base_frd, wp.vec3(0, 0, 1))
            # body_frd frame points down, so if prop z axis also points down (dot product positive), thrust sign is negative
            thrust_sign = -wp.sign(wp.dot(prop_i_pos_z_world, body_frd_pos_z_world))

            prop_i_rot_axis_world = wp.quat_rotate(q_prop_i, wp.vec3(0, 0, 1))
            # Drag torque opposes positive joint-coordinate spin
            drag_world = -thrust * self.motor_torque_coeff * prop_i_rot_axis_world

            thrust_world = wp.quat_rotate(
                q_prop_i, wp.vec3(0, 0, 1) * thrust_sign * thrust
            )
            body_f_list.extend([*thrust_world, 0, 0, 0])
            for idx in range(3):
                drag_world_total_list[idx] += drag_world[idx]

        body_f_list[3:6] = drag_world_total_list
        body_f.assign(body_f_list)

    def update_rotor_visuals(self, state, model, dt: float) -> None:
        """Set rotor joint angles and velocities from motor RPMs, then run FK."""
        if model.joint_dof_count <= 6:
            return  # No rotor joints (e.g. primitive model)

        joint_q = state.joint_q.numpy()
        joint_qd = state.joint_qd.numpy()

        for i in range(4):
            omega = self.rpms[i] * 2 * math.pi / 60
            joint_q[7 + i] += omega * dt
            joint_qd[6 + i] = omega

        state.joint_q.assign(joint_q)
        state.joint_qd.assign(joint_qd)
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)


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
