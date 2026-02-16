import math

import newton
import warp as wp
from numpy import dtype

from .actuator_base import ActuatorBase


@wp.struct
class MotorModel:
    rpm_max: float  # 3800
    tau: float  # 0.033
    ct: float  # 0.000003463 (max_thrust/max_rpm^2)
    cd: float  # 0.05
    dt: float  # 0.004


@wp.kernel
def step_motor_model(
    actuator_controls: wp.array(dtype=float),
    params: MotorModel,
    out_rpms: wp.array(dtype=float),
):
    i = wp.tid()  # motor index
    rpm_des_norm = wp.max(0.0, wp.min(1.0, actuator_controls[i]))
    rpm_des = rpm_des_norm * params.rpm_max
    alpha = 1.0 - wp.exp(-params.dt / params.tau)
    out_rpms[i] += alpha * (rpm_des - out_rpms[i])


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


@wp.kernel
def update_body_f(
    params: MotorModel,
    body_q: wp.array(dtype=wp.transform),
    rpms: wp.array(dtype=float),
    out_body_f: wp.array(dtype=wp.spatial_vector),
):
    i = wp.tid()  # motor index
    body_z_world = wp.quat_rotate(body_q[0].q, wp.vec3(0.0, 0.0, 1.0))
    q_prop = body_q[i + 1].q
    prop_z_world = wp.quat_rotate(q_prop, wp.vec3(0.0, 0.0, 1.0))
    # body_frd frame points down, so if prop z axis also points down (dot product positive), thrust sign is negative
    thrust_sign = -wp.sign(wp.dot(prop_z_world, body_z_world))
    thrust = params.ct * rpms[i] * rpms[i]
    thrust_world = wp.quat_rotate(q_prop, wp.vec3(0.0, 0.0, 1.0) * thrust_sign * thrust)
    # Drag torque opposes positive joint-coordinate spin
    drag_world = -thrust * params.cd * prop_z_world

    base_wrench = wp.spatial_vector(w=wp.vec3(0.0, 0.0, 0.0), v=drag_world)
    wp.atomic_add(out_body_f, 0, base_wrench)

    out_body_f[i + 1] = wp.spatial_vector(w=thrust_world, v=wp.vec3(0.0, 0.0, 0.0))


@wp.kernel
def update_rotor_states(
    params: MotorModel,
    rpms: wp.array(dtype=float),
    out_joint_q: wp.array(dtype=float),
    out_joint_qd: wp.array(dtype=float),
):
    """Set rotor joint angles and velocities from motor RPMs. Requires subsequent eval_fk()."""
    i = wp.tid()  # motor index
    omega = rpms[i] * 2.0 * wp.pi / 60.0
    out_joint_q[7 + i] += omega * params.dt
    out_joint_qd[6 + i] = omega


class PropellerBasic(ActuatorBase):

    def __init__(self, cfg: dict):
        super().__init__(cfg)
