import warp as wp


@wp.struct
class MotorModel:
    rpm_max: float  # maximum RPM [1/min]
    tau: float  # time constant for first order motor RPM model [-]
    ct: float  # thrust coefficient [N/min^2]
    cd: float  # drag torque coefficient [m]
    dt: float  # simulation time step [s]


@wp.kernel
def step_motor_model(
    actuator_controls: wp.array(dtype=float),
    params: MotorModel,
    out_rpms: wp.array(dtype=float),
):
    """First-order motor RPM model."""
    i = wp.tid()  # motor index

    rpm_des_norm = wp.max(0.0, wp.min(1.0, actuator_controls[i]))
    rpm_des = rpm_des_norm * params.rpm_max
    alpha = 1.0 - wp.exp(-params.dt / params.tau)
    out_rpms[i] += alpha * (rpm_des - out_rpms[i])


@wp.kernel
def update_body_f(
    params: MotorModel,
    body_q: wp.array(dtype=wp.transform),
    rpms: wp.array(dtype=float),
    out_body_f: wp.array(dtype=wp.spatial_vector),
):
    """Compute the external wrench acting on the vehicle. This is composed of:
    - Thrust force (propeller pushes air down -> reaction force pushes vehicle up)
    - Drag torque (by air on propeller -> motor cancels it out -> reaction torque on body)
    - TODO: Aerodynamic drag
    - TODO: Dynamic lift
    - TODO: Induced drag
    - TODO: Aerodynamic torque

    Note: gravity is handled by Newton
    """
    i = wp.tid()  # motor index

    # Body z axis in world frame
    body_z_world = wp.quat_rotate(body_q[0].q, wp.vec3(0.0, 0.0, 1.0))

    q_prop = body_q[i + 1].q
    # Rotor z axis in world frame
    rotor_z_world = wp.quat_rotate(q_prop, wp.vec3(0.0, 0.0, 1.0))

    # Body z axis is down, so if rotor z axis also points down (dot product positive), thrust sign is negative
    thrust_sign = -wp.sign(wp.dot(rotor_z_world, body_z_world))
    thrust = params.ct * rpms[i] * rpms[i]
    thrust_world = wp.quat_rotate(q_prop, wp.vec3(0.0, 0.0, 1.0) * thrust_sign * thrust)

    # Rotor z axis is positive spin direction. Drag torque always opposes this, hence negative sign
    drag_world = -thrust * params.cd * rotor_z_world

    base_wrench = wp.spatial_vector(w=wp.vec3(0.0, 0.0, 0.0), v=drag_world)
    # Sum drag torque contributions of all props on body
    wp.atomic_add(out_body_f, 0, base_wrench)

    # Apply each prop's thrust force to its body
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
