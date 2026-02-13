from abc import ABC, abstractmethod

import newton


class ActuatorBase(ABC):
    def __init__(self, cfg):
        self.cfg = cfg
        self.rpms = [0.0, 0.0, 0.0, 0.0]

    @abstractmethod
    def apply_forces_and_torques(
        self,
        actuator_controls: list[float],
        model: newton.Model,
        current_state,
        body_f,
    ):
        """Compute control input for Newton based on actuator control inputs from PX4.
        Newton takes control input in form of a wrench (with respect to world frame) for every body.
        Wrench: [fx, fy, fz, tx, ty, tz] where f is force and t is torque.
        """

    def update_rotor_visuals(self, state, model, dt: float) -> None:
        """Update rotor joint angles/velocities for visualization. No-op by default."""
