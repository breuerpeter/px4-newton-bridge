from abc import ABC, abstractmethod

import newton


class VehicleModel(ABC):
    """Interface that all Newton vehicle models must implement."""

    @abstractmethod
    def build(self, builder: newton.ModelBuilder, body: int) -> None:
        """Build the model using Newton's model builder."""

    @abstractmethod
    def compute_control_wrench(
        self, actuator_controls: list[float], body_q
    ) -> list[float]:
        """Compute control input for Newton based on actuator control inputs from PX4.
        Newton takes control input in form of a wrench (with respect to world frame) for every body.
        Wrench: [fx, fy, fz, tx, ty, tz] where f is force and t is torque.
        """
