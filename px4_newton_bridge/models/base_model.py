from abc import ABC, abstractmethod

import newton
import warp as wp


class VehicleModel(ABC):
    """Interface that all Newton vehicle models must implement."""

    @abstractmethod
    def build(self, builder: newton.ModelBuilder, body: int) -> None:
        """Add shapes (body, booms, motors, etc.) to the Newton model builder."""

    @abstractmethod
    def compute_wrench(
        self, actuator_controls: list[float], body_rot: wp.quat
    ) -> list[float]:
        """Convert actuator commands to world-frame wrench.

        Returns [fx, fy, fz, tx, ty, tz] in world frame.
        """
