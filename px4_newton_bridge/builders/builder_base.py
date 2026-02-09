from abc import ABC, abstractmethod
from pathlib import Path

from newton import ModelBuilder


class BuilderBase(ABC):
    def __init__(self, cfg: dict, vehicle_dir: Path):
        self.cfg = cfg
        self.vehicle_dir = vehicle_dir

    @abstractmethod
    def build(self, builder: ModelBuilder) -> None:
        """Build the model using Newton's model builder."""
