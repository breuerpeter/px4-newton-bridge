from abc import ABC, abstractmethod
from pathlib import Path

from newton import Model, ModelBuilder

from ..logging import logger


class BuilderBase(ABC):
    def __init__(self, cfg: dict, vehicle_dir: Path):
        self.cfg = cfg
        self.vehicle_dir = vehicle_dir

    @abstractmethod
    def build(self, builder: ModelBuilder) -> None:
        """Build the model using Newton's model builder."""

    def model_debug_print(self, model: Model) -> None:
        for i, key in enumerate(model.body_key):
            mass = model.body_mass.numpy()[i]
            inertia = model.body_inertia.numpy()[i]
            logger.debug(f"Body {i} ({key}): mass = {mass}, inertia =\n{inertia}")
