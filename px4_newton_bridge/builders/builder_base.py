from abc import ABC, abstractmethod

from newton import ModelBuilder


class BuilderBase(ABC):
    def __init__(self, cfg):
        self.cfg = cfg

    @abstractmethod
    def build(self, builder: ModelBuilder, body: int) -> None:
        """Build the model using Newton's model builder."""
