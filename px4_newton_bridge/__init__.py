"""PX4 SITL bridge for Newton physics engine."""

import pathlib

import yaml
from .actuators.actuator_base import ActuatorBase
from .actuators.propeller_basic import PropellerBasic
from .builders.builder_base import BuilderBase
from .builders.quad_x_primitive import QuadXPrimitive
from .builders.urdf import URDFBuilder

_BUILDER_TYPES: dict[str, type[BuilderBase]] = {
    "quad_x_primitive": QuadXPrimitive,
    "urdf": URDFBuilder,
}

_ACTUATOR_TYPES: dict[str, type[ActuatorBase]] = {
    "propeller_basic": PropellerBasic,
}

_CONFIGS_DIR = pathlib.Path(__file__).parent / "vehicles"


def load_model(name: str) -> tuple[BuilderBase, ActuatorBase]:
    """Load a model (build structure and get actuator object) by config name (e.g. 'quad_x_primitive')."""
    cfg_path = _CONFIGS_DIR / name / "model.yaml"
    if not cfg_path.exists():
        available = [
            p.name
            for p in _CONFIGS_DIR.iterdir()
            if p.is_dir() and (p / "model.yaml").exists()
        ]
        raise FileNotFoundError(
            f"Vehicle config '{name}' not found. Available: {available}"
        )

    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    builder_type = cfg.get("builder")
    if builder_type not in _BUILDER_TYPES:
        raise ValueError(
            f"Unknown builder type '{builder_type}'. Registered: {list(_BUILDER_TYPES)}"
        )

    actuator_type = cfg.get("actuator")
    if actuator_type not in _ACTUATOR_TYPES:
        raise ValueError(
            f"Unknown actuator type '{actuator_type}'. Registered: {list(_ACTUATOR_TYPES)}"
        )

    vehicle_dir = cfg_path.parent
    return _BUILDER_TYPES[builder_type](cfg, vehicle_dir), _ACTUATOR_TYPES[actuator_type](cfg)
