"""PX4 SITL bridge for the Newton physics engine."""

import pathlib

import yaml

from .builders.builder_base import BuilderBase
from .builders.quad_x_primitive import QuadXPrimitive
from .builders.urdf import URDFBuilder

_BUILDER_TYPES: dict[str, type[BuilderBase]] = {
    "quad_x_primitive": QuadXPrimitive,
    "urdf": URDFBuilder,
}

_BRIDGE_DIR = pathlib.Path(__file__).parent
_MAIN_CFG = _BRIDGE_DIR / "config.yaml"
_MODEL_CFG_DIR = _BRIDGE_DIR / "vehicles"


def get_cfg() -> dict:
    with open(_MAIN_CFG) as f:
        cfg = yaml.safe_load(f)
    return cfg


def load_model(name: str) -> BuilderBase:
    cfg_path = _MODEL_CFG_DIR / name / "model.yaml"
    if not cfg_path.exists():
        available = [
            p.name
            for p in _MODEL_CFG_DIR.iterdir()
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

    vehicle_dir = cfg_path.parent
    return _BUILDER_TYPES[builder_type](cfg, vehicle_dir)
