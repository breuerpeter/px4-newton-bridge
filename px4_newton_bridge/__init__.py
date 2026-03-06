"""PX4 SITL bridge for the Newton physics engine."""

import json
import os
import pathlib
import sys

# Ensure the newton submodule is importable
_NEWTON_DIR = str(pathlib.Path(__file__).resolve().parent.parent / "newton")
if _NEWTON_DIR not in sys.path:
    sys.path.insert(0, _NEWTON_DIR)

import yaml

from .logging import logger
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


def _deep_merge(base: dict, override: dict, path: str = "") -> dict:
    for k, v in override.items():
        key_path = f"{path}.{k}" if path else k
        if k in base and isinstance(base[k], dict) and isinstance(v, dict):
            _deep_merge(base[k], v, key_path)
        else:
            logger.info(f"Config override: {key_path} = {v!r}")
            base[k] = v
    return base


def get_cfg() -> dict:
    with open(_MAIN_CFG) as f:
        cfg = yaml.safe_load(f)

    env_override = os.environ.get("NEWTON_CONFIG_OVERRIDE")
    if env_override:
        _deep_merge(cfg, json.loads(env_override))

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
