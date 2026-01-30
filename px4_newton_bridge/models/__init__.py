import pathlib

import yaml

from .base_model import VehicleModel
from .quadrotor import QuadrotorModel

_MODEL_TYPES: dict[str, type[VehicleModel]] = {
    "quadrotor": QuadrotorModel,
}

_CONFIGS_DIR = pathlib.Path(__file__).parent / "configs"


def load_model(name: str) -> VehicleModel:
    """Load a vehicle model by config name (e.g. 'quad_x')."""
    cfg_path = _CONFIGS_DIR / f"{name}.yaml"
    if not cfg_path.exists():
        available = [p.stem for p in _CONFIGS_DIR.glob("*.yaml")]
        raise FileNotFoundError(
            f"Model config '{name}' not found. Available: {available}"
        )

    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    model_type = cfg.get("type")
    if model_type not in _MODEL_TYPES:
        raise ValueError(
            f"Unknown model type '{model_type}'. Registered: {list(_MODEL_TYPES)}"
        )

    return _MODEL_TYPES[model_type](cfg)
