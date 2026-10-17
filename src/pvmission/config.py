"""Read a site file (YAML) into plant, camera, requirements and drone settings."""

from __future__ import annotations

from dataclasses import fields
from pathlib import Path

import numpy as np
import yaml

from .camera import CAMERAS, Camera
from .geometry import Requirements
from .planner import Drone
from .plant import Plant


def _pick(cls, data: dict | None):
    data = dict(data or {})
    names = {f.name for f in fields(cls)}
    unknown = set(data) - names
    if unknown:
        raise ValueError(f"unknown {cls.__name__} setting(s): {', '.join(sorted(unknown))}")
    for k, v in data.items():
        if isinstance(v, list):
            data[k] = tuple(v)
    return cls(**data)


def load(path: str | Path) -> dict:
    cfg = yaml.safe_load(Path(path).read_text())
    site, layout = cfg["site"], cfg.get("layout", {})
    plant = Plant.regular(
        site.get("name", Path(path).stem),
        site["latitude"],
        site["longitude"],
        site.get("altitude", 0.0),
        site.get("timezone", "UTC"),
        **layout,
    )
    cam = cfg.get("camera", "h20t")
    camera = CAMERAS[cam] if isinstance(cam, str) else _pick(Camera, cam)
    home = cfg.get("home")
    return {
        "plant": plant,
        "camera": camera,
        "req": _pick(Requirements, cfg.get("requirements")),
        "drone": _pick(Drone, cfg.get("drone")),
        "home": None if home is None else np.array([home[0], home[1], 0.0]),
    }
