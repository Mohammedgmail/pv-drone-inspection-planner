"""Thermal camera model: field of view and ground sample distance (pinhole)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Camera:
    name: str
    width_px: int  # image width; the planner lines this side up with the row
    height_px: int
    pixel_pitch_um: float
    focal_mm: float
    pitch_range_deg: tuple[float, float] = (-90.0, 0.0)  # usable gimbal pitch, down negative
    time_constant_ms: float = 10.0  # microbolometer thermal time constant (motion blur)

    @property
    def hfov(self) -> float:
        """Horizontal (width) field of view, radians."""
        return 2 * np.arctan(self.width_px * self.pixel_pitch_um / 2e3 / self.focal_mm)

    @property
    def vfov(self) -> float:
        return 2 * np.arctan(self.height_px * self.pixel_pitch_um / 2e3 / self.focal_mm)

    @property
    def dfov(self) -> float:
        diag = np.hypot(self.width_px, self.height_px) * self.pixel_pitch_um / 2e3
        return 2 * np.arctan(diag / self.focal_mm)

    def gsd(self, distance_m) -> np.ndarray:
        """Size of one pixel (m) on a surface square to the view at ``distance_m``."""
        return np.asarray(distance_m) * self.pixel_pitch_um * 1e-3 / self.focal_mm


CAMERAS = {
    # DJI specifications page (enterprise.dji.com/zenmuse-h20-series/specs):
    # 640x512 VOx microbolometer, 12 um pitch, 13.5 mm lens, DFOV 40.6 deg,
    # gimbal pitch -120 to +30 deg (the planner only looks down: -90 to 0).
    "h20t": Camera("DJI Zenmuse H20T (thermal)", 640, 512, 12.0, 13.5, (-90.0, 0.0)),
}
