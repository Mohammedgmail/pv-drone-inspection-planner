"""PV plant layout in a local east-north-up (ENU) frame, plus WGS84 conversion.

Coordinates are metres: x east, y north, z up, origin at the site reference point on the
ground. Azimuths are degrees clockwise from north (180 = modules facing south).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# WGS84 ellipsoid
_A = 6378137.0
_E2 = 6.69437999014e-3


def _radii(lat_deg: float) -> tuple[float, float]:
    """Meridian and prime-vertical radii of curvature (m) at a latitude."""
    s = np.sin(np.radians(lat_deg))
    w = np.sqrt(1.0 - _E2 * s * s)
    return _A * (1.0 - _E2) / w**3, _A / w


@dataclass(frozen=True)
class GeoFrame:
    """Local tangent plane at (lat, lon, alt). Accurate to millimetres over a few km."""

    latitude: float
    longitude: float
    altitude: float = 0.0  # ground altitude above mean sea level (m)

    def to_geodetic(self, east, north) -> tuple[np.ndarray, np.ndarray]:
        r_n, r_e = _radii(self.latitude)
        lat = self.latitude + np.degrees(np.asarray(north) / r_n)
        lon = self.longitude + np.degrees(
            np.asarray(east) / (r_e * np.cos(np.radians(self.latitude)))
        )
        return lat, lon

    def to_local(self, lat, lon) -> tuple[np.ndarray, np.ndarray]:
        r_n, r_e = _radii(self.latitude)
        north = np.radians(np.asarray(lat) - self.latitude) * r_n
        east = (
            np.radians(np.asarray(lon) - self.longitude) * r_e * np.cos(np.radians(self.latitude))
        )
        return east, north


def direction(azimuth_deg: float, elevation_deg: float = 0.0) -> np.ndarray:
    """Unit ENU vector for an azimuth (clockwise from north) and elevation."""
    az, el = np.radians(azimuth_deg), np.radians(elevation_deg)
    return np.array([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)])


@dataclass(frozen=True)
class Row:
    """One fixed-tilt table: a long tilted rectangle of modules.

    ``center`` is the middle of the module surface (ENU, m). The surface spans ``length``
    along the row and ``width`` up the slope.
    """

    index: int
    center: np.ndarray
    length: float
    width: float
    tilt: float
    azimuth: float
    modules_along: int
    modules_across: int

    @property
    def normal(self) -> np.ndarray:
        """Unit normal of the module surface (points to the sky side the modules face)."""
        return direction(self.azimuth, 90.0 - self.tilt)

    @property
    def along(self) -> np.ndarray:
        """Unit vector along the row (horizontal, 90 degrees clockwise from the azimuth)."""
        return direction(self.azimuth + 90.0)

    @property
    def upslope(self) -> np.ndarray:
        """Unit vector in the surface from the lower edge to the upper edge."""
        return np.cross(self.along, self.normal)

    def point(self, s, t) -> np.ndarray:
        """Point at ``s`` metres along the row and ``t`` metres up the slope from the centre."""
        s, t = np.asarray(s, dtype=float), np.asarray(t, dtype=float)
        return self.center + s[..., None] * self.along + t[..., None] * self.upslope

    def corners(self) -> np.ndarray:
        h, w = self.length / 2, self.width / 2
        return self.point(np.array([-h, h, h, -h]), np.array([-w, -w, w, w]))

    def module_id(self, s: float, t: float) -> str | None:
        """ID 'R<row>-C<column>-<level>' of the module at (s, t), or None if outside."""
        col = int(np.floor((s + self.length / 2) / self.length * self.modules_along))
        lvl = int(np.floor((t + self.width / 2) / self.width * self.modules_across))
        if not (0 <= col < self.modules_along and 0 <= lvl < self.modules_across):
            return None
        return f"R{self.index + 1:02d}-C{col + 1:03d}-{lvl + 1}"


@dataclass
class Plant:
    name: str
    frame: GeoFrame
    rows: list[Row]
    timezone: str = "UTC"
    module_power_w: float = 0.0
    extras: dict = field(default_factory=dict)

    @classmethod
    def regular(
        cls,
        name: str,
        latitude: float,
        longitude: float,
        altitude: float = 0.0,
        timezone: str = "UTC",
        rows: int = 20,
        row_length: float = 120.0,
        row_pitch: float = 7.0,
        tilt: float = 30.0,
        azimuth: float = 180.0,
        modules_across: int = 2,
        module_length: float = 2.278,
        module_width: float = 1.134,
        lower_edge_height: float = 0.8,
        module_power_w: float = 0.0,
        orientation: str = "portrait",
    ) -> Plant:
        """Parallel rows of identical tables, centred on the reference point.

        ``orientation`` is "portrait" (long side of the module up the slope, e.g. 2P tables)
        or "landscape" (long side along the row, e.g. 4L tables).
        """
        if orientation not in ("portrait", "landscape"):
            raise ValueError(f"orientation must be portrait or landscape, not {orientation!r}")
        up, along = (
            (module_length, module_width)
            if orientation == "portrait"
            else (module_width, module_length)
        )
        width = modules_across * up
        modules_along = int(row_length // along)
        z = lower_edge_height + width / 2 * np.sin(np.radians(tilt))
        facing = direction(azimuth)
        out = []
        for i in range(rows):
            offset = ((rows - 1) / 2 - i) * row_pitch  # row 1 is the front (facing side) row
            center = offset * facing + np.array([0.0, 0.0, z])
            out.append(
                Row(
                    i,
                    center,
                    modules_along * along,
                    width,
                    tilt,
                    azimuth,
                    modules_along,
                    modules_across,
                )
            )
        return cls(name, GeoFrame(latitude, longitude, altitude), out, timezone, module_power_w)

    @property
    def n_modules(self) -> int:
        return sum(r.modules_along * r.modules_across for r in self.rows)

    @property
    def capacity_kwp(self) -> float:
        return self.n_modules * self.module_power_w / 1000.0

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        pts = np.vstack([r.corners() for r in self.rows])
        return pts.min(axis=0), pts.max(axis=0)
