"""Sun position, irradiance on the module plane and the valid inspection window.

The window is where plane-of-array (POA) irradiance is at least the minimum (600 W/m2 in
IEC TS 62446-3, as cited by IEA PVPS and industry guides) and, with a forecast, the wind is
below the limit (28 km/h). Without a forecast, a clear sky is assumed (pvlib Ineichen with
the Linke turbidity climatology), which gives the best case for that date.
"""

from __future__ import annotations

import json
import urllib.parse
import urllib.request
from dataclasses import dataclass

import numpy as np
import pandas as pd
import pvlib

from .plant import Plant, direction

OPEN_METEO = "https://api.open-meteo.com/v1/forecast"


def sun_vector(elevation_deg, azimuth_deg) -> np.ndarray:
    """Unit ENU vector(s) pointing to the sun."""
    el, az = np.radians(np.asarray(elevation_deg)), np.radians(np.asarray(azimuth_deg))
    return np.stack([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)], axis=-1)


def reflection(sun: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """Direction of the sun's mirror reflection off a surface with unit ``normal``."""
    return 2.0 * (sun @ normal)[..., None] * normal - sun


def solar_position(plant: Plant, times: pd.DatetimeIndex) -> pd.DataFrame:
    loc = pvlib.location.Location(
        plant.frame.latitude, plant.frame.longitude, plant.timezone, plant.frame.altitude
    )
    return loc.get_solarposition(times)


def clear_sky_poa(plant: Plant, times: pd.DatetimeIndex) -> pd.DataFrame:
    """Clear-sky irradiance on the module plane (W/m2) and the sun position."""
    loc = pvlib.location.Location(
        plant.frame.latitude, plant.frame.longitude, plant.timezone, plant.frame.altitude
    )
    sp = loc.get_solarposition(times)
    cs = loc.get_clearsky(times, solar_position=sp)
    row = plant.rows[0]
    poa = pvlib.irradiance.get_total_irradiance(
        row.tilt,
        row.azimuth,
        sp.apparent_zenith,
        sp.azimuth,
        cs.dni,
        cs.ghi,
        cs.dhi,
        dni_extra=pvlib.irradiance.get_extra_radiation(times),
        model="haydavies",
    )
    return pd.DataFrame(
        {
            "poa": poa.poa_global.fillna(0.0),
            "elevation": sp.apparent_elevation,
            "azimuth": sp.azimuth,
            "wind_kmh": np.nan,
        },
        index=times,
    )


def fetch_forecast(plant: Plant, days: int = 3, timeout: float = 30) -> dict:
    """Hourly irradiance and wind forecast from Open-Meteo (free, no key; CC BY 4.0)."""
    query = urllib.parse.urlencode(
        {
            "latitude": plant.frame.latitude,
            "longitude": plant.frame.longitude,
            "hourly": "shortwave_radiation,direct_normal_irradiance,diffuse_radiation,"
            "wind_speed_10m,wind_gusts_10m,cloud_cover",
            "forecast_days": days,
            "timezone": "UTC",
            "wind_speed_unit": "kmh",
        }
    )
    req = urllib.request.Request(f"{OPEN_METEO}?{query}", headers={"User-Agent": "pvmission"})
    with urllib.request.urlopen(req, timeout=timeout) as r:  # noqa: S310 (fixed https URL)
        return json.load(r)


def forecast_poa(plant: Plant, payload: dict, times: pd.DatetimeIndex) -> pd.DataFrame:
    """POA irradiance and wind from an Open-Meteo payload, interpolated to ``times``.

    Open-Meteo radiation values are means over the preceding hour, so they are placed at
    the middle of that hour before interpolating.
    """
    h = payload["hourly"]
    idx = pd.to_datetime(h["time"]).tz_localize("UTC") - pd.Timedelta("30min")
    wx = pd.DataFrame(
        {
            "ghi": h["shortwave_radiation"],
            "dni": h["direct_normal_irradiance"],
            "dhi": h["diffuse_radiation"],
            "wind_kmh": h["wind_speed_10m"],
            "gust_kmh": h.get("wind_gusts_10m", [np.nan] * len(idx)),
            "cloud": h.get("cloud_cover", [np.nan] * len(idx)),
        },
        index=idx,
        dtype=float,
    )
    t = times.tz_convert("UTC")
    wx = wx.reindex(wx.index.union(t)).interpolate("time").reindex(t)
    sp = solar_position(plant, times)
    row = plant.rows[0]
    poa = pvlib.irradiance.get_total_irradiance(
        row.tilt,
        row.azimuth,
        sp.apparent_zenith.to_numpy(),
        sp.azimuth.to_numpy(),
        wx.dni.to_numpy(),
        wx.ghi.to_numpy(),
        wx.dhi.to_numpy(),
        dni_extra=pvlib.irradiance.get_extra_radiation(times).to_numpy(),
        model="haydavies",
    )
    return pd.DataFrame(
        {
            "poa": np.nan_to_num(np.asarray(poa["poa_global"], dtype=float)),
            "elevation": sp.apparent_elevation.to_numpy(),
            "azimuth": sp.azimuth.to_numpy(),
            "wind_kmh": wx.wind_kmh.to_numpy(),
            "cloud": wx.cloud.to_numpy(),
        },
        index=times,
    )


@dataclass
class Window:
    start: pd.Timestamp
    end: pd.Timestamp

    @property
    def hours(self) -> float:
        return (self.end - self.start).total_seconds() / 3600


def valid_windows(
    conditions: pd.DataFrame, min_poa: float = 600.0, max_wind_kmh: float = 28.0
) -> list[Window]:
    """Contiguous periods meeting the irradiance and wind limits (NaN wind = not limiting)."""
    ok = (conditions.poa >= min_poa) & ~(conditions.wind_kmh > max_wind_kmh)
    windows, start = [], None
    for t, good in ok.items():
        if good and start is None:
            start = t
        elif not good and start is not None:
            windows.append(Window(start, t))
            start = None
    if start is not None:
        windows.append(Window(start, ok.index[-1]))
    return windows


def day_times(plant: Plant, date: str, step: str = "1min") -> pd.DatetimeIndex:
    day = pd.Timestamp(date).tz_localize(plant.timezone)
    return pd.date_range(day, day + pd.Timedelta("1D"), freq=step, inclusive="left")


def facing(plant: Plant) -> np.ndarray:
    return direction(plant.rows[0].azimuth)
