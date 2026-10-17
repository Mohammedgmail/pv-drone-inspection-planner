import numpy as np
import pandas as pd
import pytest

from pvmission.camera import CAMERAS
from pvmission.plant import GeoFrame, Plant, direction
from pvmission.sun import (
    clear_sky_poa,
    day_times,
    forecast_poa,
    reflection,
    sun_vector,
    valid_windows,
)


def test_local_frame_round_trip_and_distances():
    fr = GeoFrame(55.05, -1.70, 80)
    east, north = np.array([0.0, 500.0, -300.0]), np.array([0.0, 0.0, 400.0])
    lat, lon = fr.to_geodetic(east, north)
    e2, n2 = fr.to_local(lat, lon)
    np.testing.assert_allclose(e2, east, atol=1e-6)
    np.testing.assert_allclose(n2, north, atol=1e-6)
    geod = pytest.importorskip("geographiclib.geodesic").Geodesic.WGS84
    for i, j in ((0, 1), (0, 2), (1, 2)):  # exact distances on the WGS84 ellipsoid
        d = geod.Inverse(lat[i], lon[i], lat[j], lon[j])["s12"]
        assert d == pytest.approx(np.hypot(east[j] - east[i], north[j] - north[i]), abs=0.05)


def test_row_frame_is_orthonormal_and_tilted():
    p = Plant.regular("t", 55.0, -1.7, rows=3, tilt=30, azimuth=180)
    r = p.rows[0]
    m = np.stack([r.along, r.upslope, r.normal])
    np.testing.assert_allclose(m @ m.T, np.eye(3), atol=1e-12)
    assert np.degrees(np.arccos(r.normal[2])) == pytest.approx(30)
    assert r.normal[1] < 0 and r.upslope[1] > 0  # faces south, rises to the north
    lo, hi = r.corners()[:, 2].min(), r.corners()[:, 2].max()
    assert lo == pytest.approx(0.8) and hi == pytest.approx(0.8 + r.width * 0.5)
    assert p.rows[0].center[1] < p.rows[1].center[1]  # row 1 is the front (south) row


def test_module_ids_cover_the_row():
    r = Plant.regular("t", 55.0, -1.7, rows=1).rows[0]
    assert r.module_id(-r.length / 2 + 0.01, -r.width / 2 + 0.01) == "R01-C001-1"
    assert r.module_id(r.length / 2 - 0.01, r.width / 2 - 0.01) == f"R01-C{r.modules_along:03d}-2"
    assert r.module_id(r.length, 0) is None


def test_camera_matches_the_datasheet():
    cam = CAMERAS["h20t"]
    assert np.degrees(cam.dfov) == pytest.approx(40.6, abs=1.0)  # DJI: DFOV 40.6 deg
    assert cam.gsd(33.75) == pytest.approx(0.03)  # 12 um / 13.5 mm = 0.889 mm per metre


def test_reflection_law():
    rng = np.random.default_rng(1)
    n = direction(180, 60)
    for _ in range(100):
        s = sun_vector(rng.uniform(5, 85), rng.uniform(0, 360))
        r = reflection(s, n)
        assert np.linalg.norm(r) == pytest.approx(1)
        assert r @ n == pytest.approx(s @ n)  # equal angles to the normal
        assert np.cross(s, n) @ r == pytest.approx(0, abs=1e-12)  # in the plane of s and n


def test_clear_sky_window_by_season(cfg):
    plant = cfg["plant"]
    june = valid_windows(clear_sky_poa(plant, day_times(plant, "2026-06-21")))
    dec = clear_sky_poa(plant, day_times(plant, "2026-12-21"))
    assert len(june) == 1 and 6.5 < june[0].hours < 8
    assert june[0].start.hour < 13 < june[0].end.hour  # around solar noon (13:07 BST)
    assert dec.poa.max() < 600 and valid_windows(dec) == []


def test_wind_closes_the_window():
    idx = pd.date_range("2026-06-21 10:00", periods=6, freq="1h", tz="UTC")
    cond = pd.DataFrame({"poa": 800.0, "wind_kmh": [10, 10, 35, 35, 10, np.nan]}, index=idx)
    w = valid_windows(cond)
    assert [(x.start.hour, x.end.hour) for x in w] == [(10, 12), (14, 15)]


def test_forecast_payload_matches_clear_sky_on_a_clear_day(cfg):
    plant = cfg["plant"]
    times = day_times(plant, "2026-06-21", "10min")
    cs = clear_sky_poa(plant, pd.date_range(times[0], times[-1], freq="1min"))
    loc = __import__("pvlib").location.Location(55.05, -1.70, plant.timezone, 80)
    hours = pd.date_range("2026-06-20 23:00", periods=26, freq="1h", tz="UTC")
    mid = hours - pd.Timedelta("30min")  # hourly means over the preceding hour
    sky = loc.get_clearsky(mid)
    payload = {
        "hourly": {
            "time": [t.strftime("%Y-%m-%dT%H:%M") for t in hours.tz_localize(None)],
            "shortwave_radiation": sky.ghi.round(1).tolist(),
            "direct_normal_irradiance": sky.dni.round(1).tolist(),
            "diffuse_radiation": sky.dhi.round(1).tolist(),
            "wind_speed_10m": [12.0] * 26,
            "cloud_cover": [5] * 26,
        }
    }
    fc = forecast_poa(plant, payload, times)
    noon = fc.between_time("12:30", "13:30")
    ref = cs.poa.reindex(noon.index)
    assert np.abs(noon.poa - ref).max() < 40  # W/m2, only hourly-interpolation error
    assert (fc.wind_kmh == 12).all()
