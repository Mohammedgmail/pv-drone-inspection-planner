"""The placement search, checked against an independent re-derivation of every constraint."""

import numpy as np
import pytest

from pvmission.geometry import Requirements, best_config, evaluate
from pvmission.plant import direction
from pvmission.sun import reflection, solar_position, sun_vector


def independent_check(rows, cfg, sun, camera, req, others, n=400, seed=0):
    """Random points on the covered modules, each seen from its nearest frame, re-checked
    from the gimbal pitch and heading (not from the optical axis the planner used)."""
    rng = np.random.default_rng(seed)
    o = direction(cfg.yaw_deg, cfg.pitch_deg)  # optical axis from heading + pitch
    x = direction(cfg.yaw_deg + 90)  # image width: horizontal, square to the heading
    y = np.cross(o, x)
    tx, ty = np.tan(camera.hfov / 2), np.tan(camera.vfov / 2)
    refl = reflection(sun, rows[0].normal)
    worst = {"glare": 180.0, "view_lo": 90.0, "view_hi": 0.0, "gsd": 0.0}
    for _ in range(n):
        r = rows[rng.integers(len(rows))]
        s, t = rng.uniform(-r.length / 2, r.length / 2), rng.uniform(-r.width / 2, r.width / 2)
        P = r.point(s, t)
        s_frame = np.round(s / cfg.trigger_m) * cfg.trigger_m  # nearest image along the line
        C = cfg.position + s_frame * r.along
        v = (C - P) / np.linalg.norm(C - P)
        view = np.degrees(np.arccos(v @ r.normal))
        glare = np.degrees(np.arccos(np.clip(v @ refl, -1, 1)))
        gsd = camera.gsd(np.linalg.norm(C - P)) / np.cos(np.radians(view))
        d = P - C
        assert d @ o > 0
        assert abs(d @ x) / (d @ o) <= tx and abs(d @ y) / (d @ o) <= ty, "outside the image"
        for other in others:  # nothing in between
            denom = (C - P) @ other.normal
            k = ((other.center - P) @ other.normal) / denom
            if 0 < k < 1:
                X = P + k * (C - P) - other.center
                assert not (
                    abs(X @ other.along) <= other.length / 2
                    and abs(X @ other.upslope) <= other.width / 2
                ), "occluded"
        worst["glare"] = min(worst["glare"], glare)
        worst["view_lo"], worst["view_hi"] = (
            min(worst["view_lo"], view),
            max(worst["view_hi"], view),
        )
        worst["gsd"] = max(worst["gsd"], gsd)
    lo, hi = req.view_angle_deg
    assert worst["glare"] >= req.min_glare_deg - 0.5
    assert lo - 0.5 <= worst["view_lo"] and worst["view_hi"] <= hi + 0.5
    assert worst["gsd"] <= req.max_gsd_m * 1.02
    return worst


@pytest.mark.parametrize(
    "date_time", ["2026-06-21 13:07", "2026-06-21 09:45", "2026-06-21 16:30", "2026-03-21 12:10"]
)
@pytest.mark.parametrize("k", [1, 2])
def test_chosen_placement_meets_every_constraint(cfg, date_time, k):
    plant, camera, req = cfg["plant"], cfg["camera"], cfg["req"]
    import pandas as pd

    sp = solar_position(plant, pd.DatetimeIndex([pd.Timestamp(date_time, tz=plant.timezone)]))
    sun = sun_vector(sp.apparent_elevation.iloc[0], sp.azimuth.iloc[0])
    rows = plant.rows[8 : 8 + k]
    others = [r for r in plant.rows if r not in rows]
    c = best_config(rows, sun, camera, req, others)
    assert c is not None
    worst = independent_check(rows, c, sun, camera, req, others)
    assert c.min_glare_deg == pytest.approx(worst["glare"], abs=3.0)


def test_noon_sun_rules_out_the_morning_placement(cfg, morning, noon):
    """The plan made for 09:30 would see the sun's reflection at noon: the planner moves."""
    plant, camera, req = cfg["plant"], cfg["camera"], cfg["req"]
    m = morning.lines[4].config
    rows = [plant.rows[i] for i in morning.lines[4].rows]
    import pandas as pd

    sp = solar_position(
        plant, pd.DatetimeIndex([pd.Timestamp("2026-06-21 13:07", tz=plant.timezone)])
    )
    sun = sun_vector(sp.apparent_elevation.iloc[0], sp.azimuth.iloc[0])
    ev = evaluate(rows, np.array([m.height]), np.array([m.offset]), sun, camera, req)
    assert ev["glare"][0] < req.min_glare_deg and not ev["valid"][0]
    n = noon.lines[0].config
    assert (n.height, n.offset) != (m.height, m.offset)
    assert n.min_glare_deg >= req.min_glare_deg


def test_three_rows_do_not_fit_in_the_image(cfg):
    plant, camera, req = cfg["plant"], cfg["camera"], cfg["req"]
    sun = sun_vector(45, 120)
    assert best_config(plant.rows[8:11], sun, camera, Requirements(), plant.rows[6:8]) is None
    assert best_config(plant.rows[8:10], sun, camera, req, plant.rows[6:8]) is not None


def test_occlusion_by_the_next_row(cfg):
    from pvmission.geometry import _occluded

    plant = cfg["plant"]
    row, front = plant.rows[10], plant.rows[9]  # row 9 stands 7 m in front of row 10
    P = row.point(0.0, 0.0)
    low = P + np.array([0.0, -12.0, -0.4])  # 12 m in front, lower than the module centre
    high = P + np.array([0.0, -12.0, 25.0])
    assert _occluded(P[None], low[None], [front])[0]
    assert not _occluded(P[None], high[None], [front])[0]
    assert not _occluded(P[None], low[None], [plant.rows[11]])[
        0
    ]  # the row behind is not in the way


def test_no_glare_when_the_sun_is_behind_the_modules(cfg):
    plant, camera, req = cfg["plant"], cfg["camera"], cfg["req"]
    ev = evaluate(
        [plant.rows[5]], np.array([30.0]), np.array([0.0]), sun_vector(10, 0), camera, req
    )
    assert ev["glare"][0] == 180.0
