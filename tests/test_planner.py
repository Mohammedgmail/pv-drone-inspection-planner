import json

import numpy as np
import pytest

from pvmission.planner import WP_RADIUS_M, Drone, move_s, plan


def test_noon_plan_is_feasible_and_covers_every_row_once(cfg, noon):
    assert noon.feasible, noon.message
    covered = sorted(r for ln in noon.lines for r in ln.rows)
    assert covered == list(range(len(cfg["plant"].rows)))
    assert noon.rows_per_line == 2


def test_lines_meet_the_requirements(cfg, noon):
    req = cfg["req"]
    for ln in noon.lines:
        c = ln.config
        assert c.min_glare_deg >= req.min_glare_deg
        assert (
            req.view_angle_deg[0]
            <= c.view_angle_deg[0]
            <= c.view_angle_deg[1]
            <= req.view_angle_deg[1]
        )
        assert c.max_gsd_m <= req.max_gsd_m + 1e-9
        assert -90 <= c.pitch_deg <= 0
        assert 0 < c.speed <= cfg["drone"].max_speed
        # motion blur in pixels at the closest point (finest pixels) stays within the limit
        tau = cfg["camera"].time_constant_ms * 1e-3
        rows = [cfg["plant"].rows[i] for i in ln.rows]
        edges = np.array(
            [r.point(0.0, t) for r in rows for t in np.linspace(-r.width / 2, r.width / 2, 9)]
        )
        closest = np.linalg.norm(edges - c.position, axis=1).min()
        assert c.speed * tau / cfg["camera"].gsd(closest) <= req.max_blur_px + 1e-6


def test_schedule_is_inside_the_window_and_the_battery(cfg, noon):
    w = noon.window
    assert w.start <= noon.start and noon.end <= w.end
    budget = cfg["drone"].budget_s / 60
    for s in noon.sorties:
        assert s.minutes <= budget + 1e-6
    for a, b in zip(noon.lines, noon.lines[1:], strict=False):
        assert a.t_end <= b.t_start


def test_small_battery_splits_into_sorties_with_swaps(cfg):
    drone = Drone(usable_minutes=8, reserve=0.2, swap_minutes=5)
    m = plan(cfg["plant"], "2026-06-21", cfg["camera"], cfg["req"], drone, start="11:00")
    assert m.feasible, m.message
    assert len(m.sorties) >= 3
    for a, b in zip(m.sorties, m.sorties[1:], strict=False):
        assert (b.t_takeoff - a.t_land).total_seconds() == pytest.approx(300, abs=1)
        assert a.minutes <= 8 * 0.8 + 1e-6
    assert sum(len(s.lines) for s in m.sorties) == len(m.lines)


def test_december_has_no_valid_window(cfg):
    m = plan(cfg["plant"], "2026-12-21", cfg["camera"], cfg["req"], cfg["drone"])
    assert not m.feasible and "peaks at" in m.message and m.lines == []


def test_start_time_is_respected(cfg, noon):
    assert noon.start.strftime("%H:%M") == "13:00"


def test_late_start_that_overruns_the_window_is_reported(cfg):
    m = plan(cfg["plant"], "2026-06-21", cfg["camera"], cfg["req"], cfg["drone"], start="16:40")
    assert not m.feasible and "after the irradiance window closes" in m.message


def test_image_count_matches_autopilot_triggering(noon):
    for ln in noon.lines:
        expected = int((ln.length - WP_RADIUS_M) // ln.config.trigger_m) + 1
        assert ln.n_images == expected == len(ln.image_positions())


def test_move_time_profile():
    assert move_s(0, 5, 2.5) == 0
    long = move_s(100, 5, 2.5, 1.0)
    assert long == pytest.approx(100 / 5 + 5 / 2.5 + 2.5)
    short = move_s(4, 8, 2.5)  # never reaches 8 m/s
    assert short == pytest.approx(2 * np.sqrt(4 / 2.5))
    assert move_s(9.99, 5, 2.5) == pytest.approx(move_s(10.01, 5, 2.5), abs=0.01)  # continuous


def test_summary_is_json_serialisable(noon):
    s = json.loads(json.dumps(noon.summary()))
    assert s["feasible"] and s["images"] == noon.n_images
