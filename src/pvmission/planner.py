"""Mission planning: flight lines, camera placement per line, batteries and timing.

Rows are covered ``k`` at a time (k = 1, 2 or 3, whichever finishes soonest). Lines are flown
front to back in alternating directions. Each line gets the fastest valid camera placement
for the sun positions at its own start, middle and end, so the plan follows the sun through
the morning. Lines are packed into battery sorties, and the mission must fit inside one valid
irradiance window.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd

from .camera import Camera
from .geometry import LineConfig, Requirements, best_config
from .plant import Plant, Row, direction
from .sun import Window, clear_sky_poa, day_times, sun_vector, valid_windows

WP_RADIUS_M = 2.0  # ArduCopter counts a waypoint as reached this far out (WPNAV_RADIUS_M)


def move_s(distance: float, speed: float, accel: float, jerk: float | None = None) -> float:
    """Time to move ``distance`` from rest to rest with speed, acceleration and jerk limits.

    Trapezoidal profile plus ``accel / jerk`` for the S-curve ramps; short moves that never
    reach ``speed`` are accelerate-then-brake.
    """
    if distance <= 0:
        return 0.0
    ramp = accel / jerk if jerk else 0.0
    if distance >= speed * speed / accel:
        return distance / speed + speed / accel + ramp
    return 2.0 * float(np.sqrt(distance / accel)) + ramp


@dataclass(frozen=True)
class Drone:
    """Flight performance. The defaults after ``transit_speed`` are ArduCopter 4.7's:
    WPNAV_SPD_UP 2.5 m/s, WPNAV_SPD_DN 1.5 m/s, WPNAV_ACC 2.5 m/s2, WPNAV_JERK 1 m/s3,
    WPNAV_ACC_Z 1 m/s2, final landing at 0.5 m/s below 10 m, RTL_ALT 15 m and a 5 s
    loiter above home before landing. ArduCopter stops at every waypoint."""

    max_speed: float = 5.0  # m/s along lines (motion blur usually sets a lower limit)
    transit_speed: float = 8.0  # m/s between lines and to and from home
    climb_speed: float = 2.5  # m/s
    descent_speed: float = 1.5  # m/s
    accel: float = 2.5  # m/s2, horizontal
    jerk: float = 1.0  # m/s3, horizontal
    accel_z: float = 1.0  # m/s2, vertical
    land_speed: float = 0.5  # m/s for the last land_slow_m metres
    land_slow_m: float = 10.0
    rtl_height_m: float = 15.0  # minimum height on the way home
    rtl_loiter_s: float = 5.0
    hold_s: float = 2.0  # pause at each line start so heading and gimbal settle
    usable_minutes: float = 25.0  # flight time per battery before the reserve
    reserve: float = 0.2  # fraction of usable_minutes kept for contingencies
    swap_minutes: float = 5.0  # landing to next take-off

    @property
    def budget_s(self) -> float:
        return self.usable_minutes * 60 * (1 - self.reserve)

    def vertical_s(self, dz: float) -> float:
        speed = self.climb_speed if dz > 0 else self.descent_speed
        return move_s(abs(dz), speed, self.accel_z)

    def travel_s(self, a: np.ndarray, b: np.ndarray) -> float:
        """Waypoint to waypoint (horizontal and vertical motion happen together)."""
        horizontal = move_s(
            float(np.linalg.norm(b[:2] - a[:2])), self.transit_speed, self.accel, self.jerk
        )
        return max(horizontal, self.vertical_s(float(b[2] - a[2])))

    def line_s(self, length: float, speed: float) -> float:
        return self.hold_s + move_s(length, speed, self.accel, self.jerk)

    def home_and_land_s(self, pos: np.ndarray, home: np.ndarray) -> float:
        """Return from ``pos`` and land at ``home`` (ArduCopter RTL)."""
        h = max(float(pos[2]), self.rtl_height_m)
        t = self.vertical_s(h - pos[2]) + self.travel_s(
            np.array([*pos[:2], h]), np.array([*home[:2], h])
        )
        slow = min(h, self.land_slow_m)
        t += self.rtl_loiter_s + move_s(h - slow, self.descent_speed, self.accel_z)
        return float(t + slow / self.land_speed)


@dataclass
class Line:
    index: int
    rows: list[int]
    config: LineConfig
    start: np.ndarray  # ENU of the first and last camera position (z = height AGL)
    end: np.ndarray
    t_start: pd.Timestamp
    t_end: pd.Timestamp
    sortie: int = 0

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.end[:2] - self.start[:2]))

    @property
    def n_images(self) -> int:
        """Images triggered by distance from the line start until the end waypoint counts as
        reached (WP_RADIUS_M early), when the next mission item stops the camera."""
        return int(np.floor((self.length - WP_RADIUS_M) / self.config.trigger_m)) + 1

    def image_positions(self) -> np.ndarray:
        n = self.n_images
        u = (self.end - self.start) / max(self.length, 1e-9)
        return self.start + np.outer(np.arange(n) * self.config.trigger_m, u)


@dataclass
class Sortie:
    index: int
    lines: list[Line]
    t_takeoff: pd.Timestamp
    t_land: pd.Timestamp

    @property
    def minutes(self) -> float:
        return (self.t_land - self.t_takeoff).total_seconds() / 60


@dataclass
class Mission:
    plant: Plant
    camera: Camera
    req: Requirements
    drone: Drone
    home: np.ndarray
    date: str
    conditions: pd.DataFrame
    window: Window | None
    rows_per_line: int = 0
    lines: list[Line] = field(default_factory=list)
    sorties: list[Sortie] = field(default_factory=list)
    feasible: bool = False
    message: str = ""
    source: str = "clear sky"

    @property
    def n_images(self) -> int:
        return sum(ln.n_images for ln in self.lines)

    @property
    def start(self) -> pd.Timestamp | None:
        return self.sorties[0].t_takeoff if self.sorties else None

    @property
    def end(self) -> pd.Timestamp | None:
        return self.sorties[-1].t_land if self.sorties else None

    def summary(self) -> dict:
        lines = self.lines
        return {
            "feasible": self.feasible,
            "message": self.message,
            "conditions": self.source,
            "window": None
            if self.window is None
            else [self.window.start.isoformat(), self.window.end.isoformat()],
            "rows_per_line": self.rows_per_line,
            "lines": len(lines),
            "sorties": len(self.sorties),
            "images": self.n_images,
            "start": None if self.start is None else self.start.isoformat(),
            "end": None if self.end is None else self.end.isoformat(),
            "flight_minutes": round(sum(s.minutes for s in self.sorties), 1),
            "min_glare_deg": round(min(ln.config.min_glare_deg for ln in lines), 1)
            if lines
            else None,
            "view_angle_deg": [
                round(min(ln.config.view_angle_deg[0] for ln in lines), 1),
                round(max(ln.config.view_angle_deg[1] for ln in lines), 1),
            ]
            if lines
            else None,
            "max_gsd_cm": round(100 * max(ln.config.max_gsd_m for ln in lines), 2)
            if lines
            else None,
        }


def default_home(plant: Plant) -> np.ndarray:
    """Take-off point 15 m in front of the front row, level with its end."""
    front = plant.rows[0]
    f = direction(front.azimuth)
    p = front.center - front.along * (front.length / 2) + f * (front.width / 2 + 15.0)
    return np.array([p[0], p[1], 0.0])


def _suns(plant: Plant, times: list[pd.Timestamp]) -> np.ndarray:
    from .sun import solar_position

    sp = solar_position(plant, pd.DatetimeIndex(times))
    return sun_vector(sp.apparent_elevation.to_numpy(), sp.azimuth.to_numpy())


def _travel_s(a: np.ndarray, b: np.ndarray, drone: Drone) -> float:
    return drone.travel_s(a, b)


def _line_geometry(rows: list[Row], cfg: LineConfig, forward: bool):
    """Line ends: half an image spacing beyond the rows (so every module has an image within
    half a spacing of its centre, where the geometry was checked), plus the waypoint radius
    at the far end because triggering stops when the end waypoint counts as reached."""
    a = rows[0].along
    half = rows[0].length / 2 + cfg.trigger_m / 2 + WP_RADIUS_M
    p0, p1 = cfg.position - half * a, cfg.position + half * a
    return (p0, p1) if forward else (p1, p0)


def _schedule(
    plant: Plant,
    k: int,
    t0: pd.Timestamp,
    window: Window,
    camera: Camera,
    req: Requirements,
    drone: Drone,
    home: np.ndarray,
) -> tuple[list[Line], list[Sortie], str]:
    groups = [list(range(i, min(i + k, len(plant.rows)))) for i in range(0, len(plant.rows), k)]
    lines: list[Line] = []
    sorties: list[Sortie] = []
    t = t0
    pos = home.copy()
    in_air, sortie_start, sortie_lines = False, None, []
    budget = drone.budget_s
    used = 0.0

    def land(t, pos):
        nonlocal in_air
        t = t + pd.Timedelta(seconds=drone.home_and_land_s(pos, home))
        sorties.append(Sortie(len(sorties), list(sortie_lines), sortie_start, t))
        in_air = False
        return t

    for gi, group in enumerate(groups):
        rows = [plant.rows[i] for i in group]
        neighbours = [
            r for r in plant.rows if r.index not in group and abs(r.index - group[0]) <= k + 2
        ]
        forward = gi % 2 == 0
        cfg = None
        for _ in range(2):  # placement depends on time, and line duration on placement
            guess = 120.0 if cfg is None else (rows[0].length + cfg.footprint_m) / cfg.speed
            times = [t, t + pd.Timedelta(seconds=guess / 2), t + pd.Timedelta(seconds=guess + 60)]
            cfg = best_config(rows, _suns(plant, times), camera, req, neighbours, drone.max_speed)
            if cfg is None:
                return (
                    lines,
                    sorties,
                    (
                        f"no valid camera placement for rows {group[0] + 1}-{group[-1] + 1} "
                        f"around {t:%H:%M}"
                    ),
                )
        p0, p1 = _line_geometry(rows, cfg, forward)
        fly = drone.line_s(float(np.linalg.norm(p1 - p0)), cfg.speed)
        if in_air:
            to_line = _travel_s(pos, p0, drone)
            needed = used + to_line + fly + drone.home_and_land_s(p1, home)
            if needed > budget:  # next line does not fit: go home and swap batteries
                t = land(t, pos) + pd.Timedelta(minutes=drone.swap_minutes)
                pos = home.copy()
        if not in_air:
            in_air, sortie_start, sortie_lines, used = True, t, [], 0.0
            climb = home + np.array([0, 0, p0[2]])
            to_line = drone.vertical_s(float(p0[2])) + _travel_s(climb, p0, drone)
            if to_line + fly + drone.home_and_land_s(p1, home) > budget:
                return lines, sorties, "a single line is longer than one battery allows"
        t_start = t + pd.Timedelta(seconds=to_line)
        t_end = t_start + pd.Timedelta(seconds=fly)
        line = Line(len(lines), group, cfg, p0, p1, t_start, t_end, len(sorties))
        lines.append(line)
        sortie_lines.append(line)
        used += to_line + fly
        t, pos = t_end, p1
    t = land(t, pos)
    if t > window.end:
        late = (t - window.end).total_seconds() / 60
        return lines, sorties, f"finishes {late:.0f} min after the irradiance window closes"
    return lines, sorties, ""


def plan(
    plant: Plant,
    date: str,
    camera: Camera,
    req: Requirements | None = None,
    drone: Drone | None = None,
    home: np.ndarray | None = None,
    start: str | None = None,
    conditions: pd.DataFrame | None = None,
    source: str = "clear sky",
) -> Mission:
    """Plan the inspection of ``plant`` on ``date`` (local), optionally from ``start`` (HH:MM)."""
    req = Requirements() if req is None else req
    drone = Drone() if drone is None else drone
    home = default_home(plant) if home is None else np.asarray(home, dtype=float)
    if conditions is None:
        conditions = clear_sky_poa(plant, day_times(plant, date))
    mission = Mission(plant, camera, req, drone, home, date, conditions, None, source=source)
    windows = valid_windows(conditions, req.min_poa, req.max_wind_kmh)
    if start is not None:
        t_req = pd.Timestamp(f"{date} {start}").tz_localize(plant.timezone)
        windows = [w for w in windows if w.end > t_req]
    if not windows:
        peak = conditions.poa.max()
        mission.message = (
            f"no valid window: plane-of-array irradiance peaks at {peak:.0f} W/m2"
            f" (needs {req.min_poa:.0f})"
            if peak < req.min_poa
            else "no valid window (wind above the limit while irradiance is high enough)"
        )
        return mission
    window = max(windows, key=lambda w: w.hours)
    mission.window = window
    t0 = window.start if start is None else max(window.start, t_req)
    best = None
    for k in (1, 2, 3):
        lines, sorties, problem = _schedule(plant, k, t0, window, camera, req, drone, home)
        if problem or not sorties:
            continue
        if best is None or sorties[-1].t_land < best[1][-1].t_land:
            best = (lines, sorties, k)
    if best is None:  # report the reason for one row per line
        lines, sorties, problem = _schedule(plant, 1, t0, window, camera, req, drone, home)
        mission.lines, mission.sorties, mission.rows_per_line = lines, sorties, 1
        mission.message = problem or "infeasible"
        return mission
    mission.lines, mission.sorties, mission.rows_per_line = best
    mission.feasible = True
    mission.message = "ok"
    return mission
