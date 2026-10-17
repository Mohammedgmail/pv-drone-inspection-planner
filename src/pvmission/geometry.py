"""Viewing geometry of one flight line: which camera placements give valid thermal images.

The drone flies parallel to the rows at height ``H`` above ground and horizontal offset
``d`` from the centre of the rows it covers (positive = on the side the modules face).
The camera looks across the rows at their centre, with the long side of the image along
the row, so a line covers every module of its rows in a sequence of overlapping frames.

A placement is valid when, for sample points on every covered module surface (lower edge,
middle and upper edge, at the start, middle and end of a frame interval):

* the module front is visible and the viewing angle to the module normal is within the
  allowed range (5-60 degrees by default);
* the sun's mirror reflection is at least ``min_glare_deg`` away from the line of sight;
* the resolution on the module surface is finer than ``max_gsd_m``;
* the point is inside the image (with a small margin) and not hidden by another row.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .camera import Camera
from .plant import Row, direction
from .sun import reflection


@dataclass(frozen=True)
class Requirements:
    min_poa: float = 600.0  # W/m2 on the module plane (IEC TS 62446-3, as cited)
    max_wind_kmh: float = 28.0  # (IEC TS 62446-3, as cited)
    max_gsd_m: float = 0.03  # 3 cm per pixel on the module (IEC TS 62446-3, as cited)
    view_angle_deg: tuple[float, float] = (5.0, 60.0)  # common practice
    min_glare_deg: float = 15.0  # design margin from the sun's reflection (not in the standard)
    forward_overlap: float = 0.7
    max_blur_px: float = 1.0
    fov_margin: float = 0.05  # keep this fraction of each half-FOV free at the edges
    altitude_range: tuple[float, float] = (12.0, 60.0)  # m above ground
    offset_range: float = 30.0  # m, search |d| up to this


@dataclass(frozen=True)
class LineConfig:
    height: float  # m above ground
    offset: float  # m along the facing direction from the rows' centre
    pitch_deg: float  # gimbal pitch, negative = down
    yaw_deg: float  # heading the camera (and drone) faces, clockwise from north
    speed: float  # m/s
    trigger_m: float  # distance between images
    footprint_m: float  # along-row image footprint at the rows' centre
    min_glare_deg: float  # smallest angle to the sun's reflection over the samples
    view_angle_deg: tuple[float, float]  # min and max viewing angle over the samples
    max_gsd_m: float  # coarsest resolution over the samples
    position: np.ndarray  # camera position (ENU) abeam the rows' centre, s = 0


def _sample_points(rows: list[Row], s: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Points (N, S, 3) on the rows' surfaces and the matching row index (N,)."""
    pts, idx = [], []
    for k, r in enumerate(rows):
        for t in (-r.width / 2, 0.0, r.width / 2):
            pts.append(r.point(s, np.full_like(s, t)))
            idx.append(k)
    return np.stack(pts, axis=1), np.array(idx)  # (C, N, S, 3) when s is (C, S)


def _occluded(P: np.ndarray, C: np.ndarray, others: list[Row]) -> np.ndarray:
    """True where the segment P -> C crosses one of the ``others`` tables."""
    hit = np.zeros(P.shape[:-1], dtype=bool)
    v = C - P
    for r in others:
        n = r.normal
        denom = v @ n
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((r.center - P) @ n) / denom
        X = P + t[..., None] * v - r.center
        inside = (np.abs(X @ r.along) <= r.length / 2) & (np.abs(X @ r.upslope) <= r.width / 2)
        hit |= (t > 1e-6) & (t < 1 - 1e-6) & inside
    return hit


def _static(
    rows: tuple[Row, ...],
    neighbours: tuple[Row, ...],
    heights: np.ndarray,
    offsets: np.ndarray,
    camera: Camera,
    req: Requirements,
    max_speed: float,
) -> dict[str, np.ndarray]:
    """Everything about the placements that does not depend on the sun."""
    H, D = np.asarray(heights, float), np.asarray(offsets, float)
    f = direction(rows[0].azimuth)
    a = rows[0].along
    z = np.array([0.0, 0.0, 1.0])
    centre = np.mean([r.center for r in rows], axis=0)
    q_c = centre @ f
    C = ((q_c + D)[:, None] * f) + H[:, None] * z  # camera abeam the group centre
    o = centre - C
    o /= np.linalg.norm(o, axis=1, keepdims=True)  # optical axis
    y_c = np.cross(o, a)  # image vertical (across the rows)
    r0 = np.linalg.norm(centre - C, axis=1)
    footprint = 2 * r0 * np.tan(camera.hfov / 2)
    spacing = (1 - req.forward_overlap) * footprint
    s = np.stack([-spacing / 2, np.zeros_like(spacing), spacing / 2], axis=1)  # (C, 3)
    P, ridx = _sample_points(list(rows), s)  # (C, N, 3, 3)

    Cb = C[:, None, None, :]
    v = Cb - P
    R = np.linalg.norm(v, axis=-1)
    vh = v / R[..., None]
    normals = np.array([rows[k].normal for k in ridx])[None, :, None, :]
    cos_view = np.sum(vh * normals, axis=-1)
    view = np.degrees(np.arccos(np.clip(cos_view, -1, 1)))
    rel = P - Cb
    zc = np.sum(rel * o[:, None, None, :], axis=-1)
    xc = rel @ a
    yc = np.sum(rel * y_c[:, None, None, :], axis=-1)
    tx = np.tan(camera.hfov / 2) * (1 - req.fov_margin)
    ty = np.tan(camera.vfov / 2) * (1 - req.fov_margin)
    with np.errstate(divide="ignore", invalid="ignore"):
        in_fov = (zc > 0) & (np.abs(xc / zc) <= tx) & (np.abs(yc / zc) <= ty)
    gsd_along = camera.gsd(R)
    gsd_eff = gsd_along / np.clip(cos_view, 1e-6, None)
    occl = _occluded(P, Cb, list(neighbours)) if neighbours else np.zeros_like(in_fov)
    lo, hi = req.view_angle_deg
    ok = (cos_view > 0) & (view >= lo) & (view <= hi) & in_fov & (gsd_eff <= req.max_gsd_m) & ~occl
    pitch = np.degrees(np.arcsin(o[:, 2]))
    p_lo, p_hi = camera.pitch_range_deg
    horiz = o - o[:, 2:3] * z
    yaw = np.degrees(np.arctan2(horiz[:, 0], horiz[:, 1])) % 360
    yaw = np.where(np.linalg.norm(horiz, axis=1) < 1e-6, (rows[0].azimuth + 180) % 360, yaw)
    yaw = np.where(yaw > 359.95, 0.0, np.round(yaw, 3))
    n = len(H)
    blur_speed = req.max_blur_px * gsd_along.reshape(n, -1).min(axis=1)
    blur_speed /= camera.time_constant_ms * 1e-3
    return {
        "ok": ok.reshape(n, -1).all(axis=1) & (pitch >= p_lo - 1e-9) & (pitch <= p_hi),
        "vh": vh.reshape(n, -1, 3),
        "speed": np.minimum(max_speed, blur_speed),
        "pitch": pitch,
        "yaw": yaw,
        "view_min": view.reshape(n, -1).min(axis=1),
        "view_max": view.reshape(n, -1).max(axis=1),
        "gsd": gsd_eff.reshape(n, -1).max(axis=1),
        "footprint": footprint,
        "spacing": spacing,
        "position": C,
    }


_CACHE: dict = {}


def _static_cached(rows, neighbours, heights, offsets, camera, req, max_speed):
    """``_static`` keyed on the rows' geometry relative to the group centre.

    In a regular plant every interior group looks the same from its own centre, so the
    expensive part is computed once per distinct neighbourhood instead of once per line.
    """
    centre = np.mean([r.center for r in rows], axis=0)

    def shape(rs):
        return tuple(
            (
                round(r.tilt, 6),
                round(r.azimuth, 6),
                round(r.length, 6),
                round(r.width, 6),
                *np.round(r.center - centre, 6),
            )
            for r in rs
        )

    key = (
        shape(rows),
        shape(neighbours),
        heights.tobytes(),
        offsets.tobytes(),
        camera,
        req,
        max_speed,
    )
    if key not in _CACHE:
        if len(_CACHE) > 64:
            _CACHE.clear()
        st = _static(tuple(rows), tuple(neighbours), heights, offsets, camera, req, max_speed)
        st["position_rel"] = st["position"] - centre
        _CACHE[key] = st
    st = dict(_CACHE[key])
    st["position"] = st["position_rel"] + centre
    return st


def evaluate(
    rows: list[Row],
    heights: np.ndarray,
    offsets: np.ndarray,
    sun: np.ndarray,
    camera: Camera,
    req: Requirements,
    neighbours: list[Row] = (),
    max_speed: float = 5.0,
) -> dict[str, np.ndarray]:
    """Score every (height, offset) placement for one group of rows and one sun direction.

    ``heights`` and ``offsets`` are 1-D arrays of equal length (one entry per candidate).
    Returns arrays per candidate: ``valid``, ``speed``, ``pitch``, ``yaw`` and the margins.
    """
    heights = np.ascontiguousarray(heights, dtype=float)
    offsets = np.ascontiguousarray(offsets, dtype=float)
    st = _static_cached(tuple(rows), tuple(neighbours), heights, offsets, camera, req, max_speed)
    normal = rows[0].normal
    if sun[2] > 0 and sun @ normal > 0:
        refl = reflection(sun, normal)
        glare = np.degrees(np.arccos(np.clip(st["vh"] @ refl, -1, 1))).min(axis=1)
    else:  # sun down or behind the modules: no specular reflection off the front glass
        glare = np.full(len(heights), 180.0)
    out = dict(st)
    out["glare"] = glare
    out["valid"] = st["ok"] & (glare >= req.min_glare_deg)
    return out


def best_config(
    rows: list[Row],
    suns: np.ndarray,
    camera: Camera,
    req: Requirements,
    neighbours: list[Row] = (),
    max_speed: float = 5.0,
    step: float = 0.5,
) -> LineConfig | None:
    """Fastest placement valid for every sun direction in ``suns`` (e.g. line start, middle
    and end); ties within 2% of the best speed go to the larger glare margin."""
    suns = np.atleast_2d(suns)
    h = np.arange(req.altitude_range[0], req.altitude_range[1] + 1e-9, step)
    d = np.arange(-req.offset_range, req.offset_range + 1e-9, step)
    HH, DD = (g.ravel() for g in np.meshgrid(h, d, indexing="ij"))
    evs = [evaluate(rows, HH, DD, s, camera, req, neighbours, max_speed) for s in suns]
    ev = dict(evs[0])
    ev["valid"] = np.logical_and.reduce([e["valid"] for e in evs])
    ev["glare"] = np.minimum.reduce([e["glare"] for e in evs])
    ev["view_min"] = np.minimum.reduce([e["view_min"] for e in evs])
    ev["view_max"] = np.maximum.reduce([e["view_max"] for e in evs])
    if not ev["valid"].any():
        return None
    speed = np.where(ev["valid"], ev["speed"], -np.inf)
    near_best = speed >= 0.98 * speed.max()
    i = int(np.argmax(np.where(near_best, ev["glare"], -np.inf)))
    return LineConfig(
        height=float(HH[i]),
        offset=float(DD[i]),
        pitch_deg=float(ev["pitch"][i]),
        yaw_deg=float(ev["yaw"][i]),
        speed=float(ev["speed"][i]),
        trigger_m=float(ev["spacing"][i]),
        footprint_m=float(ev["footprint"][i]),
        min_glare_deg=float(ev["glare"][i]),
        view_angle_deg=(float(ev["view_min"][i]), float(ev["view_max"][i])),
        max_gsd_m=float(ev["gsd"][i]),
        position=ev["position"][i],
    )
