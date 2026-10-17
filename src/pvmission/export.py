"""Write a mission: ArduPilot waypoints, QGroundControl .plan, GeoJSON and mission.json.

Mission items per sortie (ArduPilot Copter; set WP_YAW_BEHAVIOR=0 so headings hold):

    NAV_TAKEOFF
    per line:  CONDITION_YAW -> DO_MOUNT_CONTROL (gimbal pitch) -> DO_CHANGE_SPEED (transit)
               -> NAV_WAYPOINT (line start, 2 s settle) -> DO_CHANGE_SPEED (line speed)
               -> DO_SET_CAM_TRIGG_DIST -> NAV_WAYPOINT (line end) -> DO_SET_CAM_TRIGG_DIST 0
    NAV_RETURN_TO_LAUNCH

``mission.json`` is the hand-off to the defect-mapping step: the plant layout with module
IDs, the camera, and the predicted pose of every image.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .planner import Line, Mission, Sortie

# MAVLink command and frame numbers
NAV_WAYPOINT, NAV_RTL, NAV_TAKEOFF = 16, 20, 22
CONDITION_YAW, DO_CHANGE_SPEED, DO_MOUNT_CONTROL, DO_SET_CAM_TRIGG_DIST = 115, 178, 205, 206
FRAME_GLOBAL, FRAME_MISSION, FRAME_RELATIVE_ALT = 0, 2, 3
MAV_MOUNT_MODE_MAVLINK_TARGETING = 2
MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_QUADROTOR = 3, 2
YAW_RATE = 30.0  # deg/s

RECOMMENDED_PARAMS = """\
# ArduPilot Copter parameters for flying pvmission plans
WP_YAW_BEHAVIOR 0   # keep the heading set by CONDITION_YAW (the drone flies the lines sideways)
"""


def _lla(mission: Mission, p: np.ndarray) -> tuple[float, float, float]:
    lat, lon = mission.plant.frame.to_geodetic(p[0], p[1])
    return float(lat), float(lon), float(p[2])


def mission_items(mission: Mission, sortie: Sortie) -> list[dict]:
    """MAVLink mission items (without the home item) for one sortie."""
    items = []

    def add(command, frame, p1=0.0, p2=0.0, p3=0.0, p4=0.0, x=0.0, y=0.0, z=0.0):
        items.append({"command": command, "frame": frame, "params": [p1, p2, p3, p4, x, y, z]})

    first = sortie.lines[0]
    hlat, hlon, _ = _lla(mission, mission.home)
    add(NAV_TAKEOFF, FRAME_RELATIVE_ALT, x=hlat, y=hlon, z=round(first.start[2], 2))
    for line in sortie.lines:
        c = line.config
        add(CONDITION_YAW, FRAME_MISSION, round(c.yaw_deg, 2), YAW_RATE, 0, 0)
        add(
            DO_MOUNT_CONTROL,
            FRAME_MISSION,
            round(c.pitch_deg, 2),
            0,
            0,
            0,
            0,
            0,
            MAV_MOUNT_MODE_MAVLINK_TARGETING,
        )
        add(DO_CHANGE_SPEED, FRAME_MISSION, 1, mission.drone.transit_speed, -1)
        lat, lon, alt = _lla(mission, line.start)
        add(
            NAV_WAYPOINT, FRAME_RELATIVE_ALT, mission.drone.hold_s, 0, 0, 0, lat, lon, round(alt, 2)
        )
        add(DO_CHANGE_SPEED, FRAME_MISSION, 1, round(c.speed, 2), -1)
        add(DO_SET_CAM_TRIGG_DIST, FRAME_MISSION, round(c.trigger_m, 2), 0, 1)
        lat, lon, alt = _lla(mission, line.end)
        add(NAV_WAYPOINT, FRAME_RELATIVE_ALT, 0, 0, 0, 0, lat, lon, round(alt, 2))
        add(DO_SET_CAM_TRIGG_DIST, FRAME_MISSION, 0, 0, 0)
    add(DO_CHANGE_SPEED, FRAME_MISSION, 1, mission.drone.transit_speed, -1)  # fly home fast
    add(NAV_RTL, FRAME_MISSION)
    return items


def write_waypoints(mission: Mission, sortie: Sortie, path: Path) -> None:
    """'QGC WPL 110' text file (Mission Planner, MAVProxy, QGroundControl, pymavlink)."""
    hlat, hlon, _ = _lla(mission, mission.home)
    rows = [
        [0, 1, FRAME_GLOBAL, NAV_WAYPOINT, 0, 0, 0, 0, hlat, hlon, mission.plant.frame.altitude, 1]
    ]
    for i, it in enumerate(mission_items(mission, sortie), start=1):
        rows.append([i, 0, it["frame"], it["command"], *it["params"], 1])
    with open(path, "w") as f:
        f.write("QGC WPL 110\n")
        for r in rows:
            ints = [str(int(v)) for v in r[:4]]
            params = [
                f"{float(v):.8f}" if i in (4, 5) else f"{float(v):.6g}"
                for i, v in enumerate(r[4:11])
            ]
            f.write("\t".join([*ints, *params, str(int(r[11]))]) + "\n")


def write_plan(mission: Mission, sortie: Sortie, path: Path) -> None:
    """QGroundControl .plan (JSON) for ArduPilot."""
    hlat, hlon, _ = _lla(mission, mission.home)
    items = []
    for i, it in enumerate(mission_items(mission, sortie), start=1):
        nav = it["frame"] == FRAME_RELATIVE_ALT
        item = {
            "type": "SimpleItem",
            "autoContinue": True,
            "command": it["command"],
            "doJumpId": i,
            "frame": it["frame"],
            "params": it["params"],
        }
        if nav:
            item.update(Altitude=it["params"][6], AltitudeMode=1, AMSLAltAboveTerrain=None)
        items.append(item)
    plan = {
        "fileType": "Plan",
        "version": 1,
        "groundStation": "QGroundControl",
        "mission": {
            "version": 2,
            "firmwareType": MAV_AUTOPILOT_ARDUPILOTMEGA,
            "vehicleType": MAV_TYPE_QUADROTOR,
            "cruiseSpeed": mission.drone.transit_speed,
            "hoverSpeed": mission.drone.transit_speed,
            "globalPlanAltitudeMode": 1,
            "plannedHomePosition": [hlat, hlon, mission.plant.frame.altitude],
            "items": items,
        },
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
    }
    path.write_text(json.dumps(plan, indent=2))


def _line_dict(mission: Mission, line: Line) -> dict:
    c = line.config
    return {
        "index": line.index,
        "sortie": line.sortie,
        "rows": [r + 1 for r in line.rows],
        "t_start": line.t_start.isoformat(),
        "t_end": line.t_end.isoformat(),
        "height_m": c.height,
        "offset_m": c.offset,
        "gimbal_pitch_deg": round(c.pitch_deg, 3),
        "yaw_deg": round(c.yaw_deg, 3),
        "speed_mps": round(c.speed, 3),
        "trigger_m": round(c.trigger_m, 3),
        "min_glare_deg": round(c.min_glare_deg, 2),
        "view_angle_deg": [round(v, 2) for v in c.view_angle_deg],
        "max_gsd_cm": round(100 * c.max_gsd_m, 3),
        "start_lla": _lla(mission, line.start),
        "end_lla": _lla(mission, line.end),
        "images": line.n_images,
    }


def mission_json(mission: Mission) -> dict:
    plant = mission.plant
    images = []
    for line in mission.lines:
        c = line.config
        dt = (line.t_end - line.t_start) / max(line.n_images, 1)
        for k, p in enumerate(line.image_positions()):
            lat, lon, alt = _lla(mission, p)
            images.append(
                {
                    "line": line.index,
                    "seq": len(images),
                    "time": (line.t_start + k * dt).isoformat(),
                    "enu": [round(float(v), 3) for v in p],
                    "lat": lat,
                    "lon": lon,
                    "alt_agl": alt,
                    "yaw_deg": round(c.yaw_deg, 3),
                    "gimbal_pitch_deg": round(c.pitch_deg, 3),
                }
            )
    return {
        "format": "pvmission/1",
        "summary": mission.summary(),
        "plant": {
            "name": plant.name,
            "latitude": plant.frame.latitude,
            "longitude": plant.frame.longitude,
            "altitude": plant.frame.altitude,
            "timezone": plant.timezone,
            "module_id_format": "R<row>-C<column>-<level, 1 = lower>",
            "rows": [
                {
                    "row": r.index + 1,
                    "center_enu": [round(float(v), 4) for v in r.center],
                    "length_m": r.length,
                    "width_m": r.width,
                    "tilt_deg": r.tilt,
                    "azimuth_deg": r.azimuth,
                    "modules_along": r.modules_along,
                    "modules_across": r.modules_across,
                }
                for r in plant.rows
            ],
        },
        "camera": asdict(mission.camera),
        "requirements": asdict(mission.req),
        "drone": asdict(mission.drone),
        "home_lla": _lla(mission, mission.home),
        "sorties": [
            {
                "sortie": s.index + 1,
                "takeoff": s.t_takeoff.isoformat(),
                "land": s.t_land.isoformat(),
                "minutes": round(s.minutes, 2),
                "lines": [ln.index for ln in s.lines],
            }
            for s in mission.sorties
        ],
        "lines": [_line_dict(mission, ln) for ln in mission.lines],
        "images": images,
    }


def geojson(mission: Mission) -> dict:
    feats = []
    fr = mission.plant.frame

    def ll(points):
        lat, lon = fr.to_geodetic(points[:, 0], points[:, 1])
        return [[float(a), float(b)] for a, b in zip(lon, lat, strict=True)]

    for r in mission.plant.rows:
        ring = ll(np.vstack([r.corners(), r.corners()[:1]]))
        feats.append(
            {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [ring]},
                "properties": {"kind": "row", "row": r.index + 1},
            }
        )
    for line in mission.lines:
        feats.append(
            {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": ll(np.vstack([line.start, line.end])),
                },
                "properties": {"kind": "flight_line", **_line_dict(mission, line)},
            }
        )
    hlat, hlon, _ = _lla(mission, mission.home)
    feats.append(
        {
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [hlon, hlat]},
            "properties": {"kind": "home"},
        }
    )
    return {"type": "FeatureCollection", "features": feats}


def report(mission: Mission) -> str:
    s = mission.summary()
    out = [f"# Inspection plan: {mission.plant.name}", ""]
    out.append(f"- Date: {mission.date} ({mission.plant.timezone}); conditions: {s['conditions']}")
    if not mission.feasible:
        out += [f"- **Not feasible:** {mission.message}", ""]
        return "\n".join(out)
    w = mission.window
    out += [
        f"- Valid window (POA >= {mission.req.min_poa:.0f} W/m2"
        f"{'' if s['conditions'] == 'clear sky' else ', wind within limit'}): "
        f"{w.start:%H:%M}-{w.end:%H:%M}",
        f"- Flying: {mission.start:%H:%M}-{mission.end:%H:%M}, {s['sorties']} battery sortie(s),"
        f" {s['flight_minutes']} min in the air",
        f"- {s['lines']} lines covering {s['rows_per_line']} row(s) each, {s['images']} images",
        f"- Worst case over all images: resolution {s['max_gsd_cm']} cm/px on the module, viewing"
        f" angle {s['view_angle_deg'][0]}-{s['view_angle_deg'][1]} deg, sun reflection at least"
        f" {s['min_glare_deg']} deg from the line of sight",
        "",
        "| Line | Rows | Sortie | Start | Height (m) | Offset (m) | Gimbal (deg) | Heading (deg)"
        " | Speed (m/s) | Images | Glare margin (deg) |",
        "|--:|--|--:|--|--:|--:|--:|--:|--:|--:|--:|",
    ]
    for ln in mission.lines:
        c = ln.config
        out.append(
            f"| {ln.index + 1} | {'-'.join(str(r + 1) for r in ln.rows)} | {ln.sortie + 1} |"
            f" {ln.t_start:%H:%M:%S} | {c.height:.1f} | {c.offset:+.1f} | {c.pitch_deg:.1f} |"
            f" {c.yaw_deg:.0f} | {c.speed:.2f} | {ln.n_images} | {c.min_glare_deg:.1f} |"
        )
    out += [
        "",
        "Offset: horizontal distance from the rows' centre, positive on the side the",
        "modules face. ArduPilot: set WP_YAW_BEHAVIOR=0 (see recommended.parm).",
        "",
    ]
    return "\n".join(out)


def write_all(mission: Mission, out: Path) -> list[Path]:
    out.mkdir(parents=True, exist_ok=True)
    written = [out / "report.md", out / "mission.json"]
    (out / "report.md").write_text(report(mission))
    (out / "mission.json").write_text(json.dumps(mission_json(mission), indent=1))
    if mission.feasible:
        (out / "plan.geojson").write_text(json.dumps(geojson(mission)))
        (out / "recommended.parm").write_text(RECOMMENDED_PARAMS)
        written += [out / "plan.geojson", out / "recommended.parm"]
        for s in mission.sorties:
            for ext, writer in ((".waypoints", write_waypoints), (".plan", write_plan)):
                p = out / f"sortie{s.index + 1}{ext}"
                writer(mission, s, p)
                written.append(p)
    return written
