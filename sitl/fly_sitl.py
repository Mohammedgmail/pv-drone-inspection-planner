"""Fly a planned sortie in ArduPilot's software-in-the-loop simulator (SITL) and compare.

Needs an ArduPilot source tree with ArduCopter SITL built (./waf configure --board sitl &&
./waf copter) and pymavlink (pip install pymavlink).

    python sitl/fly_sitl.py --ardupilot ~/ardupilot --mission mission/ --sortie 1
    python sitl/fly_sitl.py --ardupilot ~/ardupilot --mission mission/ --wind 6 --wind-dir 270

Writes track_<tag>.csv (position and heading), triggers_<tag>.csv (camera trigger
positions) and sitl_<tag>.json (waypoints reached, images, heading and cross-track errors,
flight time) into the mission folder.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil, mavwp

ARM_TIMEOUT_S = 180
VISIT_RADIUS_M = 2.5  # a waypoint counts as visited if the track passes this close


def start_sitl(ardupilot: Path, home, speedup: float, params: dict, workdir: Path):
    binary = ardupilot / "build" / "sitl" / "bin" / "arducopter"
    defaults = ardupilot / "Tools" / "autotest" / "default_params" / "copter.parm"
    extra = workdir / "pvmission.parm"  # applied at boot: camera settings need a reboot
    extra.write_text("".join(f"{k} {v}\n" for k, v in params.items()))
    lat, lon, alt = home
    cmd = [
        str(binary),
        "--model",
        "quad",
        "--home",
        f"{lat},{lon},{alt},0",
        "--defaults",
        f"{defaults},{extra}",
        "--speedup",
        str(speedup),
        "-I0",
    ]
    log = open(workdir / "sitl.log", "w")  # noqa: SIM115 (closed with the process)
    return subprocess.Popen(cmd, cwd=workdir, stdout=log, stderr=subprocess.STDOUT)


def connect() -> mavutil.mavfile:
    for _ in range(30):
        try:
            m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
            m.wait_heartbeat(timeout=10)
            m.target_component = m.target_component or 1
            return m
        except Exception:  # noqa: BLE001 (SITL still starting)
            time.sleep(1)
    raise RuntimeError("no heartbeat from SITL")


def request_streams(m) -> None:
    for msg_id, hz in ((33, 10), (30, 10), (42, 2), (24, 2)):  # position, attitude, mission, GPS
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            1e6 / hz,
            0,
            0,
            0,
            0,
            0,
        )


def upload(m, loader: mavwp.MAVWPLoader) -> None:
    n = loader.count()
    kinds = ["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"]
    for _ in range(30):  # right after boot, storage is not ready and answers NO_SPACE
        m.mav.mission_count_send(m.target_system, m.target_component, n)
        req = m.recv_match(type=kinds, blocking=True, timeout=5)
        if req is not None and req.get_type() != "MISSION_ACK":
            break
        time.sleep(2)
    else:
        raise RuntimeError("SITL did not accept a mission")
    while True:
        w = loader.wp(req.seq)
        m.mav.mission_item_int_send(
            m.target_system,
            m.target_component,
            w.seq,
            w.frame,
            w.command,
            w.current,
            w.autocontinue,
            w.param1,
            w.param2,
            w.param3,
            w.param4,
            int(round(w.x * 1e7)),
            int(round(w.y * 1e7)),
            w.z,
        )
        req = m.recv_match(type=kinds, blocking=True, timeout=10)
        if req is None:
            raise RuntimeError("mission upload timed out")
        if req.get_type() == "MISSION_ACK":
            if req.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"mission rejected: {req.type}")
            return


def arm_and_start(m) -> None:
    m.set_mode(m.mode_mapping()["AUTO"])
    deadline = time.time() + ARM_TIMEOUT_S
    while time.time() < deadline:
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            m.mav.command_long_send(
                m.target_system,
                m.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            return
        time.sleep(2)  # EKF / GPS still settling
    raise RuntimeError("could not arm")


def enu(lat, lon, lat0, lon0):
    r = 6378137.0
    return (np.radians(lon - lon0) * r * math.cos(math.radians(lat0)), np.radians(lat - lat0) * r)


def fly(args) -> dict:
    mdir = Path(args.mission)
    plan = json.loads((mdir / "mission.json").read_text())
    lines = [ln for ln in plan["lines"] if ln["sortie"] == args.sortie - 1]
    sortie = plan["sorties"][args.sortie - 1]
    home = plan["home_lla"]
    lat0, lon0 = plan["plant"]["latitude"], plan["plant"]["longitude"]  # ENU origin of the plan
    loader = mavwp.MAVWPLoader()
    loader.load(str(mdir / f"sortie{args.sortie}.waypoints"))
    waypoints = [loader.wp(i) for i in range(1, loader.count()) if loader.wp(i).command == 16]
    params = {
        "WP_YAW_BEHAVIOR": 0,
        "AUTO_OPTIONS": 3,
        "CAM1_TYPE": 1,
        "SERVO9_FUNCTION": 10,
        "SIM_WIND_SPD": args.wind,
        "SIM_WIND_DIR": args.wind_dir,
        "SIM_WIND_TURB": args.turbulence,
    }
    work = Path(tempfile.mkdtemp(prefix="pvmission_sitl_"))
    sitl = start_sitl(
        Path(args.ardupilot).expanduser(),
        (home[0], home[1], plan["plant"]["altitude"]),
        args.speedup,
        params,
        work,
    )
    track, triggers = [], []
    try:
        m = connect()
        request_streams(m)
        upload(m, loader)
        arm_and_start(m)
        armed_seen, t_takeoff, t_land = False, None, None
        while True:
            msg = m.recv_match(blocking=True, timeout=30)
            if msg is None:
                raise RuntimeError("SITL stopped sending data")
            kind = msg.get_type()
            if kind == "GLOBAL_POSITION_INT":
                t, alt = msg.time_boot_ms / 1e3, msg.relative_alt / 1e3
                track.append((t, msg.lat / 1e7, msg.lon / 1e7, alt, msg.hdg / 100))
                if t_takeoff is None and alt > 0.2:
                    t_takeoff = t
            elif kind == "CAMERA_FEEDBACK":
                triggers.append((msg.lat / 1e7, msg.lng / 1e7, msg.alt_rel, msg.img_idx))
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                armed_seen |= armed
                if armed_seen and not armed:
                    t_land = track[-1][0]
                    break
    finally:
        sitl.terminate()
        sitl.wait(timeout=10)

    tr = np.array(track)
    x, y = enu(tr[:, 1], tr[:, 2], lat0, lon0)
    head_err, xte = [], []
    for ln in lines:  # track samples on each line (between its waypoints, at line height)
        (a_lat, a_lon, a_alt), (b_lat, b_lon, _) = ln["start_lla"], ln["end_lla"]
        ax, ay = enu(a_lat, a_lon, lat0, lon0)
        bx, by = enu(b_lat, b_lon, lat0, lon0)
        u = np.array([bx - ax, by - ay])
        length = float(np.linalg.norm(u))
        u /= length
        rel = np.stack([x - ax, y - ay], axis=1)
        along, across = rel @ u, rel @ np.array([-u[1], u[0]])
        on = (along > 5) & (along < length - 5) & (np.abs(across) < 5)
        on &= np.abs(tr[:, 3] - a_alt) < 2
        if on.any():
            head_err.extend(np.abs((tr[on, 4] - ln["yaw_deg"] + 180) % 360 - 180))
            xte.extend(np.abs(across[on]))
    visited = 0
    for w in waypoints:
        wx, wy = enu(w.x, w.y, lat0, lon0)
        d = np.sqrt((x - wx) ** 2 + (y - wy) ** 2 + (tr[:, 3] - w.z) ** 2)
        visited += int(d.min() <= VISIT_RADIUS_M)
    covered, modules = 0, 0  # every module needs an image within half a spacing of it
    trig = np.array(triggers) if triggers else np.empty((0, 4))
    tx, ty = enu(trig[:, 0], trig[:, 1], lat0, lon0) if len(trig) else (np.array([]), np.array([]))
    rows = {r["row"]: r for r in plan["plant"]["rows"]}
    for ln in lines:
        (a_lat, a_lon, _), (b_lat, b_lon, _) = ln["start_lla"], ln["end_lla"]
        ax, ay = enu(a_lat, a_lon, lat0, lon0)
        bx, by = enu(b_lat, b_lon, lat0, lon0)
        u = np.array([bx - ax, by - ay]) / np.hypot(bx - ax, by - ay)
        near = np.abs((tx - ax) * -u[1] + (ty - ay) * u[0]) < 5  # triggers on this line
        for r in ln["rows"]:
            row = rows[r]
            c = np.array(row["center_enu"][:2])
            s_trig = (np.stack([tx[near], ty[near]], axis=1) - c) @ u
            pitch = row["length_m"] / row["modules_along"]
            s_mod = (np.arange(row["modules_along"]) + 0.5) * pitch - row["length_m"] / 2
            gap = (
                np.abs(s_mod[:, None] - s_trig[None, :]).min(axis=1)
                if s_trig.size
                else s_mod * np.inf
            )
            covered += int((gap <= ln["trigger_m"] / 2 + 0.25).sum()) * row["modules_across"]
            modules += row["modules_along"] * row["modules_across"]
    tag = f"wind{args.wind:g}_from{args.wind_dir:g}"
    summary = {
        "sortie": args.sortie,
        "wind_mps": args.wind,
        "wind_from_deg": args.wind_dir,
        "turbulence": args.turbulence,
        "waypoints": len(waypoints),
        "waypoints_visited": visited,
        "camera_triggers": len(triggers),
        "planned_images": sum(ln["images"] for ln in lines),
        "modules_with_centred_image_pct": round(100 * covered / modules, 2),
        "flight_minutes_sitl": round((t_land - t_takeoff) / 60, 2),
        "flight_minutes_planned": sortie["minutes"],
        "heading_error_deg_p95": round(float(np.percentile(head_err, 95)), 2),
        "cross_track_m_p95": round(float(np.percentile(xte, 95)), 3),
        "max_altitude_m": round(float(tr[:, 3].max()), 2),
    }
    with open(mdir / f"track_{tag}.csv", "w", newline="") as f:
        csv.writer(f).writerows([("t", "lat", "lon", "alt_rel", "heading"), *track])
    with open(mdir / f"triggers_{tag}.csv", "w", newline="") as f:
        csv.writer(f).writerows([("lat", "lon", "alt_rel", "image"), *triggers])
    (mdir / f"sitl_{tag}.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--ardupilot", required=True, help="ArduPilot source tree with SITL built")
    ap.add_argument("--mission", required=True, help="folder written by 'pvmission plan'")
    ap.add_argument("--sortie", type=int, default=1)
    ap.add_argument("--speedup", type=float, default=5.0)
    ap.add_argument("--wind", type=float, default=0.0, help="simulated wind speed (m/s)")
    ap.add_argument("--wind-dir", type=float, default=270.0, help="wind from (deg)")
    ap.add_argument("--turbulence", type=float, default=0.0, help="SIM_WIND_TURB")
    args = ap.parse_args()
    print(json.dumps(fly(args), indent=2))


if __name__ == "__main__":
    main()
