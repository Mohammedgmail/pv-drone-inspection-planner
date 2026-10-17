"""Draw the README figures (needs matplotlib).

python docs/make_figures.py
"""

import csv
import json
from dataclasses import replace
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from pvmission import sun as sunmod
from pvmission.config import load
from pvmission.geometry import evaluate
from pvmission.planner import plan
from pvmission.plant import GeoFrame

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
CFG = load(ROOT / "examples" / "northumberland.yaml")
PLANT, CAM, REQ, DRONE = CFG["plant"], CFG["camera"], CFG["req"], CFG["drone"]
DATE = "2026-06-21"


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=INK_2, length=0, labelsize=9)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)


def title(ax, text, size=11.5):
    ax.set_title(text, loc="left", color=INK, fontsize=size, fontweight="bold")


def sun_at(t):
    sp = sunmod.solar_position(PLANT, pd.DatetimeIndex([t]))
    return sunmod.sun_vector(sp.apparent_elevation.iloc[0], sp.azimuth.iloc[0]), sp


def cross_section(morning, noon):
    """Side view (looking west) of one pair of rows at 13:07 and three camera placements."""
    from pvmission.geometry import _occluded

    t = pd.Timestamp(f"{DATE} 13:07", tz=PLANT.timezone)
    s, sp = sun_at(t)
    m_line = morning.lines[4]
    pair = [PLANT.rows[i] for i in m_line.rows]
    centre = np.mean([r.center for r in pair], axis=0)
    others = [r for r in PLANT.rows if r not in pair]
    f = np.array([0.0, -1.0, 0.0])  # facing direction (south) -> plot x to the right
    n_line = next(ln for ln in noon.lines if ln.rows == [pair[0].index])

    def xz(p):
        return float((p - centre) @ f), float(p[2])

    fig, ax = plt.subplots(figsize=(9.0, 5.8), facecolor=SURFACE)
    style(ax)
    ax.grid(False)
    for r in PLANT.rows[pair[0].index - 2 : pair[-1].index + 3]:
        lo, hi = r.point(0, -r.width / 2), r.point(0, r.width / 2)
        ax.plot(
            *zip(xz(lo), xz(hi), strict=True),
            color=INK if r in pair else "#b5b3ad",
            lw=4 if r in pair else 3,
            solid_capstyle="butt",
        )
    ax.axhline(0, color=INK_2, lw=1)
    refl = sunmod.reflection(s, PLANT.rows[0].normal)
    sx, sz = float(s @ f), float(s[2])
    rx, rz = float(refl @ f), float(refl[2])
    elev = sp.apparent_elevation.iloc[0]

    def glare_at(cam, q):
        return float(
            np.degrees(np.arccos(np.clip((cam - q) @ refl / np.linalg.norm(cam - q), -1, 1)))
        )

    def sun_arrows(P):
        px, pz = xz(P)
        ax.annotate(
            "",
            (px, pz),
            (px + sx * 8, pz + sz * 8),
            arrowprops={"arrowstyle": "-|>", "color": YELLOW, "lw": 2.2},
        )
        ax.annotate(
            "",
            (px + rx * 20, pz + rz * 20),
            (px, pz),
            arrowprops={"arrowstyle": "-|>", "color": YELLOW, "lw": 2.0, "ls": (0, (4, 3))},
        )

    def camera(pos, colour, text, dy):
        cx, cz = xz(pos)
        ax.plot(cx, cz, "o", ms=9, color=colour, mec=SURFACE, mew=2, zorder=5)
        ax.text(cx + 0.8, cz + dy, text, color=INK, fontsize=8.8, va="bottom")
        return cx, cz

    # A: the morning two-row placement, kept until 13:07
    c = m_line.config
    cam = c.position
    pts = [r.point(0.0, t) for r in pair for t in (-r.width / 2, 0.0, r.width / 2)]
    P = min(pts, key=lambda q: glare_at(cam, q))
    sun_arrows(P)
    cx, cz = camera(
        cam,
        ORANGE,
        f"A  morning placement, {c.height:.0f} m up: the reflection is\n"
        f"{glare_at(cam, P):.0f}° from the line of sight (rejected: glare)",
        0.8,
    )
    ax.plot([cx, xz(P)[0]], [cz, xz(P)[1]], color=ORANGE, lw=1.6)
    # B: covering both rows from low and in front clears the glare, but row 1 hides row 2
    cam = centre + np.array([0.0, -19.5, 15.0 - centre[2]])
    Q = pair[1].point(0.0, -pair[1].width / 2 + 0.05)
    assert _occluded(Q[None], cam[None], [pair[0]])[0]
    ev = evaluate(pair, np.array([15.0]), np.array([19.5]), s, CAM, REQ, others)
    cx, cz = camera(
        cam,
        AQUA,
        f"B  both rows from 15 m up, 19.5 m out: glare {ev['glare'][0]:.0f}° is fine,\n"
        "but the front row hides the back row's lowest cells (rejected)",
        0.8,
    )
    ax.plot([cx, xz(Q)[0]], [cz, xz(Q)[1]], color=AQUA, lw=1.6, ls=(0, (2, 2)))
    # C: the planner's choice at noon, one row per pass
    c = n_line.config
    cam = c.position
    pts = [pair[0].point(0.0, t) for t in (-pair[0].width / 2, 0.0, pair[0].width / 2)]
    P = min(pts, key=lambda q: glare_at(cam, q))
    cx, cz = camera(
        cam,
        BLUE,
        f"C  chosen: one row per pass, {c.height:.0f} m up, camera {c.pitch_deg:.0f}°,\n"
        f"reflection {glare_at(cam, P):.0f}° away, nothing hidden",
        -3.6,
    )
    ax.plot([cx, xz(P)[0]], [cz, xz(P)[1]], color=BLUE, lw=1.6)
    from matplotlib.lines import Line2D

    handles = [
        Line2D([], [], color=YELLOW, lw=2.4, label=f"sunlight ({elev:.0f}° up, from the south)"),
        Line2D(
            [],
            [],
            color=YELLOW,
            lw=2.0,
            ls=(0, (4, 3)),
            label="its mirror reflection off the glass",
        ),
        Line2D([], [], color=INK, lw=4, label="the two rows of a morning pass"),
    ]
    ax.legend(
        handles=handles,
        loc="lower right",
        bbox_to_anchor=(1.0, 0.03),
        frameon=False,
        labelcolor=INK,
        fontsize=9,
    )
    ax.set_xlim(-10, 40)
    ax.set_ylim(-0.5, 34)
    ax.set_aspect("equal")
    ax.set_xlabel("metres towards the south (the way the modules face)", color=INK_2)
    ax.set_ylabel("height (m)", color=INK_2)
    title(ax, "21 June, 13:07: why the noon plan flies one row per pass", 11)
    fig.tight_layout()
    fig.savefig(DOCS / "cross_section.png", dpi=130, facecolor=SURFACE)
    plt.close(fig)


def through_the_day(morning):
    starts = pd.date_range(f"{DATE} 09:40", f"{DATE} 16:20", freq="20min", tz=PLANT.timezone)
    fixed = morning.lines[4].config
    rows = [PLANT.rows[i] for i in morning.lines[4].rows]
    fixed_glare, plan_glare, minutes = [], [], []
    for t in starts:
        s, _ = sun_at(t)
        ev = evaluate(rows, np.array([fixed.height]), np.array([fixed.offset]), s, CAM, REQ)
        fixed_glare.append(ev["glare"][0])
        m = plan(PLANT, DATE, CAM, REQ, DRONE, start=t.strftime("%H:%M"))
        plan_glare.append(m.summary()["min_glare_deg"] if m.feasible else np.nan)
        minutes.append(sum(x.minutes for x in m.sorties) if m.feasible else np.nan)
    hours = starts.hour + starts.minute / 60
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.0), facecolor=SURFACE)
    for ax in (a1, a2):
        style(ax)
        ax.set_xlim(9.5, 16.5)
        ax.set_xticks([10, 12, 14, 16], ["10:00", "12:00", "14:00", "16:00"])
        ax.set_xlabel("take-off time, 21 June (BST)", color=INK_2)
    a1.axhspan(0, REQ.min_glare_deg, color="#f4e3dc", lw=0)
    a1.plot(hours, fixed_glare, color=ORANGE, lw=2, label="morning placement kept all day")
    a1.plot(hours, plan_glare, color=BLUE, lw=2, label="placement re-planned for the time")
    a1.text(
        9.6,
        REQ.min_glare_deg / 2,
        "too close: sun reflection in the image",
        color=INK,
        fontsize=9,
        va="center",
    )
    a1.annotate(
        "morning placement kept",
        (hours[-1], fixed_glare[-1]),
        xytext=(-4, 8),
        textcoords="offset points",
        ha="right",
        color=INK,
        fontsize=9,
    )
    a1.annotate(
        "re-planned",
        (hours[9], plan_glare[9]),
        xytext=(0, 8),
        textcoords="offset points",
        ha="center",
        color=INK,
        fontsize=9,
    )
    a1.set_ylim(0, None)
    a1.set_ylabel("sun reflection to line of sight, worst image (deg)", color=INK_2, fontsize=9)
    title(a1, "Glare margin through the day")
    a2.plot(hours, minutes, color=BLUE, lw=2)
    a2.set_ylim(0, max(np.nanmax(minutes) * 1.25, 1))
    a2.set_ylabel("flight time for the whole plant (min)", color=INK_2, fontsize=9)
    title(a2, "Price of avoiding glare: slower, lower passes")
    a1.legend(
        loc="lower left",
        bbox_to_anchor=(0, -0.42),
        ncol=2,
        frameon=False,
        labelcolor=INK,
        fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(DOCS / "through_the_day.png", dpi=130, facecolor=SURFACE)
    plt.close(fig)
    return dict(
        zip(
            [t.strftime("%H:%M") for t in starts],
            zip(fixed_glare, plan_glare, minutes, strict=True),
            strict=True,
        )
    )


def season():
    sites = {"Northumberland (55.0° N)": PLANT}
    seville = replace(PLANT, frame=GeoFrame(37.39, -5.98, 10), timezone="Europe/Madrid")
    sites["Seville (37.4° N)"] = seville
    days = pd.date_range("2026-01-01", "2026-12-31", freq="3D")
    fig, ax = plt.subplots(figsize=(9.2, 3.8), facecolor=SURFACE)
    style(ax)
    out = {}
    for (name, plant), colour in zip(sites.items(), (BLUE, ORANGE), strict=True):
        hours = []
        for d in days:
            c = sunmod.clear_sky_poa(plant, sunmod.day_times(plant, str(d.date()), "5min"))
            w = sunmod.valid_windows(c, REQ.min_poa)
            hours.append(max((x.hours for x in w), default=0.0))
        ax.plot(days, hours, color=colour, lw=2, label=name)
        i = 18 if colour == BLUE else 108  # mid-February / mid-November
        ax.annotate(
            name,
            (days[i], hours[i]),
            xytext=(8, -4 if colour == BLUE else 8),
            textcoords="offset points",
            ha="left",
            color=INK,
            fontsize=9,
        )
        out[name] = pd.Series(hours, index=days)
    ax.set_ylabel("hours per day", color=INK_2)
    ax.set_ylim(0, 10)
    ax.xaxis.set_major_formatter(plt.matplotlib.dates.DateFormatter("%b"))
    title(ax, "How long a clear day stays above 600 W/m² on the modules (30° tilt, south)")
    ax.legend(loc="lower center", frameon=False, labelcolor=INK, fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(DOCS / "season.png", dpi=130, facecolor=SURFACE)
    plt.close(fig)
    return out


def sitl_map(noon):
    track = np.array(
        [
            [float(v) for v in r]
            for r in list(csv.reader(open(DOCS / "sitl" / "crosswind_track.csv")))[1:]
        ]
    )
    trig = np.array(
        [
            [float(v) for v in r]
            for r in list(csv.reader(open(DOCS / "sitl" / "crosswind_triggers.csv")))[1:]
        ]
    )
    summary = json.loads((DOCS / "sitl" / "crosswind.json").read_text())
    fr = PLANT.frame
    tx, ty = fr.to_local(track[:, 1], track[:, 2])
    gx, gy = fr.to_local(trig[:, 0], trig[:, 1])
    fig, ax = plt.subplots(figsize=(8.4, 6.4), facecolor=SURFACE)
    style(ax)
    ax.grid(False)
    for r in PLANT.rows:
        c = r.corners()
        ax.fill(c[:, 0], c[:, 1], color="#d9d7d1", lw=0)
    for ln in [ln for ln in noon.lines if ln.sortie == summary["sortie"] - 1]:
        ax.plot(
            [ln.start[0], ln.end[0]],
            [ln.start[1], ln.end[1]],
            color=BLUE,
            lw=5,
            alpha=0.35,
            solid_capstyle="butt",
        )
    ax.plot(tx, ty, color=INK, lw=0.9)
    ax.plot(gx, gy, "o", ms=2.2, color=ORANGE)
    ax.plot(*noon.home[:2], "s", ms=8, color=INK)
    ax.text(noon.home[0] + 2, noon.home[1] - 4, "take-off", color=INK, fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)", color=INK_2)
    ax.set_ylabel("north (m)", color=INK_2)
    title(
        ax,
        f"Noon plan, battery {summary['sortie']} of {len(noon.sorties)}, flown in ArduPilot SITL "
        f"with a {summary['wind_mps'] * 3.6:.0f} km/h crosswind\n",
    )
    ax.text(
        0,
        1.015,
        f"{summary['camera_triggers']} of {summary['planned_images']} images taken, "
        f"{summary['waypoints_visited']}/{summary['waypoints']} waypoints, "
        f"{summary['flight_minutes_sitl']} min flown vs "
        f"{summary['flight_minutes_planned']} planned, "
        f"track within {100 * summary['cross_track_m_p95']:.0f} cm",
        transform=ax.transAxes,
        color=INK_2,
        fontsize=9,
    )
    from matplotlib.lines import Line2D

    handles = [
        Line2D([], [], color=BLUE, lw=5, alpha=0.35, label="planned lines"),
        Line2D([], [], color=INK, lw=1, label="flown track"),
        Line2D([], [], color=ORANGE, marker="o", ls="", ms=4, label="camera triggers"),
        Line2D([], [], color="#d9d7d1", lw=6, label="module rows"),
    ]
    ax.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=4,
        frameon=False,
        labelcolor=INK,
        fontsize=9,
    )
    fig.tight_layout()
    fig.savefig(DOCS / "sitl.png", dpi=130, facecolor=SURFACE)
    plt.close(fig)


def main():
    morning = plan(PLANT, DATE, CAM, REQ, DRONE)
    noon = plan(PLANT, DATE, CAM, REQ, DRONE, start="13:00")
    cross_section(morning, noon)
    day = through_the_day(morning)
    seasons = season()
    sitl_map(noon)
    print("through the day (fixed glare, planned glare, minutes):")
    for k, v in day.items():
        print(f"  {k}: {v[0]:5.1f} {v[1]:5.1f} {v[2]:5.1f}")
    for name, s in seasons.items():
        m = s.groupby(s.index.month).max().round(1)
        print(name, dict(zip(m.index, m.to_numpy(), strict=True)))


if __name__ == "__main__":
    main()
