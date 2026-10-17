"""Command line.

python -m pvmission plan examples/northumberland.yaml --date 2026-06-21 --out missions/june
python -m pvmission plan examples/northumberland.yaml --date 2026-06-21 --start 13:00
python -m pvmission plan examples/northumberland.yaml --forecast        # next 3 days, live
python -m pvmission season examples/northumberland.yaml --year 2026    # valid hours by month
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def cmd_plan(args) -> None:
    import pandas as pd

    from . import export, sun
    from .config import load
    from .planner import plan

    cfg = load(args.site)
    plant = cfg["plant"]
    if args.forecast:
        payload = sun.fetch_forecast(plant, days=3)
        times = pd.date_range(
            pd.Timestamp.now(tz=plant.timezone).normalize(), periods=3 * 24 * 60, freq="1min"
        )
        cond = sun.forecast_poa(plant, payload, times)
        days = sorted({t.date() for t in times})
        dates = [str(d) for d in days] if args.date is None else [args.date]
        source = "Open-Meteo forecast"
    else:
        dates, cond, source = [args.date or str(pd.Timestamp.now().date())], None, "clear sky"
    for date in dates:
        c = None if cond is None else cond[cond.index.strftime("%Y-%m-%d") == date]
        mission = plan(
            plant, date, cfg["camera"], cfg["req"], cfg["drone"], cfg["home"], args.start, c, source
        )
        print(export.report(mission).split("\n| Line")[0].strip(), "\n")
        if mission.feasible:
            out = Path(args.out) / date if len(dates) > 1 else Path(args.out)
            files = export.write_all(mission, out)
            print(f"wrote {len(files)} files to {out}/")
            break
    if source != "clear sky":
        print("Weather data by Open-Meteo.com (CC BY 4.0)")


def cmd_season(args) -> None:
    import pandas as pd

    from . import sun
    from .config import load

    cfg = load(args.site)
    plant, req = cfg["plant"], cfg["req"]
    hours = {}
    for day in pd.date_range(f"{args.year}-01-01", f"{args.year}-12-31", freq="D"):
        c = sun.clear_sky_poa(plant, sun.day_times(plant, str(day.date()), "5min"))
        w = sun.valid_windows(c, req.min_poa)
        hours[str(day.date())] = round(max((x.hours for x in w), default=0.0), 2)
    s = pd.Series(hours)
    s.index = pd.to_datetime(s.index)
    monthly = s.groupby(s.index.month).agg(["min", "mean", "max"]).round(1)
    monthly.index = [pd.Timestamp(2000, m, 1).strftime("%b") for m in monthly.index]
    print(f"{plant.name}: longest clear-sky window per day (h), POA >= {req.min_poa:.0f} W/m2")
    print(monthly.to_string())
    if args.json:
        Path(args.json).write_text(json.dumps(hours))


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        prog="python -m pvmission",
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("plan", help="plan an inspection")
    p.add_argument("site", help="site YAML file")
    p.add_argument("--date", help="local date YYYY-MM-DD (default today)")
    p.add_argument("--start", help="earliest take-off, local HH:MM")
    p.add_argument("--forecast", action="store_true", help="use the Open-Meteo forecast")
    p.add_argument("--out", default="mission", help="output folder")
    p.set_defaults(fn=cmd_plan)
    s = sub.add_parser("season", help="clear-sky inspection window over a year")
    s.add_argument("site")
    s.add_argument("--year", type=int, default=2026)
    s.add_argument("--json", help="save daily hours to this file")
    s.set_defaults(fn=cmd_season)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
