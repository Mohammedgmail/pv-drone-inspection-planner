import json

import pytest

from pvmission import export

CMD = {16: "WAYPOINT", 20: "RTL", 22: "TAKEOFF", 115: "YAW", 178: "SPEED", 205: "MOUNT", 206: "CAM"}


def test_item_sequence(noon):
    items = export.mission_items(noon, noon.sorties[0])
    names = [CMD[i["command"]] for i in items]
    assert names[0] == "TAKEOFF" and names[-2:] == ["SPEED", "RTL"]
    per_line = ["YAW", "MOUNT", "SPEED", "WAYPOINT", "SPEED", "CAM", "WAYPOINT", "CAM"]
    body = names[1:-2]
    assert body == per_line * len(noon.sorties[0].lines)
    yaw = [i for i in items if i["command"] == 115]
    assert [y["params"][0] for y in yaw] == [
        round(ln.config.yaw_deg, 2) for ln in noon.sorties[0].lines
    ]


def test_waypoints_file_loads_in_pymavlink(noon, tmp_path):
    mavwp = pytest.importorskip("pymavlink.mavwp")
    path = tmp_path / "s.waypoints"
    export.write_waypoints(noon, noon.sorties[0], path)
    loader = mavwp.MAVWPLoader()
    n = loader.load(str(path))
    assert n == len(export.mission_items(noon, noon.sorties[0])) + 1  # + home
    home = loader.wp(0)
    lat, lon, _ = export._lla(noon, noon.home)
    assert home.x == pytest.approx(lat) and home.y == pytest.approx(lon)
    first_line = noon.sorties[0].lines[0]
    wp = [loader.wp(i) for i in range(n) if loader.wp(i).command == 16][1]
    assert wp.z == pytest.approx(first_line.config.height)
    assert wp.param1 == noon.drone.hold_s


def test_qgc_plan_structure(noon, tmp_path):
    path = tmp_path / "s.plan"
    export.write_plan(noon, noon.sorties[0], path)
    plan = json.loads(path.read_text())
    assert plan["fileType"] == "Plan" and plan["version"] == 1
    m = plan["mission"]
    assert m["firmwareType"] == 3 and m["vehicleType"] == 2 and m["version"] == 2
    assert len(m["plannedHomePosition"]) == 3
    ids = [it["doJumpId"] for it in m["items"]]
    assert ids == list(range(1, len(ids) + 1))
    for it in m["items"]:
        assert it["type"] == "SimpleItem" and len(it["params"]) == 7
    assert plan["geoFence"]["version"] == 2 and plan["rallyPoints"]["version"] == 2


def test_mission_json_is_the_handoff_for_defect_mapping(noon):
    mj = json.loads(json.dumps(export.mission_json(noon)))
    assert mj["format"] == "pvmission/1"
    assert len(mj["images"]) == noon.n_images
    assert len(mj["plant"]["rows"]) == len(noon.plant.rows)
    assert mj["sorties"][0]["minutes"] == pytest.approx(noon.sorties[0].minutes, abs=0.01)
    img = mj["images"][0]
    assert {"lat", "lon", "alt_agl", "yaw_deg", "gimbal_pitch_deg", "time", "enu"} <= set(img)


def test_geojson_and_report(noon):
    gj = export.geojson(noon)
    kinds = [f["properties"]["kind"] for f in gj["features"]]
    assert kinds.count("row") == len(noon.plant.rows)
    assert kinds.count("flight_line") == len(noon.lines) and kinds.count("home") == 1
    text = export.report(noon)
    assert "sun reflection at least" in text and text.count("\n| ") == len(noon.lines) + 1


def test_write_all(noon, tmp_path):
    files = export.write_all(noon, tmp_path)
    names = {f.name for f in files}
    assert {
        "mission.json",
        "report.md",
        "plan.geojson",
        "sortie1.plan",
        "sortie1.waypoints",
    } <= names
