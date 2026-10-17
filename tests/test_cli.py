from conftest import SITE

from pvmission.__main__ import main


def test_plan_command(tmp_path, capsys):
    main(["plan", str(SITE), "--date", "2026-06-21", "--start", "13:00", "--out", str(tmp_path)])
    out = capsys.readouterr().out
    assert "battery sortie" in out and (tmp_path / "sortie1.waypoints").exists()


def test_plan_command_reports_an_infeasible_day(tmp_path, capsys):
    main(["plan", str(SITE), "--date", "2026-12-21", "--out", str(tmp_path)])
    assert "Not feasible" in capsys.readouterr().out
    assert not (tmp_path / "sortie1.plan").exists()
