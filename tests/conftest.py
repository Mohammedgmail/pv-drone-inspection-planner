from pathlib import Path

import pytest

from pvmission.config import load
from pvmission.planner import plan

SITE = Path(__file__).resolve().parents[1] / "examples" / "northumberland.yaml"


@pytest.fixture(scope="session")
def cfg():
    return load(SITE)


@pytest.fixture(scope="session")
def noon(cfg):
    """21 June from 13:00: the sun is high in the south and glare constrains the plan."""
    return plan(cfg["plant"], "2026-06-21", cfg["camera"], cfg["req"], cfg["drone"], start="13:00")


@pytest.fixture(scope="session")
def morning(cfg):
    return plan(cfg["plant"], "2026-06-21", cfg["camera"], cfg["req"], cfg["drone"])
