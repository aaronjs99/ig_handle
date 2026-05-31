from __future__ import annotations

from pathlib import Path

import pytest

from ig_handle_testing.hardware_contracts import load_hardware_contract


@pytest.fixture(scope="session")
def package_dir() -> Path:
    return Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def sonar_profile_config(package_dir: Path) -> Path:
    return package_dir / "config" / "sonar" / "profiles.yaml"


@pytest.fixture(scope="session")
def hardware_contract(package_dir: Path):
    return load_hardware_contract(package_dir / "config" / "hardware" / "sensors.yaml")
