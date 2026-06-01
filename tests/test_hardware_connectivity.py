"""Live connectivity checks for IG Handle hardware endpoints."""

from pathlib import Path
import os
import subprocess

import pytest
import yaml


LIVE_HARDWARE_ENV = "IG_HANDLE_RUN_LIVE_HARDWARE_TESTS"


def _env_enabled(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


pytestmark = [
    pytest.mark.live_hardware,
    pytest.mark.skipif(
        not _env_enabled(LIVE_HARDWARE_ENV),
        reason=f"set {LIVE_HARDWARE_ENV}=1 to run live hardware connectivity tests",
    ),
]

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SENSOR_NETWORK_CONFIG = PACKAGE_DIR / "config" / "network" / "sensor_network.yaml"
ENABLE_DISABLED_ENV = "IG_HANDLE_ENABLE_DISABLED_SENSORS"
SENSOR_KEYS = [
    "camera_f1",
    "camera_f2",
    "camera_f3",
    "camera_f4",
    "lidar_h",
    "lidar_v",
    "sonar",
    "imu",
    "heron",
]


def _endpoints() -> dict:
    data = yaml.safe_load(SENSOR_NETWORK_CONFIG.read_text(encoding="utf-8")) or {}
    return dict(data["sensor_network"]["endpoints"])


def _assert_endpoint_reachable(sensor_key: str, sensor: dict) -> None:
    label = sensor.get("label", sensor_key)
    if not sensor.get("enabled_by_default", True) and not _env_enabled(
        ENABLE_DISABLED_ENV
    ):
        pytest.skip(sensor.get("disabled_reason") or f"{sensor_key} disabled")

    if sensor.get("host"):
        host = sensor["host"]
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", host],
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0, (
            f"{label} host {host} is not reachable with ping. "
            f"stdout={result.stdout!r} stderr={result.stderr!r}"
        )
        return

    if sensor.get("device_path"):
        path = Path(sensor["device_path"])
        assert path.exists(), f"{label} device path is missing: {path}"
        return

    raise AssertionError(f"{label} has no connectivity probe configured")


@pytest.mark.parametrize("sensor_key", SENSOR_KEYS)
def test_sensor_endpoint_is_reachable(sensor_key):
    _assert_endpoint_reachable(sensor_key, _endpoints()[sensor_key])
