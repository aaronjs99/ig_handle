"""Physical endpoint checks for IG Handle live hardware tests."""

from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from .env import env_enabled
from .hardware_contracts import SensorEndpoint


ENABLE_DISABLED_ENV = "IG_HANDLE_ENABLE_DISABLED_SENSORS"


def assert_endpoint_reachable(
    sensor: SensorEndpoint, *, timeout_sec: float = 2.0
) -> None:
    if not sensor.enabled_by_default and not env_enabled(ENABLE_DISABLED_ENV):
        pytest.skip(
            sensor.disabled_reason or "%s disabled by hardware contract" % sensor.key
        )

    if sensor.host:
        command = ["ping", "-c", "1", "-W", str(max(1, int(timeout_sec))), sensor.host]
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        assert result.returncode == 0, (
            f"{sensor.label} host {sensor.host} is not reachable with ping. "
            f"stdout={result.stdout!r} stderr={result.stderr!r}"
        )
        return

    if sensor.device_path:
        path = Path(sensor.device_path)
        assert path.exists(), f"{sensor.label} device path is missing: {path}"
        return

    raise AssertionError(f"{sensor.label} has no connectivity probe configured")
