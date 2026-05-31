"""Hardware endpoint contract loaded from IG Handle config."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional

import yaml


@dataclass(frozen=True)
class SensorEndpoint:
    key: str
    label: str
    topic: str
    expected_type: str
    host: Optional[str] = None
    device_path: Optional[str] = None
    enabled_by_default: bool = True
    disabled_reason: str = ""


@dataclass(frozen=True)
class HardwareContract:
    sensors: Mapping[str, SensorEndpoint]


def load_hardware_contract(config_path: Optional[Path] = None) -> HardwareContract:
    path = Path(config_path) if config_path is not None else _default_config_path()
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    sensors = {
        str(key): SensorEndpoint(key=str(key), **dict(value))
        for key, value in _sensors(data, path).items()
    }
    return HardwareContract(sensors=sensors)


def _sensors(data: object, path: Path) -> Mapping:
    if not isinstance(data, Mapping):
        raise ValueError("hardware contract must be a mapping: %s" % path)
    sensors = data.get("sensors", {})
    if not isinstance(sensors, Mapping):
        raise ValueError("hardware contract sensors must be a mapping: %s" % path)
    return sensors


def _default_config_path() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "hardware" / "sensors.yaml"
