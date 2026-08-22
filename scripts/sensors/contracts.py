#!/usr/bin/env python3
"""Load the IG Handle sensor contract and evaluate sensor availability."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Any, Dict, Set

import yaml

from .network import network_value


def load_contract(package_root: str, contract_file: str = "") -> Dict[str, Any]:
    path = _resolve_contract_path(package_root, contract_file)
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return dict(raw.get("sensor_contract", raw) or {})


def sensor_value(
    contract: Dict[str, Any], sensor_id: str, field: str, default: Any = ""
) -> Any:
    sensor = _sensor(contract, sensor_id)
    value: Any = sensor
    for part in str(field or "").split("."):
        if not part:
            continue
        if not isinstance(value, dict) or part not in value:
            return default
        value = value[part]
    if isinstance(value, bool):
        return _bool_text(value)
    return default if value is None else value


def sensor_requested(
    contract: Dict[str, Any],
    sensor_id: str,
    extra_sensor_ids: str = "",
    disabled_sensor_ids: str = "",
) -> bool:
    sensor = _sensor(contract, sensor_id)
    disabled = _id_set(disabled_sensor_ids)
    if sensor_id in disabled:
        return False
    return bool(sensor.get("default_requested", False)) or sensor_id in _id_set(
        extra_sensor_ids
    )


def sensor_reachable(
    contract: Dict[str, Any],
    package_root: str,
    sensor_id: str,
    reachability_check: bool = True,
) -> bool:
    if not reachability_check:
        return True
    sensor = _sensor(contract, sensor_id)
    mode = str(sensor.get("reachability", "none") or "none").strip().lower()
    if mode in ("", "none"):
        return True
    if mode == "device":
        return os.path.exists(str(sensor.get("device_path", "") or ""))
    if mode == "network":
        host = str(
            network_value(
                str(sensor.get("endpoint_key", "") or ""),
                package_root=package_root,
                default="",
            )
            or ""
        ).strip()
        if not host:
            return False
        timeout = int(
            float(
                (contract.get("defaults", {}) or {}).get(
                    "reachability_timeout_sec", 1.0
                )
            )
        )
        return (
            subprocess.run(
                ["ping", "-c", "1", "-W", str(max(1, timeout)), host],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            ).returncode
            == 0
        )
    raise ValueError("unsupported reachability mode %r for %s" % (mode, sensor_id))


def _resolve_contract_path(package_root: str, contract_file: str) -> Path:
    if contract_file:
        return Path(str(contract_file).replace("package://ig_handle", package_root))
    return Path(package_root) / "config" / "sensors" / "sensor_contract.yaml"


def _sensor(contract: Dict[str, Any], sensor_id: str) -> Dict[str, Any]:
    sensors = dict(contract.get("sensors", {}) or {})
    if sensor_id not in sensors:
        raise ValueError("unknown sensor id in contract: %s" % sensor_id)
    return dict(sensors[sensor_id] or {})


def _id_set(value: str) -> Set[str]:
    return {
        item.strip()
        for item in str(value or "").replace(";", ",").split(",")
        if item.strip()
    }


def _bool_text(value: bool) -> str:
    return "true" if bool(value) else "false"
