#!/usr/bin/env python3
"""Read the IG Handle sensor contract for launch-time wiring."""

from __future__ import annotations

import os
import json
import subprocess
from pathlib import Path
from typing import Any, Dict, Iterable, List, Set

import yaml

from network_config import network_value


def launch_value(
    package_root: str,
    command: str,
    sensor_id: str = "",
    field: str = "",
    default: str = "",
    contract_file: str = "",
    extra_sensor_ids: str = "",
    disabled_sensor_ids: str = "",
    reachability_check: str = "true",
) -> str:
    """Return a string value for roslaunch eval expressions."""
    contract = load_contract(package_root, contract_file)
    command = str(command or "").strip()
    sensor_id = str(sensor_id or "").strip()
    if command == "value":
        value = sensor_value(contract, sensor_id, field, default)
        if isinstance(value, str):
            value = value.replace("package://ig_handle", package_root)
        return str(value)
    if command == "contract":
        value = contract_value(contract, field, default)
        if isinstance(value, str):
            value = value.replace("package://ig_handle", package_root)
        return str(value)
    if command == "binding":
        value = binding_value(contract, sensor_id, field, default)
        if isinstance(value, str):
            value = value.replace("package://ig_handle", package_root)
        return str(value)
    if command == "binding_sensor":
        return str(binding_value(contract, sensor_id, "sensor", default))
    if command == "bindings":
        return ",".join(
            str(value)
            for value in binding_values(
                contract,
                package_root,
                sensor_id,
                field,
                default,
                extra_sensor_ids,
                disabled_sensor_ids,
                str(reachability_check).lower() == "true",
            )
            if str(value).strip()
        )
    if command == "bindings_json":
        return json.dumps(
            binding_records(
                contract,
                package_root,
                sensor_id,
                extra_sensor_ids,
                disabled_sensor_ids,
                str(reachability_check).lower() == "true",
            ),
            sort_keys=True,
            separators=(",", ":"),
        )
    if command == "records_field":
        return ",".join(record_values_from_json(sensor_id, field, first_only=False))
    if command == "records_first":
        values = record_values_from_json(sensor_id, field, first_only=True)
        return values[0] if values else str(default or "")
    if command == "binding_enabled":
        backing_sensor_id = binding_value(contract, sensor_id, "sensor", "")
        return _bool_text(
            sensor_requested(
                contract, backing_sensor_id, extra_sensor_ids, disabled_sensor_ids
            )
            and sensor_reachable(
                contract,
                package_root,
                backing_sensor_id,
                str(reachability_check).lower() == "true",
            )
        )
    if command == "endpoint":
        sensor = _sensor(contract, sensor_id)
        return str(
            network_value(
                str(sensor.get("endpoint_key", "") or ""),
                package_root=package_root,
                default=default,
            )
            or ""
        )
    if command == "network":
        key = sensor_value(contract, sensor_id, field, "")
        return str(
            network_value(str(key or ""), package_root=package_root, default=default)
            or ""
        )
    if command == "requested":
        return _bool_text(
            sensor_requested(contract, sensor_id, extra_sensor_ids, disabled_sensor_ids)
        )
    if command == "reachable":
        return _bool_text(
            sensor_reachable(
                contract,
                package_root,
                sensor_id,
                str(reachability_check).lower() == "true",
            )
        )
    if command == "enabled":
        return _bool_text(
            sensor_requested(contract, sensor_id, extra_sensor_ids, disabled_sensor_ids)
            and sensor_reachable(
                contract,
                package_root,
                sensor_id,
                str(reachability_check).lower() == "true",
            )
        )
    if command == "default_requested_ids":
        return ",".join(default_requested_ids(contract))
    raise ValueError("unknown sensor contract command: %s" % command)


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


def contract_value(contract: Dict[str, Any], field: str, default: Any = "") -> Any:
    value: Any = contract
    for part in str(field or "").split("."):
        if not part:
            continue
        if not isinstance(value, dict) or part not in value:
            return default
        value = value[part]
    if isinstance(value, bool):
        return _bool_text(value)
    return default if value is None else value


def binding_value(
    contract: Dict[str, Any], binding_id: str, field: str, default: Any = ""
) -> Any:
    binding = _binding(contract, binding_id)
    sensor_id = str(binding.get("sensor", "") or "").strip()
    sensor = _sensor(contract, sensor_id)
    field_key = str(field or "").strip()
    if not field_key or field_key in ("binding", "binding_id", "id"):
        return binding_id
    if field_key == "sensor":
        return sensor_id
    if field_key in binding:
        mapped_field = str(binding[field_key] or "").strip()
        if mapped_field:
            if mapped_field in sensor:
                return sensor[mapped_field]
            return sensor_value(contract, sensor_id, mapped_field, default)
    return sensor_value(contract, sensor_id, field_key, default)


def binding_values(
    contract: Dict[str, Any],
    package_root: str,
    binding_group: str,
    field: str,
    default: Any = "",
    extra_sensor_ids: str = "",
    disabled_sensor_ids: str = "",
    reachability_check: bool = True,
) -> List[Any]:
    values: List[Any] = []
    for binding_id in _binding_ids(contract, binding_group):
        sensor_id = str(binding_value(contract, binding_id, "sensor", "") or "")
        if not sensor_requested(
            contract, sensor_id, extra_sensor_ids, disabled_sensor_ids
        ):
            continue
        if not sensor_reachable(contract, package_root, sensor_id, reachability_check):
            continue
        values.append(binding_value(contract, binding_id, field, default))
    return values


def binding_records(
    contract: Dict[str, Any],
    package_root: str,
    binding_group: str,
    extra_sensor_ids: str = "",
    disabled_sensor_ids: str = "",
    reachability_check: bool = True,
) -> List[Dict[str, str]]:
    records: List[Dict[str, str]] = []
    fields = ("binding", "sensor", "role", "topic", "frame", "info_topic")
    for binding_id in _binding_ids(contract, binding_group):
        sensor_id = str(binding_value(contract, binding_id, "sensor", "") or "")
        if not sensor_requested(
            contract, sensor_id, extra_sensor_ids, disabled_sensor_ids
        ):
            continue
        if not sensor_reachable(contract, package_root, sensor_id, reachability_check):
            continue
        record = {}
        for field in fields:
            if field == "binding":
                value = binding_id
            else:
                value = binding_value(contract, binding_id, field, "")
            record[field] = str(value or "")
        records.append(record)
    return records


def record_values_from_json(
    records_json: str, field: str, first_only: bool
) -> List[str]:
    values: List[str] = []
    field_key = str(field or "").strip()
    if not field_key:
        return values
    for item in json.loads(str(records_json or "[]")):
        if not isinstance(item, dict):
            continue
        value = str(item.get(field_key, "") or "").strip()
        values.append(value)
        if first_only:
            break
    return values


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


def default_requested_ids(contract: Dict[str, Any]) -> Iterable[str]:
    sensors = dict(contract.get("sensors", {}) or {})
    return [
        sensor_id
        for sensor_id in sorted(
            sensors,
            key=lambda item: (
                (sensors.get(item) or {}).get("startup_order", 1000),
                str(item),
            ),
        )
        if bool((sensors.get(sensor_id) or {}).get("default_requested", False))
    ]


def _resolve_contract_path(package_root: str, contract_file: str) -> Path:
    if contract_file:
        return Path(str(contract_file).replace("package://ig_handle", package_root))
    return Path(package_root) / "config" / "sensors" / "sensor_contract.yaml"


def _sensor(contract: Dict[str, Any], sensor_id: str) -> Dict[str, Any]:
    sensors = dict(contract.get("sensors", {}) or {})
    if sensor_id not in sensors:
        raise ValueError("unknown sensor id in contract: %s" % sensor_id)
    return dict(sensors[sensor_id] or {})


def _binding(contract: Dict[str, Any], binding_id: str) -> Dict[str, Any]:
    bindings = dict(contract.get("deployment_bindings", {}) or {})
    if binding_id not in bindings:
        raise ValueError("unknown deployment binding in contract: %s" % binding_id)
    return dict(bindings[binding_id] or {})


def _binding_ids(contract: Dict[str, Any], binding_group: str) -> List[str]:
    groups = dict(contract.get("deployment_binding_groups", {}) or {})
    key = str(binding_group or "").strip()
    if key in groups:
        raw_items = groups[key]
    else:
        raw_items = str(binding_group or "").replace(";", ",").split(",")
    ids: List[str] = []
    for item in raw_items or []:
        binding_id = str(item or "").strip()
        if binding_id and binding_id not in ids:
            ids.append(binding_id)
    return ids


def _id_set(value: str) -> Set[str]:
    return {
        item.strip()
        for item in str(value or "").replace(";", ",").split(",")
        if item.strip()
    }


def _bool_text(value: bool) -> str:
    return "true" if bool(value) else "false"
