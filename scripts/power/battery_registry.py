"""Validated physical-battery registry and identity resolution.

Battery identity is established from explicit hardware metadata or an operator
installation record. Electrical behavior is never treated as identity.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional

import yaml

SCHEMA_VERSION = 1
UNKNOWN_BATTERY_ID = "UNKNOWN"
_BATTERY_ID_RE = re.compile(r"^[A-Z0-9][A-Z0-9_-]{2,63}$")
_ROLES = {"compute", "propulsion"}
_IDENTITY_METHODS = {"jk_bms", "operator_selection"}
_JK_FIELDS = (
    "device_address",
    "device_name",
    "model",
    "hardware_version",
    "software_version",
    "serial_number",
    "manufacturing_date",
)
_UNIQUE_HARDWARE_FIELDS = (
    "device_address",
    "device_name",
    "serial_number",
)


class BatteryRegistryError(ValueError):
    """Raised when physical battery identity configuration is ambiguous."""


def _required_text(mapping: Mapping[str, object], key: str) -> str:
    value = str(mapping.get(key, "") or "").strip()
    if not value:
        raise BatteryRegistryError("{} must be non-empty text".format(key))
    return value


def _normalized_hardware_id(key: str, value: object) -> str:
    text = str(value or "").strip()
    if key == "device_address":
        return text.upper()
    return text


@dataclass(frozen=True)
class BatteryRecord:
    battery_id: str
    role: str
    platform: str
    asset_label: str
    commissioned: bool
    identity_method: str
    hardware: Dict[str, str]

    def hardware_id(self) -> str:
        for key in ("serial_number", "device_address"):
            value = self.hardware.get(key, "")
            if value:
                return value
        return ""


class BatteryRegistry:
    def __init__(self, records: Iterable[BatteryRecord]) -> None:
        self._records: Dict[str, BatteryRecord] = {}
        unique_hardware: Dict[tuple, str] = {}
        for record in records:
            if record.battery_id in self._records:
                raise BatteryRegistryError(
                    "duplicate battery_id {}".format(record.battery_id)
                )
            self._records[record.battery_id] = record
            for key in _UNIQUE_HARDWARE_FIELDS:
                value = _normalized_hardware_id(key, record.hardware.get(key, ""))
                if not value:
                    continue
                unique_key = (key, value)
                previous = unique_hardware.get(unique_key)
                if previous is not None:
                    raise BatteryRegistryError(
                        "{} {} is shared by {} and {}".format(
                            key, value, previous, record.battery_id
                        )
                    )
                unique_hardware[unique_key] = record.battery_id
        if not self._records:
            raise BatteryRegistryError("battery registry must not be empty")

    @classmethod
    def load(cls, path: Path) -> "BatteryRegistry":
        try:
            payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise BatteryRegistryError(
                "cannot read battery registry {}: {}".format(path, exc)
            ) from exc
        if not isinstance(payload, dict):
            raise BatteryRegistryError("battery registry root must be a mapping")
        if payload.get("schema_version") != SCHEMA_VERSION:
            raise BatteryRegistryError("unsupported battery registry schema")
        batteries = payload.get("batteries")
        if not isinstance(batteries, dict):
            raise BatteryRegistryError("batteries must be a mapping")
        records = []
        for battery_id, raw in batteries.items():
            battery_id = str(battery_id or "").strip()
            if not _BATTERY_ID_RE.fullmatch(battery_id):
                raise BatteryRegistryError(
                    "battery_id {!r} must be normalized uppercase text".format(
                        battery_id
                    )
                )
            if battery_id == UNKNOWN_BATTERY_ID:
                raise BatteryRegistryError(
                    "UNKNOWN is reserved for unresolved identity"
                )
            if not isinstance(raw, dict):
                raise BatteryRegistryError(
                    "{} configuration must be a mapping".format(battery_id)
                )
            role = _required_text(raw, "role").lower()
            if role not in _ROLES:
                raise BatteryRegistryError(
                    "{} has unsupported role {}".format(battery_id, role)
                )
            platform = _required_text(raw, "platform").lower()
            asset_label = _required_text(raw, "asset_label")
            commissioned = raw.get("commissioned")
            if not isinstance(commissioned, bool):
                raise BatteryRegistryError(
                    "{} commissioned must be true or false".format(battery_id)
                )
            identity = raw.get("identity")
            if not isinstance(identity, dict):
                raise BatteryRegistryError(
                    "{} identity must be a mapping".format(battery_id)
                )
            method = _required_text(identity, "method").lower()
            if method not in _IDENTITY_METHODS:
                raise BatteryRegistryError(
                    "{} has unsupported identity method {}".format(battery_id, method)
                )
            hardware_raw = identity.get("hardware", {})
            if not isinstance(hardware_raw, dict):
                raise BatteryRegistryError(
                    "{} identity.hardware must be a mapping".format(battery_id)
                )
            hardware = {
                str(key): _normalized_hardware_id(str(key), value)
                for key, value in hardware_raw.items()
                if str(value or "").strip()
            }
            if method == "jk_bms":
                missing = [field for field in _JK_FIELDS if not hardware.get(field)]
                if missing:
                    raise BatteryRegistryError(
                        "{} JK identity is missing {}".format(
                            battery_id, ", ".join(missing)
                        )
                    )
                if role != "compute":
                    raise BatteryRegistryError(
                        "{} JK BMS identity must have compute role".format(battery_id)
                    )
                if not commissioned:
                    raise BatteryRegistryError(
                        "{} JK BMS identity must be commissioned".format(battery_id)
                    )
            if method == "operator_selection" and role != "propulsion":
                raise BatteryRegistryError(
                    "{} operator selection is reserved for propulsion packs".format(
                        battery_id
                    )
                )
            if method == "operator_selection" and hardware:
                raise BatteryRegistryError(
                    "{} operator selection must not imply hardware identity".format(
                        battery_id
                    )
                )
            if method == "operator_selection" and asset_label != battery_id:
                raise BatteryRegistryError(
                    "{} operator-selected asset_label must exactly match its battery_id".format(
                        battery_id
                    )
                )
            records.append(
                BatteryRecord(
                    battery_id=battery_id,
                    role=role,
                    platform=platform,
                    asset_label=asset_label,
                    commissioned=commissioned,
                    identity_method=method,
                    hardware=hardware,
                )
            )
        return cls(records)

    def records(self) -> list:
        return [self._records[key] for key in sorted(self._records)]

    def require(self, battery_id: str, *, role: Optional[str] = None) -> BatteryRecord:
        battery_id = str(battery_id or "").strip()
        record = self._records.get(battery_id)
        if record is None:
            raise BatteryRegistryError("unknown battery_id {}".format(battery_id))
        if role is not None and record.role != role:
            raise BatteryRegistryError(
                "{} has role {}, expected {}".format(battery_id, record.role, role)
            )
        return record

    def resolve_jk_bms(self, identity: Mapping[str, object]) -> Optional[BatteryRecord]:
        normalized = {
            field: _normalized_hardware_id(field, identity.get(field, ""))
            for field in _JK_FIELDS
        }
        matches = []
        for record in self._records.values():
            if record.identity_method != "jk_bms":
                continue
            if all(
                record.hardware.get(field, "") == normalized[field]
                for field in _JK_FIELDS
            ):
                matches.append(record)
        if not matches:
            return None
        if len(matches) != 1:
            raise BatteryRegistryError("JK BMS identity is ambiguous")
        return matches[0]


def jk_identity_from_config(config: Mapping[str, object]) -> Dict[str, str]:
    """Translate the commissioned JK config keys into registry identity keys."""

    mapping = {
        "device_address": "device_address",
        "device_name": "expected_device_name",
        "model": "expected_model",
        "hardware_version": "expected_hardware_version",
        "software_version": "expected_software_version",
        "serial_number": "expected_serial_number",
        "manufacturing_date": "expected_manufacturing_date",
    }
    return {
        target: _normalized_hardware_id(target, config.get(source, ""))
        for target, source in mapping.items()
    }


def require_jk_config_match(
    registry: BatteryRegistry, config: Mapping[str, object]
) -> BatteryRecord:
    record = registry.resolve_jk_bms(jk_identity_from_config(config))
    if record is None:
        raise BatteryRegistryError(
            "commissioned JK config does not match any registered battery"
        )
    return record
