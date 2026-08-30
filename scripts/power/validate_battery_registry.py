#!/usr/bin/env python3
"""Validate physical battery identity and JK configuration consistency."""

from __future__ import annotations

import argparse
import copy
import json
import tempfile
import threading
from pathlib import Path
from typing import Dict, List

import yaml
import rospkg

from power.battery_registry import (
    BatteryRegistry,
    BatteryRegistryError,
    jk_identity_from_config,
    require_jk_config_match,
)
from power.bluez_ble import BluezBleClient, BluezError
from power.reconnect_guard import (
    MAX_PROTOCOL_ERROR_RECONNECT_THRESHOLD,
    ConsecutiveErrorThreshold,
    ReconnectRequest,
)
from power.jk_bms_protocol import (
    INITIAL_READ_QUERY_COMMANDS,
    PERIODIC_READ_QUERY_COMMANDS,
    ProtocolError,
    ReadQueryGate,
    response_kind,
)

PACKAGE_ROOT = Path(rospkg.RosPack().get_path("ig_handle"))
DEFAULT_REGISTRY = PACKAGE_ROOT / "config" / "sensors" / "battery_registry.yaml"
DEFAULT_JK_CONFIG = PACKAGE_ROOT / "config" / "sensors" / "jk_bms.yaml"
DEFAULT_CASES = PACKAGE_ROOT / "config" / "validation" / "battery_registry_cases.yaml"


def _load_yaml(path: Path) -> Dict[str, object]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("{} must contain a mapping".format(path))
    return payload


def _expect_error(payload: Dict[str, object]) -> bool:
    with tempfile.TemporaryDirectory(prefix="battery-registry-") as directory:
        path = Path(directory) / "registry.yaml"
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        try:
            BatteryRegistry.load(path)
        except BatteryRegistryError:
            return True
    return False


def validate(registry_path: Path, jk_config_path: Path, cases_path: Path) -> dict:
    checks: List[dict] = []

    def record(name: str, passed: bool, detail: str = "") -> None:
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    registry = BatteryRegistry.load(registry_path)
    registry_payload = _load_yaml(registry_path)
    jk_config = _load_yaml(jk_config_path)
    cases = _load_yaml(cases_path)
    if cases.get("schema_version") != 1:
        raise ValueError("unsupported battery registry case schema")

    records = registry.records()
    actual_ids = [record.battery_id for record in records]
    expected_ids = sorted(str(value) for value in cases["expected_battery_ids"])
    record("expected_ids", actual_ids == expected_ids, str(actual_ids))

    expected_roles = dict(cases["expected_roles"])
    actual_roles = {record.battery_id: record.role for record in records}
    record("expected_roles", actual_roles == expected_roles, str(actual_roles))
    expected_commissioned = dict(cases["expected_commissioned"])
    actual_commissioned = {
        battery.battery_id: battery.commissioned for battery in records
    }
    record(
        "expected_commissioned",
        actual_commissioned == expected_commissioned,
        str(actual_commissioned),
    )

    configured = require_jk_config_match(registry, jk_config)
    expected_jk_id = str(cases["expected_jk_battery_id"])
    record(
        "jk_config_exact_match",
        configured.battery_id == expected_jk_id,
        configured.battery_id,
    )

    identity = jk_identity_from_config(jk_config)
    for field in tuple(identity):
        mutated = dict(identity)
        mutated[field] = "MISMATCH-{}".format(field)
        record(
            "jk_reject_{}".format(field),
            registry.resolve_jk_bms(mutated) is None,
        )

    unknown = dict(identity)
    unknown["serial_number"] = str(cases["unknown_jk_serial_number"])
    record("unknown_jk_is_unassigned", registry.resolve_jk_bms(unknown) is None)

    request_period = float(jk_config["request_period_sec"])
    sample_timeout = float(jk_config["sample_timeout_sec"])
    record(
        "jk_request_cadence_precedes_timeout",
        request_period > 0.0 and sample_timeout > request_period + 1.0,
        "request_period_sec={} sample_timeout_sec={}".format(
            request_period, sample_timeout
        ),
    )
    record(
        "jk_initial_query_requests_identity_once",
        INITIAL_READ_QUERY_COMMANDS == (0x97,),
        "commands={}".format(INITIAL_READ_QUERY_COMMANDS),
    )
    record(
        "jk_periodic_query_requests_telemetry_only",
        PERIODIC_READ_QUERY_COMMANDS == (0x96,),
        "commands={}".format(PERIODIC_READ_QUERY_COMMANDS),
    )
    record(
        "jk_settings_response_is_admitted_without_measurement",
        response_kind(0x01) == "settings",
    )
    record(
        "jk_telemetry_and_identity_responses_are_distinct",
        response_kind(0x02) == "telemetry" and response_kind(0x03) == "device_info",
    )
    try:
        response_kind(0xFF)
        unsupported_response_rejected = False
    except ProtocolError:
        unsupported_response_rejected = True
    record("jk_unknown_response_is_rejected", unsupported_response_rejected)
    query_gate = ReadQueryGate()
    try:
        query_gate.admit_telemetry()
        preidentity_telemetry_rejected = False
    except ProtocolError:
        preidentity_telemetry_rejected = True
    initial_state = (
        not query_gate.identity_admitted
        and not query_gate.telemetry_admitted
        and query_gate.telemetry_query_needed
    )
    query_gate.admit_identity()
    identity_state = query_gate.identity_admitted and query_gate.telemetry_query_needed
    query_gate.admit_telemetry()
    streaming_state = (
        query_gate.telemetry_admitted and not query_gate.telemetry_query_needed
    )
    query_gate.reset()
    reset_state = (
        not query_gate.identity_admitted
        and not query_gate.telemetry_admitted
        and query_gate.telemetry_query_needed
    )
    record(
        "jk_query_gate_stops_after_admitted_telemetry_and_resets",
        preidentity_telemetry_rejected
        and initial_state
        and identity_state
        and streaming_state
        and reset_state,
    )
    transport_gate_valid = (
        not BluezBleClient.periodic_query_due(
            identity_admitted=False,
            telemetry_query_needed=True,
        )
        and BluezBleClient.periodic_query_due(
            identity_admitted=True,
            telemetry_query_needed=True,
        )
        and not BluezBleClient.periodic_query_due(
            identity_admitted=False,
            telemetry_query_needed=False,
        )
        and not BluezBleClient.periodic_query_due(
            identity_admitted=True,
            telemetry_query_needed=False,
        )
    )
    record(
        "jk_transport_query_requires_identity_and_stops_on_stream",
        transport_gate_valid,
    )
    timeout_probe = object.__new__(BluezBleClient)
    timeout_probe.initial_query_delay_sec = 0.001
    timeout_probe._stop = threading.Event()
    timeout_probe._reconnect_request = ReconnectRequest()
    timeout_probe.periodic_query_ready = lambda: False
    timeout_probe.periodic_query_needed = lambda: True
    try:
        timeout_probe._wait_for_periodic_query_gate()
        identity_timeout_rejected = False
    except BluezError as exc:
        identity_timeout_rejected = str(exc) == "BMS identity response timeout"
    record(
        "jk_identity_timeout_reconnects_without_telemetry_query",
        identity_timeout_rejected,
    )

    protocol_error_threshold = jk_config.get("protocol_error_reconnect_threshold")
    threshold_is_bounded = (
        type(protocol_error_threshold) is int
        and 1 <= protocol_error_threshold <= MAX_PROTOCOL_ERROR_RECONNECT_THRESHOLD
    )
    record(
        "jk_protocol_error_reconnect_threshold_bounded",
        threshold_is_bounded,
        "protocol_error_reconnect_threshold={}".format(protocol_error_threshold),
    )
    if threshold_is_bounded:
        error_gate = ConsecutiveErrorThreshold(protocol_error_threshold)
        reconnect_request = ReconnectRequest()
        triggered_early = False
        for _ in range(protocol_error_threshold - 1):
            triggered_early = triggered_early or error_gate.record_failure()
        triggered_at_threshold = error_gate.record_failure()
        if triggered_at_threshold:
            reconnect_request.request("persistent malformed JK telemetry")
        requested, request_reason = reconnect_request.snapshot()
        record(
            "jk_protocol_error_recovery_requests_reconnect",
            not triggered_early
            and triggered_at_threshold
            and requested
            and request_reason == "persistent malformed JK telemetry",
            "count={} requested={}".format(error_gate.count, requested),
        )
        error_gate.record_success()
        reconnect_request.reset()
        requested_after_reset, reason_after_reset = reconnect_request.snapshot()
        record(
            "jk_protocol_error_recovery_resets_after_success",
            error_gate.count == 0
            and not requested_after_reset
            and not reason_after_reset,
        )

    for battery_id in ("HERON-01", "HERON-02"):
        try:
            registry.require(battery_id, role="propulsion")
            passed = True
        except BatteryRegistryError:
            passed = False
        record("{}_registered_as_propulsion".format(battery_id.lower()), passed)
        record(
            "{}_uncommissioned_until_labeled".format(battery_id.lower()),
            passed and not registry.require(battery_id).commissioned,
        )
    try:
        registry.require(expected_jk_id, role="propulsion")
        rejected_wrong_role = False
    except BatteryRegistryError:
        rejected_wrong_role = True
    record("compute_pack_rejected_as_propulsion", rejected_wrong_role)

    duplicate = copy.deepcopy(registry_payload)
    duplicate["batteries"]["IGHANDLE-CLONE"] = copy.deepcopy(
        duplicate["batteries"]["IGHANDLE-01"]
    )
    duplicate["batteries"]["IGHANDLE-CLONE"]["asset_label"] = "IGHANDLE-CLONE"
    record("duplicate_serial_rejected", _expect_error(duplicate))

    missing_commissioning = copy.deepcopy(registry_payload)
    del missing_commissioning["batteries"]["HERON-01"]["commissioned"]
    record("missing_commissioning_rejected", _expect_error(missing_commissioning))

    mismatched_label = copy.deepcopy(registry_payload)
    mismatched_label["batteries"]["HERON-01"]["asset_label"] = "HERON-02"
    record("mismatched_asset_label_rejected", _expect_error(mismatched_label))

    malformed = copy.deepcopy(registry_payload)
    malformed["batteries"]["heron one"] = malformed["batteries"].pop("HERON-01")
    record("malformed_id_rejected", _expect_error(malformed))

    passed = all(check["passed"] for check in checks)
    return {"passed": passed, "checks": checks}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--jk-config", default=str(DEFAULT_JK_CONFIG))
    parser.add_argument("--cases", default=str(DEFAULT_CASES))
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        result = validate(Path(args.registry), Path(args.jk_config), Path(args.cases))
    except (KeyError, OSError, ValueError, yaml.YAMLError) as exc:
        result = {"passed": False, "error": str(exc), "checks": []}
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        for check in result.get("checks", []):
            print("{} {}".format("PASS" if check["passed"] else "FAIL", check["name"]))
        if result.get("error"):
            print("FAIL {}".format(result["error"]))
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
