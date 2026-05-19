"""Guarded live Heron motor-control profile test."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from live_sensor_helpers import env_enabled, float_env


pytestmark = pytest.mark.live_hardware

PROFILE_NAME = "restrained_pytest_30s"
EXPECTED_PHASES = [
    ("forward_70", 10.0, 0.70, 0.70, True),
    ("rotate_left70_right90_reverse", 5.0, 0.70, -0.90, True),
    ("reverse_90", 10.0, -0.90, -0.90, True),
    ("rotate_left90_reverse_right70", 5.0, -0.90, 0.70, True),
]
REPO_ROOT = Path(__file__).resolve().parents[2]
PROFILE_SCRIPT = (
    REPO_ROOT / "slam_grande" / "scripts" / "utils" / "heron_thruster_profile.py"
)


def _load_profile_module():
    spec = importlib.util.spec_from_file_location(
        "heron_thruster_profile_under_test", PROFILE_SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _phase_tuple(phase) -> tuple:
    return (
        phase.label,
        float(phase.duration_sec),
        float(phase.left),
        float(phase.right),
        bool(phase.motor_enable),
    )


def test_restrained_thruster_profile_definition_matches_requested_sequence():
    module = _load_profile_module()
    profile = module.ThrusterProfile.from_name(PROFILE_NAME)
    assert [_phase_tuple(phase) for phase in profile.phases] == EXPECTED_PHASES
    assert profile.total_duration_sec() == pytest.approx(30.0)
    profile.validate_safety(
        max_abs_command=1.0,
        sustained_command_threshold=0.25,
        max_sustained_sec=30.0,
    )


@pytest.mark.restrained_thruster
def test_live_restrained_thruster_profile_exercises_both_motors(tmp_path):
    bag_mode = os.environ.get("IG_HANDLE_THRUSTER_PROFILE_BAG_MODE", "control")
    command = [
        sys.executable,
        str(PROFILE_SCRIPT),
        "--profile",
        PROFILE_NAME,
        "--auto-heron",
        "--yes-i-have-restraints",
        "--output-root",
        str(tmp_path),
        "--bag-mode",
        bag_mode,
    ]
    if env_enabled("IG_HANDLE_THRUSTER_PROFILE_NO_CONTROLLER_INPUTS"):
        command.append("--no-require-controller-inputs")

    timeout_sec = float_env("IG_HANDLE_THRUSTER_PROFILE_TIMEOUT_SEC", 100.0)
    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
        check=False,
        timeout=timeout_sec,
    )
    assert result.returncode == 0, (
        "thruster profile failed\n"
        f"command={command!r}\nstdout={result.stdout}\nstderr={result.stderr}"
    )

    summary_paths = sorted(tmp_path.glob("*/profile_summary.json"))
    assert len(summary_paths) == 1
    summary = json.loads(summary_paths[0].read_text(encoding="utf-8"))
    assert summary["profile"] == PROFILE_NAME
    assert summary["return_code"] == 0
    assert summary["abort_reason"] == ""
    assert summary["command_output"] == "cmd_drive"
    assert [
        (
            phase["label"],
            float(phase["duration_sec"]),
            float(phase["left"]),
            float(phase["right"]),
            bool(phase["motor_enable"]),
        )
        for phase in summary["phases"]
    ] == EXPECTED_PHASES

    min_current_a = float_env("IG_HANDLE_THRUSTER_PROFILE_MIN_CURRENT_A", 0.25)
    telemetry = summary["telemetry_snapshot"]
    assert telemetry["max_abs_current_left_a"] >= min_current_a
    assert telemetry["max_abs_current_right_a"] >= min_current_a
