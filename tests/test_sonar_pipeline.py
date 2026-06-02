"""Unit tests for ig_handle sonar profile and DeltaT helpers."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SONAR_SCRIPT_DIR = PACKAGE_DIR / "scripts" / "sonar"
SCRIPTS_DIR = PACKAGE_DIR / "scripts"
SONAR_PROFILE_CONFIG = PACKAGE_DIR / "config" / "sensors" / "sonar" / "profiles.yaml"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(SONAR_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SONAR_SCRIPT_DIR))

from include.deltat_runner import DeltaTRunner
from include.profiles import load_sonar_profile
from network_config import network_value


def test_sonar_profile_config_selects_harbor_values():
    profile = load_sonar_profile(str(SONAR_PROFILE_CONFIG), "harbor", udp_port="5050")

    assert profile.name == "harbor"
    assert profile.range_m == 30.0
    assert profile.gain == 6
    assert profile.udp_port == 5050
    assert profile.sound_velocity_m_per_s == 1500.0


def test_sonar_profile_config_selects_pool_freshwater_sound_velocity():
    profile = load_sonar_profile(str(SONAR_PROFILE_CONFIG), "pool")

    assert profile.name == "pool"
    assert profile.range_m == 10.0
    assert profile.gain == 16
    assert profile.sound_velocity_m_per_s == 1482.0


def test_deltat_launcher_generates_ini_from_profile(tmp_path):
    launcher = DeltaTRunner(
        package_dir=PACKAGE_DIR,
        runtime_dir=tmp_path,
        binary_path=PACKAGE_DIR / "scripts" / "sonar" / "Linux_DeltaT_v1023_x86_64",
    )
    profile = load_sonar_profile(str(SONAR_PROFILE_CONFIG), "pool")

    ini_text = launcher.ini_text(profile)

    assert "Range:\n10\n" in ini_text
    assert "Gain:\n16\n" in ini_text
    assert "SoundVelocity:\n1482\n" in ini_text
    assert f"UDPAddress:\n{network_value('sonar_lan_ip')}\n" in ini_text
