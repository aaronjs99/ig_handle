"""Unit tests for the pure sonar decode/profile helpers."""

from __future__ import annotations

import struct
from pathlib import Path

from ig_handle_sonar.deltat_runner import DeltaTRunner
from ig_handle_sonar.dt100_profile_decoder import (
    DT100ProfileDecoder,
    decode_profile_packet,
)
from ig_handle_sonar.sonar_profiles import load_sonar_profile


def _profile_packet(*records, header_bytes=8, kind=b"83P"):
    header = bytearray(header_bytes)
    header[:3] = kind
    payload = b"".join(
        struct.pack("<fffH", x, y, z, intensity) for x, y, z, intensity in records
    )
    return bytes(header) + payload


def test_profile_decoder_decodes_little_endian_xyz_records():
    decoder = DT100ProfileDecoder(
        header_bytes=8,
        min_range_m=0.5,
        max_range_m=10.0,
        min_points=2,
        endian="little",
    )
    result = decoder.decode(_profile_packet((1.0, 0.0, 0.0, 7), (0.0, 2.0, 0.0, 8)))

    assert result.ok
    assert result.packet_kind == "83P"
    assert result.points == [(1.0, 0.0, 0.0), (0.0, 2.0, 0.0)]
    assert decoder.last_result == result


def test_profile_decoder_rejects_raw_beam_packets():
    result = decode_profile_packet(b"83B" + b"\0" * 16, header_bytes=8)

    assert not result.ok
    assert result.packet_kind == "83B"
    assert result.reason == "beam_or_raw_packet_not_xyz_profile"


def test_profile_decoder_does_not_try_offset_zero_fallback():
    packet = _profile_packet((1.0, 0.0, 0.0, 7), header_bytes=8)

    result = decode_profile_packet(
        packet,
        header_bytes=8,
        min_range_m=0.5,
        max_range_m=10.0,
        min_points=2,
        endian="little",
    )

    assert not result.ok
    assert "offset=8" in result.reason


def test_sonar_profile_config_selects_harbor_values(sonar_profile_config: Path):
    profile = load_sonar_profile(str(sonar_profile_config), "harbor", udp_port="5050")

    assert profile.name == "harbor"
    assert profile.range_m == 30.0
    assert profile.gain == 6
    assert profile.udp_port == 5050
    assert profile.sound_velocity_m_per_s == 1500.0


def test_sonar_profile_config_selects_pool_freshwater_sound_velocity(
    sonar_profile_config: Path,
):
    profile = load_sonar_profile(str(sonar_profile_config), "pool")

    assert profile.name == "pool"
    assert profile.range_m == 10.0
    assert profile.gain == 16
    assert profile.sound_velocity_m_per_s == 1482.0


def test_deltat_launcher_generates_ini_from_profile(
    package_dir: Path, sonar_profile_config: Path, tmp_path
):
    launcher = DeltaTRunner(
        package_dir=package_dir,
        runtime_dir=tmp_path,
        binary_path=package_dir / "scripts" / "sonar" / "Linux_DeltaT_v1023_x86_64",
    )
    profile = load_sonar_profile(str(sonar_profile_config), "pool")

    ini_text = launcher.ini_text(profile)

    assert "Range:\n10\n" in ini_text
    assert "Gain:\n16\n" in ini_text
    assert "SoundVelocity:\n1482\n" in ini_text
    assert "UDPAddress:\n192.168.0.3\n" in ini_text
