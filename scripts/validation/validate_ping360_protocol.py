#!/usr/bin/env python3
"""Deterministic, hardware-free validation for Ping360 wire parsing."""

from __future__ import annotations

import math
import struct
import sys
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[2]
SONAR_INCLUDE = PACKAGE_DIR / "scripts" / "sonar" / "include"
sys.path.insert(0, str(SONAR_INCLUDE))

from ping_protocol import (  # noqa: E402
    DEVICE_INFORMATION,
    PING360_AUTO_DEVICE_DATA,
    PING360_DEVICE_DATA,
    PROTOCOL_VERSION,
    PingProtocolError,
    build_auto_transmit,
    build_frame,
    build_general_request,
    parse_device_information,
    parse_frame,
    parse_profile,
    parse_protocol_version,
)


CHECKS = []


def check(name, condition):
    if not condition:
        raise AssertionError(name)
    CHECKS.append(name)


def expect_error(name, function):
    try:
        function()
    except PingProtocolError:
        CHECKS.append(name)
        return
    raise AssertionError("expected PingProtocolError: %s" % name)


def main():
    request = parse_frame(build_general_request(DEVICE_INFORMATION))
    check("general request id", request.message_id == 6)
    check("general request payload", request.payload == struct.pack("<H", 4))

    identity = parse_frame(
        build_frame(DEVICE_INFORMATION, struct.pack("<BBBBBB", 2, 1, 3, 4, 5, 0))
    )
    check("Ping360 identity", parse_device_information(identity)[:2] == (2, 1))
    version = parse_frame(
        build_frame(PROTOCOL_VERSION, struct.pack("<BBBB", 1, 1, 0, 0))
    )
    check("protocol version", parse_protocol_version(version)[:3] == (1, 1, 0))

    intensities = bytes(byte % 256 for byte in range(200))
    device_payload = (
        struct.pack("<BBHHHHHH", 1, 1, 100, 11, 1000, 750, 200, 200) + intensities
    )
    profile = parse_profile(
        parse_frame(build_frame(PING360_DEVICE_DATA, device_payload))
    )
    check("device profile bytes", profile.intensities == intensities)
    check("gradian angle conversion", math.isclose(profile.angle_rad, math.pi / 2))
    check(
        "one-way range interval",
        math.isclose(profile.sample_interval_m(1480.0), 0.0185),
    )

    auto_payload = (
        struct.pack("<BBHHHHHHBBHH", 1, 2, 399, 11, 1000, 750, 10, 390, 2, 0, 200, 200)
        + intensities
    )
    auto = parse_profile(
        parse_frame(build_frame(PING360_AUTO_DEVICE_DATA, auto_payload))
    )
    check("auto scan metadata", auto.auto_scan and auto.num_steps == 2)

    corrupted = bytearray(build_frame(PING360_DEVICE_DATA, device_payload))
    corrupted[-1] ^= 0xFF
    check(
        "checksum observable",
        not parse_frame(bytes(corrupted), require_valid_checksum=False).checksum_valid,
    )
    expect_error("checksum rejection", lambda: parse_frame(bytes(corrupted)))
    expect_error("truncation rejection", lambda: parse_frame(b"BR\x00"))

    bad_count = (
        struct.pack("<BBHHHHHH", 1, 1, 100, 11, 1000, 750, 201, 200) + intensities
    )
    expect_error(
        "sample metadata rejection",
        lambda: parse_profile(parse_frame(build_frame(PING360_DEVICE_DATA, bad_count))),
    )
    expect_error(
        "unsafe active configuration rejection",
        lambda: build_auto_transmit(1, 11, 79, 750, 200, 0, 399, 1, 0),
    )
    command = parse_frame(build_auto_transmit(1, 11, 1000, 750, 1200, 0, 399, 1, 0))
    check("active command framing", command.message_id == 2602)

    print("Ping360 protocol validation passed (%d checks)" % len(CHECKS))


if __name__ == "__main__":
    main()
