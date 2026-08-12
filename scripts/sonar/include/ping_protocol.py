#!/usr/bin/env python3
"""Pure Ping Protocol framing and Ping360 profile parsing.

This module deliberately has no ROS or device dependency so recorded UDP
datagrams can be validated and replayed without the sonar or its vendor SDK.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional


HEADER = struct.Struct("<2sHHBB")
CHECKSUM = struct.Struct("<H")
GENERAL_REQUEST = 6
DEVICE_INFORMATION = 4
PROTOCOL_VERSION = 5
PING360_DEVICE_DATA = 2300
PING360_AUTO_DEVICE_DATA = 2301
PING360_AUTO_TRANSMIT = 2602
PING360_MOTOR_OFF = 2903


class PingProtocolError(ValueError):
    """A frame is truncated, malformed, or inconsistent."""


def profile_identity_rejection(
    identity_valid: bool, source_device_id: int, identified_device_id: Optional[int]
) -> str:
    """Reject profiles until their Ping source matches a verified identity."""

    if not identity_valid or identified_device_id is None:
        return "profile_identity_unverified"
    if int(source_device_id) != int(identified_device_id):
        return "profile_source_device_mismatch"
    return ""


@dataclass(frozen=True)
class PingFrame:
    raw: bytes
    message_id: int
    source_device_id: int
    destination_device_id: int
    payload: bytes
    checksum_valid: bool


@dataclass(frozen=True)
class Ping360Profile:
    angle_grad: int
    gain_setting: int
    transmit_duration_us: int
    sample_period_ticks_25ns: int
    transmit_frequency_khz: int
    number_of_samples: int
    intensities: bytes
    auto_scan: bool = False
    start_angle_grad: int = 0
    stop_angle_grad: int = 0
    num_steps: int = 0

    @property
    def angle_rad(self) -> float:
        return math.radians(float(self.angle_grad) * 0.9)

    def sample_interval_m(self, sound_speed_mps: float) -> float:
        return self.sample_period_ticks_25ns * 25e-9 * sound_speed_mps / 2.0


def checksum(data: bytes) -> int:
    return sum(bytearray(data)) & 0xFFFF


def build_frame(
    message_id: int,
    payload: bytes = b"",
    source_device_id: int = 0,
    destination_device_id: int = 0,
) -> bytes:
    header = HEADER.pack(
        b"BR",
        len(payload),
        message_id,
        source_device_id,
        destination_device_id,
    )
    body = header + payload
    return body + CHECKSUM.pack(checksum(body))


def build_general_request(requested_id: int, **ids: int) -> bytes:
    return build_frame(GENERAL_REQUEST, struct.pack("<H", requested_id), **ids)


def build_auto_transmit(
    gain_setting: int,
    transmit_duration_us: int,
    sample_period_ticks_25ns: int,
    transmit_frequency_khz: int,
    number_of_samples: int,
    start_angle_grad: int,
    stop_angle_grad: int,
    num_steps: int,
    delay_ms: int,
    **ids: int,
) -> bytes:
    _in_range("gain_setting", gain_setting, 0, 2)
    _in_range("transmit_duration_us", transmit_duration_us, 1, 1000)
    _in_range("sample_period_ticks_25ns", sample_period_ticks_25ns, 80, 40000)
    _in_range("transmit_frequency_khz", transmit_frequency_khz, 500, 1000)
    _in_range("number_of_samples", number_of_samples, 200, 1200)
    _in_range("start_angle_grad", start_angle_grad, 0, 399)
    _in_range("stop_angle_grad", stop_angle_grad, 0, 399)
    _in_range("num_steps", num_steps, 1, 10)
    _in_range("delay_ms", delay_ms, 0, 100)
    payload = struct.pack(
        "<BBHHHHHHBB",
        1,
        gain_setting,
        transmit_duration_us,
        sample_period_ticks_25ns,
        transmit_frequency_khz,
        number_of_samples,
        start_angle_grad,
        stop_angle_grad,
        num_steps,
        delay_ms,
    )
    return build_frame(PING360_AUTO_TRANSMIT, payload, **ids)


def parse_frame(data: bytes, require_valid_checksum: bool = True) -> PingFrame:
    if len(data) < HEADER.size + CHECKSUM.size:
        raise PingProtocolError("frame is shorter than the 10-byte minimum")
    marker, payload_length, message_id, source_id, destination_id = HEADER.unpack_from(
        data
    )
    if marker != b"BR":
        raise PingProtocolError("frame does not start with BR")
    expected_length = HEADER.size + payload_length + CHECKSUM.size
    if len(data) != expected_length:
        raise PingProtocolError(
            "frame length %d does not match declared length %d"
            % (len(data), expected_length)
        )
    received_checksum = CHECKSUM.unpack_from(data, HEADER.size + payload_length)[0]
    valid = received_checksum == checksum(data[: HEADER.size + payload_length])
    if require_valid_checksum and not valid:
        raise PingProtocolError("checksum mismatch")
    return PingFrame(
        raw=bytes(data),
        message_id=message_id,
        source_device_id=source_id,
        destination_device_id=destination_id,
        payload=bytes(data[HEADER.size : HEADER.size + payload_length]),
        checksum_valid=valid,
    )


def parse_profile(frame: PingFrame) -> Ping360Profile:
    if not frame.checksum_valid:
        raise PingProtocolError("profile checksum is invalid")
    if frame.message_id == PING360_DEVICE_DATA:
        fixed = struct.Struct("<BBHHHHHH")
        if len(frame.payload) < fixed.size:
            raise PingProtocolError("device_data fixed payload is truncated")
        (
            mode,
            gain,
            angle,
            duration,
            sample_period,
            frequency,
            sample_count,
            data_length,
        ) = fixed.unpack_from(frame.payload)
        start_angle, stop_angle, steps = 0, 0, 0
        auto_scan = False
    elif frame.message_id == PING360_AUTO_DEVICE_DATA:
        fixed = struct.Struct("<BBHHHHHHBBHH")
        if len(frame.payload) < fixed.size:
            raise PingProtocolError("auto_device_data fixed payload is truncated")
        (
            mode,
            gain,
            angle,
            duration,
            sample_period,
            frequency,
            start_angle,
            stop_angle,
            steps,
            _delay,
            sample_count,
            data_length,
        ) = fixed.unpack_from(frame.payload)
        auto_scan = True
    else:
        raise PingProtocolError(
            "message %d is not a Ping360 profile" % frame.message_id
        )
    if mode != 1:
        raise PingProtocolError("unsupported Ping360 operating mode %d" % mode)
    _in_range("gain_setting", gain, 0, 2)
    _in_range("angle_grad", angle, 0, 399)
    _in_range("transmit_duration_us", duration, 1, 1000)
    _in_range("sample_period_ticks_25ns", sample_period, 80, 40000)
    _in_range("transmit_frequency_khz", frequency, 500, 1000)
    _in_range("number_of_samples", sample_count, 200, 1200)
    if auto_scan:
        _in_range("start_angle_grad", start_angle, 0, 399)
        _in_range("stop_angle_grad", stop_angle, 0, 399)
        _in_range("num_steps", steps, 1, 10)
    intensities = frame.payload[fixed.size :]
    if data_length != sample_count or len(intensities) != data_length:
        raise PingProtocolError(
            "profile sample metadata disagrees: count=%d data_length=%d bytes=%d"
            % (sample_count, data_length, len(intensities))
        )
    return Ping360Profile(
        angle_grad=angle,
        gain_setting=gain,
        transmit_duration_us=duration,
        sample_period_ticks_25ns=sample_period,
        transmit_frequency_khz=frequency,
        number_of_samples=sample_count,
        intensities=intensities,
        auto_scan=auto_scan,
        start_angle_grad=start_angle,
        stop_angle_grad=stop_angle,
        num_steps=steps,
    )


def parse_device_information(frame: PingFrame) -> Optional[tuple]:
    if frame.message_id != DEVICE_INFORMATION:
        return None
    if len(frame.payload) != 6:
        raise PingProtocolError("device_information payload must be 6 bytes")
    return struct.unpack("<BBBBBB", frame.payload)


def parse_protocol_version(frame: PingFrame) -> Optional[tuple]:
    if frame.message_id != PROTOCOL_VERSION:
        return None
    if len(frame.payload) != 4:
        raise PingProtocolError("protocol_version payload must be 4 bytes")
    return struct.unpack("<BBBB", frame.payload)


def _in_range(name: str, value: int, minimum: int, maximum: int) -> None:
    if not minimum <= value <= maximum:
        raise PingProtocolError(
            "%s=%d is outside [%d, %d]" % (name, value, minimum, maximum)
        )
