#!/usr/bin/env python3
"""Strict, read-only decoder for the common JK02 BLE telemetry protocol."""

from dataclasses import dataclass
import math
import re
import struct
from typing import List, Optional, Sequence, Tuple


FRAME_HEADER = b"\x55\xaa\xeb\x90"
FRAME_LENGTH = 300
SUPPORTED_PROTOCOLS = ("jk02_24s", "jk02_32s")
JK02_ALARM_NAMES = (
    "wire_resistance",
    "mosfet_overtemperature",
    "cell_count_mismatch",
    "reserved_bit_03",
    "battery_fully_charged",
    "pack_overvoltage",
    "charge_overcurrent",
    "charge_short_circuit",
    "charge_overtemperature",
    "charge_undertemperature",
    "coprocessor_communication_error",
    "cell_undervoltage",
    "pack_undervoltage",
    "discharge_overcurrent",
    "discharge_short_circuit",
    "discharge_overtemperature",
    "charge_mosfet_abnormal",
    "discharge_mosfet_abnormal",
    "gps_disconnected",
    "change_password_prompt",
    "discharge_on_failed",
    "battery_overtemperature",
    "temperature_sensor_anomaly",
    "pcl_module_anomaly",
    "short_circuit_release_failed",
    "discharge_overcurrent_level_2",
    "discharge_overcurrent_level_3",
    "discharge_undertemperature",
    "gps_remote_lock",
    "reserved_bit_29",
    "reserved_bit_30",
    "reserved_bit_31",
)
NONFAULT_STATUS_MASK = (1 << 4) | (1 << 18) | (1 << 19)


class ProtocolError(ValueError):
    """Raised when a frame is malformed, inconsistent, or implausible."""


@dataclass(frozen=True)
class DeviceInfo:
    model: str
    hardware_version: str
    software_version: str
    device_name: str
    manufacturing_date: str
    serial_number: str
    power_on_count: int


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    cell_voltages_v: Tuple[float, ...]
    cell_resistances_ohm: Tuple[float, ...]
    pack_voltage_v: float
    current_a: float
    temperatures_c: Tuple[float, ...]
    mos_temperature_c: float
    balancing_current_a: float
    balancing_active: bool
    state_of_charge: float
    state_of_health: float
    remaining_capacity_ah: float
    nominal_capacity_ah: float
    cycle_count: int
    cycle_capacity_ah: float
    runtime_sec: int
    alarm_flags: int
    charge_enabled: bool
    discharge_enabled: bool
    precharge_active: bool
    heating_active: bool

    @property
    def pack_power_w(self) -> float:
        return self.pack_voltage_v * self.current_a

    @property
    def average_cell_voltage_v(self) -> float:
        return sum(self.cell_voltages_v) / len(self.cell_voltages_v)

    @property
    def delta_cell_voltage_v(self) -> float:
        return max(self.cell_voltages_v) - min(self.cell_voltages_v)

    @property
    def active_alarms(self) -> Tuple[str, ...]:
        return tuple(
            JK02_ALARM_NAMES[bit] for bit in range(32) if self.alarm_flags & (1 << bit)
        )

    @property
    def has_critical_alarm(self) -> bool:
        return bool(self.alarm_flags & ~NONFAULT_STATUS_MASK)


def _u16(frame: bytes, offset: int) -> int:
    return struct.unpack_from("<H", frame, offset)[0]


def _i16(frame: bytes, offset: int) -> int:
    return struct.unpack_from("<h", frame, offset)[0]


def _u32(frame: bytes, offset: int) -> int:
    return struct.unpack_from("<I", frame, offset)[0]


def _i32(frame: bytes, offset: int) -> int:
    return struct.unpack_from("<i", frame, offset)[0]


def _ascii(frame: bytes, offset: int, length: int) -> str:
    value = frame[offset : offset + length].split(b"\0", 1)[0]
    try:
        return value.decode("ascii").strip()
    except UnicodeDecodeError as exc:
        raise ProtocolError("device information contains non-ASCII text") from exc


def validate_frame(frame: bytes, expected_type: Optional[int] = None) -> bytes:
    frame = bytes(frame)
    if len(frame) != FRAME_LENGTH:
        raise ProtocolError("JK frame length must be exactly 300 bytes")
    if not frame.startswith(FRAME_HEADER):
        raise ProtocolError("JK frame header mismatch")
    if sum(frame[:-1]) & 0xFF != frame[-1]:
        raise ProtocolError("JK frame checksum mismatch")
    if expected_type is not None and frame[4] != expected_type:
        raise ProtocolError("unexpected JK frame type")
    return frame


def build_query(command: int) -> bytes:
    if command not in (0x96, 0x97):
        raise ProtocolError("only read-only telemetry queries are permitted")
    frame = bytearray(20)
    frame[:4] = b"\xaa\x55\x90\xeb"
    frame[4] = command
    frame[-1] = sum(frame[:-1]) & 0xFF
    return bytes(frame)


class FrameAssembler:
    """Reassemble fragmented GATT notifications into strict 300-byte frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: Sequence[int]) -> List[bytes]:
        data = bytes(chunk)
        if not data:
            return []
        header_at = data.find(FRAME_HEADER)
        if header_at >= 0:
            self._buffer = bytearray(data[header_at:])
        elif self._buffer:
            self._buffer.extend(data)
        else:
            return []
        if len(self._buffer) > 320:
            self._buffer.clear()
            raise ProtocolError("JK notification buffer exceeded 320 bytes")
        if len(self._buffer) < FRAME_LENGTH:
            return []
        frame = bytes(self._buffer[:FRAME_LENGTH])
        self._buffer.clear()
        return [validate_frame(frame)]


class JkBmsDecoder:
    """Decode one explicitly selected JK02 layout with physical checks."""

    def __init__(
        self,
        protocol: str,
        expected_cell_count: int,
        *,
        cell_voltage_bounds_v: Tuple[float, float] = (2.0, 4.5),
        pack_voltage_bounds_v: Tuple[float, float] = (12.0, 36.0),
        temperature_bounds_c: Tuple[float, float] = (-30.0, 100.0),
        pack_cell_sum_tolerance_v: float = 1.0,
    ) -> None:
        if protocol not in SUPPORTED_PROTOCOLS:
            raise ProtocolError("unsupported or unspecified JK BLE protocol")
        if isinstance(expected_cell_count, bool) or not 1 <= expected_cell_count <= 32:
            raise ProtocolError("expected_cell_count must be an integer in [1, 32]")
        self.protocol = protocol
        self.expected_cell_count = expected_cell_count
        self.cell_voltage_bounds_v = cell_voltage_bounds_v
        self.pack_voltage_bounds_v = pack_voltage_bounds_v
        self.temperature_bounds_c = temperature_bounds_c
        self.pack_cell_sum_tolerance_v = pack_cell_sum_tolerance_v

    @property
    def _cell_slots(self) -> int:
        return 32 if self.protocol == "jk02_32s" else 24

    @property
    def _cell_offset(self) -> int:
        return 16 if self.protocol == "jk02_32s" else 0

    @property
    def _payload_offset(self) -> int:
        return self._cell_offset * 2

    def decode_device_info(self, frame: bytes) -> DeviceInfo:
        frame = validate_frame(frame, 0x03)
        info = DeviceInfo(
            model=_ascii(frame, 6, 16),
            hardware_version=_ascii(frame, 22, 8),
            software_version=_ascii(frame, 30, 8),
            device_name=_ascii(frame, 46, 16),
            manufacturing_date=_ascii(frame, 78, 8),
            serial_number=_ascii(frame, 86, 16),
            power_on_count=_u32(frame, 42),
        )
        if not info.model or not re.fullmatch(r"[ -~]+", info.model):
            raise ProtocolError("JK device model is missing or invalid")
        return info

    def decode_telemetry(self, frame: bytes) -> Telemetry:
        frame = validate_frame(frame, 0x02)
        cell_offset = self._cell_offset
        payload_offset = self._payload_offset
        enabled_mask = _u32(frame, 54 + cell_offset)
        enabled_indices = tuple(
            index for index in range(self._cell_slots) if enabled_mask & (1 << index)
        )
        if len(enabled_indices) != self.expected_cell_count:
            raise ProtocolError("enabled JK cell count does not match configuration")
        if enabled_indices != tuple(range(self.expected_cell_count)):
            raise ProtocolError("enabled JK cells must be contiguous from cell 1")
        cells = tuple(_u16(frame, 6 + 2 * index) * 0.001 for index in enabled_indices)
        resistances = tuple(
            _u16(frame, 64 + cell_offset + 2 * index) * 0.001
            for index in enabled_indices
        )
        self._require_range("cell voltage", cells, self.cell_voltage_bounds_v)
        pack_voltage = _u32(frame, 118 + payload_offset) * 0.001
        self._require_range("pack voltage", (pack_voltage,), self.pack_voltage_bounds_v)
        if abs(sum(cells) - pack_voltage) > self.pack_cell_sum_tolerance_v:
            raise ProtocolError("pack voltage disagrees with summed cell voltages")
        temperatures = [
            _i16(frame, 130 + payload_offset) * 0.1,
            _i16(frame, 132 + payload_offset) * 0.1,
        ]
        if self.protocol == "jk02_32s":
            mos_temperature = _i16(frame, 112 + payload_offset) * 0.1
            alarm_flags = _u32(frame, 134 + payload_offset)
        else:
            mos_temperature = _i16(frame, 134 + payload_offset) * 0.1
            alarm_flags = _u16(frame, 136 + payload_offset)
        self._require_range(
            "temperature",
            (*temperatures, mos_temperature),
            self.temperature_bounds_c,
        )
        soc = frame[141 + payload_offset] / 100.0
        soh = frame[158 + payload_offset] / 100.0
        if not 0.0 <= soc <= 1.0 or not 0.0 <= soh <= 1.0:
            raise ProtocolError("JK state-of-charge or health is outside [0, 1]")
        telemetry = Telemetry(
            sequence=frame[5],
            cell_voltages_v=cells,
            cell_resistances_ohm=resistances,
            pack_voltage_v=pack_voltage,
            current_a=_i32(frame, 126 + payload_offset) * 0.001,
            temperatures_c=tuple(temperatures),
            mos_temperature_c=mos_temperature,
            balancing_current_a=_i16(frame, 138 + payload_offset) * 0.001,
            balancing_active=frame[140 + payload_offset] != 0,
            state_of_charge=soc,
            state_of_health=soh,
            remaining_capacity_ah=_u32(frame, 142 + payload_offset) * 0.001,
            nominal_capacity_ah=_u32(frame, 146 + payload_offset) * 0.001,
            cycle_count=_u32(frame, 150 + payload_offset),
            cycle_capacity_ah=_u32(frame, 154 + payload_offset) * 0.001,
            runtime_sec=_u32(frame, 162 + payload_offset),
            alarm_flags=alarm_flags,
            charge_enabled=bool(frame[166 + payload_offset]),
            discharge_enabled=bool(frame[167 + payload_offset]),
            precharge_active=bool(frame[168 + payload_offset]),
            heating_active=bool(frame[183 + payload_offset]),
        )
        numeric = (
            telemetry.current_a,
            telemetry.remaining_capacity_ah,
            telemetry.nominal_capacity_ah,
            telemetry.cycle_capacity_ah,
            telemetry.balancing_current_a,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ProtocolError("JK telemetry contains a non-finite value")
        if min(numeric[1:3]) < 0.0:
            raise ProtocolError("JK capacity cannot be negative")
        return telemetry

    @staticmethod
    def _require_range(
        name: str, values: Sequence[float], bounds: Tuple[float, float]
    ) -> None:
        low, high = bounds
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ProtocolError("invalid {} validation bounds".format(name))
        if not values or any(
            not math.isfinite(value) or value < low or value > high for value in values
        ):
            raise ProtocolError("{} outside configured bounds".format(name))
