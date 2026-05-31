#!/usr/bin/env python3
"""Pure Imagenex sonar packet decoding helpers.

This module intentionally has no ROS dependency. The receiver can keep raw
vendor bytes, tests can exercise packet parsing directly, and the ROS cloud
adapter can stay focused on message conversion and publication.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple, Union


Point = Tuple[float, float, float]
SourceAddress = Tuple[str, int]

PROFILE_PACKET_KINDS = frozenset({"83A", "83P"})
RAW_PACKET_KINDS = frozenset({"83B", "837"})
XYZ_RECORD_SIZE = 14
DEFAULT_HEADER_BYTES = 256
DEFAULT_MIN_RANGE_M = 0.5
DEFAULT_MAX_RANGE_M = 100.0
DEFAULT_MIN_POINTS = 3
DEFAULT_ENDIAN = "little"

_ENDIAN_ALIASES = {
    "<": "<",
    "little": "<",
    "little-endian": "<",
    "little_endian": "<",
    "le": "<",
    ">": ">",
    "big": ">",
    "big-endian": ">",
    "big_endian": ">",
    "be": ">",
}


@dataclass(frozen=True)
class RawSonarPacket:
    """Raw vendor datagram plus optional UDP source metadata."""

    data: bytes
    source: Optional[SourceAddress] = None

    @classmethod
    def from_payload(
        cls,
        payload: Union["RawSonarPacket", bytes, bytearray, Iterable[int]],
    ) -> "RawSonarPacket":
        if isinstance(payload, RawSonarPacket):
            return payload
        if isinstance(payload, (bytes, bytearray)):
            return cls(bytes(payload))
        return cls(bytes(int(value) & 0xFF for value in payload))

    @property
    def packet_kind(self) -> str:
        if len(self.data) < 3:
            return ""
        return self.data[:3].decode("ascii", errors="replace")


@dataclass(frozen=True)
class DecodedSonarProfile:
    """Result of attempting to decode a profile packet into XYZ points."""

    points: List[Point]
    packet_kind: str
    reason: str = ""

    @property
    def ok(self) -> bool:
        return bool(self.points)


def normalize_endian(endian: str) -> str:
    key = str(endian).strip().lower()
    try:
        return _ENDIAN_ALIASES[key]
    except KeyError as exc:
        raise ValueError(
            "unsupported sonar profile endian %r; expected little or big" % endian
        ) from exc


class DT100ProfileDecoder:
    """Configured decoder for Imagenex profile-point packets."""

    def __init__(
        self,
        *,
        header_bytes: int = DEFAULT_HEADER_BYTES,
        min_range_m: float = DEFAULT_MIN_RANGE_M,
        max_range_m: float = DEFAULT_MAX_RANGE_M,
        min_points: int = DEFAULT_MIN_POINTS,
        endian: str = DEFAULT_ENDIAN,
    ) -> None:
        self.header_bytes = int(header_bytes)
        self.min_range_m = float(min_range_m)
        self.max_range_m = float(max_range_m)
        self.min_points = int(min_points)
        self.endian = normalize_endian(endian)
        self.last_packet: Optional[RawSonarPacket] = None
        self.last_result: Optional[DecodedSonarProfile] = None

    def decode(
        self, payload: Union[RawSonarPacket, bytes, bytearray, Iterable[int]]
    ) -> DecodedSonarProfile:
        packet = RawSonarPacket.from_payload(payload)
        self.last_packet = packet
        result = self._decode_packet(packet)
        self.last_result = result
        return result

    def _decode_packet(self, packet: RawSonarPacket) -> DecodedSonarProfile:
        data = packet.data
        packet_kind = packet.packet_kind
        if self.header_bytes < 0:
            return DecodedSonarProfile([], packet_kind, "invalid_header_bytes")
        if self.min_range_m < 0.0 or self.max_range_m < self.min_range_m:
            return DecodedSonarProfile([], packet_kind, "invalid_range_gate")
        if len(data) < max(3, self.header_bytes):
            return DecodedSonarProfile([], packet_kind, "packet_too_short")

        if packet_kind not in PROFILE_PACKET_KINDS:
            if packet_kind in RAW_PACKET_KINDS:
                return DecodedSonarProfile(
                    [], packet_kind, "beam_or_raw_packet_not_xyz_profile"
                )
            return DecodedSonarProfile([], packet_kind, "unsupported_packet_kind")

        profile_payload = data[self.header_bytes :]
        if profile_payload and not any(profile_payload):
            return DecodedSonarProfile(
                [], packet_kind, "profile_packet_contains_no_returns"
            )

        points = self._decode_xyz_records(data)
        if len(points) < self.min_points:
            return DecodedSonarProfile(
                [],
                packet_kind,
                "not_enough_plausible_xyz_records offset=%d endian=%s"
                % (self.header_bytes, self.endian),
            )
        return DecodedSonarProfile(points, packet_kind)

    def _decode_xyz_records(self, data: bytes) -> List[Point]:
        offset = self.header_bytes
        if offset < 0 or len(data) <= offset:
            return []

        record = struct.Struct("%sfffH" % self.endian)
        usable_bytes = ((len(data) - offset) // XYZ_RECORD_SIZE) * XYZ_RECORD_SIZE
        if usable_bytes <= 0:
            return []

        min_range_sq = self.min_range_m * self.min_range_m
        max_range_sq = self.max_range_m * self.max_range_m
        chunk = data[offset : offset + usable_bytes]
        points: List[Point] = []
        for x, y, z, _intensity in record.iter_unpack(chunk):
            point = (float(x), float(y), float(z))
            if _plausible_xyz(point, min_range_sq, max_range_sq):
                points.append(point)
        return points


def _plausible_xyz(
    point: Sequence[float], min_range_sq: float, max_range_sq: float
) -> bool:
    x, y, z = (float(point[0]), float(point[1]), float(point[2]))
    if not all(math.isfinite(value) for value in (x, y, z)):
        return False
    range_sq = x * x + y * y + z * z
    return min_range_sq <= range_sq <= max_range_sq


def decode_profile_packet(
    payload: Union[RawSonarPacket, bytes, bytearray, Iterable[int]],
    *,
    header_bytes: int = DEFAULT_HEADER_BYTES,
    min_range_m: float = DEFAULT_MIN_RANGE_M,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
    min_points: int = DEFAULT_MIN_POINTS,
    endian: str = DEFAULT_ENDIAN,
) -> DecodedSonarProfile:
    return DT100ProfileDecoder(
        header_bytes=header_bytes,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
        min_points=min_points,
        endian=endian,
    ).decode(payload)
