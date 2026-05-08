#!/usr/bin/env python3
"""Convert DT100 profile-point packets into a ROS PointCloud2 stream.

The DT100 UDP receiver intentionally remains raw: it publishes the vendor packet
bytes as `std_msgs/UInt8MultiArray`. This node is the typed downstream adapter
that turns supported profile-point packets into `sensor_msgs/PointCloud2` for
ORACLE, bags, dashboards, and topic-contract checks.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, UInt8MultiArray


Point = Tuple[float, float, float]


@dataclass(frozen=True)
class DecodeResult:
    points: List[Point]
    packet_kind: str
    reason: str = ""


def _plausible_point(point: Sequence[float], max_range_m: float) -> bool:
    if not all(math.isfinite(float(value)) for value in point):
        return False
    x, y, z = (float(point[0]), float(point[1]), float(point[2]))
    radius = math.sqrt(x * x + y * y + z * z)
    return 1e-6 <= radius <= max_range_m


def _decode_xyz_records(
    data: bytes,
    *,
    offset: int,
    endian: str,
    max_range_m: float,
) -> List[Point]:
    points: List[Point] = []
    record_size = 14  # 3 float32 coordinates + uint16 intensity.
    if offset < 0 or len(data) <= offset:
        return points
    usable_bytes = len(data) - offset
    record_count = usable_bytes // record_size
    if record_count <= 0:
        return points
    fmt = f"{endian}fffH"
    for idx in range(record_count):
        start = offset + idx * record_size
        try:
            x, y, z, _intensity = struct.unpack_from(fmt, data, start)
        except struct.error:
            break
        point = (float(x), float(y), float(z))
        if _plausible_point(point, max_range_m):
            points.append(point)
    return points


def decode_dt100_profile_packet(
    payload: Iterable[int],
    *,
    header_bytes: int = 256,
    max_range_m: float = 120.0,
    min_points: int = 3,
) -> DecodeResult:
    """Decode supported Imagenex profile-point packets.

    The converter only publishes geometry when the payload looks like a profile
    point stream containing XYZ records. Raw beam/intensity packets are kept as
    raw data and rejected here instead of being converted into invented points.
    """

    data = bytes(int(value) & 0xFF for value in payload)
    packet_kind = data[:3].decode("ascii", errors="replace") if len(data) >= 3 else ""
    if len(data) < max(3, header_bytes):
        return DecodeResult([], packet_kind, "packet_too_short")

    if packet_kind not in {"83A", "83P"}:
        if packet_kind in {"83B", "837"}:
            return DecodeResult([], packet_kind, "beam_or_raw_packet_not_xyz_profile")
        return DecodeResult([], packet_kind, "unsupported_packet_kind")

    candidates: List[Tuple[int, str, List[Point]]] = []
    for offset in (header_bytes, 0):
        for endian in ("<", ">"):
            points = _decode_xyz_records(
                data,
                offset=offset,
                endian=endian,
                max_range_m=max_range_m,
            )
            candidates.append((offset, endian, points))
    offset, endian, points = max(candidates, key=lambda item: len(item[2]))
    if len(points) < min_points:
        return DecodeResult(
            [],
            packet_kind,
            f"not_enough_plausible_xyz_records offset={offset} endian={endian}",
        )
    return DecodeResult(points, packet_kind)


class DT100ProfileToCloud:
    def __init__(self) -> None:
        self.raw_topic = rospy.get_param("~raw_topic", "/sensors/sonar/raw")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/sensors/sonar/scan")
        self.frame_id = rospy.get_param("~frame_id", "sonar_link")
        self.header_bytes = int(rospy.get_param("~header_bytes", 256))
        self.max_range_m = float(rospy.get_param("~max_range_m", 120.0))
        self.min_points = int(rospy.get_param("~min_points", 3))
        self.publish_empty_on_decode_failure = bool(
            rospy.get_param("~publish_empty_on_decode_failure", False)
        )

        self.publisher = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=10)
        self.subscriber = rospy.Subscriber(
            self.raw_topic, UInt8MultiArray, self._raw_cb, queue_size=20
        )
        rospy.loginfo(
            "dt100_profile_to_cloud raw_topic=%s cloud_topic=%s frame_id=%s",
            self.raw_topic,
            self.cloud_topic,
            self.frame_id,
        )

    def _raw_cb(self, msg: UInt8MultiArray) -> None:
        result = decode_dt100_profile_packet(
            msg.data,
            header_bytes=self.header_bytes,
            max_range_m=self.max_range_m,
            min_points=self.min_points,
        )
        if result.points:
            self._publish(result.points)
            rospy.loginfo_throttle(
                5.0,
                "dt100_profile_to_cloud_decode packet_kind=%s points=%d",
                result.packet_kind,
                len(result.points),
            )
            return

        rospy.logwarn_throttle(
            5.0,
            "dt100_profile_to_cloud_decode_failed packet_kind=%s reason=%s",
            result.packet_kind or "(unknown)",
            result.reason,
        )
        if self.publish_empty_on_decode_failure:
            self._publish([])

    def _publish(self, points: Sequence[Point]) -> None:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        self.publisher.publish(pc2.create_cloud_xyz32(header, list(points)))


def main() -> None:
    rospy.init_node("dt100_profile_to_cloud")
    DT100ProfileToCloud()
    rospy.spin()


if __name__ == "__main__":
    main()
