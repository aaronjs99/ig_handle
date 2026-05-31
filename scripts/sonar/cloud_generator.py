#!/usr/bin/env python3
"""Convert supported sonar profile-point packets into a ROS PointCloud2 stream.

The UDP receiver intentionally remains raw: it publishes vendor packet bytes as
`std_msgs/UInt8MultiArray`. This node is the typed downstream adapter that turns
supported profile-point packets into `sensor_msgs/PointCloud2` for ORACLE,
bags, dashboards, and topic-contract checks.
"""

from __future__ import annotations

import os
import sys
from typing import Optional, Sequence

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, UInt8MultiArray

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIRS = (
    THIS_DIR,
    os.path.join(
        os.path.dirname(os.path.dirname(THIS_DIR)),
        "share",
        "ig_handle",
        "scripts",
        "sonar",
    ),
)
for module_dir in reversed(MODULE_DIRS):
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

from decoder import DEFAULT_MAX_RANGE_M, Point, SonarPacketDecoder
from profiles import load_sonar_profile


class SonarPointCloudGenerator:
    """Generate ROS PointCloud2 messages from decoded XYZ sonar points."""

    def __init__(self, frame_id: str) -> None:
        self.frame_id = frame_id
        self.last_points: Sequence[Point] = []
        self.last_cloud: Optional[PointCloud2] = None

    def generate(self, points: Sequence[Point]) -> PointCloud2:
        self.last_points = list(points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        self.last_cloud = pc2.create_cloud_xyz32(header, list(self.last_points))
        return self.last_cloud


class SonarCloudNode:
    def __init__(self) -> None:
        self.raw_topic = rospy.get_param("~raw_topic", "/sensors/sonar/raw")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/sensors/sonar/scan")
        self.frame_id = rospy.get_param("~frame_id", "sonar_link")
        self.profile_config = rospy.get_param("~profile_config", "")
        self.sonar_profile = rospy.get_param("~sonar_profile", "")
        self.header_bytes = int(rospy.get_param("~header_bytes", 256))
        self.min_range_m = float(rospy.get_param("~min_range_m", 0.5))
        self.max_range_m = self._max_range_m()
        self.min_points = int(rospy.get_param("~min_points", 3))
        self.endian = rospy.get_param("~endian", "little")
        self.publish_empty_on_decode_failure = bool(
            rospy.get_param("~publish_empty_on_decode_failure", False)
        )
        self.decoder = SonarPacketDecoder(
            header_bytes=self.header_bytes,
            min_range_m=self.min_range_m,
            max_range_m=self.max_range_m,
            min_points=self.min_points,
            endian=self.endian,
        )
        self.cloud_generator = SonarPointCloudGenerator(self.frame_id)

        self.publisher = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=10)
        self.subscriber = rospy.Subscriber(
            self.raw_topic, UInt8MultiArray, self._raw_cb, queue_size=20
        )
        rospy.loginfo(
            "sonar_cloud_generator raw_topic=%s cloud_topic=%s frame_id=%s endian=%s",
            self.raw_topic,
            self.cloud_topic,
            self.frame_id,
            self.decoder.endian,
        )

    def _max_range_m(self) -> float:
        explicit_max_range_m = rospy.get_param("~max_range_m", "")
        if str(explicit_max_range_m).strip():
            return float(explicit_max_range_m)
        if self.profile_config:
            profile = load_sonar_profile(self.profile_config, self.sonar_profile)
            return float(profile.range_m)
        return DEFAULT_MAX_RANGE_M

    def _raw_cb(self, msg: UInt8MultiArray) -> None:
        result = self.decoder.decode(msg.data)
        if result.points:
            self._publish(result.points)
            rospy.loginfo_throttle(
                5.0,
                "sonar_cloud_generator_decode packet_kind=%s points=%d",
                result.packet_kind,
                len(result.points),
            )
            return

        if result.reason == "profile_packet_contains_no_returns":
            rospy.loginfo_throttle(
                5.0,
                "sonar_cloud_generator_empty packet_kind=%s reason=%s",
                result.packet_kind or "(unknown)",
                result.reason,
            )
        else:
            rospy.logwarn_throttle(
                5.0,
                "sonar_cloud_generator_decode_failed packet_kind=%s reason=%s",
                result.packet_kind or "(unknown)",
                result.reason,
            )
        if self.publish_empty_on_decode_failure:
            self._publish([])

    def _publish(self, points: Sequence[Point]) -> None:
        self.publisher.publish(self.cloud_generator.generate(points))


def main() -> None:
    rospy.init_node("sonar_cloud_generator")
    SonarCloudNode()
    rospy.spin()


if __name__ == "__main__":
    main()
