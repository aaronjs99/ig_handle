#!/usr/bin/env python3
"""Publish the configured static transform between Heron LiDAR frames."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped


@dataclass(frozen=True)
class LidarPairTransform:
    parent: str
    child: str
    translation_xyz_m: tuple[float, float, float]
    rotation_quat_xyzw: tuple[float, float, float, float]

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "LidarPairTransform":
        transform = config["lidar_pair"]["active_transform"]
        translation = tuple(float(v) for v in transform["translation_xyz_m"])
        rotation = tuple(float(v) for v in transform["rotation_quat_xyzw"])
        if len(translation) != 3:
            raise ValueError("translation_xyz_m must contain exactly 3 values")
        if len(rotation) != 4:
            raise ValueError("rotation_quat_xyzw must contain exactly 4 values")
        return cls(
            parent=str(transform["parent"]),
            child=str(transform["child"]),
            translation_xyz_m=translation,
            rotation_quat_xyzw=rotation,
        )

    def to_message(self) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.parent
        msg.child_frame_id = self.child
        msg.transform.translation.x = self.translation_xyz_m[0]
        msg.transform.translation.y = self.translation_xyz_m[1]
        msg.transform.translation.z = self.translation_xyz_m[2]
        msg.transform.rotation.x = self.rotation_quat_xyzw[0]
        msg.transform.rotation.y = self.rotation_quat_xyzw[1]
        msg.transform.rotation.z = self.rotation_quat_xyzw[2]
        msg.transform.rotation.w = self.rotation_quat_xyzw[3]
        return msg


def load_lidar_pair_transform(config_file: str | Path) -> LidarPairTransform:
    path = Path(config_file).expanduser()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} does not contain a YAML mapping")
    return LidarPairTransform.from_config(data)


def main() -> int:
    rospy.init_node("lidar_pair_tf_broadcaster", anonymous=False)
    default_config = "$(find ig_handle)/config/lidar_pair_extrinsics.yaml"
    config_file = rospy.get_param("~config_file", default_config)
    if "$(" in str(config_file):
        raise RuntimeError(
            "lidar_pair_tf_broadcaster needs a resolved ~config_file path from launch"
        )

    transform = load_lidar_pair_transform(config_file)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform.to_message())
    rospy.loginfo(
        "lidar_pair_tf_broadcaster: %s -> %s from %s",
        transform.parent,
        transform.child,
        config_file,
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
