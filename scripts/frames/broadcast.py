#!/usr/bin/env python3
"""Publish selected Heron sensor TF edges from the shared YAML config."""

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _csv_set(value):
    """Return normalized comma-separated transform names."""
    return {
        item.strip()
        for item in str(value or "").replace(";", ",").split(",")
        if item.strip()
    }


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_transform(stamp, name, cfg, logger):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = str(cfg["parent"])
    transform.child_frame_id = str(cfg["child"])

    tx, ty, tz = cfg.get("translation", [0.0, 0.0, 0.0])
    transform.transform.translation.x = float(tx)
    transform.transform.translation.y = float(ty)
    transform.transform.translation.z = float(tz)

    if "rotation_quat" in cfg:
        qx, qy, qz, qw = cfg["rotation_quat"]
    elif "rotation_quat_xyzw" in cfg:
        qx, qy, qz, qw = cfg["rotation_quat_xyzw"]
    else:
        roll, pitch, yaw = cfg.get("rotation_rpy", [0.0, 0.0, 0.0])
        qx, qy, qz, qw = quaternion_from_euler(float(roll), float(pitch), float(yaw))

    transform.transform.rotation.x = float(qx)
    transform.transform.rotation.y = float(qy)
    transform.transform.rotation.z = float(qz)
    transform.transform.rotation.w = float(qw)
    logger.info(
        "sensor_tf_broadcaster: %s %s -> %s"
        % (
            name,
            transform.header.frame_id,
            transform.child_frame_id,
        )
    )
    return transform


class SensorTfBroadcaster(Node):
    def __init__(self):
        super().__init__("sensor_tf_broadcaster")
        default_file = (
            Path(get_package_share_directory("ig_handle"))
            / "config"
            / "sensors"
            / "platform"
            / "sensor_frames.yaml"
        )
        self.declare_parameter("sensor_frames_file", str(default_file))
        self.declare_parameter("allowed_transform_names", "")
        config_path = Path(self.get_parameter("sensor_frames_file").value)
        with config_path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        section = document.get("sensor_frames", {})
        transforms = section.get("transforms", {}) if isinstance(section, dict) else {}
        allowed_names = _csv_set(
            self.get_parameter("allowed_transform_names").value
        )
        if not transforms:
            raise RuntimeError(
                "sensor_tf_broadcaster: no transforms found in %s" % config_path
            )

        selected = {
            name: cfg
            for name, cfg in transforms.items()
            if not allowed_names or name in allowed_names
        }
        unknown = allowed_names - set(transforms)
        if unknown:
            raise RuntimeError(
                "sensor_tf_broadcaster: configured transform names are unknown: {}".format(
                    sorted(unknown)
                )
            )
        if not selected:
            raise RuntimeError("sensor_tf_broadcaster: transform selection is empty")

        stamp = self.get_clock().now().to_msg()
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        tf_msgs = [
            make_transform(stamp, name, cfg, self.get_logger())
            for name, cfg in selected.items()
        ]
        self.broadcaster.sendTransform(tf_msgs)


def main(args=None):
    rclpy.init(args=args)
    node = SensorTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
