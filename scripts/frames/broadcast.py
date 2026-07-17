#!/usr/bin/env python3
"""Publish selected Heron sensor TF edges from the shared YAML config."""

import math

import rospy
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


def make_transform(stamp, name, cfg):
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
    rospy.loginfo(
        "sensor_tf_broadcaster: %s %s -> %s",
        name,
        transform.header.frame_id,
        transform.child_frame_id,
    )
    return transform


def main():
    rospy.init_node("sensor_tf_broadcaster", anonymous=False)
    config_ns = rospy.get_param("~config_ns", "/sensor_frames")
    transforms = rospy.get_param(f"{config_ns}/transforms", {})
    allowed_names = _csv_set(rospy.get_param("~allowed_transform_names", ""))
    if not transforms:
        rospy.logfatal("sensor_tf_broadcaster: no transforms found under %s", config_ns)
        raise RuntimeError(f"missing transforms under {config_ns}")

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

    stamp = rospy.Time.now()
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    tf_msgs = [make_transform(stamp, name, cfg) for name, cfg in selected.items()]
    broadcaster.sendTransform(tf_msgs)
    rospy.spin()


if __name__ == "__main__":
    raise SystemExit(main())
