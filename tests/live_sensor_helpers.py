"""Shared helpers for live IG Handle sensor tests."""

from __future__ import annotations

import os
import socket
import subprocess
import time
from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

import pytest


DISABLED_CAMERA_ENV = "IG_HANDLE_INCLUDE_DISABLED_CAMERAS"


@dataclass(frozen=True)
class SensorSpec:
    key: str
    label: str
    topic: str
    expected_type: str
    host: Optional[str] = None
    device_path: Optional[str] = None
    disabled_by_default: bool = False
    disabled_reason: str = ""


F3_DISABLED_REASON = (
    "Forge F3 is off for the current three-camera Heron test; set "
    f"{DISABLED_CAMERA_ENV}=1 only after enabling it."
)


CAMERA_F1 = SensorSpec(
    key="camera_f1",
    label="Forge F1 camera",
    host="192.168.50.101",
    topic="/sensors/camera/f1/image_raw",
    expected_type="sensor_msgs/Image",
)
CAMERA_F2 = SensorSpec(
    key="camera_f2",
    label="Forge F2 camera",
    host="192.168.50.102",
    topic="/sensors/camera/f2/image_raw",
    expected_type="sensor_msgs/Image",
)
CAMERA_F3 = SensorSpec(
    key="camera_f3",
    label="Forge F3 camera",
    host="192.168.50.103",
    topic="/sensors/camera/f3/image_raw",
    expected_type="sensor_msgs/Image",
    disabled_by_default=True,
    disabled_reason=F3_DISABLED_REASON,
)
CAMERA_F4 = SensorSpec(
    key="camera_f4",
    label="Forge F4 camera",
    host="192.168.50.104",
    topic="/sensors/camera/f4/image_raw",
    expected_type="sensor_msgs/Image",
)
LIDAR_H = SensorSpec(
    key="lidar_h",
    label="horizontal VLP-16 LiDAR",
    host="192.168.50.201",
    topic="/sensors/lidar/hori/points",
    expected_type="sensor_msgs/PointCloud2",
)
LIDAR_V = SensorSpec(
    key="lidar_v",
    label="vertical VLP-16 LiDAR",
    host="192.168.50.202",
    topic="/sensors/lidar/vert/points",
    expected_type="sensor_msgs/PointCloud2",
)
SONAR = SensorSpec(
    key="sonar",
    label="Imagenex DT100 sonar",
    host="192.168.0.2",
    topic="/sensors/sonar/raw",
    expected_type="std_msgs/UInt8MultiArray",
)
IMU = SensorSpec(
    key="imu",
    label="Xsens IMU",
    device_path="/dev/serial/by-id/usb-Xsens_MTi-30_AHRS_0368319D-if01-port0",
    topic="/sensors/imu/data",
    expected_type="sensor_msgs/Imu",
)
HERON = SensorSpec(
    key="heron",
    label="Heron base computer",
    host="192.168.131.1",
    topic="/sense",
    expected_type="heron_msgs/Sense",
)


def env_enabled(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


def require_sensor_enabled(sensor: SensorSpec) -> None:
    if sensor.disabled_by_default and not env_enabled(DISABLED_CAMERA_ENV):
        pytest.skip(sensor.disabled_reason)


def assert_sensor_connectivity(sensor: SensorSpec, *, timeout_sec: float = 2.0) -> None:
    require_sensor_enabled(sensor)

    if sensor.host:
        command = ["ping", "-c", "1", "-W", str(max(1, int(timeout_sec))), sensor.host]
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        assert result.returncode == 0, (
            f"{sensor.label} host {sensor.host} is not reachable with ping. "
            f"stdout={result.stdout!r} stderr={result.stderr!r}"
        )
        return

    if sensor.device_path:
        path = Path(sensor.device_path)
        assert path.exists(), f"{sensor.label} device path is missing: {path}"
        return

    raise AssertionError(f"{sensor.label} has no connectivity probe configured")


def wait_for_ros_message(topic: str, expected_type: str, *, timeout_sec: float = 8.0):
    _assert_ros_master_reachable()

    try:
        import rospy
        import rostopic
    except ImportError as exc:
        pytest.fail(
            f"ROS Python packages are unavailable; source the ROS workspace: {exc}"
        )

    if not rospy.core.is_initialized():
        rospy.init_node(
            "ig_handle_live_hardware_tests",
            anonymous=True,
            disable_signals=True,
        )

    deadline = time.time() + timeout_sec
    msg_class = None
    real_topic = topic
    while time.time() < deadline and msg_class is None and not rospy.is_shutdown():
        msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)
        if msg_class is None:
            time.sleep(0.1)

    assert msg_class is not None, f"topic {topic} is not registered in the ROS master"
    assert (
        msg_class._type == expected_type
    ), f"topic {topic} has type {msg_class._type}, expected {expected_type}"

    remaining = max(0.1, deadline - time.time())
    try:
        return rospy.wait_for_message(real_topic or topic, msg_class, timeout=remaining)
    except Exception as exc:
        pytest.fail(f"{topic} did not publish within {timeout_sec:.1f}s: {exc}")


def ros_topic_publishers(topic: str):
    _assert_ros_master_reachable()
    try:
        import rosgraph
    except ImportError as exc:
        pytest.fail(f"ROS Python packages are unavailable; source the workspace: {exc}")

    state = rosgraph.Master("/ig_handle_live_hardware_tests").getSystemState()
    publishers = dict(state[0])
    return set(publishers.get(topic, []))


def ros_topic_subscribers(topic: str):
    _assert_ros_master_reachable()
    try:
        import rosgraph
    except ImportError as exc:
        pytest.fail(f"ROS Python packages are unavailable; source the workspace: {exc}")

    state = rosgraph.Master("/ig_handle_live_hardware_tests").getSystemState()
    subscribers = dict(state[1])
    return set(subscribers.get(topic, []))


def float_env(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    return float(value)


def _assert_ros_master_reachable(timeout_sec: float = 2.0) -> None:
    master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    parsed = urlparse(master_uri)
    host = parsed.hostname
    port = parsed.port or 11311
    assert host, f"ROS_MASTER_URI is not a usable URI: {master_uri!r}"
    try:
        with socket.create_connection((host, port), timeout=timeout_sec):
            return
    except OSError as exc:
        pytest.fail(
            f"ROS master {master_uri} is not reachable within "
            f"{timeout_sec:.1f}s: {exc}"
        )


def assert_heron_sense_payload(msg) -> None:
    min_battery_v = float_env("IG_HANDLE_HERON_MIN_BATTERY_V", 14.0)
    values = (msg.battery, msg.current_left, msg.current_right)
    assert all(
        isfinite(value) for value in values
    ), "Heron /sense message contains non-finite numeric data"
    assert msg.battery >= min_battery_v, (
        f"Heron battery voltage is below the lab threshold: "
        f"{msg.battery:.3f} V < {min_battery_v:.3f} V"
    )


def assert_heron_status_payload(msg) -> None:
    values = (
        msg.pcb_temperature,
        msg.user_current,
        msg.user_power_consumed,
        msg.motor_power_consumed,
        msg.total_power_consumed,
    )
    assert all(
        isfinite(value) for value in values
    ), "Heron /status message contains non-finite numeric data"
    assert (
        -20.0 <= msg.pcb_temperature <= 90.0
    ), f"Heron PCB temperature is implausible: {msg.pcb_temperature:.3f} C"
    assert (
        msg.user_current >= -0.1
    ), f"Heron user current is implausibly negative: {msg.user_current:.3f} A"
    assert msg.total_power_consumed >= 0.0, (
        "Heron total_power_consumed should be nonnegative: "
        f"{msg.total_power_consumed:.3f} Wh"
    )
