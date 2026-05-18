"""Shared helpers for live IG Handle sensor tests."""

from __future__ import annotations

import os
import socket
import subprocess
import time
from dataclasses import dataclass
from math import isfinite, sqrt
from pathlib import Path
from typing import Callable, Optional
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


F2_F3_DISABLED_REASON = (
    "Forge F2/F3 are physically disconnected on the current Heron rig; set "
    f"{DISABLED_CAMERA_ENV}=1 only after reconnecting them."
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
    disabled_by_default=True,
    disabled_reason=F2_F3_DISABLED_REASON,
)
CAMERA_F3 = SensorSpec(
    key="camera_f3",
    label="Forge F3 camera",
    host="192.168.50.103",
    topic="/sensors/camera/f3/image_raw",
    expected_type="sensor_msgs/Image",
    disabled_by_default=True,
    disabled_reason=F2_F3_DISABLED_REASON,
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


def assert_sensor_has_actual_data(
    sensor: SensorSpec, *, timeout_sec: float = 8.0
) -> None:
    require_sensor_enabled(sensor)
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
            "ig_handle_live_sensor_data_tests",
            anonymous=True,
            disable_signals=True,
        )

    deadline = time.time() + timeout_sec
    msg_class = None
    real_topic = sensor.topic
    while time.time() < deadline and msg_class is None and not rospy.is_shutdown():
        msg_class, real_topic, _ = rostopic.get_topic_class(
            sensor.topic, blocking=False
        )
        if msg_class is None:
            time.sleep(0.1)

    assert (
        msg_class is not None
    ), f"{sensor.label} topic {sensor.topic} is not registered in the ROS master"
    assert msg_class._type == sensor.expected_type, (
        f"{sensor.label} topic {sensor.topic} has type {msg_class._type}, "
        f"expected {sensor.expected_type}"
    )

    remaining = max(0.1, deadline - time.time())
    try:
        msg = rospy.wait_for_message(
            real_topic or sensor.topic, msg_class, timeout=remaining
        )
    except Exception as exc:
        pytest.fail(
            f"{sensor.label} did not publish one message on {sensor.topic} "
            f"within {timeout_sec:.1f}s: {exc}"
        )

    _validator_for(sensor)(sensor, msg)


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


def _validator_for(sensor: SensorSpec) -> Callable[[SensorSpec, object], None]:
    if sensor.expected_type == "sensor_msgs/Image":
        return _assert_image_payload
    if sensor.expected_type == "sensor_msgs/PointCloud2":
        return _assert_pointcloud_payload
    if sensor.expected_type == "std_msgs/UInt8MultiArray":
        return _assert_uint8_array_payload
    if sensor.expected_type == "sensor_msgs/Imu":
        return _assert_imu_payload
    if sensor.expected_type == "heron_msgs/Sense":
        return _assert_heron_sense_payload
    raise AssertionError(
        f"No live data validator configured for {sensor.expected_type}"
    )


def _payload_len(data) -> int:
    try:
        return len(data)
    except TypeError:
        return 0


def _assert_image_payload(sensor: SensorSpec, msg) -> None:
    payload_len = _payload_len(msg.data)
    assert msg.height > 0 and msg.width > 0, (
        f"{sensor.label} image has invalid dimensions: " f"{msg.width}x{msg.height}"
    )
    assert msg.step > 0, f"{sensor.label} image has invalid row step: {msg.step}"
    assert msg.encoding, f"{sensor.label} image encoding is empty"
    assert payload_len >= msg.height * msg.step, (
        f"{sensor.label} image payload is too small: {payload_len} bytes for "
        f"{msg.width}x{msg.height}, step={msg.step}, encoding={msg.encoding!r}"
    )


def _assert_pointcloud_payload(sensor: SensorSpec, msg) -> None:
    payload_len = _payload_len(msg.data)
    assert msg.height > 0 and msg.width > 0, (
        f"{sensor.label} cloud has invalid dimensions: " f"{msg.width}x{msg.height}"
    )
    assert (
        msg.point_step > 0
    ), f"{sensor.label} cloud has invalid point_step: {msg.point_step}"
    assert (
        msg.row_step > 0
    ), f"{sensor.label} cloud has invalid row_step: {msg.row_step}"
    assert payload_len >= msg.row_step * msg.height, (
        f"{sensor.label} cloud payload is too small: {payload_len} bytes for "
        f"{msg.width}x{msg.height}, row_step={msg.row_step}"
    )
    field_names = {field.name for field in msg.fields}
    assert {"x", "y", "z"}.issubset(
        field_names
    ), f"{sensor.label} cloud is missing XYZ fields: {sorted(field_names)}"

    try:
        from sensor_msgs import point_cloud2
    except ImportError as exc:
        pytest.fail(f"sensor_msgs.point_cloud2 is unavailable: {exc}")

    finite_points = 0
    for point in point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True
    ):
        if all(isfinite(value) for value in point):
            finite_points += 1
            if finite_points >= 10:
                break

    assert finite_points > 0, f"{sensor.label} cloud contains no finite XYZ points"


def _assert_uint8_array_payload(sensor: SensorSpec, msg) -> None:
    payload_len = _payload_len(msg.data)
    assert payload_len > 0, f"{sensor.label} byte payload is empty"
    assert any(
        int(value) != 0 for value in msg.data
    ), f"{sensor.label} byte payload is all zeros ({payload_len} bytes)"


def _assert_imu_payload(sensor: SensorSpec, msg) -> None:
    orientation = msg.orientation
    angular_velocity = msg.angular_velocity
    linear_acceleration = msg.linear_acceleration
    values = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        angular_velocity.x,
        angular_velocity.y,
        angular_velocity.z,
        linear_acceleration.x,
        linear_acceleration.y,
        linear_acceleration.z,
    )
    assert all(
        isfinite(value) for value in values
    ), f"{sensor.label} IMU message contains non-finite numeric data"

    quat_norm = sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    accel_norm = sqrt(
        linear_acceleration.x * linear_acceleration.x
        + linear_acceleration.y * linear_acceleration.y
        + linear_acceleration.z * linear_acceleration.z
    )
    assert (
        0.5 <= quat_norm <= 1.5
    ), f"{sensor.label} orientation quaternion norm is implausible: {quat_norm:.3f}"
    assert (
        1.0 <= accel_norm <= 25.0
    ), f"{sensor.label} acceleration norm is implausible: {accel_norm:.3f} m/s^2"


def _assert_heron_sense_payload(sensor: SensorSpec, msg) -> None:
    values = (msg.battery, msg.current_left, msg.current_right)
    assert all(
        isfinite(value) for value in values
    ), f"{sensor.label} /sense message contains non-finite numeric data"
    assert (
        msg.battery > 5.0
    ), f"{sensor.label} /sense battery voltage is implausible: {msg.battery:.3f} V"
