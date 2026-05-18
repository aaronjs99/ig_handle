"""Shared helpers for live IG Handle sensor tests."""

from __future__ import annotations

import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import pytest


DISABLED_CAMERA_ENV = "IG_HANDLE_INCLUDE_DISABLED_CAMERAS"


@dataclass(frozen=True)
class SensorSpec:
    key: str
    label: str
    topic: str
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
)
CAMERA_F2 = SensorSpec(
    key="camera_f2",
    label="Forge F2 camera",
    host="192.168.50.102",
    topic="/sensors/camera/f2/image_raw",
    disabled_by_default=True,
    disabled_reason=F2_F3_DISABLED_REASON,
)
CAMERA_F3 = SensorSpec(
    key="camera_f3",
    label="Forge F3 camera",
    host="192.168.50.103",
    topic="/sensors/camera/f3/image_raw",
    disabled_by_default=True,
    disabled_reason=F2_F3_DISABLED_REASON,
)
CAMERA_F4 = SensorSpec(
    key="camera_f4",
    label="Forge F4 camera",
    host="192.168.50.104",
    topic="/sensors/camera/f4/image_raw",
)
LIDAR_H = SensorSpec(
    key="lidar_h",
    label="horizontal VLP-16 LiDAR",
    host="192.168.50.201",
    topic="/sensors/lidar/hori/points",
)
LIDAR_V = SensorSpec(
    key="lidar_v",
    label="vertical VLP-16 LiDAR",
    host="192.168.50.202",
    topic="/sensors/lidar/vert/points",
)
SONAR = SensorSpec(
    key="sonar",
    label="Imagenex DT100 sonar",
    host="192.168.0.2",
    topic="/sensors/sonar/raw",
)
IMU = SensorSpec(
    key="imu",
    label="Xsens IMU",
    device_path="/dev/serial/by-id/usb-Xsens_MTi-30_AHRS_0368319D-if01-port0",
    topic="/sensors/imu/data",
)
HERON = SensorSpec(
    key="heron",
    label="Heron base computer",
    host="192.168.131.1",
    topic="/sense",
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


def assert_sensor_publishes(sensor: SensorSpec, *, timeout_sec: float = 8.0) -> None:
    require_sensor_enabled(sensor)

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

    remaining = max(0.1, deadline - time.time())
    try:
        rospy.wait_for_message(real_topic or sensor.topic, msg_class, timeout=remaining)
    except Exception as exc:
        pytest.fail(
            f"{sensor.label} did not publish one message on {sensor.topic} "
            f"within {timeout_sec:.1f}s: {exc}"
        )
