"""Shared helpers for live IG Handle sensor tests."""

from __future__ import annotations

import os
import shutil
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
    data_probe: str = ""
    udp_port: Optional[int] = None
    udp_min_bytes: int = 1
    udp_prefix: bytes = b""
    serial_baud: int = 115200
    serial_min_bytes: int = 16
    camera_serial: Optional[str] = None
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
    data_probe="spinnaker",
    camera_serial="25110056",
)
CAMERA_F2 = SensorSpec(
    key="camera_f2",
    label="Forge F2 camera",
    host="192.168.50.102",
    topic="/sensors/camera/f2/image_raw",
    expected_type="sensor_msgs/Image",
    data_probe="spinnaker",
    camera_serial="25152257",
    disabled_by_default=True,
    disabled_reason=F2_F3_DISABLED_REASON,
)
CAMERA_F3 = SensorSpec(
    key="camera_f3",
    label="Forge F3 camera",
    host="192.168.50.103",
    topic="/sensors/camera/f3/image_raw",
    expected_type="sensor_msgs/Image",
    data_probe="spinnaker",
    camera_serial="25110061",
    disabled_by_default=True,
    disabled_reason=F2_F3_DISABLED_REASON,
)
CAMERA_F4 = SensorSpec(
    key="camera_f4",
    label="Forge F4 camera",
    host="192.168.50.104",
    topic="/sensors/camera/f4/image_raw",
    expected_type="sensor_msgs/Image",
    data_probe="spinnaker",
    camera_serial="25110065",
)
LIDAR_H = SensorSpec(
    key="lidar_h",
    label="horizontal VLP-16 LiDAR",
    host="192.168.50.201",
    topic="/sensors/lidar/hori/points",
    expected_type="sensor_msgs/PointCloud2",
    data_probe="udp",
    udp_port=2368,
    udp_min_bytes=1206,
)
LIDAR_V = SensorSpec(
    key="lidar_v",
    label="vertical VLP-16 LiDAR",
    host="192.168.50.202",
    topic="/sensors/lidar/vert/points",
    expected_type="sensor_msgs/PointCloud2",
    data_probe="udp",
    udp_port=2369,
    udp_min_bytes=1206,
)
SONAR = SensorSpec(
    key="sonar",
    label="Imagenex DT100 sonar",
    host="192.168.0.2",
    topic="/sensors/sonar/raw",
    expected_type="std_msgs/UInt8MultiArray",
    data_probe="udp",
    udp_port=4040,
    udp_min_bytes=8,
    udp_prefix=b"83P",
)
IMU = SensorSpec(
    key="imu",
    label="Xsens IMU",
    device_path="/dev/serial/by-id/usb-Xsens_MTi-30_AHRS_0368319D-if01-port0",
    topic="/sensors/imu/data",
    expected_type="sensor_msgs/Imu",
    data_probe="serial",
    serial_baud=115200,
    serial_min_bytes=32,
)
HERON = SensorSpec(
    key="heron",
    label="Heron base computer",
    host="192.168.131.1",
    topic="/sense",
    expected_type="heron_msgs/Sense",
    data_probe="ros",
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

    if sensor.data_probe == "udp":
        _assert_udp_datagram(sensor, timeout_sec=timeout_sec)
        return
    if sensor.data_probe == "serial":
        _assert_serial_bytes(sensor, timeout_sec=timeout_sec)
        return
    if sensor.data_probe == "spinnaker":
        _assert_spinnaker_frame(sensor, timeout_sec=timeout_sec)
        return
    if sensor.data_probe == "ros":
        _assert_ros_topic_data(sensor, timeout_sec=timeout_sec)
        return

    raise AssertionError(f"{sensor.label} has no data probe configured")


def _assert_udp_datagram(sensor: SensorSpec, *, timeout_sec: float) -> None:
    assert sensor.udp_port is not None, f"{sensor.label} has no UDP port configured"
    deadline = time.monotonic() + timeout_sec
    seen = []

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except OSError:
                pass
        sock.bind(("", sensor.udp_port))
        sock.settimeout(0.25)

        while time.monotonic() < deadline:
            try:
                payload, address = sock.recvfrom(65535)
            except socket.timeout:
                continue

            source_ip = address[0]
            seen.append((source_ip, len(payload), payload[:8].hex()))
            if sensor.host and source_ip != sensor.host:
                continue
            if len(payload) < sensor.udp_min_bytes:
                continue
            if sensor.udp_prefix and not payload.startswith(sensor.udp_prefix):
                continue
            if not any(byte != 0 for byte in payload):
                continue
            return
    except OSError as exc:
        pytest.fail(
            f"{sensor.label} could not listen for direct UDP packets on "
            f"0.0.0.0:{sensor.udp_port}: {exc}"
        )
    finally:
        sock.close()

    pytest.fail(
        f"{sensor.label} did not produce a valid direct UDP packet on "
        f"0.0.0.0:{sensor.udp_port} within {timeout_sec:.1f}s. "
        f"Expected source={sensor.host!r}, min_bytes={sensor.udp_min_bytes}, "
        f"prefix={sensor.udp_prefix!r}. Seen packets={seen[:8]!r}"
    )


def _assert_serial_bytes(sensor: SensorSpec, *, timeout_sec: float) -> None:
    assert sensor.device_path, f"{sensor.label} has no serial device configured"
    path = Path(sensor.device_path)
    assert path.exists(), f"{sensor.label} device path is missing: {path}"

    try:
        import serial
    except ImportError as exc:
        pytest.fail(f"pyserial is required for {sensor.label} direct data tests: {exc}")

    deadline = time.monotonic() + timeout_sec
    collected = bytearray()
    try:
        with serial.Serial(
            str(path),
            baudrate=sensor.serial_baud,
            timeout=0.2,
        ) as port:
            while time.monotonic() < deadline:
                chunk = port.read(256)
                if chunk:
                    collected.extend(chunk)
                if len(collected) >= sensor.serial_min_bytes and any(collected):
                    return
    except serial.SerialException as exc:
        pytest.fail(
            f"{sensor.label} could not read direct serial data from {path} at "
            f"{sensor.serial_baud} baud: {exc}"
        )

    pytest.fail(
        f"{sensor.label} did not produce at least {sensor.serial_min_bytes} "
        f"nonzero serial bytes from {path} within {timeout_sec:.1f}s; "
        f"received={len(collected)} bytes"
    )


def _assert_spinnaker_frame(sensor: SensorSpec, *, timeout_sec: float) -> None:
    assert sensor.camera_serial, f"{sensor.label} has no camera serial configured"
    binary = _ensure_spinnaker_probe_binary()
    command = [
        str(binary),
        "--serial",
        sensor.camera_serial,
        "--timeout-ms",
        str(max(1000, int(timeout_sec * 1000))),
    ]
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    assert result.returncode == 0, (
        f"{sensor.label} did not return a direct Spinnaker image frame. "
        f"command={command!r} stdout={result.stdout!r} stderr={result.stderr!r}"
    )
    assert "frame_ok" in result.stdout, (
        f"{sensor.label} Spinnaker probe exited without frame_ok marker. "
        f"stdout={result.stdout!r} stderr={result.stderr!r}"
    )


def _ensure_spinnaker_probe_binary() -> Path:
    source = Path(__file__).with_name("spinnaker_frame_probe.c")
    build_dir = Path(__file__).with_name("build")
    binary = build_dir / "spinnaker_frame_probe"

    if binary.exists() and binary.stat().st_mtime >= source.stat().st_mtime:
        return binary

    compiler = shutil.which("gcc")
    assert compiler, "gcc is required to build the direct Spinnaker camera probe"
    build_dir.mkdir(exist_ok=True)
    command = [
        compiler,
        str(source),
        "-I/opt/spinnaker/include/spinc",
        "-I/opt/spinnaker/include",
        "-L/opt/spinnaker/lib",
        "-Wl,-rpath,/opt/spinnaker/lib",
        "-lSpinnaker_C",
        "-lSpinnaker",
        "-o",
        str(binary),
    ]
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    assert result.returncode == 0, (
        "failed to build direct Spinnaker camera probe. "
        f"command={command!r} stdout={result.stdout!r} stderr={result.stderr!r}"
    )
    return binary


def _assert_ros_topic_data(sensor: SensorSpec, *, timeout_sec: float) -> None:
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

    _assert_heron_sense_payload(sensor, msg)


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


def _assert_heron_sense_payload(sensor: SensorSpec, msg) -> None:
    values = (msg.battery, msg.current_left, msg.current_right)
    assert all(
        isfinite(value) for value in values
    ), f"{sensor.label} /sense message contains non-finite numeric data"
    assert (
        msg.battery > 5.0
    ), f"{sensor.label} /sense battery voltage is implausible: {msg.battery:.3f} V"
