from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_ig_handle_repo_keeps_real_platform_launch_and_data_collection_contracts():
    assert (REPO_ROOT / "ig_handle/launch/robots/heron.launch").exists()
    assert (REPO_ROOT / "ig_handle/launch/sensors/start_cam.launch").exists()
    assert (REPO_ROOT / "ig_handle/scripts/sonar/dt100_profile_to_cloud.py").exists()
    assert (REPO_ROOT / "ig_handle/scripts/pipeline/process_raw_bag.py").exists()
    assert (REPO_ROOT / "ig_handle/scripts/teensy_rosserial_launcher.py").exists()
    assert (REPO_ROOT / "ig_handle/scripts/mocap/natnet_pose_bridge.py").exists()


def test_real_sensor_launch_keeps_dlio_safe_tf_and_camera_info_contracts():
    imu = (REPO_ROOT / "ig_handle/launch/sensors/start_imu.launch").read_text(
        encoding="utf-8"
    )
    camera = (REPO_ROOT / "ig_handle/launch/sensors/start_cam.launch").read_text(
        encoding="utf-8"
    )
    suite = (REPO_ROOT / "ig_handle/launch/robots/sensor_suite.launch").read_text(
        encoding="utf-8"
    )

    assert 'name="pub_transform" value="false"' in imu
    assert 'name="frame_id" value="imu_link"' in imu
    assert 'name="camera_info_topic"' in camera
    assert '<remap from="camera_info" to="$(arg camera_info_topic)"/>' in camera
    assert 'value="$(arg topic_cam_f1_info)"' in suite
    assert 'value="$(arg topic_cam_f2_info)"' in suite
    assert 'value="$(arg topic_cam_f3_info)"' in suite
    assert 'value="$(arg topic_cam_f4_info)"' in suite


def test_standalone_camera_defaults_match_f1_to_f4_layout():
    suite = (REPO_ROOT / "ig_handle/launch/robots/sensor_suite.launch").read_text(
        encoding="utf-8"
    )
    for relpath in (
        "ig_handle/launch/robots/heron.launch",
        "ig_handle/launch/robots/handle.launch",
        "ig_handle/launch/robots/husky.launch",
        "ig_handle/launch/robots/sensor_suite.launch",
    ):
        text = (REPO_ROOT / relpath).read_text(encoding="utf-8")
        assert "package://slam_grande/config/calibration/f1.yaml" in text
        assert "package://slam_grande/config/calibration/f2.yaml" in text
        assert (
            '<arg name="camera_f3_info_url" default="package://slam_grande/config/calibration/f3.yaml"/>'
            in text
        )
        assert (
            '<arg name="camera_f4_info_url" default="package://slam_grande/config/calibration/f4.yaml"/>'
            in text
        )
    assert '<arg name="camera_name"   value="F3"/>' in suite
    assert '<arg name="frame_id"      value="F3_optical"/>' in suite
    assert '<arg name="camera_info_topic" value="$(arg topic_cam_f3_info)"/>' in suite
    assert '<arg name="camera_name"   value="F4"/>' in suite
    assert '<arg name="frame_id"      value="F4_optical"/>' in suite
    assert '<arg name="camera_info_topic" value="$(arg topic_cam_f4_info)"/>' in suite
    for camera_arg in (
        "use_camera_f1",
        "use_camera_f2",
        "use_camera_f3",
        "use_camera_f4",
    ):
        assert f'<arg name="{camera_arg}" default="true"/>' in suite
        assert f"arg('{camera_arg}')" in suite


def test_vertical_lidar_packets_use_canonical_vert_namespace():
    suite = (REPO_ROOT / "ig_handle/launch/robots/sensor_suite.launch").read_text(
        encoding="utf-8"
    )

    assert '<arg name="name"      value="vert"/>' in suite
    assert 'value="lidar_v"' not in suite


def test_mocap_and_teensy_paths_are_explicit_and_modular():
    natnet_launch = (
        REPO_ROOT / "ig_handle/launch/core/natnet_bridge.launch"
    ).read_text(encoding="utf-8")
    rosserial_launch = (
        REPO_ROOT / "ig_handle/launch/core/start_rosserial.launch"
    ).read_text(encoding="utf-8")
    suite = (REPO_ROOT / "ig_handle/launch/robots/sensor_suite.launch").read_text(
        encoding="utf-8"
    )
    heron = (REPO_ROOT / "ig_handle/launch/robots/heron.launch").read_text(
        encoding="utf-8"
    )
    cmake = (REPO_ROOT / "ig_handle/CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (REPO_ROOT / "ig_handle/package.xml").read_text(encoding="utf-8")

    assert 'type="natnet_pose_bridge.py" name="natnet_pose_bridge"' in natnet_launch
    assert 'name="server_ip" default="192.168.1.199"' in natnet_launch
    assert 'name="client_ip" default="192.168.1.8"' in natnet_launch
    assert 'name="topic_prefix" default="/mocap"' in natnet_launch
    assert 'type="teensy_rosserial_launcher.py"' in rosserial_launch
    assert 'name="port"' in rosserial_launch
    assert 'name="baud"' in rosserial_launch
    assert '<remap from="/imu/time" to="$(arg imu_time_topic)"/>' in rosserial_launch
    assert '<arg name="use_teensy" default="false"/>' in suite
    assert '<arg name="use_teensy" default="false"/>' in heron
    assert "scripts/mocap/natnet_pose_bridge.py" in cmake
    assert "scripts/teensy_rosserial_launcher.py" in cmake
    assert "<exec_depend>geometry_msgs</exec_depend>" in package_xml
    assert "<exec_depend>tf2_ros</exec_depend>" in package_xml
    assert "<exec_depend>rosserial_python</exec_depend>" in package_xml


def test_dt100_raw_driver_is_kept_separate_from_pointcloud_adapter():
    start_sonar = (REPO_ROOT / "ig_handle/launch/sensors/start_sonar.launch").read_text(
        encoding="utf-8"
    )
    suite = (REPO_ROOT / "ig_handle/launch/robots/sensor_suite.launch").read_text(
        encoding="utf-8"
    )
    cmake = (REPO_ROOT / "ig_handle/CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (REPO_ROOT / "ig_handle/package.xml").read_text(encoding="utf-8")

    assert 'name="raw_topic"      default="/sensors/sonar/raw"' in start_sonar
    assert 'name="cloud_topic"    default="/sensors/sonar/scan"' in start_sonar
    assert 'name="min_range_m"    default="0.5"' in start_sonar
    assert 'name="max_range_m"    default="100.0"' in start_sonar
    assert 'type="dt100_rx.py" name="dt100_raw_driver"' in start_sonar
    assert (
        'type="dt100_profile_to_cloud.py" name="dt100_profile_to_cloud"' in start_sonar
    )
    assert '<param name="min_range_m" value="$(arg min_range_m)"/>' in start_sonar
    assert '<param name="max_range_m" value="$(arg max_range_m)"/>' in start_sonar
    assert '<arg name="raw_topic" value="$(arg topic_sonar_raw)"/>' in suite
    assert '<arg name="cloud_topic" value="$(arg topic_sonar)"/>' in suite
    assert '"/sensors/sonar/raw"' in (
        REPO_ROOT / "ig_handle/scripts/sonar/dt100_rx.py"
    ).read_text(encoding="utf-8")
    assert "scripts/sonar/dt100_profile_to_cloud.py" in cmake
    assert "<exec_depend>sensor_msgs</exec_depend>" in package_xml


def _load_dt100_converter_module():
    path = REPO_ROOT / "ig_handle/scripts/sonar/dt100_profile_to_cloud.py"
    spec = importlib.util.spec_from_file_location("dt100_profile_to_cloud", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_dt100_profile_converter_decodes_xyz_profile_records():
    converter = _load_dt100_converter_module()
    payload = bytearray(b"83P" + bytes(253))
    for point in ((1.0, 0.0, 0.0), (2.0, 0.5, -0.1), (3.0, -0.2, 0.1)):
        payload.extend(struct.pack("<fffH", *point, 42))

    result = converter.decode_dt100_profile_packet(payload, max_range_m=10.0)

    assert result.packet_kind == "83P"
    assert result.reason == ""
    assert len(result.points) == 3
    for actual, expected in zip(
        result.points, [(1.0, 0.0, 0.0), (2.0, 0.5, -0.1), (3.0, -0.2, 0.1)]
    ):
        assert actual == pytest.approx(expected)


def test_dt100_profile_converter_applies_physical_range_gate():
    converter = _load_dt100_converter_module()
    payload = bytearray(b"83P" + bytes(253))
    for point in (
        (0.2, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (99.0, 0.0, 0.0),
        (101.0, 0.0, 0.0),
    ):
        payload.extend(struct.pack("<fffH", *point, 42))

    result = converter.decode_dt100_profile_packet(payload)

    assert result.reason == ""
    assert result.points == pytest.approx(
        [(0.5, 0.0, 0.0), (1.0, 0.0, 0.0), (99.0, 0.0, 0.0)]
    )


def test_dt100_profile_converter_rejects_raw_beam_packets_without_fake_points():
    converter = _load_dt100_converter_module()
    payload = b"83B" + bytes(512)

    result = converter.decode_dt100_profile_packet(payload)

    assert result.points == []
    assert result.packet_kind == "83B"
    assert result.reason == "beam_or_raw_packet_not_xyz_profile"
