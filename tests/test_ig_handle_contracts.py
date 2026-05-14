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
    assert (REPO_ROOT / "ig_handle/scripts/teensy_launcher.py").exists()


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


def test_vertical_lidar_packets_use_canonical_vert_namespace():
    suite = (REPO_ROOT / "ig_handle/launch/robots/sensor_suite.launch").read_text(
        encoding="utf-8"
    )

    assert '<arg name="name"      value="vert"/>' in suite
    assert 'value="lidar_v"' not in suite


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
