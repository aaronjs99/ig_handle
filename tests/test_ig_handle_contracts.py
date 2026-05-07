from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_ig_handle_repo_keeps_real_platform_launch_and_data_collection_contracts():
    assert (REPO_ROOT / "ig_handle/launch/robots/heron.launch").exists()
    assert (REPO_ROOT / "ig_handle/launch/sensors/start_cam.launch").exists()
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
