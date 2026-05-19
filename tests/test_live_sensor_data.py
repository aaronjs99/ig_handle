"""Live hardware data checks for IG Handle sensors.

The non-Heron sensors are probed at their hardware data boundary instead of
through ROS topics: Forge cameras through Spinnaker, LiDAR/sonar through UDP
datagrams, and the IMU through direct serial bytes. Heron base telemetry remains
checked at /sense because that is the Heron ROS boundary.
"""

import pytest

from live_sensor_helpers import (
    CAMERA_F1,
    CAMERA_F2,
    CAMERA_F3,
    CAMERA_F4,
    HERON,
    IMU,
    LIDAR_H,
    LIDAR_V,
    SONAR,
    assert_sensor_has_actual_data,
)


pytestmark = pytest.mark.live_hardware


def test_camera_f1_publishes_data():
    assert_sensor_has_actual_data(CAMERA_F1)


def test_camera_f2_publishes_data():
    assert_sensor_has_actual_data(CAMERA_F2)


def test_camera_f3_publishes_data():
    assert_sensor_has_actual_data(CAMERA_F3)


def test_camera_f4_publishes_data():
    assert_sensor_has_actual_data(CAMERA_F4)


def test_lidar_h_publishes_data():
    assert_sensor_has_actual_data(LIDAR_H)


def test_lidar_v_publishes_data():
    assert_sensor_has_actual_data(LIDAR_V)


def test_sonar_publishes_data():
    assert_sensor_has_actual_data(SONAR)


def test_imu_publishes_data():
    assert_sensor_has_actual_data(IMU)


def test_heron_publishes_data():
    assert_sensor_has_actual_data(HERON)
