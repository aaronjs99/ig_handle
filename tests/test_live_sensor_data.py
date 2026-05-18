"""Live ROS data checks for IG Handle sensor topics."""

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
    assert_sensor_publishes,
)


pytestmark = pytest.mark.live_hardware


def test_camera_f1_publishes_data():
    assert_sensor_publishes(CAMERA_F1)


def test_camera_f2_publishes_data():
    assert_sensor_publishes(CAMERA_F2)


def test_camera_f3_publishes_data():
    assert_sensor_publishes(CAMERA_F3)


def test_camera_f4_publishes_data():
    assert_sensor_publishes(CAMERA_F4)


def test_lidar_h_publishes_data():
    assert_sensor_publishes(LIDAR_H)


def test_lidar_v_publishes_data():
    assert_sensor_publishes(LIDAR_V)


def test_sonar_publishes_data():
    assert_sensor_publishes(SONAR)


def test_imu_publishes_data():
    assert_sensor_publishes(IMU)


def test_heron_publishes_data():
    assert_sensor_publishes(HERON)
