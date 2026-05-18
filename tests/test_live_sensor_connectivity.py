"""Live connectivity checks for IG Handle hardware endpoints."""

import pytest

from live_sensor_helpers import (
    CAMERA_F1,
    CAMERA_F2,
    CAMERA_F3,
    CAMERA_F4,
    IMU,
    LIDAR_H,
    LIDAR_V,
    SONAR,
    assert_sensor_connectivity,
)


pytestmark = pytest.mark.live_hardware


def test_camera_f1_connectivity():
    assert_sensor_connectivity(CAMERA_F1)


def test_camera_f2_connectivity():
    assert_sensor_connectivity(CAMERA_F2)


def test_camera_f3_connectivity():
    assert_sensor_connectivity(CAMERA_F3)


def test_camera_f4_connectivity():
    assert_sensor_connectivity(CAMERA_F4)


def test_lidar_h_connectivity():
    assert_sensor_connectivity(LIDAR_H)


def test_lidar_v_connectivity():
    assert_sensor_connectivity(LIDAR_V)


def test_sonar_connectivity():
    assert_sensor_connectivity(SONAR)


def test_imu_connectivity():
    assert_sensor_connectivity(IMU)
