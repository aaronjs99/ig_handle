"""Live connectivity checks for IG Handle hardware endpoints."""

import pytest

from ig_handle_testing.endpoint_checks import assert_endpoint_reachable


pytestmark = pytest.mark.live_hardware


@pytest.mark.parametrize(
    "sensor_key",
    [
        "camera_f1",
        "camera_f2",
        "camera_f3",
        "camera_f4",
        "lidar_h",
        "lidar_v",
        "sonar",
        "imu",
        "heron",
    ],
)
def test_sensor_endpoint_is_reachable(hardware_contract, sensor_key):
    assert_endpoint_reachable(hardware_contract.sensors[sensor_key])
