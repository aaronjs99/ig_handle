"""Live Heron base status and command-path checks."""

import pytest

from ig_handle_testing.heron_checks import (
    assert_heron_sense_is_healthy,
    assert_heron_status_is_healthy,
)
from ig_handle_testing.ros_checks import (
    assert_node_publishes,
    assert_node_subscribes,
    wait_for_topic_message,
)


pytestmark = pytest.mark.live_hardware


def test_heron_sense_reports_battery_health():
    msg = wait_for_topic_message("/sense", "heron_msgs/Sense")
    assert_heron_sense_is_healthy(msg)


def test_heron_status_reports_power_health():
    msg = wait_for_topic_message("/status", "heron_msgs/Status")
    assert_heron_status_is_healthy(msg)


@pytest.mark.parametrize(
    ("topic", "required_subscriber"),
    [
        ("/cmd_drive", "/serial_node"),
        ("/motor_enable", "/serial_node"),
        ("/has_wifi", "/serial_node"),
        ("/reverse_time_ms", "/serial_node"),
        ("/cmd_vel", "/controller"),
    ],
)
def test_required_command_subscribers_exist(topic, required_subscriber):
    assert_node_subscribes(topic, required_subscriber)


@pytest.mark.parametrize(
    ("topic", "required_publisher"),
    [
        ("/cmd_drive", "/controller"),
    ],
)
def test_required_command_publishers_exist(topic, required_publisher):
    assert_node_publishes(topic, required_publisher)
