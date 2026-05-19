"""Live Heron base status and command-path checks."""

import pytest

from live_sensor_helpers import (
    assert_heron_sense_payload,
    assert_heron_status_payload,
    ros_topic_publishers,
    ros_topic_subscribers,
    wait_for_ros_message,
)


pytestmark = pytest.mark.live_hardware


def _has_node(nodes, name):
    return any(node == name or node.endswith(name) for node in nodes)


def test_heron_sense_reports_battery_health():
    msg = wait_for_ros_message("/sense", "heron_msgs/Sense")
    assert_heron_sense_payload(msg)


def test_heron_status_reports_power_health():
    msg = wait_for_ros_message("/status", "heron_msgs/Status")
    assert_heron_status_payload(msg)


def test_heron_mcu_command_topics_reach_serial_node():
    for topic in ("/cmd_drive", "/motor_enable", "/has_wifi", "/reverse_time_ms"):
        subscribers = ros_topic_subscribers(topic)
        assert _has_node(subscribers, "/serial_node"), (
            f"{topic} is not subscribed by /serial_node; subscribers="
            f"{sorted(subscribers)}"
        )


def test_heron_cmd_vel_controller_path_is_available():
    cmd_vel_subscribers = ros_topic_subscribers("/cmd_vel")
    cmd_drive_publishers = ros_topic_publishers("/cmd_drive")
    assert _has_node(cmd_vel_subscribers, "/controller"), (
        "/cmd_vel is not subscribed by /controller; subscribers="
        f"{sorted(cmd_vel_subscribers)}"
    )
    assert _has_node(cmd_drive_publishers, "/controller"), (
        "/cmd_drive is not published by /controller; publishers="
        f"{sorted(cmd_drive_publishers)}"
    )
