"""Live Heron base status and command-path checks."""

from __future__ import annotations

from math import isfinite
import os
from pathlib import Path
import socket
import sys
import time
from urllib.parse import urlparse

import pytest

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from network_config import network_value


LIVE_HARDWARE_ENV = "IG_HANDLE_RUN_LIVE_HARDWARE_TESTS"


def _env_enabled(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


pytestmark = [
    pytest.mark.live_hardware,
    pytest.mark.skipif(
        not _env_enabled(LIVE_HARDWARE_ENV),
        reason=f"set {LIVE_HARDWARE_ENV}=1 to run live Heron base tests",
    ),
]


def _wait_for_topic_message(topic: str, expected_type: str, timeout_sec: float = 8.0):
    _assert_ros_master_reachable()
    try:
        import rospy
        import rostopic
    except ImportError as exc:
        pytest.fail(f"ROS Python packages are unavailable; source the workspace: {exc}")

    if not rospy.core.is_initialized():
        rospy.init_node(
            "ig_handle_live_hardware_tests",
            anonymous=True,
            disable_signals=True,
        )

    deadline = time.time() + timeout_sec
    msg_class = None
    real_topic = topic
    while time.time() < deadline and msg_class is None and not rospy.is_shutdown():
        msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=False)
        if msg_class is None:
            time.sleep(0.1)

    assert msg_class is not None, f"topic {topic} is not registered in the ROS master"
    assert (
        msg_class._type == expected_type
    ), f"topic {topic} has type {msg_class._type}, expected {expected_type}"
    try:
        return rospy.wait_for_message(
            real_topic or topic, msg_class, timeout=max(0.1, deadline - time.time())
        )
    except Exception as exc:
        pytest.fail(f"{topic} did not publish within {timeout_sec:.1f}s: {exc}")


def _assert_node_relation(
    topic: str, node_name: str, *, state_index: int, role: str
) -> None:
    nodes = _topic_nodes(topic, state_index=state_index)
    assert any(
        node == node_name or node.endswith(node_name) for node in nodes
    ), f"{topic} is not {role} by {node_name}; {role}s={sorted(nodes)}"


def _topic_nodes(topic: str, *, state_index: int) -> set[str]:
    _assert_ros_master_reachable()
    try:
        import rosgraph
    except ImportError as exc:
        pytest.fail(f"ROS Python packages are unavailable; source the workspace: {exc}")
    state = rosgraph.Master("/ig_handle_live_hardware_tests").getSystemState()
    return set(dict(state[state_index]).get(topic, []))


def _assert_ros_master_reachable(timeout_sec: float = 2.0) -> None:
    master_uri = os.environ.get("ROS_MASTER_URI", network_value("local_master_uri"))
    parsed = urlparse(master_uri)
    host = parsed.hostname
    port = parsed.port or 11311
    assert host, f"ROS_MASTER_URI is not a usable URI: {master_uri!r}"
    try:
        with socket.create_connection((host, port), timeout=timeout_sec):
            return
    except OSError as exc:
        pytest.fail(
            f"ROS master {master_uri} is not reachable within {timeout_sec:.1f}s: {exc}"
        )


def test_heron_sense_reports_battery_health():
    msg = _wait_for_topic_message("/sense", "heron_msgs/Sense")
    assert all(
        isfinite(value) for value in (msg.battery, msg.current_left, msg.current_right)
    ), "Heron /sense message contains non-finite numeric data"
    min_battery_v = float(os.environ.get("IG_HANDLE_HERON_MIN_BATTERY_V", "14.0"))
    assert msg.battery >= min_battery_v, (
        f"Heron battery voltage is below the lab threshold: "
        f"{msg.battery:.3f} V < {min_battery_v:.3f} V"
    )


def test_heron_status_reports_power_health():
    msg = _wait_for_topic_message("/status", "heron_msgs/Status")
    values = (
        msg.pcb_temperature,
        msg.user_current,
        msg.user_power_consumed,
        msg.motor_power_consumed,
        msg.total_power_consumed,
    )
    assert all(
        isfinite(value) for value in values
    ), "Heron /status message contains non-finite numeric data"
    assert (
        -20.0 <= msg.pcb_temperature <= 90.0
    ), f"Heron PCB temperature is implausible: {msg.pcb_temperature:.3f} C"
    assert (
        msg.user_current >= -0.1
    ), f"Heron user current is implausibly negative: {msg.user_current:.3f} A"
    assert msg.total_power_consumed >= 0.0, (
        "Heron total_power_consumed should be nonnegative: "
        f"{msg.total_power_consumed:.3f} Wh"
    )


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
    _assert_node_relation(topic, required_subscriber, state_index=1, role="subscribed")


@pytest.mark.parametrize(
    ("topic", "required_publisher"),
    [
        ("/cmd_drive", "/controller"),
    ],
)
def test_required_command_publishers_exist(topic, required_publisher):
    _assert_node_relation(topic, required_publisher, state_index=0, role="published")
