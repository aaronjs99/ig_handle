"""ROS graph and topic checks for IG Handle live tests."""

from __future__ import annotations

import os
import socket
import time
from urllib.parse import urlparse

import pytest


def wait_for_topic_message(topic: str, expected_type: str, *, timeout_sec: float = 8.0):
    assert_ros_master_reachable()

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

    remaining = max(0.1, deadline - time.time())
    try:
        return rospy.wait_for_message(real_topic or topic, msg_class, timeout=remaining)
    except Exception as exc:
        pytest.fail(f"{topic} did not publish within {timeout_sec:.1f}s: {exc}")


def get_topic_publishers(topic: str) -> set[str]:
    return _topic_nodes(topic, state_index=0)


def get_topic_subscribers(topic: str) -> set[str]:
    return _topic_nodes(topic, state_index=1)


def assert_node_subscribes(topic: str, node_name: str) -> None:
    subscribers = get_topic_subscribers(topic)
    assert node_name_matches(
        subscribers, node_name
    ), f"{topic} is not subscribed by {node_name}; subscribers={sorted(subscribers)}"


def assert_node_publishes(topic: str, node_name: str) -> None:
    publishers = get_topic_publishers(topic)
    assert node_name_matches(
        publishers, node_name
    ), f"{topic} is not published by {node_name}; publishers={sorted(publishers)}"


def node_name_matches(nodes: set[str], expected_name: str) -> bool:
    return any(node == expected_name or node.endswith(expected_name) for node in nodes)


def assert_ros_master_reachable(timeout_sec: float = 2.0) -> None:
    master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
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


def _topic_nodes(topic: str, *, state_index: int) -> set[str]:
    assert_ros_master_reachable()
    try:
        import rosgraph
    except ImportError as exc:
        pytest.fail(f"ROS Python packages are unavailable; source the workspace: {exc}")

    state = rosgraph.Master("/ig_handle_live_hardware_tests").getSystemState()
    return set(dict(state[state_index]).get(topic, []))
