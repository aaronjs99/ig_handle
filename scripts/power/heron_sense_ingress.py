#!/usr/bin/env python3
"""Republish the raw Heron MCU contract on IG Handle's canonical topic."""

import math

import rospy
from heron_msgs.msg import Sense


def normalize_node_name(value: str) -> str:
    value = str(value).strip()
    if not value:
        raise ValueError("expected_publisher must be nonempty")
    return value if value.startswith("/") else "/" + value


class HeronSenseIngress:
    def __init__(self) -> None:
        self.input_topic = str(rospy.get_param("~input_topic", "/sense")).strip()
        self.output_topic = str(
            rospy.get_param("~output_topic", "/sense_heron")
        ).strip()
        self.expected_publisher = normalize_node_name(
            rospy.get_param("~expected_publisher")
        )
        if not self.input_topic or not self.output_topic:
            raise ValueError("sense input/output topics must be nonempty")
        if self.input_topic == self.output_topic:
            raise ValueError("raw and canonical Heron sense topics must differ")
        self.publisher = rospy.Publisher(
            self.output_topic, Sense, queue_size=20, tcp_nodelay=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            Sense,
            self._callback,
            queue_size=20,
            tcp_nodelay=True,
        )

    def _callback(self, message: Sense) -> None:
        header = getattr(message, "_connection_header", {}) or {}
        caller = normalize_node_name(header.get("callerid", "unknown"))
        if caller != self.expected_publisher:
            rospy.logerr_throttle(
                2.0,
                "Dropping raw Heron sense from unexpected publisher %s (expected %s)",
                caller,
                self.expected_publisher,
            )
            return
        values = (message.battery, message.current_left, message.current_right)
        if not all(math.isfinite(value) for value in values):
            rospy.logerr_throttle(2.0, "Dropping non-finite raw Heron sense")
            return
        self.publisher.publish(message)


def main() -> None:
    rospy.init_node("heron_sense_ingress")
    try:
        HeronSenseIngress()
    except (KeyError, TypeError, ValueError) as exc:
        rospy.logfatal("Invalid Heron sense ingress configuration: %s", exc)
        raise SystemExit(2)
    rospy.spin()


if __name__ == "__main__":
    main()
