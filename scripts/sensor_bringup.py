#!/usr/bin/env python3
"""Launch and supervise enabled sensors from the IG Handle sensor contract."""

from __future__ import annotations

import signal
import subprocess
import sys
import time
import json
import os
from pathlib import Path
from typing import Any, Dict, Iterable, List

import rospy
import rospkg
from std_msgs.msg import String

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from network_config import network_value  # noqa: E402
from sensor_contract import (  # noqa: E402
    load_contract,
    sensor_reachable,
    sensor_requested,
    sensor_value,
)


def _csv_ids(value: str) -> List[str]:
    ids: List[str] = []
    for item in str(value or "").replace(";", ",").split(","):
        item = item.strip()
        if item and item not in ids:
            ids.append(item)
    return ids


def _resolve_package_uri(package_root: str, value: Any) -> str:
    return str(value if value is not None else "").replace(
        "package://ig_handle", package_root
    )


def _bool_text(value: bool) -> str:
    return "true" if bool(value) else "false"


def _sensor_order(sensors: Dict[str, Any]) -> Iterable[str]:
    return sorted(
        sensors.keys(),
        key=lambda sensor_id: (
            (sensors.get(sensor_id) or {}).get("startup_order", 1000),
            str(sensor_id),
        ),
    )


class SensorBringup:
    def __init__(self) -> None:
        self.package_root = rospkg.RosPack().get_path("ig_handle")
        self.contract_file = str(rospy.get_param("~sensor_contract_file", ""))
        self.extra_sensor_ids = str(rospy.get_param("~extra_sensor_ids", ""))
        self.disabled_sensor_ids = str(rospy.get_param("~disabled_sensor_ids", ""))
        self.reachability_check = self._param_bool("~sensor_reachability_check", True)
        self.processes: Dict[str, subprocess.Popen] = {}
        self.process_started: Dict[str, float] = {}
        self.topic_last_message: Dict[str, float] = {}
        self.topic_subscribers = {}
        self.last_restart: Dict[str, float] = {}
        self.health_timeout_sec = float(rospy.get_param("~topic_timeout_sec", 5.0))
        self.restart_cooldown_sec = float(
            rospy.get_param("~restart_cooldown_sec", 10.0)
        )
        self.contract = load_contract(self.package_root, self.contract_file)
        self.health_pub = rospy.Publisher("~health", String, queue_size=1, latch=True)
        rospy.on_shutdown(self.shutdown)

    @staticmethod
    def _param_bool(name: str, default: bool) -> bool:
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def start(self) -> None:
        sensors = dict(self.contract.get("sensors", {}) or {})
        for sensor_id in _sensor_order(sensors):
            sensor = dict(sensors.get(sensor_id) or {})
            if not dict(sensor.get("launch", {}) or {}):
                continue
            enabled = sensor_requested(
                self.contract,
                sensor_id,
                self.extra_sensor_ids,
                self.disabled_sensor_ids,
            )
            reachable = enabled and sensor_reachable(
                self.contract,
                self.package_root,
                sensor_id,
                self.reachability_check,
            )
            rospy.set_param(f"/ig_handle/sensors/{sensor_id}/requested", bool(enabled))
            rospy.set_param(
                f"/ig_handle/sensors/{sensor_id}/reachable", bool(reachable)
            )
            rospy.set_param(f"/ig_handle/sensors/{sensor_id}/enabled", bool(reachable))
            if not reachable:
                rospy.loginfo(
                    "sensor_bringup skipping id=%s requested=%s reachable=%s",
                    sensor_id,
                    _bool_text(enabled),
                    _bool_text(reachable),
                )
                continue
            self._launch_sensor(sensor_id, sensor)
            self._subscribe_sensor_topics(sensor_id, sensor)
        self._publish_health()

    def spin(self) -> None:
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            sensors = dict(self.contract.get("sensors", {}) or {})
            for sensor_id, proc in list(self.processes.items()):
                returncode = proc.poll()
                reason = ""
                if returncode is not None:
                    reason = "child_exited:{}".format(returncode)
                else:
                    reason = self._sensor_unhealthy_reason(
                        sensor_id, sensors[sensor_id]
                    )
                if reason:
                    self._restart_sensor(sensor_id, sensors[sensor_id], reason)
            self._publish_health()
            rate.sleep()

    def shutdown(self) -> None:
        for proc in list(self.processes.values()):
            self._stop_process(proc)
        deadline = time.time() + 8.0
        for proc in list(self.processes.values()):
            while proc.poll() is None and time.time() < deadline:
                time.sleep(0.1)
            if proc.poll() is None:
                self._stop_process(proc, signal.SIGTERM)

    def _launch_sensor(self, sensor_id: str, sensor: Dict[str, Any]) -> None:
        launch = dict(sensor.get("launch", {}) or {})
        launch_file = _resolve_package_uri(self.package_root, launch.get("file", ""))
        if not launch_file:
            raise RuntimeError(f"sensor {sensor_id} launch stanza has no file")
        args = ["roslaunch", launch_file]
        for name, spec in dict(launch.get("args", {}) or {}).items():
            args.append(f"{name}:={self._arg_value(sensor_id, spec)}")
        rospy.loginfo("sensor_bringup launching id=%s file=%s", sensor_id, launch_file)
        self.processes[sensor_id] = subprocess.Popen(args, start_new_session=True)
        self.process_started[sensor_id] = time.monotonic()

    def _required_topics(self, sensor: Dict[str, Any]) -> List[str]:
        topics = dict(sensor.get("topics", {}) or {})
        required = dict(sensor.get("required_topics", {}) or {})
        return [
            str(topic)
            for name, topic in topics.items()
            if topic and required.get(name, True)
        ]

    def _subscribe_sensor_topics(self, sensor_id: str, sensor: Dict[str, Any]) -> None:
        for topic in self._required_topics(sensor):
            if topic in self.topic_subscribers:
                continue
            self.topic_subscribers[topic] = rospy.Subscriber(
                topic,
                rospy.AnyMsg,
                lambda _msg, observed=topic: self.topic_last_message.__setitem__(
                    observed, time.monotonic()
                ),
                queue_size=1,
            )

    @staticmethod
    def _stop_process(proc: subprocess.Popen, sig=signal.SIGINT) -> None:
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return

    def _sensor_unhealthy_reason(self, sensor_id: str, sensor: Dict[str, Any]) -> str:
        started = self.process_started.get(sensor_id, time.monotonic())
        launch = dict(sensor.get("launch", {}) or {})
        configured_delay = float(
            launch.get("args", {}).get("startup_delay_sec", {}).get("literal", 0.0)
            if isinstance(launch.get("args", {}).get("startup_delay_sec", {}), dict)
            else sensor.get("startup_delay_sec", 0.0)
        )
        grace_sec = self.health_timeout_sec + max(
            configured_delay, float(sensor.get("startup_delay_sec", 0.0))
        )
        if time.monotonic() - started <= grace_sec:
            return ""
        now = time.monotonic()
        for topic in self._required_topics(sensor):
            observed = self.topic_last_message.get(topic, float("-inf"))
            if now - observed > self.health_timeout_sec:
                return "topic_stale:{}".format(topic)
        return ""

    def _restart_sensor(
        self, sensor_id: str, sensor: Dict[str, Any], reason: str
    ) -> None:
        now = time.monotonic()
        if (
            now - self.last_restart.get(sensor_id, float("-inf"))
            < self.restart_cooldown_sec
        ):
            return
        self.last_restart[sensor_id] = now
        proc = self.processes.get(sensor_id)
        if proc is not None:
            self._stop_process(proc)
            deadline = time.monotonic() + 8.0
            while proc.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if proc.poll() is None:
                self._stop_process(proc, signal.SIGTERM)
                deadline = time.monotonic() + 2.0
                while proc.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.1)
            if proc.poll() is None:
                rospy.logerr(
                    "sensor_bringup did not reap id=%s; replacement refused", sensor_id
                )
                return
        self.processes.pop(sensor_id, None)
        rospy.logwarn("sensor_bringup restarting id=%s reason=%s", sensor_id, reason)
        self._launch_sensor(sensor_id, sensor)

    def _publish_health(self) -> None:
        sensors = dict(self.contract.get("sensors", {}) or {})
        now = time.monotonic()
        status = {}
        for sensor_id, proc in self.processes.items():
            sensor = sensors.get(sensor_id, {})
            topics = self._required_topics(sensor)
            status[sensor_id] = {
                "alive": proc.poll() is None,
                "age_sec": round(now - self.process_started.get(sensor_id, now), 3),
                "topics": {
                    topic: (
                        {"state": "fresh", "age_sec": round(now - observed, 3)}
                        if (observed := self.topic_last_message.get(topic)) is not None
                        else {"state": "never_seen", "age_sec": None}
                    )
                    for topic in topics
                },
            }
        self.health_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def contract_path_arg(self) -> str:
        if self.contract_file:
            return _resolve_package_uri(self.package_root, self.contract_file)
        return str(
            Path(self.package_root) / "config" / "sensors" / "sensor_contract.yaml"
        )

    def _arg_value(self, sensor_id: str, spec: Any) -> str:
        if isinstance(spec, dict):
            if "field" in spec:
                return _resolve_package_uri(
                    self.package_root,
                    sensor_value(self.contract, sensor_id, str(spec["field"]), ""),
                )
            if "endpoint" in spec:
                endpoint_key = sensor_value(
                    self.contract, sensor_id, "endpoint_key", ""
                )
                return str(
                    network_value(
                        str(endpoint_key or ""),
                        package_root=self.package_root,
                        default="",
                    )
                    or ""
                )
            if "literal" in spec:
                return _resolve_package_uri(self.package_root, spec.get("literal", ""))
            if "contract_file" in spec:
                return self.contract_path_arg()
            if "sensor_id" in spec:
                return str(sensor_id)
            if "reachability_check" in spec:
                return _bool_text(self.reachability_check)
        return _resolve_package_uri(self.package_root, spec)


def main() -> None:
    rospy.init_node("sensor_bringup")
    bringup = SensorBringup()
    bringup.start()
    bringup.spin()


if __name__ == "__main__":
    main()
