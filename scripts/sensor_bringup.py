#!/usr/bin/env python3
"""Launch enabled sensors from the IG Handle sensor contract."""

from __future__ import annotations

import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List

import rospy
import rospkg

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
        self.contract = load_contract(self.package_root, self.contract_file)
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

    def spin(self) -> None:
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            for sensor_id, proc in list(self.processes.items()):
                returncode = proc.poll()
                if returncode is not None:
                    rospy.logerr(
                        "sensor_bringup child for id=%s exited with code %s",
                        sensor_id,
                        returncode,
                    )
                    self.processes.pop(sensor_id, None)
            rate.sleep()

    def shutdown(self) -> None:
        for proc in list(self.processes.values()):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        deadline = time.time() + 8.0
        for proc in list(self.processes.values()):
            while proc.poll() is None and time.time() < deadline:
                time.sleep(0.1)
            if proc.poll() is None:
                proc.terminate()

    def _launch_sensor(self, sensor_id: str, sensor: Dict[str, Any]) -> None:
        launch = dict(sensor.get("launch", {}) or {})
        launch_file = _resolve_package_uri(self.package_root, launch.get("file", ""))
        if not launch_file:
            raise RuntimeError(f"sensor {sensor_id} launch stanza has no file")
        args = ["roslaunch", launch_file]
        for name, spec in dict(launch.get("args", {}) or {}).items():
            args.append(f"{name}:={self._arg_value(sensor_id, spec)}")
        rospy.loginfo("sensor_bringup launching id=%s file=%s", sensor_id, launch_file)
        self.processes[sensor_id] = subprocess.Popen(args)

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
