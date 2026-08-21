#!/usr/bin/env python3
"""Persist one externally owned physical sensor provider across ROS restarts."""

from __future__ import annotations

import argparse
import fcntl
import http.client
import math
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time
from typing import Any, Dict, Iterable, List, Set
from xmlrpc.client import ServerProxy, Transport

import rospy
import rospkg
from sensor_msgs.msg import Imu

from ig_handle_runtime.network_config import network_value
from ig_handle_runtime.sensor_contract import load_contract, sensor_value


TEMPORARY_FAILURE = 75


def _required_topics(sensor: Dict[str, Any]) -> List[str]:
    topics = dict(sensor.get("topics", {}) or {})
    required = dict(sensor.get("required_topics", {}) or {})
    return [
        str(topic)
        for name, topic in topics.items()
        if topic and required.get(name, True)
    ]


class _TimeoutTransport(Transport):
    def __init__(self, timeout_sec: float):
        super().__init__()
        self._timeout_sec = timeout_sec

    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=self._timeout_sec)


def _master_call(master_uri: str, method: str, *args):
    proxy = ServerProxy(master_uri, transport=_TimeoutTransport(0.75))
    code, message, value = getattr(proxy, method)("/ig_handle_external_sensor", *args)
    if int(code) != 1:
        raise RuntimeError("ROS master {} failed: {}".format(method, message))
    return value


def _master_pid(master_uri: str) -> int:
    return int(_master_call(master_uri, "getPid"))


def _publisher_graph(master_uri: str) -> Dict[str, Set[str]]:
    publishers, _subscribers, _services = _master_call(master_uri, "getSystemState")
    return {str(topic): set(nodes) for topic, nodes in publishers}


def _node_reachable(master_uri: str, node_name: str) -> bool:
    try:
        uri = str(_master_call(master_uri, "lookupNode", node_name))
        proxy = ServerProxy(uri, transport=_TimeoutTransport(0.75))
        code, _message, pid = proxy.getPid("/ig_handle_external_sensor")
    except Exception:
        return False
    return bool(int(code) == 1 and int(pid) > 0)


def _acquire_provider_lock(sensor_id: str):
    runtime_dir = Path("/run/user") / str(os.getuid())
    if not runtime_dir.is_dir():
        raise RuntimeError(
            "user runtime directory is unavailable: {}".format(runtime_dir)
        )
    lock_path = runtime_dir / "ig-handle-external-sensor-{}.lock".format(sensor_id)
    handle = lock_path.open("a+")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError(
            "external sensor {} already has a local lifecycle owner".format(sensor_id)
        ) from exc
    handle.seek(0)
    handle.truncate()
    handle.write("{}\n".format(os.getpid()))
    handle.flush()
    return handle


def _same_character_device(configured: str, identity: str) -> bool:
    try:
        configured_path = Path(configured)
        identity_path = Path(identity)
        configured_stat = configured_path.stat()
        identity_stat = identity_path.stat()
    except (FileNotFoundError, OSError):
        return False
    return bool(
        stat.S_ISCHR(configured_stat.st_mode)
        and stat.S_ISCHR(identity_stat.st_mode)
        and configured_stat.st_rdev == identity_stat.st_rdev
    )


def _device_owner_pids(device_path: str) -> Set[int]:
    try:
        device_stat = Path(device_path).stat()
    except OSError:
        return set()
    owners: Set[int] = set()
    for proc_dir in Path("/proc").iterdir():
        if not proc_dir.name.isdigit():
            continue
        try:
            for fd_path in (proc_dir / "fd").iterdir():
                try:
                    fd_stat = fd_path.stat()
                except OSError:
                    continue
                if (
                    stat.S_ISCHR(fd_stat.st_mode)
                    and fd_stat.st_rdev == device_stat.st_rdev
                ):
                    owners.add(int(proc_dir.name))
                    break
        except OSError:
            continue
    return owners


def _resolve_package_uri(package_root: str, value: Any) -> str:
    return str(value if value is not None else "").replace(
        "package://ig_handle", package_root
    )


def _launch_arguments(
    package_root: str, contract: Dict[str, Any], sensor_id: str
) -> List[str]:
    sensor = dict((contract.get("sensors", {}) or {}).get(sensor_id, {}) or {})
    launch = dict(sensor.get("launch", {}) or {})
    launch_file = _resolve_package_uri(package_root, launch.get("file", ""))
    if not launch_file:
        raise RuntimeError("external sensor {} has no launch file".format(sensor_id))
    arguments = ["roslaunch", launch_file]
    for name, spec in dict(launch.get("args", {}) or {}).items():
        if isinstance(spec, dict) and "field" in spec:
            value = sensor_value(contract, sensor_id, str(spec["field"]), "")
        elif isinstance(spec, dict) and "literal" in spec:
            value = spec.get("literal", "")
        else:
            value = spec
        arguments.append(
            "{}:={}".format(name, _resolve_package_uri(package_root, value))
        )
    return arguments


def _alive_pids(root_pid: int) -> List[int]:
    children: Dict[int, List[int]] = {}
    for status_path in Path("/proc").glob("[0-9]*/status"):
        try:
            values = {}
            for line in status_path.read_text().splitlines():
                if line.startswith(("Pid:", "PPid:")):
                    key, value = line.split(":", 1)
                    values[key] = int(value.strip())
            children.setdefault(values["PPid"], []).append(values["Pid"])
        except (FileNotFoundError, PermissionError, KeyError, ValueError):
            continue
    descendants = []

    def visit(parent: int) -> None:
        for child in children.get(parent, []):
            visit(child)
            descendants.append(child)

    visit(root_pid)
    result = [root_pid, *descendants]
    alive = []
    for pid in result:
        try:
            fields = Path("/proc/{}/stat".format(pid)).read_text().split()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if len(fields) > 2 and fields[2] != "Z":
            alive.append(pid)
    return alive


def _stop_process_group(proc: subprocess.Popen) -> None:
    for sig, timeout_sec in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 2.0),
    ):
        alive = _alive_pids(proc.pid)
        if not alive:
            return
        for pid in alive:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not _alive_pids(proc.pid):
                return
            time.sleep(0.1)
    if _alive_pids(proc.pid):
        raise RuntimeError("external sensor process group could not be reaped")


class ProviderMonitor:
    def __init__(
        self,
        *,
        expected_publisher: str,
        expected_frame: str,
        future_tolerance_sec: float,
        stamp_timeout_sec: float,
    ):
        self.expected_publisher = expected_publisher
        self.expected_frame = expected_frame
        self.future_tolerance_sec = future_tolerance_sec
        self.stamp_timeout_sec = stamp_timeout_sec
        self.last_message_wall = float("-inf")
        self.last_stamp = float("-inf")
        self.fault = ""

    def callback(self, message: Imu) -> None:
        source = str(
            (getattr(message, "_connection_header", None) or {}).get("callerid", "")
            or ""
        )
        stamp_sec = float(message.header.stamp.to_sec())
        age_sec = float(rospy.Time.now().to_sec()) - stamp_sec
        values: Iterable[float] = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        if source != self.expected_publisher:
            self.fault = "unexpected_message_publisher:{}".format(source)
        elif str(message.header.frame_id) != self.expected_frame:
            self.fault = "unexpected_frame:{}".format(message.header.frame_id)
        elif not all(math.isfinite(float(value)) for value in values):
            self.fault = "nonfinite_imu_value"
        elif not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            self.fault = "invalid_stamp"
        elif stamp_sec <= self.last_stamp:
            self.fault = "nonadvancing_stamp"
        elif age_sec < -self.future_tolerance_sec:
            self.fault = "future_stamp"
        elif age_sec > self.stamp_timeout_sec:
            self.fault = "stale_stamp"
        if self.fault:
            return
        self.last_stamp = stamp_sec
        self.last_message_wall = time.monotonic()


def _wait_for_prerequisites(
    master_uri: str, device_path: str, identity_path: str
) -> int:
    last_report = float("-inf")
    while True:
        device_ready = _same_character_device(device_path, identity_path)
        try:
            master_pid = _master_pid(master_uri)
            master_ready = master_pid > 0
        except Exception:
            master_pid = -1
            master_ready = False
        if device_ready and master_ready:
            return master_pid
        if time.monotonic() - last_report >= 30.0:
            print(
                "waiting for Xsens prerequisites: device_identity={} master={}".format(
                    "ready" if device_ready else "missing_or_wrong",
                    "ready" if master_ready else "unreachable",
                ),
                flush=True,
            )
            last_report = time.monotonic()
        time.sleep(2.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-id", default="1")
    parser.add_argument("--startup-grace-sec", type=float, default=15.0)
    parser.add_argument("--topic-timeout-sec", type=float, default=2.0)
    parser.add_argument("--stamp-timeout-sec", type=float, default=0.5)
    parser.add_argument("--future-tolerance-sec", type=float, default=0.05)
    args = parser.parse_args()

    package_root = rospkg.RosPack().get_path("ig_handle")
    contract = load_contract(package_root)
    sensor = dict((contract.get("sensors", {}) or {}).get(args.sensor_id, {}) or {})
    if str(sensor.get("lifecycle_owner", "")) != "external_service":
        raise RuntimeError("selected sensor is not externally owned")
    required_topics = _required_topics(sensor)
    if required_topics != [str(sensor_value(contract, args.sensor_id, "topics.data"))]:
        raise RuntimeError("external Xsens service requires exactly its data topic")
    topic = required_topics[0]
    expected_publisher = str(sensor.get("expected_publisher", "") or "").strip()
    expected_frame = str(sensor.get("frame", "") or "").strip()
    device_path = str(sensor.get("device_path", "") or "").strip()
    identity_path = str(sensor.get("device_identity_path", "") or "").strip()
    if not all((expected_publisher, expected_frame, device_path, identity_path)):
        raise RuntimeError("external sensor identity contract is incomplete")

    heron_ip = network_value("heron_ip", package_root=package_root)
    local_ip = network_value("heron_local_ip", package_root=package_root)
    master_uri = "http://{}:11311".format(heron_ip)
    os.environ["ROS_MASTER_URI"] = master_uri
    os.environ["ROS_IP"] = local_ip

    provider_lock = _acquire_provider_lock(args.sensor_id)
    initial_master_pid = _wait_for_prerequisites(master_uri, device_path, identity_path)
    existing = _publisher_graph(master_uri).get(topic, set())
    unexpected = existing - {expected_publisher}
    if unexpected:
        raise RuntimeError(
            "duplicate publisher before launch: {}".format(sorted(unexpected))
        )
    if expected_publisher in existing and _node_reachable(
        master_uri, expected_publisher
    ):
        raise RuntimeError(
            "live expected publisher already exists: {}".format(expected_publisher)
        )
    serial_owners = _device_owner_pids(device_path)
    if serial_owners:
        raise RuntimeError(
            "serial device already owned by pids: {}".format(sorted(serial_owners))
        )

    rospy.init_node(
        "external_sensor_provider_{}".format(args.sensor_id), disable_signals=True
    )
    monitor = ProviderMonitor(
        expected_publisher=expected_publisher,
        expected_frame=expected_frame,
        future_tolerance_sec=args.future_tolerance_sec,
        stamp_timeout_sec=args.stamp_timeout_sec,
    )
    rospy.Subscriber(topic, Imu, monitor.callback, queue_size=20, tcp_nodelay=True)
    command = _launch_arguments(package_root, contract, args.sensor_id)
    print(
        "starting externally owned sensor {}: {}".format(args.sensor_id, command),
        flush=True,
    )
    proc = subprocess.Popen(command, start_new_session=True)
    started = time.monotonic()
    stopping = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    fault = ""
    try:
        while not stopping:
            returncode = proc.poll()
            if returncode is not None:
                fault = "driver_exited:{}".format(returncode)
                break
            if not _same_character_device(device_path, identity_path):
                fault = "device_missing_or_wrong_identity"
                break
            try:
                if _master_pid(master_uri) != initial_master_pid:
                    fault = "ros_master_replaced"
                    break
                publishers = _publisher_graph(master_uri).get(topic, set())
            except Exception:
                fault = "ros_master_unreachable"
                break
            if time.monotonic() - started > args.startup_grace_sec:
                if publishers != {expected_publisher}:
                    fault = "publisher_ownership:{}".format(sorted(publishers))
                    break
                if monitor.fault:
                    fault = monitor.fault
                    break
                if (
                    time.monotonic() - monitor.last_message_wall
                    > args.topic_timeout_sec
                ):
                    fault = "topic_stale:{}".format(topic)
                    break
            time.sleep(0.5)
    finally:
        _stop_process_group(proc)
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass
    if fault:
        print("external sensor provider fault: {}".format(fault), file=sys.stderr)
        return TEMPORARY_FAILURE
    del provider_lock
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
