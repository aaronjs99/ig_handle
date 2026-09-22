#!/usr/bin/env python3
"""Persist one externally owned physical sensor provider across ROS restarts."""

from __future__ import annotations

import argparse
import fcntl
import http.client
import ipaddress
import math
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple
from urllib.parse import urlparse
from xmlrpc.client import ServerProxy, Transport

import rospy
import rospkg
from sensor_msgs.msg import Imu

from sensors.network import network_value
from sensors.process_identity import read_process_stat
from sensors.contracts import load_contract, sensor_value


TEMPORARY_FAILURE = 75
PROFILE_INTEGRATED = "integrated"
PROFILE_STANDALONE = "standalone"
PROFILE_GROUND = "ground"
SUPPORTED_PROFILES = (PROFILE_STANDALONE, PROFILE_INTEGRATED, PROFILE_GROUND)
ProcessContext = Tuple[int, int, int, int]


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


def _local_node_pid(master_uri: str, node_name: str, local_ip: str) -> int:
    uri = str(_master_call(master_uri, "lookupNode", node_name) or "").strip()
    parsed = urlparse(uri)
    allowed_hosts = {
        "127.0.0.1",
        "localhost",
        str(local_ip or "").strip().lower(),
    }
    allowed_hosts.discard("")
    if (
        parsed.scheme != "http"
        or str(parsed.hostname or "").lower() not in allowed_hosts
    ):
        raise RuntimeError("publisher node is not on the selected local host")
    proxy = ServerProxy(uri, transport=_TimeoutTransport(0.75))
    code, message, pid = proxy.getPid("/ig_handle_external_sensor")
    if int(code) != 1 or int(pid) <= 0:
        raise RuntimeError("publisher getPid failed: {}".format(message))
    return int(pid)


def _acquire_provider_lock(sensor_id: str):
    safe_sensor_id = str(sensor_id or "").strip()
    if not safe_sensor_id or not all(
        character.isalnum() or character in ("-", "_") for character in safe_sensor_id
    ):
        raise RuntimeError("external sensor id is unsafe for lock ownership")
    lock_path = Path("/tmp") / "ig-handle-external-sensor-{}-{}.lock".format(
        os.getuid(), safe_sensor_id
    )
    flags = os.O_CREAT | os.O_RDWR | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(lock_path), flags, 0o600)
    handle = os.fdopen(descriptor, "r+")
    try:
        lock_stat = os.fstat(handle.fileno())
        if (
            not stat.S_ISREG(lock_stat.st_mode)
            or lock_stat.st_uid != os.getuid()
            or lock_stat.st_nlink != 1
        ):
            raise RuntimeError("external sensor lock path is not a private file")
        os.fchmod(handle.fileno(), 0o600)
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError(
            "external sensor {} already has a lifecycle owner".format(sensor_id)
        ) from exc
    except Exception:
        handle.close()
        raise
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
    # The selected master has an independent lifecycle owner. Never let a
    # provider-side roslaunch create a replacement core during a failure race.
    arguments = ["roslaunch", "--wait", launch_file]
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


def _identity_matches(pid: int, start_time_ticks: int) -> bool:
    try:
        process = read_process_stat(pid)
    except (
        FileNotFoundError,
        PermissionError,
        ProcessLookupError,
        RuntimeError,
        ValueError,
    ):
        return False
    return bool(
        process["start_time_ticks"] == int(start_time_ticks)
        and process.get("state") != "Z"
    )


def _pid_descends_from(
    pid: int,
    root_pid: int,
    root_start_time_ticks: Optional[int] = None,
) -> bool:
    current = int(pid)
    root = int(root_pid)
    visited = set()
    while current > 0 and current not in visited:
        try:
            process = read_process_stat(current)
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            RuntimeError,
            ValueError,
        ):
            return False
        if current == root:
            return root_start_time_ticks is None or process["start_time_ticks"] == int(
                root_start_time_ticks
            )
        visited.add(current)
        current = process["parent_pid"]
    return False


def _managed_tree(
    root_pid: int, root_start_time_ticks: int
) -> Dict[int, Tuple[int, int]]:
    try:
        root = read_process_stat(root_pid)
    except (
        FileNotFoundError,
        PermissionError,
        ProcessLookupError,
        RuntimeError,
        ValueError,
    ):
        return {}
    if root["start_time_ticks"] != int(root_start_time_ticks):
        return {}
    tree: Dict[int, Tuple[int, int]] = {int(root_pid): (int(root_start_time_ticks), 0)}
    pending = [(int(root_pid), 0)]
    while pending:
        parent, depth = pending.pop()
        children = set()
        try:
            task_directories = list((Path("/proc") / str(parent) / "task").iterdir())
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            RuntimeError,
            ValueError,
        ):
            continue
        for task_directory in task_directories:
            try:
                children.update(
                    int(value)
                    for value in (task_directory / "children").read_text().split()
                )
            except (
                FileNotFoundError,
                PermissionError,
                ProcessLookupError,
                RuntimeError,
                ValueError,
            ):
                continue
        for child in sorted(children):
            if child in tree:
                continue
            try:
                child_process = read_process_stat(child)
            except (
                FileNotFoundError,
                PermissionError,
                ProcessLookupError,
                RuntimeError,
                ValueError,
            ):
                continue
            if child_process["parent_pid"] != parent:
                continue
            tree[child] = (child_process["start_time_ticks"], depth + 1)
            pending.append((child, depth + 1))
    return tree


def _refresh_tracked_identities(
    proc: subprocess.Popen,
    tracked: Dict[int, int],
    contexts: Optional[Dict[int, ProcessContext]] = None,
) -> Dict[int, Tuple[int, int]]:
    root_pid = int(proc.pid)
    root_start = tracked.get(root_pid)
    if root_start is None:
        if proc.poll() is not None:
            return {}
        try:
            root_start = read_process_stat(root_pid)["start_time_ticks"]
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            RuntimeError,
            ValueError,
        ):
            return {}
        tracked[root_pid] = root_start
    tree = _managed_tree(root_pid, root_start)
    verified_tree: Dict[int, Tuple[int, int]] = {}
    for pid, (start_time_ticks, depth) in tree.items():
        try:
            process = read_process_stat(pid)
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            RuntimeError,
            ValueError,
        ):
            continue
        if (
            process["state"] == "Z"
            or process["start_time_ticks"] != int(start_time_ticks)
            or (pid != root_pid and not _pid_descends_from(pid, root_pid, root_start))
        ):
            continue
        previous_start = tracked.get(pid)
        if previous_start is not None and previous_start != int(start_time_ticks):
            # Never adopt a reused PID into an existing ownership record.
            continue
        context: ProcessContext = (
            int(start_time_ticks),
            int(process["process_group_id"]),
            int(process["session_id"]),
            int(depth),
        )
        if contexts is not None:
            previous_context = contexts.get(pid)
            if previous_context is not None and previous_context != context:
                # A managed process that changes group/session is no longer safe
                # to signal using the original ownership proof.
                continue
            contexts.setdefault(pid, context)
        tracked.setdefault(pid, int(start_time_ticks))
        verified_tree[pid] = (int(start_time_ticks), int(depth))
    return verified_tree


def _owned_process_stat_now(
    pid: int,
    proc: subprocess.Popen,
    tracked: Dict[int, int],
    contexts: Dict[int, ProcessContext],
    *,
    allow_reparented: bool,
) -> Optional[Dict[str, Any]]:
    """Return current process state only with an exact ownership proof."""

    root_pid = int(proc.pid)
    root_start = tracked.get(root_pid)
    context = contexts.get(int(pid))
    if root_start is None or context is None:
        return None
    start_time_ticks, process_group_id, session_id, _depth = context
    if tracked.get(int(pid)) != start_time_ticks:
        return None
    try:
        process = read_process_stat(int(pid))
    except (
        FileNotFoundError,
        PermissionError,
        ProcessLookupError,
        RuntimeError,
        ValueError,
    ):
        return None
    if (
        process["state"] == "Z"
        or process["start_time_ticks"] != start_time_ticks
        or process["process_group_id"] != process_group_id
        or process["session_id"] != session_id
    ):
        return None
    if int(pid) == root_pid:
        return process if start_time_ticks == root_start else None
    if _pid_descends_from(int(pid), root_pid, root_start):
        return process
    if not allow_reparented:
        return None
    root_context = contexts.get(root_pid)
    if root_context is None:
        return None
    # A child discovered while it was in the verified launch tree may outlive
    # and be reparented after roslaunch exits. Its immutable PID/start identity
    # plus unchanged original session and process group are the remaining proof.
    if root_context[2] != root_pid or session_id != root_context[2]:
        return None
    return process


def _publisher_process_owned(
    publisher_pid: int,
    proc: subprocess.Popen,
    tracked: Dict[int, int],
    contexts: Dict[int, ProcessContext],
) -> bool:
    tree = _refresh_tracked_identities(proc, tracked, contexts)
    if int(publisher_pid) not in tree:
        return False
    return (
        _owned_process_stat_now(
            int(publisher_pid),
            proc,
            tracked,
            contexts,
            allow_reparented=False,
        )
        is not None
    )


def _tracked_process_alive(
    proc: subprocess.Popen,
    tracked: Dict[int, int],
    contexts: Optional[Dict[int, ProcessContext]] = None,
) -> bool:
    _refresh_tracked_identities(proc, tracked, contexts)
    if proc.poll() is None and int(proc.pid) not in tracked:
        return True
    return any(
        _identity_matches(pid, start_time_ticks)
        for pid, start_time_ticks in tracked.items()
    )


def _signal_verified_tree(
    proc: subprocess.Popen,
    tracked: Dict[int, int],
    contexts: Dict[int, ProcessContext],
    sig: int,
) -> None:
    _refresh_tracked_identities(proc, tracked, contexts)
    for pid, context in sorted(
        contexts.items(), key=lambda item: (item[1][3], item[0]), reverse=True
    ):
        if (
            _owned_process_stat_now(
                pid,
                proc,
                tracked,
                contexts,
                allow_reparented=True,
            )
            is None
        ):
            continue
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except OSError:
            pass


def _stop_process_group(
    proc: subprocess.Popen,
    known_identities: Optional[Dict[int, int]] = None,
    known_contexts: Optional[Dict[int, ProcessContext]] = None,
) -> None:
    tracked = dict(known_identities or {})
    contexts = dict(known_contexts or {})
    for sig, timeout_sec in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 2.0),
    ):
        if not _tracked_process_alive(proc, tracked, contexts):
            try:
                proc.wait(timeout=0)
            except subprocess.TimeoutExpired:
                pass
            return
        _signal_verified_tree(proc, tracked, contexts, sig)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not _tracked_process_alive(proc, tracked, contexts):
                try:
                    proc.wait(timeout=0)
                except subprocess.TimeoutExpired:
                    pass
                return
            time.sleep(0.1)
    if _tracked_process_alive(proc, tracked, contexts):
        raise RuntimeError("external sensor process group could not be reaped")


def _profile_network(package_root: str, profile: str) -> Tuple[str, str]:
    selected = str(profile or "").strip().lower()
    if selected == PROFILE_STANDALONE:
        master_uri = network_value("standalone_master_uri", package_root=package_root)
        local_ip = network_value("sensor_lan_ip", package_root=package_root)
    elif selected == PROFILE_GROUND:
        master_uri = "http://127.0.0.1:11321"
        local_ip = network_value("sensor_lan_ip", package_root=package_root)
    elif selected == PROFILE_INTEGRATED:
        heron_ip = network_value("heron_ip", package_root=package_root)
        master_uri = "http://{}:11311".format(heron_ip)
        local_ip = network_value("heron_local_ip", package_root=package_root)
    else:
        raise RuntimeError("unsupported ROS profile: {}".format(profile))

    parsed = urlparse(str(master_uri or ""))
    if (
        parsed.scheme != "http"
        or not parsed.hostname
        or parsed.username
        or parsed.password
        or parsed.query
        or parsed.fragment
        or parsed.path not in ("", "/")
    ):
        raise RuntimeError("invalid ROS master URI for {} profile".format(selected))
    try:
        port = parsed.port
    except ValueError as exc:
        raise RuntimeError(
            "invalid ROS master port for {} profile".format(selected)
        ) from exc
    if port is None or port <= 0 or port > 65535:
        raise RuntimeError("invalid ROS master port for {} profile".format(selected))
    try:
        advertised_ip = ipaddress.ip_address(str(local_ip or ""))
    except ValueError as exc:
        raise RuntimeError("invalid ROS_IP for {} profile".format(selected)) from exc
    if (
        advertised_ip.version != 4
        or advertised_ip.is_unspecified
        or advertised_ip.is_loopback
        or advertised_ip.is_multicast
    ):
        raise RuntimeError("unsafe ROS_IP for {} profile".format(selected))
    if selected == PROFILE_STANDALONE and (
        parsed.hostname != "127.0.0.1" or port != 11311
    ):
        raise RuntimeError("standalone profile requires http://127.0.0.1:11311")
    return str(master_uri), str(advertised_ip)


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
    parser.add_argument("--sensor-contract-file", default="")
    parser.add_argument(
        "--profile",
        choices=SUPPORTED_PROFILES,
        default=PROFILE_INTEGRATED,
        help=(
            "fixed ROS graph profile; changing profiles requires restarting the "
            "provider and the rest of the sensing stack"
        ),
    )
    parser.add_argument("--startup-grace-sec", type=float, default=15.0)
    parser.add_argument("--topic-timeout-sec", type=float, default=2.0)
    parser.add_argument("--stamp-timeout-sec", type=float, default=0.5)
    parser.add_argument("--future-tolerance-sec", type=float, default=0.05)
    args = parser.parse_args(rospy.myargv()[1:])

    package_root = rospkg.RosPack().get_path("ig_handle")
    contract = load_contract(package_root, args.sensor_contract_file)
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

    master_uri, local_ip = _profile_network(package_root, args.profile)
    os.environ["ROS_MASTER_URI"] = master_uri
    os.environ["ROS_IP"] = local_ip
    os.environ.pop("ROS_HOSTNAME", None)

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
    managed_identities: Dict[int, int] = {}
    managed_contexts: Dict[int, ProcessContext] = {}
    _refresh_tracked_identities(proc, managed_identities, managed_contexts)
    started = time.monotonic()
    stopping = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    fault = ""
    try:
        if (
            int(proc.pid) not in managed_identities
            or int(proc.pid) not in managed_contexts
        ):
            raise RuntimeError("could not establish external provider process identity")
        while not stopping:
            _refresh_tracked_identities(proc, managed_identities, managed_contexts)
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
            unexpected_publishers = publishers - {expected_publisher}
            if unexpected_publishers:
                fault = "unexpected_publishers:{}".format(sorted(unexpected_publishers))
                break
            if expected_publisher in publishers:
                try:
                    publisher_pid = _local_node_pid(
                        master_uri, expected_publisher, local_ip
                    )
                except Exception:
                    fault = "publisher_identity_unavailable"
                    break
                if not _publisher_process_owned(
                    publisher_pid,
                    proc,
                    managed_identities,
                    managed_contexts,
                ):
                    fault = "publisher_process_mismatch:{}".format(publisher_pid)
                    break
            if monitor.fault:
                fault = monitor.fault
                break
            if time.monotonic() - started > args.startup_grace_sec:
                if publishers != {expected_publisher}:
                    fault = "publisher_ownership:{}".format(sorted(publishers))
                    break
                if (
                    time.monotonic() - monitor.last_message_wall
                    > args.topic_timeout_sec
                ):
                    fault = "topic_stale:{}".format(topic)
                    break
            time.sleep(0.5)
    finally:
        _stop_process_group(proc, managed_identities, managed_contexts)
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
