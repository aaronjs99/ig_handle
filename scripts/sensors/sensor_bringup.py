#!/usr/bin/env python3
"""Launch and supervise enabled sensors from the IG Handle sensor contract."""

from __future__ import annotations

import concurrent.futures
import fcntl
import json
import math
import os
import pwd
import signal
import socket
import stat
import subprocess
import threading
import time
import xmlrpc.client
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import urlparse

import rospy
import rospkg
import rosgraph
from std_msgs.msg import String

from sensors.network import network_value
from sensors.process_identity import read_process_stat
from sensors.contracts import (
    load_contract,
    sensor_reachable,
    sensor_requested,
    sensor_value,
)


HOST_BRINGUP_LOCK = Path("/run/lock/ig-handle-sensor-bringup.lock")
BRINGUP_OWNER_ACCOUNT = "ig-handle"


def _acquire_bringup_lock(lock_path: Path = HOST_BRINGUP_LOCK):
    """Hold the one host-wide camera/LiDAR bringup ownership lease."""

    path = Path(lock_path)
    flags = os.O_CREAT | os.O_RDWR | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(str(path), flags, 0o600)
    handle = os.fdopen(descriptor, "r+")
    try:
        try:
            expected_uid = int(pwd.getpwnam(BRINGUP_OWNER_ACCOUNT).pw_uid)
        except KeyError:
            expected_uid = os.getuid()
        lock_stat = os.fstat(handle.fileno())
        if os.geteuid() == 0 and lock_stat.st_uid == 0 and expected_uid != 0:
            os.fchown(handle.fileno(), expected_uid, -1)
            lock_stat = os.fstat(handle.fileno())
        if (
            not stat.S_ISREG(lock_stat.st_mode)
            or lock_stat.st_uid != expected_uid
            or lock_stat.st_nlink != 1
        ):
            raise RuntimeError("sensor bringup lock path is not a private file")
        os.fchmod(handle.fileno(), 0o600)
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        handle.close()
        raise RuntimeError(
            "another standalone or integrated sensor bringup owns this host"
        ) from exc
    except Exception:
        handle.close()
        raise
    handle.seek(0)
    handle.truncate()
    handle.write(
        "pid={} master={} ros_ip={}\n".format(
            os.getpid(),
            os.environ.get("ROS_MASTER_URI", ""),
            os.environ.get("ROS_IP", ""),
        )
    )
    handle.flush()
    return handle


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


class _TimeoutXmlRpcTransport(xmlrpc.client.Transport):
    """XML-RPC transport with a per-connection timeout."""

    def __init__(self, timeout_sec: float) -> None:
        super().__init__()
        self.timeout_sec = float(timeout_sec)

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout_sec
        return connection


class SensorBringup:
    def __init__(self) -> None:
        self.package_root = rospkg.RosPack().get_path("ig_handle")
        self.contract_file = str(rospy.get_param("~sensor_contract_file", ""))
        self.extra_sensor_ids = str(rospy.get_param("~extra_sensor_ids", ""))
        self.disabled_sensor_ids = str(rospy.get_param("~disabled_sensor_ids", ""))
        self.reachability_check = self._param_bool("~sensor_reachability_check", True)
        self.processes: Dict[str, subprocess.Popen] = {}
        self.sensor_selection: Dict[str, Dict[str, bool]] = {}
        self.process_started: Dict[str, float] = {}
        self.topic_last_message: Dict[str, float] = {}
        self.topic_subscribers = {}
        self._stopping = threading.Event()
        self._lifecycle_lock = threading.RLock()
        self.restart_count: Dict[str, int] = {}
        self.failure_count: Dict[str, int] = {}
        self.publisher_anomaly_count: Dict[str, int] = {}
        self.publisher_ownership: Dict[str, Dict[str, Any]] = {}
        self.managed_process_groups: Dict[str, set] = {}
        self.managed_process_identities: Dict[str, Dict[int, int]] = {}
        self.stop_failure_count: Dict[str, int] = {}
        self.next_retry: Dict[str, float] = {}
        self.healthy_since: Dict[str, float] = {}
        self.current_fault: Dict[str, str] = {}
        self.last_fault: Dict[str, str] = {}
        self.last_fault_time: Dict[str, float] = {}
        self.last_transition: Dict[str, str] = {}
        self.last_transition_time: Dict[str, float] = {}
        self.reachability_failures: Dict[str, int] = {}
        self.quarantine_reason: Dict[str, str] = {}
        self.last_reachability_probe = float("-inf")
        self.master_uri = rosgraph.get_master_uri()
        self.xmlrpc_timeout_sec = float(rospy.get_param("~xmlrpc_timeout_sec", 1.0))
        self.health_timeout_sec = float(rospy.get_param("~topic_timeout_sec", 5.0))
        self.restart_cooldown_sec = float(
            rospy.get_param("~restart_cooldown_sec", 10.0)
        )
        self.restart_backoff_max_sec = float(
            rospy.get_param("~restart_backoff_max_sec", 120.0)
        )
        self.restart_backoff_reset_sec = float(
            rospy.get_param("~restart_backoff_reset_sec", 60.0)
        )
        self.reachability_probe_period_sec = float(
            rospy.get_param("~reachability_probe_period_sec", 2.0)
        )
        self.reachability_failure_threshold = int(
            rospy.get_param("~reachability_failure_threshold", 3)
        )
        self.publisher_anomaly_threshold = int(
            rospy.get_param("~publisher_anomaly_threshold", 3)
        )
        for name, value in (
            ("topic_timeout_sec", self.health_timeout_sec),
            ("restart_cooldown_sec", self.restart_cooldown_sec),
            ("restart_backoff_max_sec", self.restart_backoff_max_sec),
            ("restart_backoff_reset_sec", self.restart_backoff_reset_sec),
            ("reachability_probe_period_sec", self.reachability_probe_period_sec),
            ("xmlrpc_timeout_sec", self.xmlrpc_timeout_sec),
        ):
            if not math.isfinite(value) or value <= 0.0 or value > 3600.0:
                raise ValueError(f"{name} must be finite and in (0, 3600]")
        if self.restart_backoff_max_sec < self.restart_cooldown_sec:
            raise ValueError(
                "restart_backoff_max_sec must be at least restart_cooldown_sec"
            )
        if self.reachability_failure_threshold < 1:
            raise ValueError("reachability_failure_threshold must be at least one")
        if self.publisher_anomaly_threshold < 1:
            raise ValueError("publisher_anomaly_threshold must be at least one")
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
            self.sensor_selection[sensor_id] = {
                "requested": bool(enabled),
                "reachable": False,
                "probe_reachable": False,
                "lifecycle_owner": self._lifecycle_owner(sensor),
            }
            self.restart_count[sensor_id] = 0
            self.failure_count[sensor_id] = 0
            self.next_retry[sensor_id] = 0.0
            self.publisher_anomaly_count[sensor_id] = 0
            self.managed_process_groups[sensor_id] = set()
            self.managed_process_identities[sensor_id] = {}
            self.stop_failure_count[sensor_id] = 0
            self.current_fault[sensor_id] = ""
            self.last_fault[sensor_id] = ""
            self.reachability_failures[sensor_id] = 0
            self._transition(sensor_id, "initializing")
            self._set_runtime_param(
                f"/ig_handle/sensors/{sensor_id}/requested", bool(enabled)
            )
            lifecycle_owner = self._lifecycle_owner(sensor)
            self._startup_grace_sec(sensor)
            if lifecycle_owner == "external_service":
                if not str(sensor.get("expected_publisher", "") or "").strip():
                    raise RuntimeError(
                        "externally owned sensor {} requires expected_publisher".format(
                            sensor_id
                        )
                    )
            elif enabled:
                self._launch_args(sensor_id, sensor)
            if enabled:
                self._subscribe_sensor_topics(sensor_id, sensor)
        publisher_graph = self._publisher_graph()
        self._refresh_reachability(sensors, force=True)
        self._reconcile(sensors, publisher_graph)
        self._publish_health(publisher_graph)

    def spin(self, master_lease) -> None:
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown() and not self._stopping.is_set():
            master_lease.check()
            sensors = dict(self.contract.get("sensors", {}) or {})
            publisher_graph = self._publisher_graph()
            self._refresh_reachability(sensors)
            self._reconcile(sensors, publisher_graph)
            self._publish_health(publisher_graph)
            rate.sleep()

    def shutdown(self) -> None:
        with self._lifecycle_lock:
            if self._stopping.is_set():
                return
            self._stopping.set()
            processes = dict(self.processes)
            known_process_identities = {
                sensor_id: dict(self.managed_process_identities.get(sensor_id, {}))
                for sensor_id in processes
            }
        unreaped = self._stop_and_reap_processes(
            processes, known_process_identities=known_process_identities
        )
        with self._lifecycle_lock:
            for sensor_id, proc in processes.items():
                if sensor_id not in unreaped and self.processes.get(sensor_id) is proc:
                    self.processes.pop(sensor_id, None)
        for sensor_id in sorted(unreaped):
            rospy.logerr(
                "sensor_bringup could not reap id=%s during bounded shutdown",
                sensor_id,
            )

    def _transition(self, sensor_id: str, state: str, fault: str = "") -> None:
        previous = self.last_transition.get(sensor_id)
        previous_fault = self.current_fault.get(sensor_id, "")
        if previous != state or previous_fault != fault:
            self.last_transition[sensor_id] = state
            self.last_transition_time[sensor_id] = time.monotonic()
        self.current_fault[sensor_id] = fault
        if fault:
            self.last_fault[sensor_id] = fault
            self.last_fault_time[sensor_id] = time.monotonic()

    def _set_reachability(self, sensor_id: str, reachable: bool) -> None:
        selection = self.sensor_selection[sensor_id]
        previous = bool(selection.get("reachable", False))
        selection["reachable"] = bool(reachable)
        self._set_runtime_param(
            f"/ig_handle/sensors/{sensor_id}/reachable", bool(reachable)
        )
        self._set_runtime_param(
            f"/ig_handle/sensors/{sensor_id}/enabled", bool(reachable)
        )
        if previous == bool(reachable):
            if self.last_transition.get(sensor_id) == "initializing":
                self._transition(
                    sensor_id,
                    "reachable" if reachable else "unreachable",
                    "" if reachable else "unreachable_at_startup",
                )
            return
        if reachable:
            self._transition(sensor_id, "reachable")
            rospy.loginfo("sensor_bringup reachability restored id=%s", sensor_id)
        else:
            self._transition(sensor_id, "unreachable", "reachability_lost")
            rospy.logwarn("sensor_bringup reachability lost id=%s", sensor_id)

    @staticmethod
    def _runtime_evidence_reachable(
        probe_reachable: bool,
        topics_fresh: bool,
        process_running: bool,
        externally_owned: bool,
    ) -> bool:
        return bool(
            probe_reachable or (topics_fresh and (process_running or externally_owned))
        )

    def _refresh_reachability(
        self, sensors: Dict[str, Any], force: bool = False
    ) -> None:
        if self._stopping.is_set() or rospy.is_shutdown():
            return
        now = time.monotonic()
        if (
            not force
            and now - self.last_reachability_probe < self.reachability_probe_period_sec
        ):
            return
        self.last_reachability_probe = now
        requested = [
            sensor_id
            for sensor_id in _sensor_order(sensors)
            if sensor_id in self.sensor_selection
            and bool(self.sensor_selection[sensor_id].get("requested", False))
        ]
        results: Dict[str, bool] = {}
        if requested:
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=min(8, len(requested))
            ) as executor:
                futures = {
                    executor.submit(
                        sensor_reachable,
                        self.contract,
                        self.package_root,
                        sensor_id,
                        self.reachability_check,
                    ): sensor_id
                    for sensor_id in requested
                }
                for future, sensor_id in futures.items():
                    try:
                        results[sensor_id] = bool(future.result())
                    except Exception as exc:
                        rospy.logerr_throttle(
                            5.0,
                            "sensor_bringup reachability probe failed id=%s error=%s",
                            sensor_id,
                            exc,
                        )
                        results[sensor_id] = False
        for sensor_id, selection in self.sensor_selection.items():
            if not bool(selection.get("requested", False)):
                self.reachability_failures[sensor_id] = 0
                self._set_reachability(sensor_id, False)
                self._transition(sensor_id, "not_requested")
                continue
            observed = bool(results.get(sensor_id, False))
            selection["probe_reachable"] = observed
            if observed:
                self.reachability_failures[sensor_id] = 0
                self._set_reachability(sensor_id, True)
                continue
            failures = self.reachability_failures.get(sensor_id, 0) + 1
            self.reachability_failures[sensor_id] = failures
            currently_reachable = bool(selection.get("reachable", False))
            with self._lifecycle_lock:
                proc = self.processes.get(sensor_id)
            proc_running = proc is not None and proc.poll() is None
            sensor = dict(sensors.get(sensor_id) or {})
            topics_fresh = self._sensor_topics_fresh(sensor)
            if self._runtime_evidence_reachable(
                observed,
                topics_fresh,
                proc_running,
                selection.get("lifecycle_owner") == "external_service",
            ):
                self._set_reachability(sensor_id, True)
                self._transition(
                    sensor_id,
                    "reachability_degraded",
                    "probe_failed_topics_fresh",
                )
                continue
            if force or not currently_reachable:
                self._set_reachability(sensor_id, False)
            elif failures >= self.reachability_failure_threshold:
                self._set_reachability(sensor_id, False)

    def _observed_publishers(
        self, sensor: Dict[str, Any], publisher_graph
    ) -> Dict[str, set]:
        return {
            topic: set((publisher_graph or {}).get(topic, set()))
            for topic in self._required_topics(sensor)
        }

    def _publisher_anomaly_reason(self, sensor: Dict[str, Any], publisher_graph) -> str:
        if publisher_graph is None:
            return ""
        expected = str(sensor.get("expected_publisher", "") or "").strip()
        missing = []
        for topic, observed in self._observed_publishers(
            sensor, publisher_graph
        ).items():
            if expected:
                unexpected = observed - {expected}
                if unexpected:
                    return "publisher_unexpected:{}:{}".format(
                        topic, ",".join(sorted(unexpected))
                    )
                if expected not in observed:
                    missing.append(topic)
                    continue
            if not expected and len(observed) > 1:
                return "publisher_conflict:{}:{}".format(
                    topic, ",".join(sorted(observed))
                )
        if expected and missing:
            return "publisher_missing:{}:{}".format(missing[0], expected)
        return ""

    def _publisher_present_reason(self, sensor: Dict[str, Any], publisher_graph) -> str:
        if publisher_graph is None:
            return ""
        expected = str(sensor.get("expected_publisher", "") or "").strip()
        for topic, observed in self._observed_publishers(
            sensor, publisher_graph
        ).items():
            if expected in observed:
                try:
                    node_uri = str(
                        self._bounded_xmlrpc_call(
                            self.master_uri, "lookupNode", expected
                        )
                    )
                except Exception:
                    pass
                else:
                    if self._node_uri_is_local(node_uri):
                        try:
                            self._bounded_xmlrpc_call(node_uri, "getPid")
                        except ConnectionRefusedError:
                            # An interrupted run can leave its name in the
                            # master after the local node has exited. A fresh
                            # launch replaces that dead registration.
                            observed.discard(expected)
                        except Exception:
                            pass
            if observed:
                return "publisher_present_without_managed_process:{}:{}".format(
                    topic, ",".join(sorted(observed))
                )
        return ""

    @staticmethod
    def _node_uri_is_local(node_uri: str) -> bool:
        parsed_uri = urlparse(str(node_uri or ""))
        host = str(parsed_uri.hostname or "").strip().lower()
        if not host:
            return False
        local_names = {
            "localhost",
            "127.0.0.1",
            "::1",
            str(socket.gethostname() or "").strip().lower(),
            str(socket.getfqdn() or "").strip().lower(),
            str(os.environ.get("ROS_IP", "") or "").strip().lower(),
            str(os.environ.get("ROS_HOSTNAME", "") or "").strip().lower(),
        }
        local_names.discard("")
        if host in local_names:
            return True

        def resolved_addresses(name: str) -> set:
            try:
                return {
                    str(entry[4][0]).split("%", 1)[0]
                    for entry in socket.getaddrinfo(name, None)
                }
            except OSError:
                return set()

        host_addresses = resolved_addresses(host)
        local_addresses = set()
        for name in local_names:
            local_addresses.update(resolved_addresses(name))
        if host_addresses and host_addresses.intersection(local_addresses):
            return True

        # A UDP connect performs route selection without transmitting a packet.
        # If the selected source address equals the node address, that address
        # belongs to this host even when ROS_IP is not exported interactively.
        try:
            candidates = socket.getaddrinfo(
                host,
                parsed_uri.port or 1,
                type=socket.SOCK_DGRAM,
            )
        except OSError:
            return False
        for family, socktype, protocol, _canonical_name, address in candidates:
            try:
                with socket.socket(family, socktype, protocol) as route_socket:
                    route_socket.connect(address)
                    local_address = str(route_socket.getsockname()[0]).split("%", 1)[0]
                    node_address = str(address[0]).split("%", 1)[0]
                    if local_address == node_address:
                        return True
            except OSError:
                continue
        return False

    def _bounded_xmlrpc_call(self, uri: str, method_name: str, *args: Any) -> Any:
        """Call one ROS XML-RPC method with a bounded network timeout."""
        transport = _TimeoutXmlRpcTransport(timeout_sec=self.xmlrpc_timeout_sec)
        with xmlrpc.client.ServerProxy(
            str(uri), allow_none=True, transport=transport
        ) as endpoint:
            response = getattr(endpoint, str(method_name))(rospy.get_name(), *args)
        if not isinstance(response, (list, tuple)) or len(response) != 3:
            raise RuntimeError(
                "{} returned an invalid ROS XML-RPC response".format(method_name)
            )
        code, message, value = response
        if int(code) != 1:
            raise RuntimeError("{} failed: {}".format(method_name, message))
        return value

    def _set_runtime_param(self, name: str, value: Any) -> bool:
        """Best-effort diagnostic parameter update without an unbounded master call."""
        try:
            self._bounded_xmlrpc_call(self.master_uri, "setParam", name, value)
        except Exception as exc:
            rospy.logerr_throttle(
                5.0,
                "sensor_bringup could not update parameter %s: %s",
                name,
                exc,
            )
            return False
        return True

    def _lookup_publisher_pid(self, publisher_name: str) -> int:
        node_uri = str(
            self._bounded_xmlrpc_call(
                self.master_uri, "lookupNode", str(publisher_name)
            )
            or ""
        ).strip()
        if not node_uri:
            raise RuntimeError("publisher node URI is unavailable")
        if not self._node_uri_is_local(node_uri):
            raise RuntimeError(
                "publisher node is not local: {}".format(
                    str(urlparse(node_uri).hostname or "unknown")
                )
            )
        publisher_pid = int(self._bounded_xmlrpc_call(node_uri, "getPid"))
        if publisher_pid <= 0:
            raise RuntimeError("publisher getPid returned an invalid PID")
        return publisher_pid

    def _managed_publisher_evidence(
        self,
        sensor_id: str,
        sensor: Dict[str, Any],
        publisher_graph,
        proc: subprocess.Popen,
    ) -> Dict[str, Any]:
        evidence: Dict[str, Any] = {
            "verified": False,
            "reason": "",
            "publisher_pid": None,
            "publisher_process_group_id": None,
            "managed_root_pid": int(proc.pid),
        }
        expected = str(sensor.get("expected_publisher", "") or "").strip()
        if not expected:
            evidence["reason"] = "expected_publisher_undeclared"
            return evidence
        if publisher_graph is None:
            evidence["reason"] = "publisher_graph_unavailable"
            return evidence
        name_reason = self._publisher_anomaly_reason(sensor, publisher_graph)
        if name_reason:
            evidence["reason"] = name_reason
            return evidence
        try:
            publisher_pid = self._lookup_publisher_pid(expected)
            publisher_stat = read_process_stat(publisher_pid)
            publisher_process_group_id = publisher_stat["process_group_id"]
        except Exception as exc:
            evidence["reason"] = "publisher_identity_unavailable:{}:{}".format(
                expected, exc
            )
            return evidence
        evidence["publisher_pid"] = publisher_pid
        evidence["publisher_process_group_id"] = publisher_process_group_id
        tree = self._remember_managed_tree(sensor_id, proc)
        if (
            publisher_pid not in tree
            or tree[publisher_pid][0] != publisher_stat["start_time_ticks"]
        ):
            evidence["reason"] = (
                "publisher_process_mismatch:{}:pid={}:pgid={}:managed_root={}".format(
                    expected,
                    publisher_pid,
                    publisher_process_group_id,
                    proc.pid,
                )
            )
            return evidence
        evidence["verified"] = True
        return evidence

    def _reconcile(self, sensors: Dict[str, Any], publisher_graph=None) -> None:
        if self._stopping.is_set() or rospy.is_shutdown():
            return
        now = time.monotonic()
        for sensor_id in _sensor_order(sensors):
            if self._stopping.is_set() or rospy.is_shutdown():
                return
            if sensor_id not in self.sensor_selection:
                continue
            sensor = dict(sensors.get(sensor_id) or {})
            selection = self.sensor_selection[sensor_id]
            requested = bool(selection.get("requested", False))
            reachable = bool(selection.get("reachable", False))
            lifecycle_owner = str(selection.get("lifecycle_owner", ""))
            if lifecycle_owner == "external_service":
                continue
            with self._lifecycle_lock:
                proc = self.processes.get(sensor_id)

            quarantine = self.quarantine_reason.get(sensor_id, "")
            if quarantine:
                if quarantine.startswith("stop_failed:"):
                    if proc is None:
                        self.quarantine_reason.pop(sensor_id, None)
                        self.stop_failure_count[sensor_id] = 0
                        quarantine = ""
                    elif now >= self.next_retry.get(sensor_id, 0.0):
                        original_reason = quarantine.split(":", 1)[1]
                        if self._stop_sensor(sensor_id, sensor, original_reason):
                            self.quarantine_reason.pop(sensor_id, None)
                            if self._publisher_fault_requires_quarantine(
                                original_reason
                            ):
                                self.quarantine_reason[sensor_id] = original_reason
                                self._transition(
                                    sensor_id, "quarantined", original_reason
                                )
                            elif requested:
                                self._enter_restart_backoff(sensor_id, original_reason)
                            else:
                                self._transition(sensor_id, "not_requested")
                        continue
                    else:
                        self._transition(sensor_id, "quarantined", quarantine)
                        continue
                if quarantine and quarantine.startswith("publisher_") and proc is None:
                    conflict = self._publisher_present_reason(sensor, publisher_graph)
                    if publisher_graph is not None and not conflict:
                        self.quarantine_reason.pop(sensor_id, None)
                        self.publisher_anomaly_count[sensor_id] = 0
                        self._transition(sensor_id, "quarantine_cleared")
                    else:
                        self._transition(
                            sensor_id, "quarantined", conflict or quarantine
                        )
                        continue
                elif quarantine:
                    self._transition(sensor_id, "quarantined", quarantine)
                    continue

            if not requested:
                if proc is not None:
                    self._stop_sensor(sensor_id, sensor, "not_requested")
                continue
            if not reachable:
                if proc is not None:
                    self._schedule_retry(sensor_id, sensor, "reachability_lost")
                continue
            if proc is None:
                self.publisher_ownership.pop(sensor_id, None)
                if not str(sensor.get("expected_publisher", "") or "").strip():
                    reason = "expected_publisher_undeclared"
                    self.quarantine_reason[sensor_id] = reason
                    self._transition(sensor_id, "quarantined", reason)
                    continue
                if publisher_graph is None:
                    self._transition(
                        sensor_id,
                        "publisher_graph_unavailable",
                        "publisher_graph_unavailable",
                    )
                    continue
                conflict = self._publisher_present_reason(sensor, publisher_graph)
                if conflict:
                    self.quarantine_reason[sensor_id] = conflict
                    self._transition(sensor_id, "quarantined", conflict)
                    continue
                if now >= self.next_retry.get(sensor_id, 0.0):
                    if self._launch_sensor(sensor_id, sensor):
                        self._transition(sensor_id, "starting")
                continue
            returncode = proc.poll()
            if returncode is None:
                # Capture the complete live launch tree even before a provider
                # registers with the ROS master. Cleanup must not depend on a
                # healthy topic appearing first.
                self._remember_managed_tree(sensor_id, proc)
            reason = (
                "child_exited:{}".format(returncode)
                if returncode is not None
                else self._sensor_unhealthy_reason(sensor_id, sensor)
            )
            if reason:
                self.publisher_ownership.pop(sensor_id, None)
                self._schedule_retry(sensor_id, sensor, reason)
                continue
            publisher_evidence = self._managed_publisher_evidence(
                sensor_id, sensor, publisher_graph, proc
            )
            self.publisher_ownership[sensor_id] = publisher_evidence
            if publisher_graph is None:
                self._transition(
                    sensor_id,
                    "publisher_graph_unavailable",
                    "publisher_graph_unavailable",
                )
                continue
            elapsed = time.monotonic() - self.process_started.get(sensor_id, now)
            in_startup_grace = elapsed <= self._startup_grace_sec(sensor)
            publisher_reason = str(publisher_evidence.get("reason", "") or "")
            if in_startup_grace and publisher_reason.startswith("publisher_missing:"):
                # Absence is normal while a provider registers. Any observed
                # unexpected publisher or unverifiable owner is not normal and
                # is counted during startup just as it is during steady state.
                self._transition(sensor_id, "starting")
                continue
            if publisher_reason:
                anomalies = self.publisher_anomaly_count.get(sensor_id, 0) + 1
                self.publisher_anomaly_count[sensor_id] = anomalies
                if anomalies >= self.publisher_anomaly_threshold:
                    if self._stop_sensor(sensor_id, sensor, publisher_reason):
                        if self._publisher_fault_requires_quarantine(publisher_reason):
                            self.quarantine_reason[sensor_id] = publisher_reason
                            self._transition(sensor_id, "quarantined", publisher_reason)
                        else:
                            self._enter_restart_backoff(sensor_id, publisher_reason)
                else:
                    self._transition(sensor_id, "publisher_degraded", publisher_reason)
                continue
            self.publisher_anomaly_count[sensor_id] = 0
            if self._sensor_topics_fresh(sensor):
                healthy_since = self.healthy_since.setdefault(sensor_id, now)
                if now - healthy_since >= self.restart_backoff_reset_sec:
                    self.failure_count[sensor_id] = 0
                    self.next_retry[sensor_id] = 0.0
                self._transition(sensor_id, "running")

    @staticmethod
    def _publisher_fault_requires_quarantine(reason: str) -> bool:
        return str(reason or "").startswith(
            (
                "publisher_unexpected:",
                "publisher_conflict:",
                "publisher_process_mismatch:",
                "publisher_present_without_managed_process:",
            )
        )

    def _clear_sensor_observations(self, sensor: Dict[str, Any]) -> None:
        for topic in self._required_topics(sensor):
            self.topic_last_message.pop(topic, None)

    def _stop_sensor(self, sensor_id: str, sensor: Dict[str, Any], reason: str) -> bool:
        with self._lifecycle_lock:
            proc = self.processes.get(sensor_id)
            known_process_identities = dict(
                self.managed_process_identities.get(sensor_id, {})
            )
        if proc is None:
            return True
        if not self._stop_and_reap_process(proc, known_process_identities):
            rospy.logerr("sensor_bringup did not reap id=%s", sensor_id)
            stop_failures = self.stop_failure_count.get(sensor_id, 0) + 1
            self.stop_failure_count[sensor_id] = stop_failures
            self.quarantine_reason[sensor_id] = "stop_failed:{}".format(reason)
            delay = self._backoff_delay(stop_failures)
            self.next_retry[sensor_id] = time.monotonic() + delay
            self._transition(sensor_id, "stop_failed", "stop_failed:{}".format(reason))
            return False
        with self._lifecycle_lock:
            if self.processes.get(sensor_id) is proc:
                self.processes.pop(sensor_id, None)
        self.process_started.pop(sensor_id, None)
        self.healthy_since.pop(sensor_id, None)
        self.publisher_ownership.pop(sensor_id, None)
        self.managed_process_groups[sensor_id] = set()
        self.managed_process_identities[sensor_id] = {}
        self.stop_failure_count[sensor_id] = 0
        self._clear_sensor_observations(sensor)
        self._transition(sensor_id, "stopped", reason)
        return True

    def _schedule_retry(
        self, sensor_id: str, sensor: Dict[str, Any], reason: str
    ) -> None:
        if not self._stop_sensor(sensor_id, sensor, reason):
            return
        self._enter_restart_backoff(sensor_id, reason)

    def _enter_restart_backoff(self, sensor_id: str, reason: str) -> None:
        failures = self.failure_count.get(sensor_id, 0) + 1
        self.failure_count[sensor_id] = failures
        self.restart_count[sensor_id] = self.restart_count.get(sensor_id, 0) + 1
        delay = self._backoff_delay(failures)
        self.next_retry[sensor_id] = time.monotonic() + delay
        self._transition(sensor_id, "backoff", reason)
        rospy.logwarn(
            "sensor_bringup scheduled retry id=%s delay=%.1f reason=%s",
            sensor_id,
            delay,
            reason,
        )

    def _schedule_launch_retry(self, sensor_id: str, reason: str) -> None:
        failures = self.failure_count.get(sensor_id, 0) + 1
        self.failure_count[sensor_id] = failures
        self.restart_count[sensor_id] = self.restart_count.get(sensor_id, 0) + 1
        delay = self._backoff_delay(failures)
        self.next_retry[sensor_id] = time.monotonic() + delay
        self._transition(sensor_id, "backoff", reason)
        rospy.logwarn(
            "sensor_bringup scheduled launch retry id=%s delay=%.1f reason=%s",
            sensor_id,
            delay,
            reason,
        )

    def _backoff_delay(self, failures: int) -> float:
        ratio = self.restart_backoff_max_sec / self.restart_cooldown_sec
        maximum_exponent = (
            max(0, int(math.ceil(math.log(ratio, 2.0)))) if ratio > 1.0 else 0
        )
        exponent = min(max(0, int(failures) - 1), maximum_exponent)
        return min(
            self.restart_cooldown_sec * (2**exponent),
            self.restart_backoff_max_sec,
        )

    def _launch_args(self, sensor_id: str, sensor: Dict[str, Any]) -> List[str]:
        launch = dict(sensor.get("launch", {}) or {})
        launch_file = _resolve_package_uri(self.package_root, launch.get("file", ""))
        if not launch_file:
            raise RuntimeError(f"sensor {sensor_id} launch stanza has no file")
        if not Path(launch_file).is_file():
            raise RuntimeError(
                f"sensor {sensor_id} launch file does not exist: {launch_file}"
            )
        # ROS core is owned outside every sensor provider. A launch racing a
        # master outage must wait or fail, never create a private replacement.
        args = ["roslaunch", "--wait", launch_file]
        for name, spec in dict(launch.get("args", {}) or {}).items():
            args.append(f"{name}:={self._arg_value(sensor_id, spec)}")
        return args

    def _launch_sensor(self, sensor_id: str, sensor: Dict[str, Any]) -> bool:
        if self._lifecycle_owner(sensor) != "sensor_bringup":
            raise RuntimeError(
                "sensor_bringup cannot launch externally owned sensor {}".format(
                    sensor_id
                )
            )
        if self._stopping.is_set() or rospy.is_shutdown():
            return False
        try:
            args = self._launch_args(sensor_id, sensor)
            launch_file = next(
                (value for value in args[1:] if not str(value).startswith("-")),
                "unknown",
            )
            rospy.loginfo(
                "sensor_bringup launching id=%s file=%s", sensor_id, launch_file
            )
            with self._lifecycle_lock:
                if self._stopping.is_set() or rospy.is_shutdown():
                    return False
                if sensor_id in self.processes:
                    return False
                proc = subprocess.Popen(args, start_new_session=True)
                self.processes[sensor_id] = proc
                try:
                    root_stat = read_process_stat(proc.pid)
                    self.managed_process_identities[sensor_id] = {
                        proc.pid: root_stat["start_time_ticks"]
                    }
                    self.managed_process_groups[sensor_id] = {
                        root_stat["process_group_id"]
                    }
                except (
                    FileNotFoundError,
                    PermissionError,
                    ProcessLookupError,
                    RuntimeError,
                    ValueError,
                ):
                    self.managed_process_identities[sensor_id] = {}
                    self.managed_process_groups[sensor_id] = set()
                self.process_started[sensor_id] = time.monotonic()
                self.healthy_since.pop(sensor_id, None)
                self.next_retry[sensor_id] = 0.0
        except Exception as exc:
            self._schedule_launch_retry(sensor_id, "launch_failed:{}".format(exc))
            return False
        return True

    def _required_topics(self, sensor: Dict[str, Any]) -> List[str]:
        topics = dict(sensor.get("topics", {}) or {})
        required = dict(sensor.get("required_topics", {}) or {})
        return [
            str(topic)
            for name, topic in topics.items()
            if topic and required.get(name, True)
        ]

    @staticmethod
    def _configured_topics(sensor: Dict[str, Any]) -> List[str]:
        return list(
            dict.fromkeys(
                str(topic)
                for topic in dict(sensor.get("topics", {}) or {}).values()
                if topic
            )
        )

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

    @classmethod
    def _identity_matches(cls, pid: int, start_time_ticks: int) -> bool:
        try:
            stat = read_process_stat(pid)
            return (
                stat["start_time_ticks"] == int(start_time_ticks)
                and stat.get("state") != "Z"
            )
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            RuntimeError,
            ValueError,
        ):
            return False

    @classmethod
    def _pid_descends_from(
        cls,
        pid: int,
        root_pid: int,
        root_start_time_ticks: Optional[int] = None,
    ) -> bool:
        current = int(pid)
        root = int(root_pid)
        visited = set()
        while current > 0 and current not in visited:
            try:
                stat = read_process_stat(current)
            except (
                FileNotFoundError,
                PermissionError,
                ProcessLookupError,
                RuntimeError,
                ValueError,
            ):
                return False
            if current == root:
                return root_start_time_ticks is None or stat["start_time_ticks"] == int(
                    root_start_time_ticks
                )
            visited.add(current)
            current = stat["parent_pid"]
        return False

    @classmethod
    def _managed_tree(
        cls, root_pid: int, root_start_time_ticks: int
    ) -> Dict[int, Tuple[int, int]]:
        """Return current descendants as PID -> (start ticks, tree depth)."""
        try:
            root_stat = read_process_stat(root_pid)
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            RuntimeError,
            ValueError,
        ):
            return {}
        if root_stat["start_time_ticks"] != int(root_start_time_ticks):
            return {}
        tree: Dict[int, Tuple[int, int]] = {
            int(root_pid): (int(root_start_time_ticks), 0)
        }
        pending = [(int(root_pid), 0)]
        while pending:
            parent, depth = pending.pop()
            children = set()
            try:
                task_directories = list(
                    (Path("/proc") / str(parent) / "task").iterdir()
                )
            except (
                FileNotFoundError,
                PermissionError,
                ProcessLookupError,
                RuntimeError,
                ValueError,
            ):
                continue
            # Linux records children against the thread that spawned them, so
            # inspecting only /task/<tgid>/children can miss roslaunch workers.
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
                    child_stat = read_process_stat(child)
                except (
                    FileNotFoundError,
                    PermissionError,
                    ProcessLookupError,
                    RuntimeError,
                    ValueError,
                ):
                    continue
                if child_stat["parent_pid"] != parent:
                    continue
                tree[child] = (child_stat["start_time_ticks"], depth + 1)
                pending.append((child, depth + 1))
        return tree

    def _remember_managed_tree(
        self, sensor_id: str, proc: subprocess.Popen
    ) -> Dict[int, Tuple[int, int]]:
        with self._lifecycle_lock:
            known = dict(self.managed_process_identities.get(sensor_id, {}))
        root_start = known.get(int(proc.pid))
        if root_start is None and proc.poll() is None:
            try:
                root_start = read_process_stat(proc.pid)["start_time_ticks"]
            except (
                FileNotFoundError,
                PermissionError,
                ProcessLookupError,
                RuntimeError,
                ValueError,
            ):
                return {}
        if root_start is None:
            return {}
        tree = self._managed_tree(proc.pid, root_start)
        if not tree:
            return {}
        groups = set()
        for pid, (start_time_ticks, _depth) in tree.items():
            try:
                stat = read_process_stat(pid)
            except (
                FileNotFoundError,
                PermissionError,
                ProcessLookupError,
                RuntimeError,
                ValueError,
            ):
                continue
            if stat["start_time_ticks"] == start_time_ticks:
                groups.add(stat["process_group_id"])
        with self._lifecycle_lock:
            if self.processes.get(sensor_id) is proc:
                identities = self.managed_process_identities.setdefault(sensor_id, {})
                identities.update(
                    {
                        pid: start_time_ticks
                        for pid, (start_time_ticks, _) in tree.items()
                    }
                )
                self.managed_process_groups[sensor_id] = groups
        return tree

    @classmethod
    def _refresh_tracked_identities(
        cls, proc: subprocess.Popen, tracked: Dict[int, int]
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
        tree = cls._managed_tree(root_pid, root_start)
        tracked.update(
            {pid: start_time_ticks for pid, (start_time_ticks, _) in tree.items()}
        )
        return tree

    @classmethod
    def _tracked_process_alive(
        cls, proc: subprocess.Popen, tracked: Dict[int, int]
    ) -> bool:
        cls._refresh_tracked_identities(proc, tracked)
        if proc.poll() is None and int(proc.pid) not in tracked:
            # The child exists, but its immutable identity could not be read.
            return True
        return any(
            cls._identity_matches(pid, start_time_ticks)
            for pid, start_time_ticks in tracked.items()
        )

    @classmethod
    def _signal_verified_tree(
        cls, proc: subprocess.Popen, tracked: Dict[int, int], sig: int
    ) -> None:
        tree = cls._refresh_tracked_identities(proc, tracked)
        root_pid = int(proc.pid)
        root_start = tracked.get(root_pid)
        if root_start is None:
            return
        # Descendants are signalled before roslaunch so their ancestry remains
        # provable at the instant each signal is sent. The root is always last.
        for pid, (start_time_ticks, depth) in sorted(
            tree.items(), key=lambda item: (item[1][1], item[0]), reverse=True
        ):
            if not cls._identity_matches(pid, start_time_ticks):
                continue
            if pid != root_pid and not cls._pid_descends_from(
                pid, root_pid, root_start
            ):
                continue
            # Re-read the immutable start tick immediately before signalling.
            if not cls._identity_matches(pid, start_time_ticks):
                continue
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except OSError:
                # Uncertain ownership is never escalated to a group-wide signal.
                pass

    @classmethod
    def _stop_and_reap_processes(
        cls,
        processes: Dict[str, subprocess.Popen],
        known_process_identities: Dict[str, Dict[int, int]] = None,
    ) -> Dict[str, subprocess.Popen]:
        remaining = dict(processes)
        tracked = {
            sensor_id: dict((known_process_identities or {}).get(sensor_id, {}))
            for sensor_id in remaining
        }

        def reap_stopped() -> None:
            for sensor_id, proc in list(remaining.items()):
                if cls._tracked_process_alive(proc, tracked[sensor_id]):
                    continue
                try:
                    proc.wait(timeout=0)
                except subprocess.TimeoutExpired:
                    continue
                remaining.pop(sensor_id, None)

        for sig, timeout_sec in (
            (signal.SIGINT, 8.0),
            (signal.SIGTERM, 2.0),
            (signal.SIGKILL, 2.0),
        ):
            reap_stopped()
            if not remaining:
                break
            for sensor_id, proc in list(remaining.items()):
                cls._signal_verified_tree(proc, tracked[sensor_id], sig)
            deadline = time.monotonic() + timeout_sec
            while remaining and time.monotonic() < deadline:
                reap_stopped()
                if remaining:
                    time.sleep(0.1)
        reap_stopped()
        return remaining

    @classmethod
    def _stop_and_reap_process(
        cls, proc: subprocess.Popen, known_process_identities: Dict[int, int] = None
    ) -> bool:
        return not cls._stop_and_reap_processes(
            {"process": proc},
            known_process_identities={"process": dict(known_process_identities or {})},
        )

    def _startup_grace_sec(self, sensor: Dict[str, Any]) -> float:
        launch = dict(sensor.get("launch", {}) or {})
        configured_delay = float(
            launch.get("args", {}).get("startup_delay_sec", {}).get("literal", 0.0)
            if isinstance(launch.get("args", {}).get("startup_delay_sec", {}), dict)
            else sensor.get("startup_delay_sec", 0.0)
        )
        configured_delay = max(
            configured_delay, float(sensor.get("startup_delay_sec", 0.0))
        )
        startup_grace_sec = float(
            sensor.get("startup_grace_sec", self.health_timeout_sec)
        )
        if (
            not math.isfinite(configured_delay)
            or configured_delay < 0.0
            or configured_delay > 3600.0
        ):
            raise ValueError("sensor startup_delay_sec must be finite and in [0, 3600]")
        if (
            not math.isfinite(startup_grace_sec)
            or startup_grace_sec <= 0.0
            or startup_grace_sec > 3600.0
        ):
            raise ValueError("sensor startup_grace_sec must be finite and in (0, 3600]")
        return configured_delay + max(self.health_timeout_sec, startup_grace_sec)

    def _sensor_topics_fresh(self, sensor: Dict[str, Any]) -> bool:
        topics = self._required_topics(sensor)
        now = time.monotonic()
        return bool(topics) and all(
            now - self.topic_last_message.get(topic, float("-inf"))
            <= self.health_timeout_sec
            for topic in topics
        )

    def _sensor_unhealthy_reason(self, sensor_id: str, sensor: Dict[str, Any]) -> str:
        if not bool(sensor.get("restart_on_missing_data", True)):
            return ""
        started = self.process_started.get(sensor_id, time.monotonic())
        if time.monotonic() - started <= self._startup_grace_sec(sensor):
            return ""
        now = time.monotonic()
        for topic in self._required_topics(sensor):
            observed = self.topic_last_message.get(topic, float("-inf"))
            if now - observed > self.health_timeout_sec:
                return "topic_stale:{}".format(topic)
        return ""

    def _publish_health(self, publisher_graph=None) -> None:
        sensors = dict(self.contract.get("sensors", {}) or {})
        now = time.monotonic()
        if publisher_graph is None and not self._stopping.is_set():
            publisher_graph = self._publisher_graph()
        status = {}
        for sensor_id, selection in self.sensor_selection.items():
            sensor = sensors.get(sensor_id, {})
            requested = bool(selection.get("requested", False))
            lifecycle_owner = str(selection.get("lifecycle_owner", ""))
            reachable = bool(selection.get("reachable", False))
            proc = self.processes.get(sensor_id)
            required_topics = self._required_topics(sensor)
            configured_topics = self._configured_topics(sensor)
            topics = required_topics if requested else []
            topic_status = {
                topic: (
                    {
                        "state": (
                            "fresh"
                            if now - observed <= self.health_timeout_sec
                            else "stale"
                        ),
                        "age_sec": round(max(0.0, now - observed), 3),
                    }
                    if (observed := self.topic_last_message.get(topic)) is not None
                    else {"state": "never_seen", "age_sec": None}
                )
                for topic in topics
            }
            expected_publisher = str(sensor.get("expected_publisher", "") or "").strip()
            observed_publishers = {
                topic: sorted((publisher_graph or {}).get(topic, set()))
                for topic in configured_topics
            }
            unexpected_publisher_while_disabled = bool(
                not requested
                and publisher_graph is not None
                and any(observed_publishers[topic] for topic in configured_topics)
            )
            topics_fresh = bool(topics) and all(
                topic_status[topic]["state"] == "fresh" for topic in topics
            )
            publisher_conflict = bool(
                publisher_graph is not None
                and any(
                    len(set(observed_publishers[topic])) > 1
                    for topic in required_topics
                )
            )
            publisher_valid = bool(
                publisher_graph is not None
                and expected_publisher
                and all(
                    set(observed_publishers[topic]) == {expected_publisher}
                    for topic in required_topics
                )
            )
            proc_running = proc is not None and proc.poll() is None
            ownership_evidence = self.publisher_ownership.get(sensor_id, {})
            if lifecycle_owner == "external_service":
                publisher_owned = publisher_valid
            else:
                publisher_owned = bool(
                    expected_publisher
                    and publisher_valid
                    and ownership_evidence.get("verified", False)
                )
            if lifecycle_owner == "external_service":
                alive = bool(reachable and topics_fresh and publisher_valid)
            else:
                alive = bool(
                    reachable
                    and proc_running
                    and topics_fresh
                    and publisher_owned
                    and not publisher_conflict
                )
            transition = self.last_transition.get(sensor_id, "")
            next_retry_sec = max(0.0, self.next_retry.get(sensor_id, 0.0) - now)
            if not requested:
                state = (
                    "not_requested_publisher_present"
                    if unexpected_publisher_while_disabled
                    else "not_requested"
                )
            elif self.quarantine_reason.get(sensor_id):
                state = "quarantined"
            elif not reachable:
                state = "unreachable"
            elif publisher_graph is None:
                state = "publisher_graph_unavailable"
            elif lifecycle_owner == "external_service" and not topics_fresh:
                state = "external_topic_stale"
            elif lifecycle_owner == "external_service" and not publisher_valid:
                state = "external_publisher_invalid"
            elif lifecycle_owner == "external_service":
                state = "external_running"
            elif not bool(selection.get("probe_reachable", False)) and topics_fresh:
                state = "reachability_degraded"
            elif publisher_conflict:
                state = "publisher_conflict"
            elif not proc_running and transition == "backoff" and next_retry_sec > 0.0:
                state = "backoff"
            elif not proc_running:
                state = "exited"
            elif not topics_fresh:
                elapsed = now - self.process_started.get(sensor_id, now)
                state = (
                    "starting"
                    if elapsed <= self._startup_grace_sec(sensor)
                    else (
                        "running_no_recent_data"
                        if not bool(sensor.get("restart_on_missing_data", True))
                        else "topic_stale"
                    )
                )
            elif not publisher_owned:
                state = "publisher_invalid"
            elif alive:
                state = "running"
            else:
                state = transition or "unknown"
            status[sensor_id] = {
                "requested": requested,
                "reachable": reachable,
                "probe_reachable": bool(selection.get("probe_reachable", False)),
                "alive": alive,
                "process_running": proc_running,
                "data_available": topics_fresh,
                "state": state,
                "lifecycle_owner": lifecycle_owner,
                "expected_publisher": expected_publisher or None,
                "observed_publishers": observed_publishers,
                "publisher_graph_available": publisher_graph is not None,
                "publisher_conflict": publisher_conflict,
                "unexpected_publisher_while_disabled": (
                    unexpected_publisher_while_disabled
                ),
                "publisher_process_owned": publisher_owned,
                "publisher_ownership_reason": (
                    ownership_evidence.get("reason") or None
                    if lifecycle_owner != "external_service"
                    else None
                ),
                "publisher_pid": (
                    ownership_evidence.get("publisher_pid")
                    if lifecycle_owner != "external_service"
                    else None
                ),
                "publisher_process_group_id": (
                    ownership_evidence.get("publisher_process_group_id")
                    if lifecycle_owner != "external_service"
                    else None
                ),
                "managed_process_group_ids": (
                    sorted(self.managed_process_groups.get(sensor_id, set()))
                    if lifecycle_owner != "external_service"
                    else []
                ),
                "managed_process_ids": (
                    sorted(self.managed_process_identities.get(sensor_id, {}))
                    if lifecycle_owner != "external_service"
                    else []
                ),
                "age_sec": (
                    round(now - self.process_started.get(sensor_id, now), 3)
                    if proc is not None
                    else None
                ),
                "restart_count": self.restart_count.get(sensor_id, 0),
                "consecutive_failure_count": self.failure_count.get(sensor_id, 0),
                "publisher_anomaly_count": self.publisher_anomaly_count.get(
                    sensor_id, 0
                ),
                "stop_failure_count": self.stop_failure_count.get(sensor_id, 0),
                "current_fault": self.current_fault.get(sensor_id, "") or None,
                "last_fault": self.last_fault.get(sensor_id, "") or None,
                "last_fault_age_sec": (
                    round(max(0.0, now - self.last_fault_time[sensor_id]), 3)
                    if sensor_id in self.last_fault_time
                    else None
                ),
                "quarantine_reason": self.quarantine_reason.get(sensor_id) or None,
                "last_transition": transition or None,
                "last_transition_age_sec": (
                    round(
                        max(
                            0.0,
                            now - self.last_transition_time.get(sensor_id, now),
                        ),
                        3,
                    )
                    if sensor_id in self.last_transition_time
                    else None
                ),
                "next_retry_sec": round(next_retry_sec, 3),
                "reachability_failures": self.reachability_failures.get(sensor_id, 0),
                "topics": topic_status,
            }
        self.health_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    @staticmethod
    def _lifecycle_owner(sensor: Dict[str, Any]) -> str:
        owner = str(sensor.get("lifecycle_owner", "sensor_bringup") or "").strip()
        if owner not in {"sensor_bringup", "external_service"}:
            raise RuntimeError("unsupported sensor lifecycle_owner: {}".format(owner))
        return owner

    def _publisher_graph(self):
        try:
            system_state = self._bounded_xmlrpc_call(self.master_uri, "getSystemState")
            if not isinstance(system_state, (list, tuple)) or len(system_state) != 3:
                raise RuntimeError("getSystemState returned an invalid system state")
            publishers, _subscribers, _services = system_state
        except Exception as exc:
            rospy.logerr_throttle(
                5.0, "sensor_bringup could not inspect publisher graph: %s", exc
            )
            return None
        return {topic: set(nodes) for topic, nodes in publishers}

    def contract_path_arg(self) -> str:
        if self.contract_file:
            return _resolve_package_uri(self.package_root, self.contract_file)
        return str(
            Path(self.package_root)
            / "config"
            / "sensors"
            / "platform"
            / "sensor_contract.yaml"
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
            if "network_field" in spec:
                network_key = sensor_value(
                    self.contract, sensor_id, str(spec["network_field"]), ""
                )
                return str(
                    network_value(
                        str(network_key or ""),
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
    from sensors.ros_master import MasterLost, RosMasterLease

    bringup_lock = _acquire_bringup_lock()
    bringup = None
    try:
        master_lease = RosMasterLease(rosgraph.get_master_uri(), "/sensor_bringup")
        rospy.init_node("sensor_bringup")
        bringup = SensorBringup()
        master_lease.check()
        bringup.start()
        bringup.spin(master_lease)
    except MasterLost as exc:
        rospy.logfatal("%s", exc)
        raise SystemExit(75)
    finally:
        if bringup is not None:
            bringup.shutdown()
        bringup_lock.close()


if __name__ == "__main__":
    main()
