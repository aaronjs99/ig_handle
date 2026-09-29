"""Bind acquisition and a service-owned client launch to one ROS master."""

import argparse
import http.client
import os
import signal
import subprocess
import sys
import threading
from xmlrpc.client import ServerProxy, Transport


class MasterLost(RuntimeError):
    """Fresh node registrations are required before acquisition can resume."""


class _TimeoutTransport(Transport):
    def __init__(self, source_ip=""):
        super().__init__()
        self.source_address = (source_ip, 0) if source_ip else None

    def make_connection(self, host):
        return http.client.HTTPConnection(
            host, timeout=0.75, source_address=self.source_address
        )


class RosMasterLease:
    """Detect loss/replacement; the existing owner restarts its own processes."""

    def __init__(
        self, uri: str, caller: str, *, source_ip: str = "", require_run_id=False
    ):
        self.uri = uri
        self.caller = caller
        self.source_ip = source_ip
        self.require_run_id = require_run_id
        self.pid = self._pid()
        self.run_id = self._run_id() if require_run_id else None

    def _pid(self) -> int:
        try:
            with ServerProxy(
                self.uri, transport=_TimeoutTransport(self.source_ip)
            ) as master:
                code, message, pid = master.getPid(self.caller)
            if int(code) != 1 or int(pid) <= 0:
                raise ValueError(str(message))
            return int(pid)
        except Exception as exc:
            raise MasterLost(
                "ROS master unavailable; fresh registrations required"
            ) from exc

    def _run_id(self):
        try:
            with ServerProxy(
                self.uri, transport=_TimeoutTransport(self.source_ip)
            ) as master:
                code, message, run_id = master.getParam(self.caller, "/run_id")
            if int(code) != 1 or not isinstance(run_id, str) or not run_id.strip():
                raise ValueError(str(message))
            return run_id
        except Exception as exc:
            raise MasterLost("ROS master epoch unavailable") from exc

    def check(self) -> None:
        if self._pid() != self.pid or (
            self.require_run_id and self._run_id() != self.run_id
        ):
            raise MasterLost("ROS master replaced; fresh registrations required")


def _stop_client(process):
    if process is None or process.poll() is not None:
        return
    for operation, timeout in (
        (lambda: process.send_signal(signal.SIGINT), 10),
        (process.terminate, 3),
        (process.kill, 1),
    ):
        try:
            operation()
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def run_client(command):
    """Serve one launch; exit on master loss so its existing systemd job restarts.

    No master is created and no processes outside the child launch are signaled.
    The caller's service must use KillMode=control-group for descendant cleanup.
    This module remains compatible with Heron's Python 3.5 and needs no ROS import.
    """
    master_uri = os.environ.get("ROS_MASTER_URI", "").strip()
    source_ip = os.environ.get("ROS_IP", "").strip()
    if not master_uri or not source_ip or not command:
        print(
            "The client needs ROS_MASTER_URI, ROS_IP and a launch command.",
            file=sys.stderr,
        )
        return 78
    stop = threading.Event()
    previous = {
        signum: signal.signal(signum, lambda *_: stop.set())
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    process = None
    try:
        print("Waiting for configured ROS master " + master_uri, flush=True)
        while not stop.is_set():
            try:
                lease = RosMasterLease(
                    master_uri,
                    "/ighandle_hardware_client",
                    source_ip=source_ip,
                    require_run_id=True,
                )
                break
            except MasterLost:
                stop.wait(1.0)
        if stop.is_set():
            return 0
        process = subprocess.Popen(command)
        print("Serving ROS master epoch " + lease.run_id, flush=True)
        while not stop.wait(0.5):
            if process.poll() is not None:
                print("Owned ROS client launch exited.", file=sys.stderr)
                return process.returncode or 75
            try:
                lease.check()
            except MasterLost as exc:
                print(str(exc), file=sys.stderr)
                return 75
        return 0
    finally:
        _stop_client(process)
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=run_client.__doc__)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    arguments = parser.parse_args().command
    if arguments[:1] == ["--"]:
        arguments = arguments[1:]
    sys.exit(run_client(arguments))
