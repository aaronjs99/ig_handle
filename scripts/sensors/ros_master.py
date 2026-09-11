"""Bound an acquisition process to the ROS master that registered it."""

import http.client
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
    """Detect loss/replacement; let the owning launcher restart the whole node."""

    def __init__(self, uri: str, caller: str, *, source_ip: str = ""):
        self.uri = uri
        self.caller = caller
        self.source_ip = source_ip
        self.pid = self._pid()

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
            raise MasterLost("ROS master unavailable; restart this test") from exc

    def check(self) -> None:
        if self._pid() != self.pid:
            raise MasterLost(
                "ROS master replaced; restart this test for fresh registrations"
            )
