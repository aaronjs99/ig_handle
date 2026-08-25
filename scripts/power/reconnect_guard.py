#!/usr/bin/env python3
"""Thread-safe, hardware-independent guards for bounded BLE recovery."""

import threading
from typing import Tuple


MAX_PROTOCOL_ERROR_RECONNECT_THRESHOLD = 100


class ConsecutiveErrorThreshold:
    """Signal once when a bounded number of consecutive failures is reached."""

    def __init__(self, threshold: int) -> None:
        if type(threshold) is not int:
            raise ValueError("protocol error threshold must be a native integer")
        if not 1 <= threshold <= MAX_PROTOCOL_ERROR_RECONNECT_THRESHOLD:
            raise ValueError(
                "protocol error threshold must be between 1 and {}".format(
                    MAX_PROTOCOL_ERROR_RECONNECT_THRESHOLD
                )
            )
        self.threshold = threshold
        self._count = 0
        self._lock = threading.Lock()

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    def record_success(self) -> None:
        with self._lock:
            self._count = 0

    def record_failure(self) -> bool:
        """Return true only on the transition that reaches the threshold."""

        with self._lock:
            if self._count >= self.threshold:
                return False
            self._count += 1
            return self._count == self.threshold


class ReconnectRequest:
    """Carry one idempotent reconnect request safely between callback threads."""

    def __init__(self) -> None:
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._reason = ""

    def request(self, reason: str) -> None:
        normalized = str(reason).strip() or "requested"
        with self._lock:
            if not self._event.is_set():
                self._reason = normalized
                self._event.set()

    def snapshot(self) -> Tuple[bool, str]:
        with self._lock:
            return self._event.is_set(), self._reason

    def reset(self) -> None:
        with self._lock:
            self._reason = ""
            self._event.clear()
