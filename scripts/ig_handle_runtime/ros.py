"""Small ROS 2 runtime facade for IG Handle's migrating node logic.

The facade keeps the existing mission modules focused on orchestration while
all graph entities, parameters, clocks, services, and executors are native
``rclpy`` objects.  It is deliberately package-private; new nodes should use
``rclpy`` directly.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args


class ROSException(RuntimeError):
    pass


class ROSInitException(ROSException):
    pass


class ROSInterruptException(ROSException):
    pass


class ServiceException(ROSException):
    pass


_node: Optional[Node] = None
_executor: Optional[MultiThreadedExecutor] = None
_executor_thread: Optional[threading.Thread] = None
_callback_group = ReentrantCallbackGroup()
_service_clients: dict[tuple[str, type], Any] = {}
_throttle_stamps: dict[tuple[str, str], float] = {}
_shutdown_callbacks: list[Any] = []


def _require_node() -> Node:
    if _node is None:
        raise ROSInitException("ORACLE ROS runtime has not been initialized")
    return _node


def _parameter_name(name: str) -> str:
    value = str(name).strip()
    if value.startswith("~"):
        value = value[1:]
    value = value.lstrip("/").replace("/", ".")
    if not value or not (value[0].isalpha() or value[0] == "_"):
        raise ValueError(f"invalid ROS 2 parameter name: {name!r}")
    return value


def _parameter_tree(node: Node, prefix: str) -> dict[str, Any]:
    tree: dict[str, Any] = {}
    for suffix, parameter in node.get_parameters_by_prefix(prefix).items():
        parts = suffix.split(".")
        current = tree
        for part in parts[:-1]:
            current = current.setdefault(part, {})
        current[parts[-1]] = parameter.value
    return tree


def _declare_parameter(node: Node, name: str, default: Any = None):
    key = _parameter_name(name)
    if not node.has_parameter(key):
        if default is None or isinstance(default, dict):
            node.declare_parameter(key, Parameter.Type.NOT_SET)
        else:
            node.declare_parameter(key, default)
    value = node.get_parameter(key).value
    if value is not None:
        return value
    if isinstance(default, dict):
        return _parameter_tree(node, key) or default
    return value


class _Time:
    def __new__(cls, secs: int = 0, nsecs: int = 0):
        value = TimeMessage()
        value.sec = int(secs)
        value.nanosec = int(nsecs)
        return value

    @staticmethod
    def now() -> TimeMessage:
        return _require_node().get_clock().now().to_msg()

    @staticmethod
    def from_sec(seconds: float) -> TimeMessage:
        nanoseconds = int(round(float(seconds) * 1_000_000_000))
        value = TimeMessage()
        value.sec, value.nanosec = divmod(nanoseconds, 1_000_000_000)
        return value

    @staticmethod
    def to_sec(stamp: TimeMessage) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# ROS 1 code commonly calls to_sec() on an incoming Time.  Keep that helper
# while the containing nodes migrate to native ROS 2 message timestamps.
if not hasattr(TimeMessage, "to_sec"):
    TimeMessage.to_sec = lambda self: float(self.sec) + float(self.nanosec) * 1e-9
if not hasattr(TimeMessage, "to_nsec"):
    TimeMessage.to_nsec = lambda self: int(self.sec) * 1_000_000_000 + int(self.nanosec)
if not hasattr(TimeMessage, "secs"):
    TimeMessage.secs = property(lambda self: self.sec, lambda self, value: setattr(self, "sec", value))
if not hasattr(TimeMessage, "nsecs"):
    TimeMessage.nsecs = property(lambda self: self.nanosec, lambda self, value: setattr(self, "nanosec", value))
if not hasattr(TimeMessage, "__sub__"):
    TimeMessage.__sub__ = lambda self, other: RclpyDuration(
        nanoseconds=(int(self.sec) - int(other.sec)) * 1_000_000_000
        + int(self.nanosec) - int(other.nanosec)
    )
if not hasattr(TimeMessage, "__lt__"):
    TimeMessage.__lt__ = lambda self, other: (int(self.sec), int(self.nanosec)) < (int(other.sec), int(other.nanosec))
if not hasattr(TimeMessage, "__le__"):
    TimeMessage.__le__ = lambda self, other: (int(self.sec), int(self.nanosec)) <= (int(other.sec), int(other.nanosec))
if not hasattr(TimeMessage, "__gt__"):
    TimeMessage.__gt__ = lambda self, other: (int(self.sec), int(self.nanosec)) > (int(other.sec), int(other.nanosec))
if not hasattr(TimeMessage, "__ge__"):
    TimeMessage.__ge__ = lambda self, other: (int(self.sec), int(self.nanosec)) >= (int(other.sec), int(other.nanosec))


def Duration(seconds: float = 0.0) -> RclpyDuration:
    return RclpyDuration(seconds=float(seconds))


def init_node(name: str, **_kwargs: Any) -> Node:
    global _node, _executor, _executor_thread, _callback_group
    if _node is not None:
        return _node
    if not rclpy.ok():
        rclpy.init()
    _node = Node(
        str(name),
        automatically_declare_parameters_from_overrides=True,
    )
    _executor = MultiThreadedExecutor(num_threads=8)
    _executor.add_node(_node)
    _callback_group = ReentrantCallbackGroup()
    _executor_thread = threading.Thread(
        target=_executor.spin, name=f"{name}-rclpy-executor", daemon=True
    )
    _executor_thread.start()
    return _node


def get_node() -> Node:
    return _require_node()


def resolve_name(name: str) -> str:
    return _require_node().resolve_topic_name(str(name))


def get_param(name: str, default: Any = ...):
    node = _require_node()
    key = _parameter_name(name)
    value = _declare_parameter(node, key, None if default is ... else default)
    if isinstance(default, dict):
        return value or default
    if value is None and default is ...:
        raise KeyError(f"ROS 2 parameter {name!r} is not set")
    return default if value is None and default is not ... else value


def set_param(name: str, value: Any) -> None:
    node = _require_node()
    key = _parameter_name(name)
    if isinstance(value, dict):
        flattened = {}

        def flatten(prefix, item):
            for child, child_value in item.items():
                child_name = f"{prefix}.{child}"
                if isinstance(child_value, dict):
                    flatten(child_name, child_value)
                else:
                    flattened[child_name] = child_value

        flatten(key, value)
        for child_name, child_value in flattened.items():
            if not node.has_parameter(child_name):
                node.declare_parameter(child_name, child_value)
            else:
                node.set_parameters([Parameter(child_name, value=child_value)])
    elif not node.has_parameter(key):
        node.declare_parameter(key, value)
    else:
        node.set_parameters([Parameter(key, value=value)])


def has_param(name: str) -> bool:
    node = _require_node()
    key = _parameter_name(name)
    return node.has_parameter(key) or bool(node.get_parameters_by_prefix(key))


def on_shutdown(callback) -> None:
    _shutdown_callbacks.append(callback)
    rclpy.get_default_context().on_shutdown(callback)


def is_shutdown() -> bool:
    return not rclpy.ok()


def spin() -> None:
    if _executor is None or _executor_thread is None:
        raise ROSInitException("call init_node() before spin()")
    try:
        _executor_thread.join()
    except KeyboardInterrupt as exc:
        raise ROSInterruptException("ROS 2 executor interrupted") from exc


def shutdown() -> None:
    global _node, _executor, _executor_thread
    if _executor is not None:
        _executor.shutdown(timeout_sec=2.0)
        _executor = None
    if _node is not None:
        _node.destroy_node()
        _node = None
    if rclpy.ok():
        rclpy.shutdown()
    _executor_thread = None
    _service_clients.clear()


def _logger_method(level: str, message: Any, *args: Any) -> None:
    text = str(message) % args if args else str(message)
    getattr(_require_node().get_logger(), level)(text)


def logdebug(message: Any, *args: Any) -> None:
    _logger_method("debug", message, *args)


def loginfo(message: Any, *args: Any) -> None:
    _logger_method("info", message, *args)


def logwarn(message: Any, *args: Any) -> None:
    _logger_method("warning", message, *args)


def logerr(message: Any, *args: Any) -> None:
    _logger_method("error", message, *args)


def logfatal(message: Any, *args: Any) -> None:
    _logger_method("fatal", message, *args)


def _throttled(level: str, period: float, message: Any, args: tuple[Any, ...]) -> None:
    key = (level, str(message))
    now = time.monotonic()
    if now - _throttle_stamps.get(key, float("-inf")) >= float(period):
        _throttle_stamps[key] = now
        _logger_method(level, message, *args)


def loginfo_throttle(period: float, message: Any, *args: Any) -> None:
    _throttled("info", period, message, args)


def logwarn_throttle(period: float, message: Any, *args: Any) -> None:
    _throttled("warning", period, message, args)


def logerr_throttle(period: float, message: Any, *args: Any) -> None:
    _throttled("error", period, message, args)


def logdebug_throttle(period: float, message: Any, *args: Any) -> None:
    _throttled("debug", period, message, args)


def _qos(
    queue_size: int = 10, latch: bool = False, *, sensor_data: bool = False
) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(queue_size)),
        reliability=(
            ReliabilityPolicy.BEST_EFFORT
            if sensor_data
            else ReliabilityPolicy.RELIABLE
        ),
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if latch
            else DurabilityPolicy.VOLATILE
        ),
    )


class Publisher:
    def __init__(self, topic: str, msg_type: type, queue_size: int = 10, latch: bool = False, **_kwargs):
        sensor_data = msg_type.__module__.startswith("sensor_msgs.")
        self._publisher = _require_node().create_publisher(
            msg_type, topic, _qos(queue_size, latch, sensor_data=sensor_data)
        )

    def publish(self, message: Any) -> None:
        self._publisher.publish(message)

    def get_num_connections(self) -> int:
        return self._publisher.get_subscription_count()


class Subscriber:
    def __init__(self, topic: str, msg_type: type, callback, queue_size: int = 10, **_kwargs):
        self._node = _require_node()
        sensor_data = msg_type.__module__.startswith("sensor_msgs.")
        self._subscription = self._node.create_subscription(
            msg_type,
            topic,
            callback,
            _qos(queue_size, sensor_data=sensor_data),
            callback_group=_callback_group,
        )

    def unregister(self) -> None:
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None


class Service:
    def __init__(self, name: str, srv_type: type, callback, **_kwargs):
        self._node = _require_node()

        def dispatch(request, response):
            try:
                result = callback(request)
            except Exception as exc:
                raise ServiceException(str(exc)) from exc
            return response if result is None else result

        self._service = self._node.create_service(
            srv_type,
            name,
            dispatch,
            callback_group=_callback_group,
        )


class ServiceProxy:
    def __init__(self, name: str, srv_type: type, **_kwargs):
        self.name = str(name)
        self.srv_type = srv_type
        key = (self.name, srv_type)
        self.client = _service_clients.get(key)
        if self.client is None:
            self.client = _require_node().create_client(
                srv_type,
                self.name,
                callback_group=_callback_group,
            )
            _service_clients[key] = self.client

    def wait_for_service(self, timeout: Optional[float] = None) -> bool:
        return bool(self.client.wait_for_service(timeout_sec=timeout))

    def __call__(self, *args: Any, **kwargs: Any):
        request = self.srv_type.Request()
        fields = list(request.get_fields_and_field_types())
        if args:
            if len(args) == 1 and isinstance(args[0], self.srv_type.Request):
                request = args[0]
            else:
                if len(args) > len(fields):
                    raise TypeError(f"too many positional fields for {self.srv_type.__name__}")
                for field, value in zip(fields, args):
                    setattr(request, field, value)
        for field, value in kwargs.items():
            if field not in fields:
                raise TypeError(f"unknown request field {field!r} for {self.srv_type.__name__}")
            setattr(request, field, value)
        if not self.client.wait_for_service(timeout_sec=2.0):
            raise ServiceException(f"service {self.name!r} is not available")
        completed = threading.Event()
        future = self.client.call_async(request)
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=60.0):
            raise ServiceException(f"timed out waiting for service {self.name!r}")
        try:
            result = future.result()
        except Exception as exc:
            raise ServiceException(str(exc)) from exc
        if result is None:
            raise ServiceException(f"service {self.name!r} returned no response")
        return result


def wait_for_service(name: str, timeout: Optional[float] = None) -> None:
    node = _require_node()
    deadline = None if timeout is None else time.monotonic() + float(timeout)
    normalized = node.resolve_service_name(str(name))
    while rclpy.ok():
        available = {service_name for service_name, _types in node.get_service_names_and_types()}
        if normalized in available:
            return
        if deadline is not None and time.monotonic() >= deadline:
            raise ROSException(f"timed out waiting for service {name!r}")
        time.sleep(0.05)


def wait_for_message(topic: str, msg_type: type, timeout: Optional[float] = None):
    node = _require_node()
    event = threading.Event()
    captured: list[Any] = []

    def receive(message):
        if not captured:
            captured.append(message)
            event.set()

    subscription = node.create_subscription(
        msg_type, topic, receive, _qos(1), callback_group=_callback_group
    )
    try:
        if not event.wait(timeout=timeout):
            raise ROSException(f"timed out waiting for a message on {topic!r}")
        return captured[0]
    finally:
        node.destroy_subscription(subscription)


class Rate:
    def __init__(self, hz: float):
        self.period = 1.0 / float(hz)
        self._next = time.monotonic() + self.period

    def sleep(self) -> None:
        delay = max(0.0, self._next - time.monotonic())
        if delay:
            time.sleep(delay)
        self._next = max(self._next + self.period, time.monotonic())


def Timer(duration: RclpyDuration, callback):
    seconds = float(duration.nanoseconds) * 1e-9
    node = _require_node()
    timer = node.create_timer(
        max(0.001, seconds),
        lambda: callback(None),
        callback_group=_callback_group,
    )
    return _TimerHandle(node, timer)


class _TimerHandle:
    def __init__(self, node: Node, timer):
        self._node = node
        self._timer = timer

    def shutdown(self) -> None:
        self._timer.cancel()
        self._node.destroy_timer(self._timer)

    cancel = shutdown


def get_node_uri():
    return None if _node is None else _node.get_fully_qualified_name()


def get_name():
    return _require_node().get_fully_qualified_name()


def signal_shutdown(reason: str = "") -> None:
    if reason:
        logwarn("ROS shutdown requested: %s", reason)
    if rclpy.ok():
        rclpy.try_shutdown()


def publishers_info(topic: str):
    return _require_node().get_publishers_info_by_topic(str(topic))


def myargv(argv=None):
    return remove_ros_args(args=None if argv is None else list(argv))


def sleep(seconds: float) -> None:
    time.sleep(max(0.0, float(seconds)))


def logfatal(message: str, *args: Any) -> None:
    logerr(message, *args)
