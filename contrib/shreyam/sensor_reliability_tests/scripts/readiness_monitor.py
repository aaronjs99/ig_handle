#!/usr/bin/env python3

import subprocess
from collections import deque

import roslib.message
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, String


class SensorState:
    def __init__(self, config):
        self.name = config["name"]
        self.topic = config["topic"]
        self.message_type = config["message_type"]
        self.min_frequency_hz = float(config.get("min_frequency_hz", 0.0))
        self.timeout_sec = float(config.get("timeout_sec", 1.0))
        self.max_header_age_sec = float(config.get("max_header_age_sec", 0.0))
        self.ping_host = config.get("ping_host")

        self.last_receive_time = None
        self.last_header_stamp = None
        self.receive_times = deque(maxlen=50)
        self.subscriber = None

    def record_message(self, msg):
        now = rospy.Time.now()
        self.last_receive_time = now
        self.receive_times.append(now)

        header = getattr(msg, "header", None)
        if header is not None:
            self.last_header_stamp = header.stamp

    def estimated_frequency(self):
        if len(self.receive_times) < 2:
            return 0.0

        elapsed = (self.receive_times[-1] - self.receive_times[0]).to_sec()
        if elapsed <= 0.0:
            return 0.0

        return float(len(self.receive_times) - 1) / elapsed


class ReadinessMonitor:
    def __init__(self):
        rospy.init_node("readiness_monitor")

        params = rospy.get_param("/sensor_readiness", {})
        sensor_configs = params.get("sensors", [])
        if not sensor_configs:
            rospy.logwarn("No sensors configured under /sensor_readiness/sensors")

        self.sensors = [SensorState(config) for config in sensor_configs]
        self.ready_pub = rospy.Publisher(
            "/sensor_readiness/ready", Bool, queue_size=1, latch=True
        )
        self.summary_pub = rospy.Publisher(
            "/sensor_readiness/summary", String, queue_size=1, latch=True
        )
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=1
        )

        self._subscribe_to_sensors()

        update_rate_hz = float(params.get("update_rate_hz", 1.0))
        period = rospy.Duration(1.0 / max(update_rate_hz, 0.1))
        self.timer = rospy.Timer(period, self.publish_status)

    def _subscribe_to_sensors(self):
        for sensor in self.sensors:
            msg_class = roslib.message.get_message_class(sensor.message_type)
            if msg_class is None:
                rospy.logerr(
                    "Could not load message type '%s' for sensor '%s'",
                    sensor.message_type,
                    sensor.name,
                )
                continue

            sensor.subscriber = rospy.Subscriber(
                sensor.topic,
                msg_class,
                sensor.record_message,
                queue_size=10,
            )
            rospy.loginfo(
                "Monitoring sensor '%s' on %s as %s",
                sensor.name,
                sensor.topic,
                sensor.message_type,
            )

    def publish_status(self, _event):
        now = rospy.Time.now()
        statuses = [self._sensor_status(sensor, now) for sensor in self.sensors]
        overall_ready = bool(statuses) and all(status["ready"] for status in statuses)

        self.ready_pub.publish(Bool(data=overall_ready))
        self.summary_pub.publish(String(data=self._summary_text(statuses, overall_ready)))
        self.diagnostics_pub.publish(self._diagnostic_array(statuses, now))

    def _sensor_status(self, sensor, now):
        problems = []

        if sensor.subscriber is None:
            problems.append("subscription not active")

        has_message = sensor.last_receive_time is not None
        if not has_message:
            problems.append("no messages received")
            message_age = None
        else:
            message_age = (now - sensor.last_receive_time).to_sec()
            if message_age > sensor.timeout_sec:
                problems.append("last message too old")

        frequency = sensor.estimated_frequency()
        if sensor.min_frequency_hz > 0.0 and frequency < sensor.min_frequency_hz:
            problems.append("frequency below minimum")

        header_age = None
        if sensor.last_header_stamp is not None and sensor.max_header_age_sec > 0.0:
            header_age = (now - sensor.last_header_stamp).to_sec()
            if header_age < 0.0 or header_age > sensor.max_header_age_sec:
                problems.append("header timestamp not fresh")

        ping_ok = None
        if sensor.ping_host:
            ping_ok = self._ping(sensor.ping_host)
            if not ping_ok:
                problems.append("ping failed")

        return {
            "sensor": sensor,
            "ready": len(problems) == 0,
            "problems": problems,
            "message_age": message_age,
            "frequency": frequency,
            "header_age": header_age,
            "ping_ok": ping_ok,
        }

    def _ping(self, host):
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", host],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            return result.returncode == 0
        except OSError as exc:
            rospy.logwarn_throttle(30.0, "Could not run ping: %s", exc)
            return False

    def _summary_text(self, statuses, overall_ready):
        if not statuses:
            return "NOT READY: no sensors configured"

        state = "READY" if overall_ready else "NOT READY"
        parts = []
        for status in statuses:
            sensor = status["sensor"]
            if status["ready"]:
                parts.append("%s ok %.2f Hz" % (sensor.name, status["frequency"]))
            else:
                parts.append("%s: %s" % (sensor.name, ", ".join(status["problems"])))

        return "%s: %s" % (state, "; ".join(parts))

    def _diagnostic_array(self, statuses, now):
        array = DiagnosticArray()
        array.header.stamp = now

        for status in statuses:
            sensor = status["sensor"]
            diagnostic = DiagnosticStatus()
            diagnostic.name = "sensor_readiness/%s" % sensor.name
            diagnostic.hardware_id = sensor.name
            diagnostic.level = (
                DiagnosticStatus.OK if status["ready"] else DiagnosticStatus.ERROR
            )
            diagnostic.message = (
                "ready" if status["ready"] else ", ".join(status["problems"])
            )
            diagnostic.values = self._diagnostic_values(status)
            array.status.append(diagnostic)

        return array

    def _diagnostic_values(self, status):
        sensor = status["sensor"]
        values = [
            KeyValue("topic", sensor.topic),
            KeyValue("message_type", sensor.message_type),
            KeyValue("ready", str(status["ready"])),
            KeyValue("frequency_hz", "%.3f" % status["frequency"]),
            KeyValue("min_frequency_hz", "%.3f" % sensor.min_frequency_hz),
            KeyValue("timeout_sec", "%.3f" % sensor.timeout_sec),
        ]

        if status["message_age"] is not None:
            values.append(KeyValue("message_age_sec", "%.3f" % status["message_age"]))
        if status["header_age"] is not None:
            values.append(KeyValue("header_age_sec", "%.3f" % status["header_age"]))
        if sensor.ping_host:
            values.append(KeyValue("ping_host", sensor.ping_host))
            values.append(KeyValue("ping_ok", str(status["ping_ok"])))

        return values


if __name__ == "__main__":
    try:
        ReadinessMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
