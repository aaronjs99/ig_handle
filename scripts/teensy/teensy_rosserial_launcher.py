#!/usr/bin/env python3

"""Supervise rosserial while waiting for the optional configured Teensy."""

import os
import signal
import subprocess
import time

import rospy
from std_msgs.msg import String


class TeensyRosserialLauncher:
    """Start rosserial only when the configured optional Teensy device exists."""

    def __init__(self):
        self.port = str(rospy.get_param("~port", "/dev/teensy")).strip()
        self.baud = int(rospy.get_param("~baud", 115200))
        self.pps_time_topic = str(
            rospy.get_param("~pps_time_topic", "/sensors/pps/time")
        )
        self.camera_time_topic = str(
            rospy.get_param("~camera_time_topic", "/sensors/camera/time")
        )
        self.imu_time_topic = str(
            rospy.get_param("~imu_time_topic", "/sensors/imu/time")
        )
        self.timing_status_topic = str(
            rospy.get_param("~timing_status_topic", "/sensors/timing/status")
        )
        self.expected_firmware_build_id = str(
            rospy.get_param("~expected_firmware_build_id", "")
        ).strip()
        self.device_timeout_sec = float(rospy.get_param("~device_timeout_sec", 10.0))
        self.identity_timeout_sec = float(rospy.get_param("~identity_timeout_sec", 5.0))
        self.poll_period_sec = float(rospy.get_param("~poll_period_sec", 0.5))
        self.last_firmware_build_id = ""
        self.last_identity_received_sec = float("-inf")
        if not self.port:
            raise ValueError("~port must be nonempty")
        if self.baud <= 0:
            raise ValueError("~baud must be positive")
        if self.poll_period_sec <= 0.0:
            raise ValueError("~poll_period_sec must be positive")
        if not self.expected_firmware_build_id:
            raise ValueError(
                "~expected_firmware_build_id is required before Teensy bridge enablement"
            )
        if self.device_timeout_sec <= 0.0:
            raise ValueError("~device_timeout_sec must be positive")
        if self.identity_timeout_sec <= 0.0:
            raise ValueError("~identity_timeout_sec must be positive")
        rospy.Subscriber(
            self.timing_status_topic,
            String,
            self._timing_status_cb,
            queue_size=5,
        )

    def _timing_status_cb(self, message):
        build_id = ""
        for token in str(message.data or "").split():
            key, separator, value = token.partition("=")
            if separator and key == "firmware_build_id":
                build_id = value.strip()
                break
        self.last_firmware_build_id = build_id
        self.last_identity_received_sec = time.monotonic()

    def command(self):
        return [
            "rosrun",
            "rosserial_python",
            "serial_node.py",
            "__name:=teensy_serial_node",
            "_port:=" + self.port,
            "_baud:=" + str(self.baud),
            "/pps/time:=" + self.pps_time_topic,
            "/cam/time:=" + self.camera_time_topic,
            "/imu/time:=" + self.imu_time_topic,
            "/timing/status:=" + self.timing_status_topic,
        ]

    @staticmethod
    def stop_child(child):
        if child.poll() is not None:
            return
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            child.wait()
            return
        try:
            child.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait()

    def run(self):
        device_deadline_sec = time.monotonic() + self.device_timeout_sec
        while not rospy.is_shutdown() and not os.path.exists(self.port):
            if time.monotonic() >= device_deadline_sec:
                rospy.logfatal(
                    "Teensy device did not appear at %s within %.1fs",
                    self.port,
                    self.device_timeout_sec,
                )
                return 78
            rospy.logwarn_throttle(
                10.0,
                "Teensy device not found at %s; waiting without publishing timing.",
                self.port,
            )
            time.sleep(self.poll_period_sec)
        if rospy.is_shutdown():
            return 0

        rospy.loginfo("Teensy found at %s; starting rosserial.", self.port)
        child = subprocess.Popen(self.command(), start_new_session=True)
        child_started_sec = time.monotonic()
        identity_deadline_sec = child_started_sec + self.identity_timeout_sec
        identity_verified = False
        failure = ""
        try:
            while not rospy.is_shutdown():
                child_code = child.poll()
                if child_code is not None:
                    failure = "rosserial exited unexpectedly with code {}".format(
                        child_code
                    )
                    break
                if not os.path.exists(self.port):
                    failure = "Teensy device disappeared from {}".format(self.port)
                    break
                now_sec = time.monotonic()
                if self.last_identity_received_sec >= child_started_sec:
                    if self.last_firmware_build_id != self.expected_firmware_build_id:
                        failure = (
                            "Teensy firmware identity mismatch: expected {}, got {}"
                        ).format(
                            self.expected_firmware_build_id,
                            self.last_firmware_build_id or "<missing>",
                        )
                        break
                    if not identity_verified:
                        rospy.loginfo(
                            "Verified Teensy firmware identity %s",
                            self.expected_firmware_build_id,
                        )
                    identity_verified = True
                if not identity_verified and now_sec >= identity_deadline_sec:
                    failure = (
                        "Teensy did not publish the expected firmware identity "
                        "within {:.1f}s"
                    ).format(self.identity_timeout_sec)
                    break
                if (
                    identity_verified
                    and now_sec - self.last_identity_received_sec
                    > self.identity_timeout_sec
                ):
                    failure = "Teensy firmware identity/status stream became stale"
                    break
                time.sleep(self.poll_period_sec)
        finally:
            self.stop_child(child)
        if rospy.is_shutdown():
            return 0
        rospy.logfatal("%s; fail-stopping the enabled Teensy bridge", failure)
        return 78


def main():
    rospy.init_node("teensy_rosserial_launcher", anonymous=True)
    return TeensyRosserialLauncher().run()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
