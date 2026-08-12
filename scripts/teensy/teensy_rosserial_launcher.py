#!/usr/bin/env python3

"""Supervise rosserial while waiting for the optional configured Teensy."""

import os
import signal
import subprocess
import time

import rospy


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
        self.poll_period_sec = float(rospy.get_param("~poll_period_sec", 0.5))
        if not self.port:
            raise ValueError("~port must be nonempty")
        if self.baud <= 0:
            raise ValueError("~baud must be positive")
        if self.poll_period_sec <= 0.0:
            raise ValueError("~poll_period_sec must be positive")

    def command(self):
        return [
            "rosrun",
            "rosserial_python",
            "serial_node.py",
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
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and not os.path.exists(self.port):
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
            try:
                while not rospy.is_shutdown() and child.poll() is None:
                    if not os.path.exists(self.port):
                        rospy.logerr(
                            "Teensy device disappeared from %s; stopping rosserial.",
                            self.port,
                        )
                        break
                    time.sleep(self.poll_period_sec)
            finally:
                self.stop_child(child)
            if rospy.is_shutdown():
                return 0
            rospy.logwarn(
                "rosserial exited with code %s; waiting for Teensy before restart.",
                child.returncode,
            )
            time.sleep(self.poll_period_sec)
        return 0


def main():
    rospy.init_node("teensy_rosserial_launcher", anonymous=True)
    return TeensyRosserialLauncher().run()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
