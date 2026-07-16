#!/usr/bin/env python3

import os
import subprocess

import rospy


class TeensyRosserialLauncher:
    """Start rosserial only when the configured optional Teensy device exists."""

    def __init__(self):
        self.port = str(rospy.get_param("~port", "/dev/teensy"))
        self.baud = int(rospy.get_param("~baud", 115200))
        self.pps_time_topic = str(rospy.get_param("~pps_time_topic", "/sensors/pps/time"))
        self.camera_time_topic = str(rospy.get_param("~camera_time_topic", "/sensors/camera/time"))
        self.imu_time_topic = str(rospy.get_param("~imu_time_topic", "/sensors/imu/time"))

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
        ]

    def run(self):
        if not os.path.exists(self.port):
            rospy.logwarn(
                "Teensy device NOT found at %s. Skipping rosserial node (optional).",
                self.port,
            )
            return 0
        rospy.loginfo(
            "Teensy found at %s. Starting rosserial_python node...", self.port
        )
        try:
            return int(subprocess.call(self.command()))
        except KeyboardInterrupt:
            return 0


def main():
    rospy.init_node("teensy_rosserial_launcher", anonymous=True)
    return TeensyRosserialLauncher().run()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
