#!/usr/bin/env python3
# ig_handle/scripts/sonar/dt100_rx.py
"""Imagenex DT100 Driver: Raw UDP Packet Interceptor.
--------------------------------------------------

Lightweight Network Receiver for the Imagenex DT100 Multibeam Profiling Sonar.
This node binds to the proprietary sonar subnet, intercepts UDP datagrams
broadcast by the sonar head (83P format), and publishes them as raw byte arrays
to ROS.

This raw stream is typically recorded for offline processing or parsed in
real-time by dt100_profile_to_cloud.py to produce PointCloud2.
"""
import socket

import rospy
from std_msgs.msg import UInt8MultiArray


class DT100RawReceiver:
    """Receive Imagenex DT100 UDP datagrams and publish raw ROS byte arrays."""

    def __init__(self):
        self.port = int(rospy.get_param("~port", 4040))
        self.topic = str(rospy.get_param("~topic", "/sensors/sonar/raw"))
        self.bind_ip = str(rospy.get_param("~bind_ip", "0.0.0.0"))
        self.publisher = rospy.Publisher(self.topic, UInt8MultiArray, queue_size=50)
        self.socket = self._open_socket()

    def _open_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_ip, self.port))
        sock.settimeout(0.5)
        return sock

    def spin(self):
        rospy.loginfo(
            "Imagenex UDP listener on %s:%d -> %s",
            self.bind_ip,
            self.port,
            self.topic,
        )
        while not rospy.is_shutdown():
            self._receive_once()

    def _receive_once(self):
        try:
            data, src = self.socket.recvfrom(65535)
        except socket.timeout:
            return
        if len(data) < 8:
            return
        if data[:3] != b"83P":
            rospy.logdebug("Non-83P frame from %s len=%d", src, len(data))
        msg = UInt8MultiArray()
        msg.data = list(data)
        self.publisher.publish(msg)


def run():
    rospy.init_node("dt100_rx")
    DT100RawReceiver().spin()


if __name__ == "__main__":
    run()
