#!/usr/bin/env python3
"""Raw UDP receiver for Imagenex sonar datagrams.

This node publishes vendor datagrams unchanged. It intentionally does not filter
on packet kind, so raw bags preserve profile packets, beam packets, and any
other diagnostic traffic sent by the sonar-side transmitter.
"""
import socket
from dataclasses import dataclass
from typing import Optional, Tuple

import rospy
from std_msgs.msg import UInt8MultiArray
from network_config import network_value


@dataclass(frozen=True)
class RawSonarPacket:
    """Raw vendor datagram plus optional UDP source metadata."""

    data: bytes
    source: Tuple[str, int]


class SonarRawReceiver:
    """Receive Imagenex UDP datagrams and publish raw ROS byte arrays."""

    def __init__(self):
        self.port = int(rospy.get_param("~port", 4040))
        self.topic = str(rospy.get_param("~topic", "/sensors/sonar/raw"))
        self.bind_ip = str(
            rospy.get_param("~bind_ip", network_value("mocap_udp_bind_ip"))
        )
        self.publisher = rospy.Publisher(self.topic, UInt8MultiArray, queue_size=50)
        self.socket = self._open_socket()
        self.last_packet: Optional[RawSonarPacket] = None
        rospy.on_shutdown(self.close)

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
        except OSError:
            if not rospy.is_shutdown():
                raise
            return
        if len(data) < 8:
            return
        if data[:3] != b"83P":
            rospy.logdebug("Non-83P frame from %s len=%d", src, len(data))
        self.last_packet = RawSonarPacket(data=data, source=src)
        msg = UInt8MultiArray()
        msg.data = list(self.last_packet.data)
        self.publisher.publish(msg)

    def close(self):
        try:
            self.socket.close()
        except OSError:
            pass


def run():
    rospy.init_node("sonar_receiver")
    SonarRawReceiver().spin()


if __name__ == "__main__":
    run()
