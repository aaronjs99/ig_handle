#!/usr/bin/env python3
# ig_handle/scripts/dt100_rx.py
"""
Imagenex DT100 Sonar Receiver
-----------------------------
Listens for raw UDP packets from the Imagenex DT100 multibeam sonar.
Publishes raw byte data to a ROS topic for downstream processing.
"""
import socket, struct
import rospy
from std_msgs.msg import Header
from std_msgs.msg import UInt8MultiArray


def run():
    rospy.init_node("dt100_rx")

    port = rospy.get_param("~port", 4040)
    topic = rospy.get_param("~topic", "/sonar/scan")
    bind_ip = rospy.get_param("~bind_ip", "192.168.0.3")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, port))
    sock.settimeout(0.5)

    pub = rospy.Publisher(topic, UInt8MultiArray, queue_size=50)
    rospy.loginfo("Imagenex UDP listener on %s:%d -> %s", bind_ip, port, topic)

    while not rospy.is_shutdown():
        try:
            data, src = sock.recvfrom(65535)
            if len(data) < 8:
                continue
            if data[:3] != b"83P":
                rospy.logdebug("Non-83P frame from %s len=%d", src, len(data))
            msg = UInt8MultiArray()
            msg.data = list(data)
            pub.publish(msg)
        except socket.timeout:
            pass


if __name__ == "__main__":
    run()
