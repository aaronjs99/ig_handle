#!/usr/bin/env python3
import os
import sys
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from natnet.NatNetClient import NatNetClient


def quat_xyzw_from_natnet(q):
    x, y, z, w = q
    return float(x), float(y), float(z), float(w)


class NatNetBridge:
    def __init__(self):
        self.server_ip = rospy.get_param("~server_ip", "192.168.1.199")
        self.client_ip = rospy.get_param("~client_ip", "192.168.1.8")
        self.frame_id = rospy.get_param("~frame_id", "motive_world")
        self.pub_prefix = rospy.get_param("~topic_prefix", "/motive")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.pub_rb = {}
        self.tf_broadcaster = TransformBroadcaster() if self.publish_tf else None

        self.client = NatNetClient()

        # Configure exactly like PythonSample
        self.client.set_client_address(self.client_ip)
        self.client.set_server_address(self.server_ip)
        self.client.set_use_multicast(False)

        # IMPORTANT: suppress internal printing
        if hasattr(self.client, "set_print_level"):
            self.client.set_print_level(0)

        # Only hook rigid body listener
        self.client.rigid_body_listener = self._on_rigid_body

        rospy.loginfo(
            "NatNetBridge configured. server_ip=%s client_ip=%s"
            % (self.server_ip, self.client_ip)
        )

    def start(self):
        rospy.loginfo("Starting NatNet client...")
        ok = self.client.run("d")
        if not ok:
            raise RuntimeError("NatNetClient failed to start.")

        rospy.sleep(0.5)
        if hasattr(self.client, "connected") and not self.client.connected():
            raise RuntimeError("NatNetClient not connected.")

        rospy.loginfo("NatNet streaming.")

    def _pub_for_rb(self, rb_id):
        if rb_id not in self.pub_rb:
            topic = f"{self.pub_prefix}/rigid_body_{rb_id}/pose"
            self.pub_rb[rb_id] = rospy.Publisher(topic, PoseStamped, queue_size=50)
        return self.pub_rb[rb_id]

    def _on_rigid_body(self, rb_id, position, rotation):
        t = rospy.Time.now()
        pub = self._pub_for_rb(int(rb_id))

        msg = PoseStamped()
        msg.header.stamp = t
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])

        qx, qy, qz, qw = quat_xyzw_from_natnet(rotation)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        pub.publish(msg)

        if self.tf_broadcaster is not None:
            tfm = TransformStamped()
            tfm.header.stamp = t
            tfm.header.frame_id = self.frame_id
            tfm.child_frame_id = f"rigid_body_{int(rb_id)}"
            tfm.transform.translation.x = float(position[0])
            tfm.transform.translation.y = float(position[1])
            tfm.transform.translation.z = float(position[2])
            tfm.transform.rotation.x = qx
            tfm.transform.rotation.y = qy
            tfm.transform.rotation.z = qz
            tfm.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tfm)


def main():
    rospy.init_node("natnet_bridge", anonymous=False)
    bridge = NatNetBridge()
    bridge.start()
    rospy.spin()


if __name__ == "__main__":
    main()
