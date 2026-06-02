#!/usr/bin/env python3
import json
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIRS = (
    os.path.dirname(THIS_DIR),
    THIS_DIR,
    os.path.join(
        os.path.dirname(os.path.dirname(THIS_DIR)),
        "share",
        "ig_handle",
        "scripts",
        "mocap",
    ),
)
for module_dir in reversed(MODULE_DIRS):
    if module_dir not in sys.path:
        sys.path.insert(0, module_dir)

from udp.datacollect import DatacollectUdpReceiver, as_bool
from network_config import network_value


def quat_xyzw(q):
    x, y, z, w = q
    return float(x), float(y), float(z), float(w)


class MocapBridge:
    def __init__(self):
        self.transport = rospy.get_param("~transport", "natnet").strip().lower()
        self.server_ip = rospy.get_param(
            "~server_ip", network_value("mocap_natnet_server_ip")
        )
        self.client_ip = rospy.get_param(
            "~client_ip", network_value("mocap_natnet_client_ip")
        )
        self.natnet_use_multicast = as_bool(
            rospy.get_param("~natnet_use_multicast", False)
        )
        self.natnet_multicast_address = rospy.get_param(
            "~natnet_multicast_address",
            network_value("mocap_natnet_multicast_address"),
        ).strip()
        self.frame_id = rospy.get_param("~frame_id", "mocap_world")
        self.pub_prefix = rospy.get_param("~topic_prefix", "/mocap")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.child_frame_prefix = rospy.get_param("~child_frame_prefix", "rigid_body_")
        self.datacollect_schema = rospy.get_param(
            "~datacollect_schema", "datacollect.heron.v1"
        )
        self.datacollect_source_ip = rospy.get_param(
            "~datacollect_source_ip", ""
        ).strip()
        self.datacollect_reject_unexpected_source = as_bool(
            rospy.get_param("~datacollect_reject_unexpected_source", False)
        )
        self.udp_bind_ip = rospy.get_param(
            "~udp_bind_ip", network_value("mocap_udp_bind_ip")
        )
        self.udp_port = int(rospy.get_param("~udp_port", 5005))
        self.heron_pose_topic = rospy.get_param(
            "~heron_pose_topic", f"{self.pub_prefix}/rigid_body_1/pose"
        )
        self.markers_topic = rospy.get_param(
            "~markers_topic", f"{self.pub_prefix}/heron/markers"
        )
        self.potential_objects_topic = rospy.get_param(
            "~potential_objects_topic", f"{self.pub_prefix}/potential_objects"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", f"{self.pub_prefix}/datacollect_status"
        )
        self.stale_timeout_sec = float(rospy.get_param("~stale_timeout_sec", 1.0))

        self.pub_rb = {}
        self.heron_pose_pub = rospy.Publisher(
            self.heron_pose_topic, PoseStamped, queue_size=50
        )
        self.markers_pub = rospy.Publisher(
            self.markers_topic, PointCloud2, queue_size=10
        )
        self.potential_objects_pub = rospy.Publisher(
            self.potential_objects_topic, PointCloud2, queue_size=10
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster() if self.publish_tf else None

        self.client = None
        if self.transport == "natnet":
            from natnet.NatNetClient import NatNetClient

            self.client = NatNetClient()

            # Configure exactly like PythonSample
            self.client.set_client_address(self.client_ip)
            self.client.set_server_address(self.server_ip)
            self.client.set_use_multicast(self.natnet_use_multicast)
            if self.natnet_multicast_address:
                self.client.set_multicast_address(self.natnet_multicast_address)

            # IMPORTANT: suppress internal printing
            if hasattr(self.client, "set_print_level"):
                self.client.set_print_level(0)

            # Only hook rigid body listener
            self.client.rigid_body_listener = self._on_rigid_body
        elif self.transport != "datacollect_udp":
            raise RuntimeError(
                "unsupported mocap transport %r; expected natnet or datacollect_udp"
                % self.transport
            )

        if self.transport == "datacollect_udp":
            rospy.loginfo(
                "MocapBridge configured. transport=datacollect_udp udp=%s:%d "
                "schema=%s expected_source_ip=%s reject_unexpected_source=%s"
                % (
                    self.udp_bind_ip,
                    self.udp_port,
                    self.datacollect_schema,
                    self.datacollect_source_ip or "*",
                    self.datacollect_reject_unexpected_source,
                )
            )
        else:
            rospy.loginfo(
                "MocapBridge configured. transport=%s server_ip=%s client_ip=%s "
                "use_multicast=%s multicast_address=%s udp=%s:%d"
                % (
                    self.transport,
                    self.server_ip,
                    self.client_ip,
                    self.natnet_use_multicast,
                    self.natnet_multicast_address or "*",
                    self.udp_bind_ip,
                    self.udp_port,
                )
            )

    def start(self):
        if self.transport == "datacollect_udp":
            DatacollectUdpReceiver(
                bind_ip=self.udp_bind_ip,
                port=self.udp_port,
                schema=self.datacollect_schema,
                expected_source_ip=self.datacollect_source_ip,
                reject_unexpected_source=self.datacollect_reject_unexpected_source,
                stale_timeout_sec=self.stale_timeout_sec,
                publish_status=self._publish_status,
                publish_pose=self._publish_datacollect_pose,
                publish_points=self._publish_datacollect_points,
            ).run()
            return

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

    def _pose_msg(self, stamp, position, rotation):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])

        qx, qy, qz, qw = quat_xyzw(rotation)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_pose_tf(self, stamp, rb_id, position, rotation):
        if self.tf_broadcaster is None:
            return
        qx, qy, qz, qw = quat_xyzw(rotation)
        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = self.frame_id
        tfm.child_frame_id = f"{self.child_frame_prefix}{int(rb_id)}"
        tfm.transform.translation.x = float(position[0])
        tfm.transform.translation.y = float(position[1])
        tfm.transform.translation.z = float(position[2])
        tfm.transform.rotation.x = qx
        tfm.transform.rotation.y = qy
        tfm.transform.rotation.z = qz
        tfm.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tfm)

    def _on_rigid_body(self, rb_id, position, rotation):
        t = rospy.Time.now()
        pub = self._pub_for_rb(int(rb_id))

        msg = self._pose_msg(t, position, rotation)
        pub.publish(msg)

        self._publish_pose_tf(t, rb_id, position, rotation)

    def _cloud_msg(self, stamp, points):
        header = Header(stamp=stamp, frame_id=self.frame_id)
        return pc2.create_cloud_xyz32(header, points)

    def _publish_status(
        self, packet, status_state, stamp, source_address=None, tracking_valid=None
    ):
        status = {
            "schema": packet.get("schema"),
            "status": status_state,
            "device": packet.get("device", ""),
            "frame": packet.get("frame"),
            "stamp": stamp.to_sec(),
        }
        if self.datacollect_source_ip:
            status["expected_source_ip"] = self.datacollect_source_ip
        if source_address is not None:
            status["source_ip"] = source_address[0]
            status["source_port"] = source_address[1]
        if tracking_valid is not None:
            status["tracking_valid"] = bool(tracking_valid)
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def _publish_datacollect_pose(self, stamp, rb_id, position, rotation):
        self.heron_pose_pub.publish(self._pose_msg(stamp, position, rotation))
        self._publish_pose_tf(stamp, rb_id, position, rotation)

    def _publish_datacollect_points(self, stamp, marker_points, potential_points):
        self.markers_pub.publish(self._cloud_msg(stamp, marker_points))
        self.potential_objects_pub.publish(self._cloud_msg(stamp, potential_points))


def main():
    rospy.init_node("mocap_bridge", anonymous=False)
    bridge = MocapBridge()

    while not rospy.is_shutdown():
        try:
            bridge.start()
            break
        except RuntimeError as e:
            rospy.logwarn("[MocapBridge] %s; retrying in 5s", e)
            rospy.sleep(5.0)

    if not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
