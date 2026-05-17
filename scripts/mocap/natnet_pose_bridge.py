#!/usr/bin/env python3
import json
import math
import os
import socket
import sys
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from natnet.NatNetClient import NatNetClient


def quat_xyzw_from_natnet(q):
    x, y, z, w = q
    return float(x), float(y), float(z), float(w)


def _finite(values):
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _xyz_from_packet(value):
    if value is None:
        return None
    if isinstance(value, dict):
        xyz = [value.get("x"), value.get("y"), value.get("z")]
    else:
        try:
            xyz = list(value[:3])
        except (TypeError, IndexError):
            return None
        if len(xyz) != 3:
            return None
    if not _finite(xyz):
        return None
    return [float(v) for v in xyz]


def _quat_from_packet(value):
    if value is None:
        return None
    if isinstance(value, dict):
        quat = [value.get("x"), value.get("y"), value.get("z"), value.get("w")]
    else:
        try:
            quat = list(value[:4])
        except (TypeError, IndexError):
            return None
        if len(quat) != 4:
            return None
    if not _finite(quat):
        return None
    return [float(v) for v in quat]


def _point_from_packet(item):
    if not isinstance(item, dict):
        return None
    point = item.get("position_m", item.get("position", item.get("point")))
    if point is None:
        return None
    return _xyz_from_packet(point)


class NatNetBridge:
    def __init__(self):
        self.transport = rospy.get_param("~transport", "natnet").strip().lower()
        self.server_ip = rospy.get_param("~server_ip", "192.168.1.199")
        self.client_ip = rospy.get_param("~client_ip", "192.168.1.8")
        self.frame_id = rospy.get_param("~frame_id", "mocap_world")
        self.pub_prefix = rospy.get_param("~topic_prefix", "/mocap")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.child_frame_prefix = rospy.get_param("~child_frame_prefix", "rigid_body_")
        self.datacollect_schema = rospy.get_param(
            "~datacollect_schema", "datacollect.heron.v1"
        )
        self.udp_bind_ip = rospy.get_param("~udp_bind_ip", "0.0.0.0")
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
        self.last_packet_time = None

        self.client = None
        if self.transport == "natnet":
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
        elif self.transport != "datacollect_udp":
            raise RuntimeError(
                "unsupported mocap transport %r; expected natnet or datacollect_udp"
                % self.transport
            )

        rospy.loginfo(
            "NatNetBridge configured. transport=%s server_ip=%s client_ip=%s udp=%s:%d"
            % (
                self.transport,
                self.server_ip,
                self.client_ip,
                self.udp_bind_ip,
                self.udp_port,
            )
        )

    def start(self):
        if self.transport == "datacollect_udp":
            self._run_datacollect_udp()
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

        qx, qy, qz, qw = quat_xyzw_from_natnet(rotation)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_pose_tf(self, stamp, rb_id, position, rotation):
        if self.tf_broadcaster is None:
            return
        qx, qy, qz, qw = quat_xyzw_from_natnet(rotation)
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

    def _publish_status(self, packet, status_state, stamp):
        status = {
            "schema": packet.get("schema"),
            "status": status_state,
            "device": packet.get("device", ""),
            "frame": packet.get("frame"),
            "stamp": stamp.to_sec(),
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def _publish_datacollect_packet(self, packet, source_address):
        if not isinstance(packet, dict):
            rospy.logwarn_throttle(
                5.0,
                "Ignoring non-object datacollect mocap packet from %s"
                % (source_address,),
            )
            return
        if packet.get("schema") != self.datacollect_schema:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring datacollect mocap packet with schema=%r from %s"
                % (packet.get("schema"), source_address),
            )
            return

        stamp = rospy.Time.now()
        status = packet.get("status", {})
        status_state = status.get("state", "ok") if isinstance(status, dict) else "ok"
        heron = packet.get("heron", {})
        if not isinstance(heron, dict):
            heron = {}
        rigid_body = heron.get("rigid_body", {})
        if not isinstance(rigid_body, dict):
            rigid_body = {}
        tracking_valid = bool(heron.get("tracking_valid", False))

        self._publish_status(packet, status_state, stamp)
        if status_state != "ok" or not tracking_valid:
            return

        position = _xyz_from_packet(rigid_body.get("position_m"))
        rotation = _quat_from_packet(rigid_body.get("orientation_xyzw"))
        if position is None or rotation is None:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring datacollect mocap packet with invalid Heron pose from %s"
                % (source_address,),
            )
            return

        try:
            rb_id = int(rigid_body.get("id", 1))
        except (TypeError, ValueError):
            rb_id = 1
        self.heron_pose_pub.publish(self._pose_msg(stamp, position, rotation))
        self._publish_pose_tf(stamp, rb_id, position, rotation)

        markers = heron.get("markers", []) or []
        potential_objects = heron.get("potential_objects", []) or []
        marker_points = [
            point
            for point in (_point_from_packet(item) for item in markers)
            if point is not None
        ]
        potential_points = [
            point
            for point in (_point_from_packet(item) for item in potential_objects)
            if point is not None
        ]
        self.markers_pub.publish(self._cloud_msg(stamp, marker_points))
        self.potential_objects_pub.publish(self._cloud_msg(stamp, potential_points))
        self.last_packet_time = rospy.Time.now()

    def _run_datacollect_udp(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.udp_bind_ip, self.udp_port))
        sock.settimeout(0.2)
        rospy.loginfo(
            "Listening for datacollect Heron mocap UDP packets on %s:%d"
            % (self.udp_bind_ip, self.udp_port)
        )
        while not rospy.is_shutdown():
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                if (
                    self.last_packet_time is not None
                    and (rospy.Time.now() - self.last_packet_time).to_sec()
                    > self.stale_timeout_sec
                ):
                    stale_packet = {
                        "schema": self.datacollect_schema,
                        "status": {"state": "stale"},
                        "device": "",
                        "frame": None,
                    }
                    self._publish_status(stale_packet, "stale", rospy.Time.now())
                    self.last_packet_time = None
                continue
            try:
                packet = json.loads(data.decode("utf-8"))
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "Invalid datacollect mocap JSON from %s: %s" % (address, exc)
                )
                continue
            self._publish_datacollect_packet(packet, address)


def main():
    rospy.init_node("natnet_pose_bridge", anonymous=False)
    bridge = NatNetBridge()

    while not rospy.is_shutdown():
        try:
            bridge.start()
            break
        except RuntimeError as e:
            rospy.logwarn("[NatNetBridge] %s; retrying in 5s", e)
            rospy.sleep(5.0)

    if not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
