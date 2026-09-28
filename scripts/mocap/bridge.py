#!/usr/bin/env python3
"""Bridge NatNet or DataCollect motion-capture observations into ROS topics."""

import json
import math
import time
import threading
import uuid

from ig_handle_runtime import ros as rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, String
from ig_handle.msg import AcquisitionTimingEvent
from tf2_ros import TransformBroadcaster

from sensors.network import network_value
from sensors.parameters import strict_bool
from mocap.udp.datacollect import DatacollectUdpReceiver
from teensy.timing_event_adapter import TimingEventAdapter


def quat_xyzw(q):
    x, y, z, w = q
    return float(x), float(y), float(z), float(w)


def status_values(value):
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("+Infinity" if value > 0 else "-Infinity")
    if isinstance(value, dict):
        return {key: status_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [status_values(item) for item in value]
    return value


class MocapBridge:
    def __init__(self):
        self.transport = rospy.get_param("~transport", "natnet").strip().lower()
        self.server_ip = rospy.get_param(
            "~server_ip", network_value("mocap_natnet_server_ip")
        )
        self.client_ip = rospy.get_param(
            "~client_ip", network_value("mocap_natnet_client_ip")
        )
        self.natnet_use_multicast = strict_bool(
            rospy.get_param("~natnet_use_multicast", False),
            name="~natnet_use_multicast",
        )
        self.natnet_multicast_address = rospy.get_param(
            "~natnet_multicast_address",
            network_value("mocap_natnet_multicast_address"),
        ).strip()
        self.frame_id = rospy.get_param("~frame_id", "mocap_world")
        self.pub_prefix = rospy.get_param("~topic_prefix", "/mocap")
        self.publish_tf = strict_bool(
            rospy.get_param("~publish_tf", True),
            name="~publish_tf",
        )
        self.child_frame_prefix = rospy.get_param("~child_frame_prefix", "rigid_body_")
        self.datacollect_schema = rospy.get_param(
            "~datacollect_schema", "datacollect.heron.v1"
        )
        self.datacollect_source_ip = rospy.get_param(
            "~datacollect_source_ip", ""
        ).strip()
        self.datacollect_reject_unexpected_source = strict_bool(
            rospy.get_param("~datacollect_reject_unexpected_source", False),
            name="~datacollect_reject_unexpected_source",
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

        self.timing = TimingEventAdapter(listen=False)
        self._datacollect_event = None
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
        self.tf_broadcaster = TransformBroadcaster(rospy.get_node()) if self.publish_tf else None

        self.client = None
        self._natnet_lock = threading.RLock()
        self._natnet_session = ""
        if self.transport not in ("natnet", "datacollect_udp"):
            raise RuntimeError("unsupported mocap transport %r" % self.transport)
        rospy.on_shutdown(self._stop_natnet)

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

        from mocap.natnet.NatNetClient import NatNetClient

        self._stop_natnet()
        client = NatNetClient()
        client.set_client_address(self.client_ip)
        client.set_server_address(self.server_ip)
        client.set_use_multicast(self.natnet_use_multicast)
        if self.natnet_multicast_address:
            client.set_multicast_address(self.natnet_multicast_address)
        if hasattr(client, "set_print_level"):
            client.set_print_level(0)
        session = str(uuid.uuid4())
        client.new_frame_with_data_listener = lambda frame: self._natnet_frame(client, session, frame)
        with self._natnet_lock:
            self.client, self._natnet_session = client, session
        try:
            rospy.loginfo("Starting NatNet client...")
            if not client.run("d"):
                raise RuntimeError("NatNetClient failed to start.")
            rospy.sleep(0.5)
            if hasattr(client, "connected") and not client.connected():
                raise RuntimeError("NatNetClient not connected.")
        except BaseException:
            self._stop_natnet()
            raise
        rospy.loginfo("NatNet streaming.")

    def _stop_natnet(self):
        with self._natnet_lock:
            client, self.client = self.client, None
            self._natnet_session = ""
            if client is not None:
                client.new_frame_with_data_listener = None
        if client is not None:
            client.shutdown()

    def _natnet_frame(self, client, session, frame):
        with self._natnet_lock:
            if client is not self.client or session != self._natnet_session:
                return
            self._on_frame(frame, source_instance=self.server_ip + ":" + session)

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

    def _reference_event(self, *, stamp, sequence, raw_seconds, clock_domain,
                         source_instance="", auxiliary_clocks=()):
        try:
            sequence = int(sequence) if sequence is not None else None
            if sequence is not None and not 0 <= sequence < (1 << 32):
                sequence = None
        except (TypeError, ValueError, OverflowError):
            sequence = None
        try:
            raw_seconds = float(raw_seconds) if raw_seconds is not None else None
        except (TypeError, ValueError, OverflowError):
            raw_seconds = None
        valid_sequence = sequence is not None
        valid_time = (raw_seconds is not None and math.isfinite(raw_seconds)
                      and 0 <= raw_seconds < (1 << 32))
        raw = rospy.Time.from_sec(float(raw_seconds)) if valid_time else rospy.Time()
        return self.timing._publish_event(
            header=Header(stamp=stamp, frame_id=self.frame_id),
            source_id="mocap_{}".format(self.transport),
            source_epoch=0, source_sequence=0 if sequence is None else int(sequence),
            source_sequence_bits=32, source_sequence_valid=valid_sequence,
            raw_source_time=raw, raw_source_time_valid=valid_time,
            raw_source_time_seconds=raw_seconds, source_clock_domain=clock_domain,
            kind=AcquisitionTimingEvent.KIND_MOCAP_FRAME,
            source_instance_id=source_instance or self.timing.bridge_instance_id,
            source_receipt_ros_time=stamp, source_receipt_monotonic_ns=time.monotonic_ns(),
            auxiliary_clocks=auxiliary_clocks)

    @staticmethod
    def _reference_stamp(event):
        if event.mapped_acquisition_time_valid and event.clock_mapping_calibrated:
            return event.mapped_acquisition_time
        return event.receipt_ros_time

    def _on_frame(self, frame, *, source_instance):
        data = frame["mocap_data"]
        suffix = data.suffix_data
        auxiliary = []
        for name in ("stamp_camera_mid_exposure", "stamp_data_received", "stamp_transmit",
                     "prec_timestamp_secs", "prec_timestamp_frac_secs"):
            value = getattr(suffix, name, -1)
            if int(value) >= 0:
                auxiliary.append((name, int(value), math.nan))
        receipt = rospy.Time.now()
        event = self._reference_event(
            stamp=receipt, sequence=int(frame["frame_number"]),
            raw_seconds=float(suffix.timestamp),
            clock_domain="natnet_server_relative_seconds_unmapped",
            source_instance=source_instance, auxiliary_clocks=auxiliary)
        stamp = self._reference_stamp(event)
        bodies = []
        for body in data.rigid_body_data.rigid_body_list:
            valid_pose = (all(math.isfinite(float(v)) for v in (*body.pos, *body.rot))
                          and sum(float(v) ** 2 for v in body.rot) > 0)
            valid = bool(body.tracking_valid) and valid_pose
            bodies.append({"id": int(body.id_num), "tracking_valid": bool(body.tracking_valid),
                           "pose_finite": valid_pose, "marker_error": float(body.error),
                           "position_m": list(body.pos), "orientation_xyzw": list(body.rot)})
            if not valid:
                continue
            msg = self._pose_msg(stamp, body.pos, body.rot)
            msg.header.seq = int(frame["frame_number"]) & 0xffffffff
            self._pub_for_rb(int(body.id_num)).publish(msg)
            if int(body.id_num) == 1:
                self.heron_pose_pub.publish(msg)
            self._publish_pose_tf(stamp, body.id_num, body.pos, body.rot)
        self.status_pub.publish(String(data=json.dumps(status_values({
            "transport": "natnet", "frame": int(frame["frame_number"]),
            "timing_event_id": event.original_event_id,
            "pose_stamp_semantics": "mapped_acquisition" if event.clock_mapping_calibrated
                                    else "receipt_unmapped",
            "rigid_bodies": bodies}), sort_keys=True, allow_nan=False)))

    def _cloud_msg(self, stamp, points):
        header = Header(stamp=stamp, frame_id=self.frame_id)
        return pc2.create_cloud_xyz32(header, points)

    def _publish_status(
        self, packet, status_state, stamp, source_address=None, tracking_valid=None
    ):
        self._datacollect_event = None
        if packet.get("frame") is not None:
            timing = packet.get("timing", {})
            if not isinstance(timing, dict):
                timing = {}
            self._datacollect_event = self._reference_event(
                stamp=stamp, sequence=packet.get("frame"),
                raw_seconds=timing.get("source_time_seconds"),
                clock_domain=str(timing.get("source_clock_domain", "datacollect_source_time_unavailable")),
                source_instance=str(timing.get("source_instance_id", packet.get("device", ""))))
        status = {
            "schema": packet.get("schema"),
            "status": status_state,
            "device": packet.get("device", ""),
            "frame": packet.get("frame"),
            "stamp": stamp.to_sec(),
            "original_packet": packet,
            "timing_event_id": (self._datacollect_event.original_event_id
                                if self._datacollect_event is not None else ""),
            "pose_stamp_semantics": ("mapped_acquisition"
                if self._datacollect_event is not None and self._datacollect_event.clock_mapping_calibrated
                else "receipt_unmapped"),
        }
        if self.datacollect_source_ip:
            status["expected_source_ip"] = self.datacollect_source_ip
        if source_address is not None:
            status["source_ip"] = source_address[0]
            status["source_port"] = source_address[1]
        if tracking_valid is not None:
            status["tracking_valid"] = bool(tracking_valid)
        self.status_pub.publish(String(data=json.dumps(status_values(status), sort_keys=True, allow_nan=False)))

    def _publish_datacollect_pose(self, stamp, rb_id, position, rotation):
        event = self._datacollect_event
        pose_stamp = self._reference_stamp(event) if event is not None else stamp
        msg = self._pose_msg(pose_stamp, position, rotation)
        if event is not None and event.source_sequence_valid:
            msg.header.seq = int(event.source_sequence) & 0xffffffff
        self.heron_pose_pub.publish(msg)
        self._publish_pose_tf(pose_stamp, rb_id, position, rotation)

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
