#!/usr/bin/env python3
"""Datacollect UDP receiver for Motive-side Heron mocap packets."""

import json
import math
import socket

import rospy

from sensors.parameters import strict_bool


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


class DatacollectUdpReceiver:
    def __init__(
        self,
        *,
        bind_ip,
        port,
        schema,
        expected_source_ip,
        reject_unexpected_source,
        stale_timeout_sec,
        publish_status,
        publish_pose,
        publish_points,
    ):
        self.bind_ip = bind_ip
        self.port = int(port)
        self.schema = schema
        self.expected_source_ip = expected_source_ip
        self.reject_unexpected_source = strict_bool(
            reject_unexpected_source,
            name="reject_unexpected_source",
        )
        self.stale_timeout_sec = float(stale_timeout_sec)
        self.publish_status = publish_status
        self.publish_pose = publish_pose
        self.publish_points = publish_points
        self.last_packet_time = None

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_ip, self.port))
        sock.settimeout(0.2)
        rospy.loginfo(
            "Listening for datacollect Heron mocap UDP packets on %s:%d",
            self.bind_ip,
            self.port,
        )
        while not rospy.is_shutdown():
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                self._publish_stale_if_needed()
                continue

            try:
                packet = json.loads(data.decode("utf-8"))
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "Invalid datacollect mocap JSON from %s: %s", address, exc
                )
                continue
            self._publish_packet(packet, address)

    def _publish_stale_if_needed(self):
        if self.last_packet_time is None:
            return
        if (
            rospy.Time.now() - self.last_packet_time
        ).to_sec() <= self.stale_timeout_sec:
            return
        stale_packet = {
            "schema": self.schema,
            "status": {"state": "stale"},
            "device": "",
            "frame": None,
        }
        self.publish_status(stale_packet, "stale", rospy.Time.now())
        self.last_packet_time = None

    def _publish_packet(self, packet, source_address):
        if not isinstance(packet, dict):
            rospy.logwarn_throttle(
                5.0,
                "Ignoring non-object datacollect mocap packet from %s",
                source_address,
            )
            return
        if packet.get("schema") != self.schema:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring datacollect mocap packet with schema=%r from %s",
                packet.get("schema"),
                source_address,
            )
            return

        stamp = rospy.Time.now()
        source_ip = source_address[0] if source_address is not None else ""
        unexpected_source = (
            bool(self.expected_source_ip) and source_ip != self.expected_source_ip
        )
        status = packet.get("status", {})
        status_state = status.get("state", "ok") if isinstance(status, dict) else "ok"
        heron = packet.get("heron", {})
        if not isinstance(heron, dict):
            heron = {}
        rigid_body = heron.get("rigid_body", {})
        if not isinstance(rigid_body, dict):
            rigid_body = {}
        if unexpected_source:
            rospy.logwarn_throttle(
                5.0,
                "Datacollect mocap packet from unexpected source %s; expected %s",
                source_ip,
                self.expected_source_ip,
            )
            if self.reject_unexpected_source:
                self.publish_status(
                    packet,
                    "unexpected_source",
                    stamp,
                    source_address=source_address,
                    tracking_valid=False,
                )
                return

        try:
            tracking_valid = strict_bool(
                heron.get("tracking_valid", False),
                name="heron.tracking_valid",
            )
        except ValueError as exc:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring datacollect mocap packet with invalid tracking state from %s: %s",
                source_address,
                exc,
            )
            self.publish_status(
                packet,
                "invalid_tracking_valid",
                stamp,
                source_address=source_address,
                tracking_valid=False,
            )
            return

        self.publish_status(
            packet,
            status_state,
            stamp,
            source_address=source_address,
            tracking_valid=tracking_valid,
        )
        if status_state != "ok" or not tracking_valid:
            return

        position = _xyz_from_packet(rigid_body.get("position_m"))
        rotation = _quat_from_packet(rigid_body.get("orientation_xyzw"))
        if position is None or rotation is None:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring datacollect mocap packet with invalid Heron pose from %s",
                source_address,
            )
            return

        try:
            rb_id = int(rigid_body.get("id", 1))
        except (TypeError, ValueError):
            rb_id = 1
        self.publish_pose(stamp, rb_id, position, rotation)

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
        self.publish_points(stamp, marker_points, potential_points)
        self.last_packet_time = stamp
