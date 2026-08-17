#!/usr/bin/env python3
"""Raw UDP receiver for Imagenex sonar datagrams.

This node byte-preserves accepted vendor datagrams. It intentionally does not
filter on packet kind, while rejecting undersized traffic that cannot carry the
minimum vendor header used by downstream provenance checks.
"""

import ipaddress
import socket
from dataclasses import dataclass
from typing import Optional, Tuple

import rospy
from ig_handle.msg import SonarRawPacket as SonarRawPacketMessage
from ig_handle_runtime.network_config import network_value
from ig_handle_runtime.parameters import strict_bool


@dataclass(frozen=True)
class RawSonarPacket:
    """Raw vendor datagram plus optional UDP source metadata."""

    data: bytes
    source: Tuple[str, int]


class SonarRawReceiver:
    """Receive Imagenex UDP datagrams with receipt and source provenance."""

    def __init__(self):
        self.port = int(rospy.get_param("~port", 4040))
        self.topic = str(rospy.get_param("~topic", "/sensors/sonar/raw"))
        self.bind_ip = str(
            rospy.get_param("~bind_ip", network_value("mocap_udp_bind_ip"))
        )
        self.provider = str(rospy.get_param("~provider", "imagenex_udp"))
        self.model = str(rospy.get_param("~model", "unknown"))
        self.hardware_commissioned = strict_bool(
            rospy.get_param("~hardware_commissioned", False),
            name="~hardware_commissioned",
        )
        self.require_expected_source = strict_bool(
            rospy.get_param("~require_expected_source", False),
            name="~require_expected_source",
        )
        self.expected_source_ip = str(
            rospy.get_param("~expected_source_ip", "") or ""
        ).strip()
        self.expected_source_port = int(
            rospy.get_param("~expected_source_port", 0) or 0
        )
        self.frame_id = str(rospy.get_param("~frame_id", "") or "").strip()
        self.extrinsic_revision = str(
            rospy.get_param("~extrinsic_revision", "") or ""
        ).strip()
        if (
            not self.provider
            or not self.model
            or not self.frame_id
            or not self.extrinsic_revision
        ):
            raise RuntimeError(
                "~provider, ~model, ~frame_id, and ~extrinsic_revision are required for raw sonar provenance"
            )
        if self.expected_source_ip:
            try:
                ipaddress.ip_address(self.expected_source_ip)
            except ValueError as exc:
                raise RuntimeError(
                    "~expected_source_ip must be a numeric IP address"
                ) from exc
        if not 0 <= self.expected_source_port <= 65535:
            raise RuntimeError("~expected_source_port must be 0 or in [1, 65535]")
        if self.require_expected_source and not self.expected_source_ip:
            raise RuntimeError(
                "~expected_source_ip is required when ~require_expected_source=true"
            )
        if self.provider == "imagenex_dt100":
            if not self.hardware_commissioned:
                raise RuntimeError(
                    "DT100 hardware ingress is uncommissioned; set ~hardware_commissioned=true only after endpoint verification"
                )
            if not self.require_expected_source:
                raise RuntimeError(
                    "DT100 hardware ingress requires ~require_expected_source=true"
                )
        self.sequence = 0
        self.publisher = rospy.Publisher(
            self.topic, SonarRawPacketMessage, queue_size=50
        )
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
            "Sonar UDP listener provider=%s model=%s frame=%s revision=%s on %s:%d -> %s expected_source=%s%s commissioned=%s",
            self.provider,
            self.model,
            self.frame_id,
            self.extrinsic_revision,
            self.bind_ip,
            self.port,
            self.topic,
            self.expected_source_ip or "*",
            (
                ":{}".format(self.expected_source_port)
                if self.expected_source_port
                else ""
            ),
            self.hardware_commissioned,
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
        source_ip, source_port = src
        if self.expected_source_ip and source_ip != self.expected_source_ip:
            rospy.logwarn_throttle(
                5.0,
                "Rejected sonar UDP source %s:%d; expected IP %s",
                source_ip,
                source_port,
                self.expected_source_ip,
            )
            return
        if self.expected_source_port and source_port != self.expected_source_port:
            rospy.logwarn_throttle(
                5.0,
                "Rejected sonar UDP source %s:%d; expected port %d",
                source_ip,
                source_port,
                self.expected_source_port,
            )
            return
        if data[:3] != b"83P":
            rospy.logdebug("Non-83P frame from %s len=%d", src, len(data))
        self.last_packet = RawSonarPacket(data=data, source=src)
        msg = SonarRawPacketMessage()
        msg.header.seq = self.sequence & 0xFFFFFFFF
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.provider = self.provider
        msg.model = self.model
        msg.packet_kind = data[:3].decode("ascii", errors="replace")
        msg.source_endpoint = "{}:{}".format(src[0], src[1])
        msg.extrinsic_revision = self.extrinsic_revision
        msg.sequence = self.sequence
        msg.payload = list(self.last_packet.data)
        self.sequence += 1
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
