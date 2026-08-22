#!/usr/bin/env python3
"""Blue Robotics Ping360 UDP provider with a read-only identity mode."""

from __future__ import annotations

import hashlib
import json
import socket
import struct
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rospy

from ig_handle.msg import Ping360RawPacket, SonarDiagnostics, SonarProfile
from sensors.parameters import strict_bool
from .ping_protocol import (
    DEVICE_INFORMATION,
    PING360_AUTO_DEVICE_DATA,
    PING360_DEVICE_DATA,
    PING360_MOTOR_OFF,
    PROTOCOL_VERSION,
    PingProtocolError,
    build_auto_transmit,
    build_frame,
    build_general_request,
    parse_device_information,
    parse_frame,
    parse_profile,
    parse_protocol_version,
    profile_identity_rejection,
)


@dataclass
class Counters:
    packets_received: int = 0
    profiles_published: int = 0
    checksum_errors: int = 0
    malformed_packets: int = 0
    receive_timeouts: int = 0


class Ping360Provider:
    """Publish lossless wire packets plus provider-neutral profiles."""

    def __init__(self) -> None:
        self.host = str(rospy.get_param("~host", "")).strip()
        self.port = int(rospy.get_param("~port", 0))
        self.bind_ip = str(rospy.get_param("~bind_ip", "0.0.0.0"))
        self.bind_port = int(rospy.get_param("~bind_port", 0))
        self.operation_mode = str(
            rospy.get_param("~operation_mode", "identity")
        ).lower()
        self.allow_active_transmit = strict_bool(
            rospy.get_param("~allow_active_transmit", False),
            name="~allow_active_transmit",
        )
        self.frame_id = str(rospy.get_param("~frame_id", "sonar_link"))
        self.extrinsic_revision = str(
            rospy.get_param("~extrinsic_revision", "") or ""
        ).strip()
        self.sound_speed_mps = float(rospy.get_param("~sound_speed_mps", 1480.0))
        self.request_period_sec = float(rospy.get_param("~request_period_sec", 2.0))
        self.socket_timeout_sec = float(rospy.get_param("~socket_timeout_sec", 0.25))
        self.source_device_id = int(rospy.get_param("~source_device_id", 0))
        self.destination_device_id = int(rospy.get_param("~destination_device_id", 0))
        self.scan = self._load_scan_configuration()
        self.configuration_hash = hashlib.sha256(
            json.dumps(self._configuration(), sort_keys=True).encode("utf-8")
        ).hexdigest()
        self._validate_configuration()

        self.raw_topic = str(
            rospy.get_param("~raw_topic", "/sensors/sonar/ping360/raw")
        )
        self.profile_topic = str(
            rospy.get_param("~profile_topic", "/sensors/sonar/imaging/profile")
        )
        self.diagnostics_topic = str(
            rospy.get_param("~diagnostics_topic", "/sensors/sonar/ping360/diagnostics")
        )
        self.raw_pub = rospy.Publisher(self.raw_topic, Ping360RawPacket, queue_size=100)
        self.profile_pub = rospy.Publisher(
            self.profile_topic, SonarProfile, queue_size=20
        )
        self.diagnostics_pub = rospy.Publisher(
            self.diagnostics_topic, SonarDiagnostics, queue_size=5, latch=True
        )
        self.socket = self._open_socket()
        self.counters = Counters()
        self.sequence = 0
        self.device_info: Optional[tuple] = None
        self.identified_device_id: Optional[int] = None
        self.device_id = self.destination_device_id
        self.protocol_version: Optional[tuple] = None
        self.active_scan_started = False
        self.active_scan_destination_device_id: Optional[int] = None
        self.scan_identity_faulted = False
        self.last_request_time = rospy.Time(0)
        self.last_source: Tuple[str, int] = (self.host, self.port)
        rospy.on_shutdown(self.close)

    def _load_scan_configuration(self) -> Dict[str, int]:
        return {
            "gain_setting": int(rospy.get_param("~gain_setting", 1)),
            "transmit_duration_us": int(rospy.get_param("~transmit_duration_us", 11)),
            "sample_period_ticks_25ns": int(
                rospy.get_param("~sample_period_ticks_25ns", 80)
            ),
            "transmit_frequency_khz": int(
                rospy.get_param("~transmit_frequency_khz", 750)
            ),
            "number_of_samples": int(rospy.get_param("~number_of_samples", 1200)),
            "start_angle_grad": int(rospy.get_param("~start_angle_grad", 0)),
            "stop_angle_grad": int(rospy.get_param("~stop_angle_grad", 399)),
            "num_steps": int(rospy.get_param("~num_steps", 1)),
            "delay_ms": int(rospy.get_param("~delay_ms", 0)),
        }

    def _configuration(self) -> dict:
        return {
            "provider": "blue_robotics_ping360",
            "operation_mode": self.operation_mode,
            "frame_id": self.frame_id,
            "extrinsic_revision": self.extrinsic_revision,
            "sound_speed_mps": self.sound_speed_mps,
            "scan": self.scan,
        }

    def _validate_configuration(self) -> None:
        if not self.host or not 1 <= self.port <= 65535:
            raise ValueError("Ping360 host and UDP port must be explicitly configured")
        if not self.frame_id or not self.extrinsic_revision:
            raise ValueError(
                "Ping360 frame_id and extrinsic_revision must be explicitly configured"
            )
        if self.operation_mode not in ("identity", "scan"):
            raise ValueError("operation_mode must be identity or scan")
        if self.operation_mode == "scan" and not self.allow_active_transmit:
            raise ValueError(
                "scan mode requires allow_active_transmit=true and operator authorization"
            )
        if not 1300.0 <= self.sound_speed_mps <= 1700.0:
            raise ValueError("sound_speed_mps is outside the supported safety range")
        if self.operation_mode == "scan":
            build_auto_transmit(**self.scan)

    def _open_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_ip, self.bind_port))
        sock.connect((self.host, self.port))
        sock.settimeout(self.socket_timeout_sec)
        return sock

    def spin(self) -> None:
        rospy.loginfo(
            "Ping360 %s mode at %s:%d frame=%s revision=%s (active transmit=%s)",
            self.operation_mode,
            self.host,
            self.port,
            self.frame_id,
            self.extrinsic_revision,
            self.allow_active_transmit,
        )
        self._request_identity()
        self._publish_diagnostics("awaiting_identity")
        while not rospy.is_shutdown():
            self._receive_once()
            if self._identity_valid() and self.operation_mode == "scan":
                self._start_scan_once()
            elif not self._identity_valid():
                elapsed = (rospy.Time.now() - self.last_request_time).to_sec()
                if elapsed >= self.request_period_sec:
                    self._request_identity()

    def _request_identity(self) -> None:
        for requested_id in (DEVICE_INFORMATION, PROTOCOL_VERSION):
            self.socket.send(
                build_general_request(
                    requested_id,
                    source_device_id=self.source_device_id,
                    destination_device_id=self.destination_device_id,
                )
            )
        self.last_request_time = rospy.Time.now()

    def _start_scan_once(self) -> None:
        if self.active_scan_started or self.scan_identity_faulted:
            return
        destination_device_id = self._command_destination_device_id()
        self.socket.send(
            build_auto_transmit(
                **self.scan,
                source_device_id=self.source_device_id,
                destination_device_id=destination_device_id,
            )
        )
        self.active_scan_destination_device_id = destination_device_id
        self.active_scan_started = True
        self._publish_diagnostics("scanning")

    def _stop_active_scan(self, state: str, *, latch_identity_fault: bool) -> bool:
        """Stop the bound scan without consulting mutable identity state."""
        if latch_identity_fault:
            self.scan_identity_faulted = True
        if not self.active_scan_started:
            return True
        destination_device_id = self.active_scan_destination_device_id
        self.active_scan_started = False
        self.active_scan_destination_device_id = None
        if destination_device_id is None:
            self._publish_diagnostics("{}_destination_missing".format(state))
            return False
        try:
            self.socket.send(
                build_frame(
                    PING360_MOTOR_OFF,
                    source_device_id=self.source_device_id,
                    destination_device_id=destination_device_id,
                )
            )
        except OSError as error:
            rospy.logerr("Ping360 MOTOR_OFF failed during %s: %s", state, error)
            self._publish_diagnostics("{}_motor_off_failed".format(state))
            return False
        self._publish_diagnostics(state)
        return True

    def _receive_once(self) -> None:
        try:
            data = self.socket.recv(65535)
            source = self.socket.getpeername()
        except socket.timeout:
            self.counters.receive_timeouts += 1
            if self.counters.receive_timeouts % 20 == 0:
                self._publish_diagnostics("timeout")
            return
        except OSError:
            if not rospy.is_shutdown():
                raise
            return
        stamp = rospy.Time.now()
        self.sequence += 1
        self.counters.packets_received += 1
        self.last_source = source
        try:
            frame = parse_frame(data, require_valid_checksum=False)
        except PingProtocolError as error:
            self.counters.malformed_packets += 1
            self._publish_unparsed_raw(data, stamp, source)
            rospy.logwarn_throttle(2.0, "Malformed Ping frame: %s", error)
            self._publish_diagnostics("malformed_packet")
            return
        packet_id = hashlib.sha256(frame.raw).hexdigest()
        self._publish_raw(frame, packet_id, stamp, source)
        if not frame.checksum_valid:
            self.counters.checksum_errors += 1
            self._publish_diagnostics("checksum_error")
            return
        try:
            self._handle_frame(frame, packet_id, stamp)
        except PingProtocolError as error:
            self.counters.malformed_packets += 1
            rospy.logwarn_throttle(2.0, "Invalid Ping payload: %s", error)
            self._publish_diagnostics("malformed_payload")

    def _handle_frame(self, frame, packet_id: str, stamp) -> None:
        information = parse_device_information(frame)
        if information is not None:
            source_id = int(frame.source_device_id)
            if (
                self.destination_device_id and source_id != self.destination_device_id
            ) or (
                self.identified_device_id is not None
                and source_id != self.identified_device_id
            ):
                self._stop_active_scan(
                    "identity_source_mismatch", latch_identity_fault=True
                )
                self.device_info = None
                self.protocol_version = None
                self.identified_device_id = None
                self.device_id = self.destination_device_id
                self._publish_diagnostics("identity_source_mismatch")
                return
            self.device_info = information
            self.identified_device_id = source_id
            self.device_id = source_id
            state = "identity_valid" if self._identity_valid() else "identity_mismatch"
            self._publish_diagnostics(state)
            return
        version = parse_protocol_version(frame)
        if version is not None:
            source_matches = (
                self.identified_device_id is not None
                and frame.source_device_id == self.identified_device_id
            )
            if source_matches:
                self.protocol_version = version
            self._publish_diagnostics(
                ("identity_valid" if self._identity_valid() else "awaiting_identity")
            )
            return
        if frame.message_id not in (PING360_DEVICE_DATA, PING360_AUTO_DEVICE_DATA):
            return
        identity_rejection = profile_identity_rejection(
            self._identity_valid(),
            frame.source_device_id,
            self.identified_device_id,
        )
        if identity_rejection:
            if self.active_scan_started:
                self._stop_active_scan(identity_rejection, latch_identity_fault=True)
            self._publish_diagnostics(identity_rejection)
            rospy.logwarn_throttle(
                2.0,
                "Rejected Ping360 profile: %s (source_device_id=%d identified=%s)",
                identity_rejection,
                frame.source_device_id,
                self.identified_device_id,
            )
            return
        profile = parse_profile(frame)
        msg = SonarProfile()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.profile_id = hashlib.sha256(
            (packet_id + self.configuration_hash).encode("ascii")
        ).hexdigest()
        msg.provider = "blue_robotics_ping360"
        msg.model = "Ping360"
        msg.raw_packet_id = packet_id
        msg.extrinsic_revision = self.extrinsic_revision
        msg.synthetic = False
        msg.sequence = self.sequence
        msg.valid = True
        msg.validity_reason = "validated_ping_protocol_profile"
        msg.angle_rad = profile.angle_rad
        msg.angle_grad = profile.angle_grad
        msg.auto_scan = profile.auto_scan
        msg.start_angle_grad = profile.start_angle_grad
        msg.stop_angle_grad = profile.stop_angle_grad
        msg.num_steps = profile.num_steps
        msg.gain_setting = profile.gain_setting
        msg.transmit_duration_us = profile.transmit_duration_us
        msg.sample_period_ticks_25ns = profile.sample_period_ticks_25ns
        msg.transmit_frequency_khz = profile.transmit_frequency_khz
        msg.sound_speed_mps = self.sound_speed_mps
        msg.sample_interval_m = profile.sample_interval_m(self.sound_speed_mps)
        msg.min_range_m = 0.0
        msg.max_range_m = profile.number_of_samples * msg.sample_interval_m
        msg.intensities = list(profile.intensities)
        self.profile_pub.publish(msg)
        self.counters.profiles_published += 1

    def _publish_raw(self, frame, packet_id: str, stamp, source) -> None:
        msg = Ping360RawPacket()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.packet_id = packet_id
        msg.provider = "blue_robotics_ping360"
        msg.extrinsic_revision = self.extrinsic_revision
        msg.source_address = source[0]
        msg.source_port = source[1]
        msg.sequence = self.sequence
        msg.message_id = frame.message_id
        msg.source_device_id = frame.source_device_id
        msg.destination_device_id = frame.destination_device_id
        msg.checksum_valid = frame.checksum_valid
        msg.data = list(frame.raw)
        self.raw_pub.publish(msg)

    def _publish_unparsed_raw(self, data: bytes, stamp, source) -> None:
        msg = Ping360RawPacket()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.packet_id = hashlib.sha256(data).hexdigest()
        msg.provider = "blue_robotics_ping360"
        msg.extrinsic_revision = self.extrinsic_revision
        msg.source_address = source[0]
        msg.source_port = source[1]
        msg.sequence = self.sequence
        msg.checksum_valid = False
        msg.data = list(data)
        self.raw_pub.publish(msg)

    def _identity_valid(self) -> bool:
        return bool(
            self.device_info is not None
            and self.device_info[0] == 2
            and self.identified_device_id is not None
            and (
                not self.destination_device_id
                or self.identified_device_id == self.destination_device_id
            )
        )

    def _command_destination_device_id(self) -> int:
        if not self._identity_valid():
            raise RuntimeError("Ping360 command requested before identity binding")
        return int(self.identified_device_id)

    def _publish_diagnostics(self, state: str) -> None:
        msg = SonarDiagnostics()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.provider = "blue_robotics_ping360"
        msg.model = "Ping360"
        msg.state = state
        msg.endpoint = "%s:%d" % (self.host, self.port)
        msg.read_only = self.operation_mode == "identity"
        msg.identity_valid = self._identity_valid()
        if self.device_info is not None:
            (
                msg.device_type,
                msg.device_revision,
                msg.firmware_major,
                msg.firmware_minor,
                msg.firmware_patch,
                _reserved,
            ) = self.device_info
        if self.protocol_version is not None:
            (
                msg.protocol_major,
                msg.protocol_minor,
                msg.protocol_patch,
                _reserved,
            ) = self.protocol_version
        msg.device_id = self.device_id
        msg.packets_received = self.counters.packets_received
        msg.profiles_published = self.counters.profiles_published
        msg.checksum_errors = self.counters.checksum_errors
        msg.malformed_packets = self.counters.malformed_packets
        msg.receive_timeouts = self.counters.receive_timeouts
        msg.configuration_hash = self.configuration_hash
        self.diagnostics_pub.publish(msg)

    def close(self) -> None:
        try:
            if self.operation_mode == "scan":
                self._stop_active_scan("shutdown", latch_identity_fault=False)
        finally:
            try:
                self.socket.close()
            except (AttributeError, OSError):
                pass


def run() -> None:
    rospy.init_node("ping360_provider")
    try:
        Ping360Provider().spin()
    except (ValueError, OSError) as error:
        rospy.logfatal("Ping360 provider configuration failed: %s", error)
        raise SystemExit(2)
