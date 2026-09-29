#!/usr/bin/env python3
"""Preserve identified sensor timing events without claiming clock calibration."""

import json
import copy
from collections import OrderedDict
import math

from sensors.clock_mapping import parse_clock_mappings, apply_clock_mapping
import threading
import time
import uuid
from pathlib import Path

import rospy
from ig_handle.msg import AcquisitionTimingEvent, FirmwareTimingEvent
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, TriggerResponse


_FIRMWARE_SOURCE_SPECS = {
    (FirmwareTimingEvent.KIND_PPS_REFERENCE, 0): (
        "lidar_pair_vlp16",
        AcquisitionTimingEvent.KIND_PPS_REFERENCE,
    ),
    (FirmwareTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT, 0): (
        "camera_f1",
        AcquisitionTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT,
    ),
    (FirmwareTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT, 1): (
        "camera_f2",
        AcquisitionTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT,
    ),
    (FirmwareTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT, 2): (
        "camera_f3",
        AcquisitionTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT,
    ),
    (FirmwareTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT, 3): (
        "camera_f4",
        AcquisitionTimingEvent.KIND_CAMERA_EXPOSURE_MIDPOINT,
    ),
    (FirmwareTimingEvent.KIND_IMU_SYNC_OUT, 0): (
        "xsens_mti30",
        AcquisitionTimingEvent.KIND_IMU_SYNC_OUT,
    ),
    (FirmwareTimingEvent.KIND_SENSOR_TRIGGER_COMMAND, 0): (
        "teensy_sensor_trigger",
        AcquisitionTimingEvent.KIND_SENSOR_TRIGGER_COMMAND,
    ),
}


def _load_timing_calibration(path):
    path = str(path or "").strip()
    if not path:
        return {}, {}
    document = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    mappings = parse_clock_mappings(document)
    correlations = {}
    unique_events = {}
    for record in document.get("correlations", []):
        capture = str(record.get("capture_id", "")).strip()
        event_ids = [str(value).strip() for value in record.get("event_ids", [])]
        shared = set(str(value) for value in record.get("shared_trigger_event_ids", []))
        revision = int(record.get("revision", 0))
        if (
            not capture
            or revision <= 0
            or len(event_ids) < 2
            or any(not value for value in event_ids)
            or len(set(event_ids)) != len(event_ids)
            or not shared.issubset(event_ids)
        ):
            raise ValueError(
                "measured correlation needs identity, revision and distinct event IDs"
            )
        normalized = {
            "capture_id": capture,
            "event_ids": event_ids,
            "shared_trigger_event_ids": sorted(shared),
            "revision": revision,
        }
        if capture in correlations and correlations[capture] != normalized:
            raise ValueError("one capture has conflicting measured correlations")
        for identity in set(event_ids) - shared:
            previous = unique_events.setdefault(identity, capture)
            if previous != capture:
                raise ValueError("one frame/exposure event is assigned to two captures")
        correlations[capture] = normalized
    return mappings, correlations


def _merge_revisions(active, proposed):
    merged = dict(active)
    for identity, record in proposed.items():
        old = active.get(identity)
        if old is not None and (
            record["revision"] < old["revision"]
            or (record["revision"] == old["revision"] and record != old)
        ):
            raise ValueError("calibration revision cannot roll back or change in place")
        merged[identity] = record
    return merged


def _host_boot_id():
    try:
        with open("/proc/sys/kernel/random/boot_id", "r", encoding="ascii") as stream:
            return stream.read().strip()
    except OSError:
        return ""


class TimingEventAdapter:
    """Translate only identified existing firmware events into typed records."""

    def __init__(self, *, listen=True):
        self.firmware_event_topic = rospy.get_param(
            "~firmware_event_topic", "/sensors/timing/firmware_event"
        )
        self.status_topic = rospy.get_param(
            "~timing_status_topic", "/sensors/timing/status"
        )
        self.output_topic = rospy.get_param(
            "~timing_events_topic", "/sensors/timing/events"
        )
        self.bridge_instance_id = str(uuid.uuid4())
        self.receipt_host_boot_id = _host_boot_id()
        self.firmware_build_id = ""
        self._source_state = {}
        self._restart_candidates = {}
        self._bridge_event_sequence = 0
        self._last_receipt_ros_ns = None
        self._ros_time_epoch = 0
        self.calibration_path = str(rospy.get_param("~clock_mappings_file", "") or "")
        try:
            self.clock_mappings, self.correlations = _load_timing_calibration(
                self.calibration_path
            )
        except (OSError, TypeError, ValueError) as error:
            self.clock_mappings, self.correlations = {}, {}
            rospy.logerr(
                "timing calibration unavailable: %s; raw clocks remain unmapped", error
            )
        self._state_lock = threading.RLock()
        self._event_history = OrderedDict()
        self._history_limit = int(rospy.get_param("~retained_timing_events", 10000))
        if self._history_limit <= 0:
            raise ValueError("retained_timing_events must be positive")
        self._mapped_sources = {}
        self._index_correlations()
        self._publisher = rospy.Publisher(
            self.output_topic, AcquisitionTimingEvent, queue_size=1000
        )
        self._reload_service = rospy.Service(
            "~reload_timing_calibration", Trigger, self._reload_calibration
        )
        if not listen:
            return
        camera_topics = rospy.get_param("~camera_frame_timing_topics", "")
        if isinstance(camera_topics, str):
            camera_topics = [
                topic.strip() for topic in camera_topics.split(",") if topic.strip()
            ]
        if camera_topics:
            from spinnaker_camera_driver.msg import CameraFrameTiming
        self._camera_subscribers = [
            rospy.Subscriber(
                topic, CameraFrameTiming, self._camera_frame, queue_size=1000
            )
            for topic in camera_topics
        ]
        if self.status_topic:
            rospy.Subscriber(self.status_topic, String, self._status, queue_size=10)
        if self.firmware_event_topic:
            rospy.Subscriber(
                self.firmware_event_topic,
                FirmwareTimingEvent,
                self._firmware_event,
                queue_size=1000,
            )

        self.lidar_pointcloud_topic = str(
            rospy.get_param("~lidar_pointcloud_topic", "") or ""
        ).strip()
        self.lidar_source_id = str(
            rospy.get_param("~lidar_source_id", "") or ""
        ).strip()
        self._lidar_subscriber = None
        if self.lidar_pointcloud_topic and self.lidar_source_id:
            self._lidar_subscriber = rospy.Subscriber(
                self.lidar_pointcloud_topic,
                PointCloud2,
                self._lidar_cloud,
                queue_size=100,
            )
        elif self.lidar_pointcloud_topic or self.lidar_source_id:
            rospy.logwarn(
                "LiDAR timing observation needs both lidar_pointcloud_topic and lidar_source_id"
            )

    def _status(self, message):
        build_id = ""
        for token in str(message.data or "").split():
            key, separator, value = token.partition("=")
            if separator and key == "firmware_build_id":
                build_id = value.strip()
                break
        with self._state_lock:
            self.firmware_build_id = build_id

    @staticmethod
    def _sequence_delta(previous, current, sequence_bits=32):
        return (current - previous) & ((1 << sequence_bits) - 1)

    @staticmethod
    def _is_restart_rollback(raw_ns, previous_raw_ns):
        return raw_ns < previous_raw_ns and raw_ns <= previous_raw_ns // 2

    def _store_source_state(
        self, source_id, generation, epoch, sequence, raw_ns, publish_ns, ros_epoch
    ):
        self._source_state[source_id] = (
            generation,
            epoch,
            sequence,
            raw_ns,
            publish_ns,
            ros_epoch,
        )

    def _order(
        self,
        source_id,
        epoch,
        sequence,
        raw_ns,
        publish_ns,
        ros_epoch,
        sequence_bits=32,
    ):
        previous = self._source_state.get(source_id)
        if previous is None:
            self._store_source_state(
                source_id, 0, epoch, sequence, raw_ns, publish_ns, ros_epoch
            )
            return 0, AcquisitionTimingEvent.ORDER_FIRST, 0

        (
            generation,
            previous_epoch,
            previous_sequence,
            previous_raw_ns,
            previous_publish_ns,
            previous_ros_epoch,
        ) = previous
        delta = self._sequence_delta(previous_sequence, sequence, sequence_bits)
        candidate = self._restart_candidates.get(source_id)

        if candidate is not None:
            (
                candidate_epoch,
                candidate_sequence,
                candidate_raw_ns,
                candidate_publish_ns,
            ) = candidate
            candidate_delta = self._sequence_delta(
                candidate_sequence, sequence, sequence_bits
            )
            candidate_advances = (
                epoch == candidate_epoch
                and 0 < candidate_delta < (1 << (sequence_bits - 1))
                and raw_ns > candidate_raw_ns
                and publish_ns > candidate_publish_ns
            )
            candidate_is_reset_scale = self._is_restart_rollback(
                candidate_raw_ns, previous_raw_ns
            )
            sequence_only_restart = (
                epoch == previous_epoch
                and candidate_raw_ns > previous_raw_ns
                and delta >= (1 << (sequence_bits - 1))
                and raw_ns > candidate_raw_ns
            )
            still_below_old_highwater = (
                epoch <= previous_epoch
                and raw_ns < previous_raw_ns
                and (publish_ns > previous_publish_ns or ros_epoch > previous_ros_epoch)
            )
            if candidate_advances and (
                (candidate_is_reset_scale and still_below_old_highwater)
                or sequence_only_restart
            ):
                generation += 1
                self._restart_candidates.pop(source_id, None)
                self._store_source_state(
                    source_id,
                    generation,
                    epoch,
                    sequence,
                    raw_ns,
                    publish_ns,
                    ros_epoch,
                )
                return (
                    generation,
                    (
                        AcquisitionTimingEvent.ORDER_SEQUENCE_RESTART_INFERRED
                        if sequence_only_restart
                        else AcquisitionTimingEvent.ORDER_SOURCE_RESTART_INFERRED
                    ),
                    0,
                )

            old_stream_advances = (
                epoch == previous_epoch
                and 0 < delta < (1 << (sequence_bits - 1))
                and raw_ns >= previous_raw_ns
                and (
                    publish_ns >= previous_publish_ns or ros_epoch > previous_ros_epoch
                )
            )
            if old_stream_advances or epoch > previous_epoch:
                self._restart_candidates.pop(source_id, None)
            elif candidate_advances:
                if candidate_is_reset_scale:
                    self._restart_candidates[source_id] = (
                        epoch,
                        sequence,
                        raw_ns,
                        publish_ns,
                    )
                else:
                    self._restart_candidates.pop(source_id, None)
                return generation, AcquisitionTimingEvent.ORDER_OUT_OF_ORDER, 0
            else:
                return generation, AcquisitionTimingEvent.ORDER_OUT_OF_ORDER, 0

        if epoch > previous_epoch:
            missing = max(0, delta - 1) if delta < (1 << (sequence_bits - 1)) else 0
            self._store_source_state(
                source_id, generation, epoch, sequence, raw_ns, publish_ns, ros_epoch
            )
            return generation, AcquisitionTimingEvent.ORDER_CLOCK_EPOCH, missing

        if epoch < previous_epoch:
            rollback = (
                self._is_restart_rollback(raw_ns, previous_raw_ns)
                and publish_ns > previous_publish_ns
            )
            if rollback:
                self._restart_candidates[source_id] = (
                    epoch,
                    sequence,
                    raw_ns,
                    publish_ns,
                )
            return generation, AcquisitionTimingEvent.ORDER_OUT_OF_ORDER, 0

        if delta == 0:
            if raw_ns < previous_raw_ns and publish_ns >= previous_publish_ns:
                self._restart_candidates[source_id] = (
                    epoch,
                    sequence,
                    raw_ns,
                    publish_ns,
                )
                return generation, AcquisitionTimingEvent.ORDER_OUT_OF_ORDER, 0
            return generation, AcquisitionTimingEvent.ORDER_DUPLICATE, 0

        if delta >= (1 << (sequence_bits - 1)):
            rollback = (
                self._is_restart_rollback(raw_ns, previous_raw_ns)
                or raw_ns > previous_raw_ns
            ) and publish_ns > previous_publish_ns
            if rollback:
                self._restart_candidates[source_id] = (
                    epoch,
                    sequence,
                    raw_ns,
                    publish_ns,
                )
            return generation, AcquisitionTimingEvent.ORDER_OUT_OF_ORDER, 0

        if raw_ns < previous_raw_ns:
            if (
                self._is_restart_rollback(raw_ns, previous_raw_ns)
                and 0 < delta < (1 << (sequence_bits - 1))
                and (publish_ns > previous_publish_ns or ros_epoch > previous_ros_epoch)
            ):
                self._restart_candidates[source_id] = (
                    epoch,
                    sequence,
                    raw_ns,
                    publish_ns,
                )
            return generation, AcquisitionTimingEvent.ORDER_OUT_OF_ORDER, 0

        disposition = (
            AcquisitionTimingEvent.ORDER_GAP
            if delta > 1
            else AcquisitionTimingEvent.ORDER_FORWARD
        )
        self._store_source_state(
            source_id, generation, epoch, sequence, raw_ns, publish_ns, ros_epoch
        )
        return generation, disposition, max(0, delta - 1)

    def _publish_event(
        self,
        header,
        source_id,
        source_epoch,
        source_sequence,
        source_sequence_bits,
        raw_source_time,
        source_clock_domain,
        kind,
        sensor_frame_counter=None,
        source_instance_id="",
        source_receipt_ros_time=None,
        source_receipt_monotonic_ns=None,
        source_clock_epoch_valid=False,
        sequence_origin=AcquisitionTimingEvent.SEQUENCE_COUNTER_SOURCE,
        correlation_sequence=None,
        correlation_source_id="",
        source_sequence_valid=True,
        raw_source_time_valid=True,
        raw_source_time_seconds=None,
        auxiliary_clocks=(),
    ):
        sequence_mask = (1 << source_sequence_bits) - 1
        source_sequence = int(source_sequence) & sequence_mask
        raw_ns = raw_source_time.to_nsec()
        publish_ns = header.stamp.to_nsec()
        receipt_monotonic_ns = (
            time.monotonic_ns()
            if source_receipt_monotonic_ns is None
            else int(source_receipt_monotonic_ns)
        )
        source_state_id = (
            source_id
            if not source_instance_id
            else "{}:{}".format(source_id, source_instance_id)
        )

        with self._state_lock:
            current_ros_time = rospy.Time.now()
            current_ros_ns = current_ros_time.to_nsec()
            if (
                self._last_receipt_ros_ns is not None
                and current_ros_ns < self._last_receipt_ros_ns
            ):
                self._ros_time_epoch += 1
            self._last_receipt_ros_ns = current_ros_ns
            ros_time_epoch = self._ros_time_epoch
            receipt_ros = (
                current_ros_time
                if source_receipt_ros_time is None
                else source_receipt_ros_time
            )
            generation, disposition, missing = (
                self._order(
                    source_state_id,
                    source_epoch,
                    source_sequence,
                    raw_ns,
                    publish_ns,
                    ros_time_epoch,
                    sequence_bits=source_sequence_bits,
                )
                if source_sequence_valid and raw_source_time_valid
                else (
                    self._source_state.get(source_state_id, (0,))[0],
                    AcquisitionTimingEvent.ORDER_UNAVAILABLE,
                    0,
                )
            )
            self._bridge_event_sequence += 1
            bridge_event_sequence = self._bridge_event_sequence
            firmware_build_id = self.firmware_build_id

        event = AcquisitionTimingEvent()
        event.header = header
        event.kind = kind
        event.identity_scope = AcquisitionTimingEvent.IDENTITY_SCOPE_BRIDGE_INSTANCE
        event.event_id = "{}:{}".format(self.bridge_instance_id, bridge_event_sequence)
        event.bridge_event_sequence = bridge_event_sequence
        epoch_key = str(source_epoch) if source_clock_epoch_valid else "unknown"
        event.source_key = "{}:{}:{}:generation-{}:epoch-{}:sequence-{}".format(
            self.bridge_instance_id,
            source_id,
            source_instance_id,
            generation,
            epoch_key,
            source_sequence,
        )
        if not source_sequence_valid:
            event.source_key += ":unidentified-event-{}".format(bridge_event_sequence)
        event.source_id = source_id
        event.source_instance_id = source_instance_id
        event.source_sequence_valid = bool(source_sequence_valid)
        event.source_sequence = source_sequence
        event.source_sequence_origin = sequence_origin
        event.bridge_instance_id = self.bridge_instance_id
        event.firmware_build_id = firmware_build_id
        event.source_boot_id_valid = False
        event.source_boot_id = ""
        event.observed_source_generation = generation
        event.source_generation_inferred = generation > 0
        event.source_clock_epoch = source_epoch
        event.source_clock_epoch_valid = source_clock_epoch_valid
        event.source_sequence_bits = source_sequence_bits
        event.source_clock_domain = source_clock_domain
        event.clock_mapping_source_instance_id = ":".join(
            (self.bridge_instance_id, source_id, source_instance_id)
        )
        event.clock_mapping_source_epoch_id = "{}:generation-{}".format(
            epoch_key, generation
        )
        event.raw_source_time_valid = bool(raw_source_time_valid)
        event.raw_source_time = raw_source_time
        event.raw_source_time_seconds = (
            math.nan
            if raw_source_time_seconds is None
            else float(raw_source_time_seconds)
        )
        event.auxiliary_clock_names = [name for name, ticks, rate in auxiliary_clocks]
        event.auxiliary_clock_ticks = [
            int(ticks) for name, ticks, rate in auxiliary_clocks
        ]
        event.auxiliary_clock_frequency_hz = [
            float(rate) for name, ticks, rate in auxiliary_clocks
        ]
        event.receipt_ros_time = receipt_ros
        event.receipt_monotonic_ns = receipt_monotonic_ns
        event.receipt_host_boot_id = self.receipt_host_boot_id
        event.ros_time_epoch = ros_time_epoch
        event.order_disposition = disposition
        event.missing_source_sequences = missing
        event.correlation_sequence_valid = correlation_sequence is not None
        event.correlation_source_id = (
            str(correlation_source_id) if correlation_sequence is not None else ""
        )
        event.correlation_sequence = (
            0 if correlation_sequence is None else int(correlation_sequence)
        )
        event.correlated_capture_ids = []
        event.correlated_event_ids = []
        event.correlation_status = AcquisitionTimingEvent.CORRELATION_UNRESOLVED
        event.correlation_revision = 0
        event.derived_revision = False
        event.original_event_id = event.event_id
        event.mapped_continuity_epoch = 0
        event.sensor_frame_counter_valid = sensor_frame_counter is not None
        event.sensor_frame_counter = (
            0 if sensor_frame_counter is None else int(sensor_frame_counter)
        )
        event.mapped_acquisition_time_valid = False
        event.mapped_acquisition_time = rospy.Time()
        event.clock_mapping_revision = 0
        event.timing_uncertainty_sec = math.nan
        event.clock_mapping_calibrated = False
        with self._state_lock:
            self._event_history[event.original_event_id] = copy.deepcopy(event)
            while len(self._event_history) > self._history_limit:
                self._event_history.popitem(last=False)
            self._derive_event(event)
            self._event_history[event.original_event_id] = copy.deepcopy(event)
            related = {
                identity
                for record in self._correlation_index.get(event.original_event_id, [])
                for identity in record["event_ids"]
                if identity != event.original_event_id
            }
            revisions = self._rederive_retained(related)
        self._publisher.publish(event)
        for revision in revisions:
            self._publisher.publish(revision)
        if disposition in (
            AcquisitionTimingEvent.ORDER_GAP,
            AcquisitionTimingEvent.ORDER_OUT_OF_ORDER,
            AcquisitionTimingEvent.ORDER_SOURCE_RESTART_INFERRED,
        ):
            rospy.logwarn_throttle(
                10.0,
                "Timing source %s ordering=%d missing=%d",
                source_id,
                disposition,
                missing,
            )
        return event

    def _derive_event(self, event):
        event.mapped_acquisition_time_valid = False
        event.mapped_acquisition_time = rospy.Time()
        event.clock_mapping_revision = 0
        event.timing_uncertainty_sec = math.nan
        event.clock_mapping_calibrated = False
        event.correlated_capture_ids = []
        event.correlated_event_ids = []
        event.correlation_revision = 0
        event.correlation_status = AcquisitionTimingEvent.CORRELATION_UNRESOLVED
        mapping_key = (
            event.source_clock_domain,
            event.clock_mapping_source_instance_id,
            event.clock_mapping_source_epoch_id,
            "ros_time",
            self.receipt_host_boot_id,
            str(event.ros_time_epoch),
        )
        mapping = self.clock_mappings.get(mapping_key)
        if mapping is not None:
            event.clock_mapping_revision = mapping["revision"]
        if (
            mapping is not None
            and event.raw_source_time_valid
            and event.order_disposition
            not in (
                AcquisitionTimingEvent.ORDER_DUPLICATE,
                AcquisitionTimingEvent.ORDER_OUT_OF_ORDER,
            )
        ):
            mapped = apply_clock_mapping(mapping, event.raw_source_time.to_nsec())
            if mapped is not None:
                mapped_ns, uncertainty, calibrated = mapped
                event.mapped_acquisition_time_valid = True
                event.mapped_acquisition_time = rospy.Time(
                    mapped_ns // 1000000000, mapped_ns % 1000000000
                )
                event.clock_mapping_revision = mapping["revision"]
                event.timing_uncertainty_sec = uncertainty
                event.clock_mapping_calibrated = calibrated
                timeline = self._mapped_sources.get(mapping_key)
                continuity = 0 if timeline is None else timeline[2]
                raw_ns = event.raw_source_time.to_nsec()
                if not event.derived_revision and (
                    timeline is None or raw_ns > timeline[0]
                ):
                    if timeline is not None and mapped_ns <= timeline[1]:
                        continuity += 1
                        rospy.logwarn(
                            "mapped timing chronology changed for %s; continuity=%d",
                            event.source_id,
                            continuity,
                        )
                    self._mapped_sources[mapping_key] = (raw_ns, mapped_ns, continuity)
                event.mapped_continuity_epoch = continuity

        links = self._correlation_index.get(event.original_event_id, [])
        complete = []
        missing = False
        for record in links:
            if all(identity in self._event_history for identity in record["event_ids"]):
                try:
                    unresolved = self._check_correlation_sources(
                        {record["capture_id"]: record}
                    )
                except ValueError:
                    event.correlation_status = (
                        AcquisitionTimingEvent.CORRELATION_CONFLICT
                    )
                    return
                if record["capture_id"] not in unresolved:
                    complete.append(record)
            else:
                missing = True
        if complete:
            event.correlated_capture_ids = sorted(
                {record["capture_id"] for record in complete}
            )
            event.correlated_event_ids = sorted(
                {identity for record in complete for identity in record["event_ids"]}
            )
            event.correlation_revision = max(record["revision"] for record in complete)
            event.correlation_status = AcquisitionTimingEvent.CORRELATION_MEASURED
        elif links and missing:
            event.correlation_status = AcquisitionTimingEvent.CORRELATION_MISSING_EVENTS

    def _index_correlations(self):
        self._correlation_index = {}
        for record in self.correlations.values():
            for identity in record["event_ids"]:
                self._correlation_index.setdefault(identity, []).append(record)

    def _check_correlation_sources(self, correlations):
        owners = {}
        unresolved = set()
        for record in correlations.values():
            for identity in set(record["event_ids"]) - set(
                record["shared_trigger_event_ids"]
            ):
                previous = owners.setdefault(identity, record["capture_id"])
                if previous != record["capture_id"]:
                    raise ValueError(
                        "one frame/exposure event has conflicting capture ownership"
                    )
            events = [
                self._event_history.get(identity) for identity in record["event_ids"]
            ]
            if any(event is None for event in events):
                continue
            frames = [
                event
                for event in events
                if event.kind == AcquisitionTimingEvent.KIND_CAMERA_FRAME
            ]
            if frames:
                if len(frames) != 1:
                    raise ValueError(
                        "camera correspondence must contain exactly one frame"
                    )
                frame = frames[0]
                prefix = "spinnaker_camera_"
                if not frame.source_id.startswith(prefix):
                    raise ValueError(
                        "camera correspondence has no identified camera source"
                    )
                serial = frame.source_id[len(prefix) :]
                capture = "camera:{}:{}:{}".format(
                    serial, frame.source_instance_id, frame.sensor_frame_counter
                )
                if capture != record["capture_id"]:
                    raise ValueError(
                        "camera correspondence does not identify its original capture"
                    )
            for identity in record["shared_trigger_event_ids"]:
                event = self._event_history[identity]
                if event.kind != AcquisitionTimingEvent.KIND_SENSOR_TRIGGER_COMMAND:
                    raise ValueError(
                        "only an identified trigger may be shared between captures"
                    )
            firmware = [event for event in events if event.correlation_sequence_valid]
            scopes = {
                (
                    event.bridge_instance_id,
                    event.source_instance_id,
                    event.correlation_source_id,
                    event.correlation_sequence,
                )
                for event in firmware
            }
            epochs = {
                event.source_clock_epoch
                for event in firmware
                if event.source_clock_epoch_valid
            }
            if len(scopes) > 1 or len(epochs) > 1:
                raise ValueError(
                    "trigger/exposure correspondence crosses a source epoch or trigger sequence"
                )
            if firmware and any(
                not event.source_clock_epoch_valid for event in firmware
            ):
                unresolved.add(record["capture_id"])
        return unresolved

    @staticmethod
    def _derived_signature(event):
        uncertainty = float(event.timing_uncertainty_sec)
        return (
            event.mapped_acquisition_time_valid,
            event.mapped_acquisition_time.to_nsec(),
            event.clock_mapping_revision,
            event.clock_mapping_calibrated,
            uncertainty if math.isfinite(uncertainty) else None,
            event.mapped_continuity_epoch,
            tuple(event.correlated_capture_ids),
            tuple(event.correlated_event_ids),
            event.correlation_status,
            event.correlation_revision,
        )

    def _rederive_retained(self, identities):
        revised = []
        for identity in identities:
            original = self._event_history.get(identity)
            if original is None:
                continue
            event = copy.deepcopy(original)
            event.derived_revision = True
            self._derive_event(event)
            if self._derived_signature(event) == self._derived_signature(original):
                continue
            self._bridge_event_sequence += 1
            event.bridge_event_sequence = self._bridge_event_sequence
            event.event_id = "{}:{}".format(
                self.bridge_instance_id, self._bridge_event_sequence
            )
            self._event_history[event.original_event_id] = copy.deepcopy(event)
            revised.append(event)
        return revised

    def _refresh_mapping_timelines(self, previous_mappings):
        """Use one declared corrected continuity for retained and future events."""
        for key, mapping in self.clock_mappings.items():
            if previous_mappings.get(key) == mapping:
                continue
            timeline = self._mapped_sources.get(key)
            eligible = [
                event
                for event in self._event_history.values()
                if event.raw_source_time_valid
                and event.order_disposition
                not in (
                    AcquisitionTimingEvent.ORDER_DUPLICATE,
                    AcquisitionTimingEvent.ORDER_OUT_OF_ORDER,
                )
                and (
                    event.source_clock_domain,
                    event.clock_mapping_source_instance_id,
                    event.clock_mapping_source_epoch_id,
                    "ros_time",
                    self.receipt_host_boot_id,
                    str(event.ros_time_epoch),
                )
                == key
            ]
            raw_ns = max(
                [event.raw_source_time.to_nsec() for event in eligible]
                + ([] if timeline is None else [timeline[0]]),
                default=None,
            )
            if raw_ns is None:
                continue
            mapped = apply_clock_mapping(mapping, raw_ns)
            if mapped is None:
                continue
            continuity = 0 if timeline is None else timeline[2]
            if timeline is not None:
                corrected_previous = apply_clock_mapping(mapping, timeline[0])
                if (
                    corrected_previous is not None
                    and corrected_previous[0] < timeline[1]
                ):
                    continuity += 1
            self._mapped_sources[key] = (raw_ns, mapped[0], continuity)

    def _reload_calibration(self, _request):
        try:
            mappings, correlations = _load_timing_calibration(
                rospy.get_param("~clock_mappings_file", self.calibration_path)
            )
            with self._state_lock:
                candidate = copy.copy(self)
                candidate.clock_mappings = _merge_revisions(
                    self.clock_mappings, mappings
                )
                candidate.correlations = _merge_revisions(
                    self.correlations, correlations
                )
                candidate._event_history = copy.deepcopy(self._event_history)
                candidate._mapped_sources = dict(self._mapped_sources)
                candidate._check_correlation_sources(candidate.correlations)
                candidate._index_correlations()
                candidate._refresh_mapping_timelines(self.clock_mappings)
                revised = candidate._rederive_retained(list(candidate._event_history))
                for attribute in (
                    "clock_mappings",
                    "correlations",
                    "_correlation_index",
                    "_event_history",
                    "_mapped_sources",
                    "_bridge_event_sequence",
                ):
                    setattr(self, attribute, getattr(candidate, attribute))
            for event in revised:
                self._publisher.publish(event)
            return TriggerResponse(
                success=True,
                message=(
                    "installed measured timing revisions; {} mappings, {} correlations, "
                    "{} retained event revisions"
                ).format(len(mappings), len(correlations), len(revised)),
            )
        except (OSError, TypeError, ValueError, OverflowError) as error:
            return TriggerResponse(success=False, message=str(error))

    def _firmware_event(self, message):
        spec = _FIRMWARE_SOURCE_SPECS.get(
            (int(message.kind), int(message.source_index))
        )
        if spec is None:
            rospy.logwarn_throttle(
                10.0,
                "Ignoring unknown typed firmware timing event kind=%d source_index=%d",
                int(message.kind),
                int(message.source_index),
            )
            return
        if (
            int(message.source_sequence_bits) != 32
            or int(message.raw_time_nsec) >= 1000000000
        ):
            rospy.logwarn_throttle(
                10.0,
                "Ignoring malformed firmware timing event sequence width or timestamp",
            )
            return

        source_id, kind = spec
        receipt_ros_time = rospy.Time.now()
        header = Header()
        header.stamp = receipt_ros_time
        raw_source_time = rospy.Time(
            int(message.raw_time_sec), int(message.raw_time_nsec)
        )
        self._publish_event(
            header=header,
            source_id=source_id,
            source_epoch=(
                int(message.source_clock_epoch)
                if message.source_clock_epoch_valid
                else 0
            ),
            source_sequence=int(message.source_sequence),
            source_sequence_bits=32,
            raw_source_time=raw_source_time,
            source_clock_domain="teensy_relative_monotonic_unmapped",
            source_instance_id=self.bridge_instance_id,
            kind=kind,
            source_receipt_ros_time=receipt_ros_time,
            source_receipt_monotonic_ns=time.monotonic_ns(),
            source_clock_epoch_valid=bool(message.source_clock_epoch_valid),
            correlation_sequence=(
                int(message.correlation_sequence)
                if message.correlation_sequence_valid
                else None
            ),
            correlation_source_id=(
                "teensy_sensor_trigger" if message.correlation_sequence_valid else ""
            ),
        )

    def _camera_frame(self, message):
        source_id = "spinnaker_camera_{}".format(message.camera_serial)
        timestamp_ns = int(message.device_timestamp_ns)
        raw_source_time = rospy.Time(
            timestamp_ns // 1000000000, timestamp_ns % 1000000000
        )
        self._publish_event(
            header=message.header,
            source_id=source_id,
            source_epoch=0,
            source_sequence=message.frame_counter,
            source_sequence_bits=64,
            raw_source_time=raw_source_time,
            source_clock_domain="spinnaker_device_timestamp_ns",
            kind=AcquisitionTimingEvent.KIND_CAMERA_FRAME,
            sensor_frame_counter=message.frame_counter,
            source_instance_id=message.stream_id,
            source_receipt_ros_time=message.header.stamp,
            source_receipt_monotonic_ns=message.host_monotonic_ns,
            source_clock_epoch_valid=False,
        )

    def _lidar_cloud(self, message):
        """Record a cloud-header/receipt pair without asserting acquisition semantics."""

        receipt_ros_time = rospy.Time.now()
        raw_source_time = message.header.stamp
        if raw_source_time == rospy.Time():
            rospy.logwarn_throttle(
                10.0,
                "LiDAR %s published a zero point-cloud header stamp; preserving it as unmapped",
                self.lidar_source_id,
            )
        event_header = Header()
        event_header.stamp = receipt_ros_time
        self._publish_event(
            header=event_header,
            source_id=self.lidar_source_id,
            source_epoch=0,
            source_sequence=int(message.header.seq),
            source_sequence_bits=32,
            raw_source_time=raw_source_time,
            raw_source_time_valid=raw_source_time != rospy.Time(),
            source_clock_domain="ros_pointcloud_header_unverified",
            kind=AcquisitionTimingEvent.KIND_LIDAR_CLOUD_HEADER,
            source_instance_id=self.bridge_instance_id,
            source_receipt_ros_time=receipt_ros_time,
            source_receipt_monotonic_ns=time.monotonic_ns(),
            source_clock_epoch_valid=False,
            sequence_origin=AcquisitionTimingEvent.SEQUENCE_COUNTER_ROS_HEADER,
        )


def main():
    rospy.init_node("teensy_timing_event_adapter")
    TimingEventAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
