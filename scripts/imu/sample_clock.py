#!/usr/bin/env python3
"""Join IMU measurements to typed raw device-clock events without resampling."""

from copy import deepcopy
from pathlib import Path
from threading import Lock
from time import monotonic_ns
from uuid import uuid4


class SampleClock:
    """Map sensor-clock nanoseconds while preserving measured sample intervals."""

    def __init__(self, epoch=0, mapping_revision=0):
        self.offset_ns = None
        self.receipt_ns = None
        self.sensor_ns = None
        self.epoch = int(epoch)
        self.mapping_revision = int(mapping_revision)

    def map(self, receipt_ns, sensor_ns):
        if receipt_ns <= 0 or sensor_ns < 0:
            return None, "invalid_timestamp", self.epoch
        if self.receipt_ns is not None and receipt_ns < self.receipt_ns:
            return None, "out_of_order_receipt", self.epoch
        if self.sensor_ns is not None and sensor_ns == self.sensor_ns:
            return None, "duplicate_sensor_sample", self.epoch
        if self.sensor_ns is not None and sensor_ns < self.sensor_ns:
            return None, "out_of_order_sensor_sample", self.epoch
        event = "mapped"
        if self.offset_ns is None:
            event = "anchored"
            self.offset_ns = receipt_ns - sensor_ns
            self.mapping_revision += 1
        self.receipt_ns = receipt_ns
        self.sensor_ns = sensor_ns
        return self.offset_ns + sensor_ns, event, self.epoch


class SampleClockNode:
    """Correlate each IMU packet with its matching Xsens clock event."""

    def __init__(self):
        import rospy
        from ig_handle.msg import ClockedImu
        from sensor_msgs.msg import Imu
        from std_srvs.srv import Trigger, TriggerResponse
        from xsens_mti_driver.msg import XsensClockEvent

        self.ros = rospy
        self.TriggerResponse = TriggerResponse
        self.ClockedImu = ClockedImu
        self.capacity = int(rospy.get_param("~queue_size", 256))
        if self.capacity < 1:
            raise ValueError("IMU sample-clock queue_size must be positive")
        output_topic = rospy.get_param("~output_topic")
        timed_output_topic = rospy.get_param("~timed_output_topic")
        clock_event_topic = rospy.get_param("~clock_event_topic")
        if rospy.resolve_name(clock_event_topic) == rospy.resolve_name(output_topic):
            raise ValueError("IMU clock-event input and canonical output must differ")

        self.clock = SampleClock()
        self.source_id = rospy.get_param("~source_id", "xsens_imu")
        self.source_boot_id = uuid4().hex
        self.source_instance_id = None
        self.device_host_boot_id = None
        self.source_instance_generation = None
        self.device_clock_epoch = None
        self.device_sequence_bits = None
        self.device_ros_time_epoch = None
        self.last_device_sequence = None
        self.last_device_source_ns = None
        self.retired_source_instance_ids = set()
        self.seen_host_boot_ids = set()
        self.retired_host_boot_ids = set()
        self.local_host_boot_id = self._read_host_boot_id()
        self.use_sim_time = bool(rospy.get_param("/use_sim_time", False))
        self.default_clock_domain = rospy.get_param(
            "~clock_domain", "xsens_sample_clock"
        )
        # This node anchors device time to receipt time. The resulting offset
        # includes unknown transport delay and cannot be declared calibrated.
        self.clock_mapping_calibrated = False
        self.timing_uncertainty_sec = float("nan")
        self.sequence = 0
        self.last_ros_ns = None
        self.pending_ros_reset = False
        self.explicit_reanchor_pending = False
        self.reanchor_request_monotonic_ns = None
        self.ros_reset_monotonic_ns = None
        self.lock = Lock()
        self.publisher = rospy.Publisher(output_topic, Imu, queue_size=self.capacity)
        self.timed_publisher = rospy.Publisher(
            timed_output_topic, ClockedImu, queue_size=self.capacity
        )
        self.time_subscriber = rospy.Subscriber(
            clock_event_topic, XsensClockEvent, self._time, queue_size=self.capacity
        )
        reanchor_service = rospy.get_param(
            "~reanchor_service", "~reanchor_source_clock"
        )
        self.reanchor_server = rospy.Service(
            reanchor_service, Trigger, self._reanchor_source_clock
        )
        rospy.loginfo(
            "IMU sample clock consumes atomic Xsens measurement/clock records on %s into %s and %s; calibration=%s",
            clock_event_topic,
            output_topic,
            timed_output_topic,
            self.clock_mapping_calibrated,
        )

    @staticmethod
    def _read_host_boot_id():
        try:
            return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        except OSError:
            return ""

    def _reanchor_source_clock(self, _request):
        with self.lock:
            self.explicit_reanchor_pending = True
            self.reanchor_request_monotonic_ns = monotonic_ns()
            epoch = self.clock.epoch
        return self.TriggerResponse(
            success=True,
            message=(
                "IMU source clock reanchor requested; the current mapping is "
                "preserved unless the next matched event confirms a raw-clock "
                "rollback from source epoch %d"
            )
            % (epoch + 1),
        )

    @staticmethod
    def _sequence_delta(sequence, previous, bits):
        modulus = 1 << int(bits)
        delta = (int(sequence) - int(previous)) & (modulus - 1)
        return None if delta == 0 or delta >= modulus // 2 else delta

    def _time(self, message):
        self._receive(message)

    def _receive(self, event):
        with self.lock:
            now_ns = self.ros.Time.now().to_nsec()
            if self.last_ros_ns is not None and now_ns < self.last_ros_ns:
                self.pending_ros_reset = True
                self.ros_reset_monotonic_ns = monotonic_ns()
                self.ros.logwarn(
                    "IMU sample clock observed ROS-time rollback; waiting for source-epoch evidence"
                )
            self.last_ros_ns = now_ns

            receipt_ns = event.header.stamp.to_nsec()
            if receipt_ns <= 0:
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock skipped an event without a valid ROS receipt timestamp",
                )
                return
            bits = int(event.source_sequence_bits)
            if bits not in (16, 32):
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock skipped event with unsupported sequence width %d",
                    bits,
                )
                return
            if not event.measurement_valid:
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock received timing without a valid IMU measurement",
                )
                return
            imu = event.measurement
            if imu.header.stamp != event.header.stamp:
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock skipped an atomic record with inconsistent embedded timestamps",
                )
                return
            packet_sequence = int(event.source_sequence) & ((1 << bits) - 1)

            source_instance_id = event.source_instance_id
            device_boot_id = event.device_boot_id if event.device_boot_id_valid else ""
            host_id = event.host_boot_id
            generation = int(event.process_start_monotonic_ns)
            device_clock_epoch = int(event.device_clock_epoch)
            ros_epoch = int(event.ros_time_epoch)
            source_ns = event.source_time.to_nsec()
            if (
                not source_instance_id
                or (bool(event.device_boot_id_valid) and not device_boot_id)
                or (not bool(event.device_boot_id_valid) and bool(event.device_boot_id))
                or not host_id
                or generation <= 0
                or source_ns < 0
                or int(event.receipt_monotonic_ns) <= 0
            ):
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock skipped event with invalid device identity or raw time",
                )
                return

            same_source_instance = source_instance_id == self.source_instance_id
            host_changed = host_id != self.device_host_boot_id
            source_instance_changed = not same_source_instance
            if (
                host_id in self.retired_host_boot_ids
                or source_instance_id in self.retired_source_instance_ids
            ):
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock skipped a delayed packet from a retired source epoch",
                )
                return
            if host_changed and host_id in self.seen_host_boot_ids:
                self.ros.logwarn_throttle(
                    5.0, "IMU sample clock skipped a packet from a retired host boot"
                )
                return
            if (
                same_source_instance
                and self.device_clock_epoch is not None
                and device_clock_epoch < self.device_clock_epoch
            ):
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock skipped a packet from a retired device-clock epoch",
                )
                return
            device_clock_changed = (
                same_source_instance
                and self.device_clock_epoch is not None
                and device_clock_epoch > self.device_clock_epoch
            )
            reanchor_request_active = (
                self.explicit_reanchor_pending
                and host_id == self.local_host_boot_id
                and self.reanchor_request_monotonic_ns is not None
                and int(event.receipt_monotonic_ns)
                >= self.reanchor_request_monotonic_ns
            )
            source_clock_rollback = (
                same_source_instance
                and self.last_device_source_ns is not None
                and source_ns < self.last_device_source_ns
            )
            explicit_reanchor_event = reanchor_request_active and source_clock_rollback
            if (
                same_source_instance
                and self.last_device_source_ns is not None
                and not device_clock_changed
            ):
                if source_ns == self.last_device_source_ns:
                    self.ros.logwarn_throttle(
                        5.0, "IMU sample clock skipped duplicate raw device time"
                    )
                    return
                if (
                    source_ns < self.last_device_source_ns
                    and not explicit_reanchor_event
                ):
                    self.ros.logwarn_throttle(
                        5.0, "IMU sample clock skipped out-of-order raw device time"
                    )
                    return
            if (
                same_source_instance
                and self.device_ros_time_epoch is not None
                and ros_epoch < self.device_ros_time_epoch
            ):
                self.ros.logwarn_throttle(
                    5.0, "IMU sample clock skipped an event from a prior ROS-time epoch"
                )
                return

            if reanchor_request_active and not source_clock_rollback:
                self.explicit_reanchor_pending = False
                self.reanchor_request_monotonic_ns = None
                self.ros.loginfo(
                    "IMU source clock remained chronological after reanchor request; preserving mapping epoch %d",
                    self.clock.epoch,
                )

            if self.pending_ros_reset:
                same_host_clock = (
                    host_id == self.local_host_boot_id and not self.use_sim_time
                )
                if same_host_clock and self.ros_reset_monotonic_ns is not None:
                    if int(event.receipt_monotonic_ns) < self.ros_reset_monotonic_ns:
                        self.ros.logwarn_throttle(
                            5.0,
                            "IMU sample clock rejected a pre-reset event using monotonic receipt time",
                        )
                        return
                elif (
                    not self.explicit_reanchor_pending
                    and same_source_instance
                    and self.device_ros_time_epoch is not None
                    and ros_epoch <= self.device_ros_time_epoch
                ):
                    self.ros.logwarn_throttle(
                        5.0,
                        "IMU sample clock is waiting for the driver's incremented ROS-time epoch",
                    )
                    return

            if (
                same_source_instance
                and self.last_device_sequence is not None
                and not device_clock_changed
                and not explicit_reanchor_event
            ):
                delta = self._sequence_delta(
                    packet_sequence, self.last_device_sequence, bits
                )
                if delta is None:
                    self.ros.logwarn_throttle(
                        5.0,
                        "IMU source counter is duplicate, out of order, or ambiguous after wrap/outage",
                    )
                    return
                if delta > 1:
                    self.ros.logwarn_throttle(
                        5.0,
                        "IMU source counter indicates at least %d missing packets",
                        delta - 1,
                    )

            source_epoch_changed = (
                same_source_instance
                and self.device_ros_time_epoch is not None
                and ros_epoch > self.device_ros_time_epoch
            )
            device_clock_epoch_changed = device_clock_changed or explicit_reanchor_event
            if (
                source_instance_changed
                or source_epoch_changed
                or device_clock_epoch_changed
                or self.pending_ros_reset
            ):
                self.clock = SampleClock(
                    self.clock.epoch + 1, self.clock.mapping_revision
                )
                self.sequence = 0
                self.pending_ros_reset = False
                self.explicit_reanchor_pending = False
                self.reanchor_request_monotonic_ns = None
                self.ros_reset_monotonic_ns = None
            if source_instance_changed:
                if self.source_instance_id is not None:
                    self.retired_source_instance_ids.add(self.source_instance_id)
                if host_changed and self.device_host_boot_id is not None:
                    self.retired_host_boot_ids.add(self.device_host_boot_id)
                self.seen_host_boot_ids.add(host_id)
                self.source_instance_id = source_instance_id
                self.device_host_boot_id = host_id
                self.source_instance_generation = generation
                self.device_clock_epoch = device_clock_epoch
            elif device_clock_changed:
                self.device_clock_epoch = device_clock_epoch
            self.device_sequence_bits = bits
            self.device_ros_time_epoch = ros_epoch

            mapped_ns, event_name, source_epoch = self.clock.map(receipt_ns, source_ns)
            if mapped_ns is None:
                self.ros.logwarn_throttle(
                    5.0, "IMU sample clock skipped unusable event: %s", event_name
                )
                return
            self.last_device_sequence = packet_sequence
            self.last_device_source_ns = source_ns
            if event_name == "anchored":
                self.ros.loginfo(
                    "IMU source clock anchored at epoch %d; receipt offset includes uncalibrated transport delay",
                    source_epoch,
                )

            output = deepcopy(imu)
            sec, nsec = divmod(mapped_ns, 1000000000)
            output.header.stamp = self.ros.Time(sec, nsec)
            self.publisher.publish(output)
            timed = self.ClockedImu()
            timed.header = deepcopy(output.header)
            timed.source_id = self.source_id
            timed.source_boot_id = self.source_boot_id
            timed.device_boot_id_valid = bool(event.device_boot_id_valid)
            timed.device_boot_id = device_boot_id
            timed.source_instance_id = source_instance_id
            timed.device_host_boot_id = host_id
            timed.source_instance_generation = generation
            timed.device_clock_epoch = device_clock_epoch
            timed.clock_domain = self.default_clock_domain
            timed.source_epoch = source_epoch
            timed.source_sequence = packet_sequence
            timed.source_sequence_bits = bits
            timed.source_time = event.source_time
            timed.receipt_time = event.header.stamp
            timed.receipt_host_boot_id = host_id
            timed.receipt_monotonic_ns = int(event.receipt_monotonic_ns)
            timed.ros_time_epoch = ros_epoch
            timed.timing_uncertainty_sec = self.timing_uncertainty_sec
            timed.clock_mapping_revision = self.clock.mapping_revision
            timed.clock_mapping_calibrated = self.clock_mapping_calibrated
            timed.measurement = output
            self.sequence += 1
            self.timed_publisher.publish(timed)


def main():
    import rospy

    rospy.init_node("imu_sample_clock")
    SampleClockNode()
    rospy.spin()


if __name__ == "__main__":
    main()
