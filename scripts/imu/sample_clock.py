#!/usr/bin/env python3
"""Pair Xsens IMU samples with device-clock references and preserve timing."""

from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
from threading import Lock
from time import monotonic_ns
from uuid import uuid4

import rclpy
from builtin_interfaces.msg import Time
from ig_handle.msg import ClockedImu
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, TimeReference
from std_srvs.srv import Trigger


def stamp_to_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_stamp(value):
    stamp = Time()
    stamp.sec, stamp.nanosec = divmod(int(value), 1_000_000_000)
    return stamp


class SampleClock:
    """Receipt-offset map with monotonic guards and explicit reset epochs."""

    def __init__(self):
        self.offset_ns = None
        self.receipt_ns = None
        self.sensor_ns = None
        self.epoch = 0
        self.mapping_revision = 0
        self.pending_reason = "anchored"

    def map(self, receipt_ns, sensor_ns):
        if receipt_ns <= 0 or sensor_ns < 0:
            return None, "invalid_timestamp"
        if self.receipt_ns is not None and receipt_ns <= self.receipt_ns:
            return None, "out_of_order_receipt"
        if self.sensor_ns is not None and sensor_ns == self.sensor_ns:
            return None, "duplicate_sensor_sample"
        if self.sensor_ns is not None and sensor_ns < self.sensor_ns:
            # A large rollback indicates a device clock epoch reset. Smaller
            # regressions are late or reordered packets and are discarded.
            if self.sensor_ns - sensor_ns < 1_000_000_000:
                return None, "out_of_order_sensor_sample"
            self.epoch += 1
            self.mapping_revision += 1
            self.offset_ns = None
            self.pending_reason = "sensor_clock_reset"
        event = "mapped"
        if self.offset_ns is None:
            event = self.pending_reason
            self.offset_ns = receipt_ns - sensor_ns
            if self.mapping_revision == 0:
                self.mapping_revision = 1
        self.receipt_ns = receipt_ns
        self.sensor_ns = sensor_ns
        return self.offset_ns + sensor_ns, event

    def reset_ros_time(self):
        self.mapping_revision += 1
        self.offset_ns = None
        self.receipt_ns = None
        self.sensor_ns = None
        self.pending_reason = "ros_time_reset"

    def reanchor(self):
        self.mapping_revision += 1
        self.offset_ns = None
        self.receipt_ns = None
        self.sensor_ns = None
        self.pending_reason = "operator_reanchor"


class SampleClockNode(Node):
    """Publish canonical IMU data and an auditable typed timing record."""

    def __init__(self):
        super().__init__("imu_sample_clock")
        self.declare_parameter("queue_size", 256)
        self.declare_parameter("input_topic", "")
        self.declare_parameter("time_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("timed_output_topic", "")
        self.declare_parameter("source_id", "xsens_imu")
        self.declare_parameter("clock_domain", "xsens_sample_time")
        self.declare_parameter("reanchor_service", "~/reanchor_source_clock")

        self.capacity = int(self.get_parameter("queue_size").value)
        if self.capacity < 1:
            raise ValueError("IMU sample-clock queue_size must be positive")
        self.input_topic = str(self.get_parameter("input_topic").value).strip()
        self.time_topic = str(self.get_parameter("time_topic").value).strip()
        self.output_topic = str(self.get_parameter("output_topic").value).strip()
        self.timed_output_topic = str(
            self.get_parameter("timed_output_topic").value
        ).strip()
        if not all((self.input_topic, self.time_topic, self.output_topic)):
            raise ValueError("input_topic, time_topic, and output_topic are required")
        if not self.timed_output_topic:
            self.timed_output_topic = self.output_topic + "_timed"
        if self.resolve_topic_name(self.input_topic) == self.resolve_topic_name(
            self.output_topic
        ):
            raise ValueError("IMU receipt input and canonical output must differ")

        self.source_id = str(self.get_parameter("source_id").value)
        self.clock_domain = str(self.get_parameter("clock_domain").value)
        self.source_boot_id = uuid4().hex
        self.source_instance_id = uuid4().hex
        self.host_boot_id = self._read_host_boot_id()
        self.ros_time_epoch = 0
        self.clock = SampleClock()
        self.last_ros_ns = None
        self.pending_imu = OrderedDict()
        self.pending_time = OrderedDict()
        self.lock = Lock()
        qos = QoSProfile(depth=self.capacity)
        self.publisher = self.create_publisher(Imu, self.output_topic, qos)
        self.timed_publisher = self.create_publisher(
            ClockedImu, self.timed_output_topic, qos
        )
        self.imu_subscriber = self.create_subscription(
            Imu, self.input_topic, self._imu, qos
        )
        self.time_subscriber = self.create_subscription(
            TimeReference, self.time_topic, self._time, qos
        )
        reanchor_service = str(self.get_parameter("reanchor_service").value)
        self.reanchor_server = self.create_service(
            Trigger, reanchor_service, self._reanchor_source_clock
        )
        self.get_logger().info(
            "Pairing receipt-stamped IMU and TimeReference on %s and %s; publishing %s and %s"
            % (
                self.input_topic,
                self.time_topic,
                self.output_topic,
                self.timed_output_topic,
            )
        )

    @staticmethod
    def _read_host_boot_id():
        try:
            return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        except OSError:
            return ""

    def _imu(self, message):
        self._receive(message, is_time=False)

    def _time(self, message):
        self._receive(message, is_time=True)

    def _receive(self, message, *, is_time):
        with self.lock:
            now_ns = self.get_clock().now().nanoseconds
            if self.last_ros_ns is not None and now_ns < self.last_ros_ns:
                self.clock.reset_ros_time()
                self.ros_time_epoch += 1
                self.pending_imu.clear()
                self.pending_time.clear()
                self.get_logger().warning(
                    "ROS time moved backward; starting a new IMU timing epoch"
                )
            self.last_ros_ns = now_ns

            receipt_ns = stamp_to_ns(message.header.stamp)
            pending = self.pending_time if is_time else self.pending_imu
            pending[receipt_ns] = message
            if len(pending) > self.capacity:
                pending.popitem(last=False)
                self.get_logger().warning(
                    "Dropped an unmatched IMU clock record after the pairing queue filled"
                )

            other = self.pending_imu if is_time else self.pending_time
            if receipt_ns not in other:
                return
            imu = self.pending_imu.pop(receipt_ns)
            time_reference = self.pending_time.pop(receipt_ns)
            sensor_ns = stamp_to_ns(time_reference.time_ref)
            mapped_ns, event = self.clock.map(receipt_ns, sensor_ns)
            if mapped_ns is None:
                self.get_logger().warning(
                    "Skipped IMU clock pair (%s)" % event
                )
                return
            if event == "sensor_clock_reset":
                self.get_logger().warning(
                    "Xsens sample clock reset; anchored a new receipt-time epoch"
                )

            output = deepcopy(imu)
            output.header.stamp = ns_to_stamp(mapped_ns)
            self.publisher.publish(output)
            self._publish_timing_record(
                output, time_reference, receipt_ns, sensor_ns, mapped_ns
            )

    def _reanchor_source_clock(self, _request, response):
        with self.lock:
            self.clock.reanchor()
        response.success = True
        response.message = "A new IMU source-clock mapping will start at the next paired sample"
        return response

    def _publish_timing_record(self, output, reference, receipt_ns, sensor_ns, mapped_ns):
        record = ClockedImu()
        record.header = deepcopy(output.header)
        record.measurement = deepcopy(output)
        record.source_id = self.source_id
        record.source_boot_id = self.source_boot_id
        record.device_boot_id_valid = False
        record.device_boot_id = ""
        record.source_instance_id = self.source_instance_id
        record.device_host_boot_id = self.host_boot_id
        record.source_instance_generation = 1
        record.device_clock_epoch = self.clock.epoch
        record.clock_domain = self.clock_domain
        record.source_epoch = self.clock.epoch
        # TimeReference has no hardware sequence counter; do not mislabel the
        # ROS header stamp or a locally generated counter as a source sequence.
        record.source_sequence_valid = False
        record.source_sequence = 0
        record.source_sequence_bits = 0
        record.source_time = deepcopy(reference.time_ref)
        record.receipt_time = deepcopy(reference.header.stamp)
        record.receipt_host_boot_id = self.host_boot_id
        record.receipt_monotonic_ns = monotonic_ns()
        record.ros_time_epoch = self.ros_time_epoch
        record.timing_uncertainty_sec = float("nan")
        record.clock_mapping_revision = self.clock.mapping_revision
        record.clock_mapping_calibrated = False
        self.timed_publisher.publish(record)


def main(args=None):
    rclpy.init(args=args)
    node = SampleClockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
