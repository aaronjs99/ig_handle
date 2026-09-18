#!/usr/bin/env python3
"""Publish IMU acquisition times using a receipt-paired sensor clock.

The first matched receipt anchors each sensor-clock epoch in ROS time. The
constant offset preserves measured sample intervals; it is not a calibrated
estimate of the sensor's transport delay. Raw IMU and TimeReference inputs
remain available for interpreting that uncertainty.
"""

from collections import OrderedDict
from copy import deepcopy
from threading import Lock


class SampleClock:
    """Map sensor-clock nanoseconds without resampling or replacing intervals."""

    def __init__(self):
        self.offset_ns = None
        self.receipt_ns = None
        self.sensor_ns = None

    def map(self, receipt_ns, sensor_ns):
        if receipt_ns <= 0 or sensor_ns < 0:
            return None, "invalid_timestamp"
        if self.receipt_ns is not None and receipt_ns <= self.receipt_ns:
            return None, "out_of_order_receipt"
        if self.sensor_ns is not None and sensor_ns == self.sensor_ns:
            return None, "duplicate_sensor_sample"
        if (
            self.sensor_ns is not None
            and sensor_ns < self.sensor_ns
            and self.sensor_ns - sensor_ns < 1000000000
        ):
            return None, "out_of_order_sensor_sample"

        event = "mapped"
        if self.offset_ns is None or sensor_ns < self.sensor_ns:
            event = "anchored" if self.offset_ns is None else "sensor_clock_reset"
            self.offset_ns = receipt_ns - sensor_ns
        self.receipt_ns = receipt_ns
        self.sensor_ns = sensor_ns
        return self.offset_ns + sensor_ns, event


class SampleClockNode:
    """Join existing IMU and TimeReference streams at their receipt stamp."""

    def __init__(self):
        import rospy
        from sensor_msgs.msg import Imu, TimeReference

        self.ros = rospy
        self.capacity = int(rospy.get_param("~queue_size", 256))
        if self.capacity < 1:
            raise ValueError("IMU sample-clock queue_size must be positive")
        output_topic = rospy.get_param("~output_topic")
        input_topic = rospy.get_param("~input_topic")
        time_topic = rospy.get_param("~time_topic")
        if rospy.resolve_name(input_topic) == rospy.resolve_name(output_topic):
            raise ValueError("IMU receipt input and canonical output must differ")

        self.clock = SampleClock()
        self.last_ros_ns = None
        self.pending_imu = OrderedDict()
        self.pending_time = OrderedDict()
        self.lock = Lock()
        self.publisher = rospy.Publisher(output_topic, Imu, queue_size=self.capacity)
        self.imu_subscriber = rospy.Subscriber(
            input_topic, Imu, self._imu, queue_size=self.capacity
        )
        self.time_subscriber = rospy.Subscriber(
            time_topic, TimeReference, self._time, queue_size=self.capacity
        )
        rospy.loginfo(
            "IMU sample clock pairs %s and %s into %s; "
            "the epoch offset includes uncalibrated receipt delay",
            input_topic,
            time_topic,
            output_topic,
        )

    def _imu(self, message):
        self._receive(message, False)

    def _time(self, message):
        self._receive(message, True)

    def _receive(self, message, is_time):
        with self.lock:
            now_ns = self.ros.Time.now().to_nsec()
            if self.last_ros_ns is not None and now_ns < self.last_ros_ns:
                self.clock = SampleClock()
                self.pending_imu.clear()
                self.pending_time.clear()
                self.ros.logwarn("IMU sample clock reanchored after ROS clock reset")
            self.last_ros_ns = now_ns

            receipt_ns = message.header.stamp.to_nsec()
            pending = self.pending_time if is_time else self.pending_imu
            pending[receipt_ns] = message
            if len(pending) > self.capacity:
                pending.popitem(last=False)
                self.ros.logwarn_throttle(
                    5.0,
                    "IMU sample clock dropped an unmatched receipt stamp: "
                    "no matching %s within the %d-message buffer",
                    "IMU" if is_time else "TimeReference",
                    self.capacity,
                )
            if (
                receipt_ns not in self.pending_imu
                or receipt_ns not in self.pending_time
            ):
                return

            imu = self.pending_imu.pop(receipt_ns)
            reference = self.pending_time.pop(receipt_ns)
            mapped_ns, event = self.clock.map(receipt_ns, reference.time_ref.to_nsec())
            if mapped_ns is None:
                self.ros.logwarn_throttle(
                    5.0, "IMU sample clock skipped measurement: %s", event
                )
                return
            if event == "sensor_clock_reset":
                self.ros.logwarn(
                    "IMU sensor clock rolled back at receipt %.9f; "
                    "the new epoch offset uses its receipt time",
                    receipt_ns * 1e-9,
                )
            output = deepcopy(imu)
            seconds, nanoseconds = divmod(mapped_ns, 1000000000)
            output.header.stamp = self.ros.Time(seconds, nanoseconds)
            self.publisher.publish(output)


def main():
    import rospy

    rospy.init_node("imu_sample_clock")
    SampleClockNode()
    rospy.spin()


if __name__ == "__main__":
    main()
