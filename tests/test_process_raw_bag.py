#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock, patch
import sys
import os
import importlib

# Add scripts to path
SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), "../scripts/pipeline")
if SCRIPTS_DIR not in sys.path:
    sys.path.append(SCRIPTS_DIR)


# Fake Time/Duration for comparison
class FakeTime:
    def __init__(self, t=0.0):
        self.t = float(t)

    def to_sec(self):
        return self.t

    def __lt__(self, other):
        return self.t < (other.t if hasattr(other, "t") else other)

    def __le__(self, other):
        return self.t <= (other.t if hasattr(other, "t") else other)

    def __gt__(self, other):
        return self.t > (other.t if hasattr(other, "t") else other)

    def __ge__(self, other):
        return self.t >= (other.t if hasattr(other, "t") else other)

    def __eq__(self, other):
        return self.t == (other.t if hasattr(other, "t") else other)

    def __add__(self, other):
        return FakeTime(self.t + other.to_sec())

    def __sub__(self, other):
        start = other.t if hasattr(other, "t") else other
        return FakeDuration(self.t - start)

    def __repr__(self):
        return f"FakeTime({self.t})"


class FakeDuration:
    def __init__(self, t=0.0):
        self.t = float(t)

    @classmethod
    def from_sec(cls, t):
        return cls(t)

    def to_sec(self):
        return self.t

    def __repr__(self):
        return f"FakeDuration({self.t})"


class TestIgPostProcess(unittest.TestCase):
    def setUp(self):
        # Mocks
        self.mock_rospy = MagicMock()
        self.mock_rospy.Time = FakeTime
        self.mock_rospy.Duration = FakeDuration

        self.mock_rosbag = MagicMock()
        self.mock_bag_instance = MagicMock()
        self.mock_rosbag.Bag.return_value = self.mock_bag_instance

        # We need real numpy or a robust mock. process_raw_bag uses np.interp.
        # Ideally we let it use real numpy. We won't patch numpy in sys.modules.
        # But we patch rospy, rosbag.

        self.modules_patcher = patch.dict(
            sys.modules, {"rospy": self.mock_rospy, "rosbag": self.mock_rosbag}
        )
        self.modules_patcher.start()

        # Import/Reload
        import process_raw_bag

        importlib.reload(process_raw_bag)
        self.mod = process_raw_bag

        # Args
        self.args = MagicMock()
        self.args.bag = "test.bag"
        self.args.bag_start = -1
        self.args.bag_end = -1
        self.args.data_restamp_topics = ["/data"]
        self.args.time_restamp_topics = ["/time"]
        self.args.clip_restamp_topics = [0.0]
        self.args.data_interp_topics = []
        self.args.time_interp_topics = []

        # Setup bag behaviors
        self.mock_bag_instance.get_start_time.return_value = 1000.0
        self.mock_bag_instance.get_end_time.return_value = 2000.0
        self.mock_bag_instance.filename = "test.bag"

    def tearDown(self):
        self.modules_patcher.stop()
        if "process_raw_bag" in sys.modules:
            del sys.modules["process_raw_bag"]

    def _make_msg(self, t, stamp=None):
        msg = MagicMock()
        if stamp:
            msg.header.stamp = stamp
        return msg

    def test_restamp_logic(self):
        # Define side effect to fake bag reading
        def read_side_effect(topics=None):
            # Ensure data_t >= stamp_t to avoid clipping
            data_msgs = [
                ("/data", self._make_msg(1012, MagicMock()), 1012),
                ("/data", self._make_msg(1022, MagicMock()), 1022),
            ]
            # Use FakeTime for timestamps
            t1 = MagicMock()
            t1.time_ref = FakeTime(1011)
            t2 = MagicMock()
            t2.time_ref = FakeTime(1021)
            time_msgs = [("/time", t1, 1011), ("/time", t2, 1021)]

            all_msgs = sorted(data_msgs + time_msgs, key=lambda x: x[2])

            if topics is None:
                for m in all_msgs:
                    yield m
            else:
                req_topics = set(topics)
                for m in all_msgs:
                    if m[0] in req_topics:
                        yield m

        self.mock_bag_instance.read_messages.side_effect = read_side_effect

        with patch("os.system"), patch("os.path.exists"):
            proc = self.mod.IgPostProcess(self.args)

        out_bag = proc.out_bag
        data_writes = [c for c in out_bag.write.call_args_list if c[0][0] == "/data"]

        self.assertEqual(len(data_writes), 2)
        # Check timestamp was updated to 1011
        self.assertEqual(data_writes[0][0][2], FakeTime(1011))

    def test_signal_dropout(self):
        def read_side_effect(topics=None):
            if topics is None:
                return []

            topic = topics[0]
            if topic == "/data":
                # 3 data messages vs 1 time message => diff 2 (>1)
                yield ("/data", MagicMock(), 1015)
                yield ("/data", MagicMock(), 1025)
                yield ("/data", MagicMock(), 1035)
            elif topic == "/time":
                # Only 1 time msg (Dropout)
                t1 = MagicMock()
                t1.time_ref = FakeTime(1011)
                yield ("/time", t1, 1010)

        self.mock_bag_instance.read_messages.side_effect = read_side_effect

        with patch("os.system"):
            with self.assertLogs(level="WARNING"):
                proc = self.mod.IgPostProcess(self.args)

        # Expect no writes because validation failed
        proc.out_bag.write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
