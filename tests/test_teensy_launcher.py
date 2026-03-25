#!/usr/bin/env python3
import importlib
import os
import sys
import unittest
from unittest.mock import MagicMock, patch


_ORIG_MODULES = {"rospy": sys.modules.get("rospy")}

mock_rospy = MagicMock()
sys.modules["rospy"] = mock_rospy

TEST_DIR = os.path.dirname(__file__)
SCRIPT_DIR = os.path.abspath(os.path.join(TEST_DIR, "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import teensy_launcher  # noqa: E402

if _ORIG_MODULES["rospy"] is None:
    sys.modules.pop("rospy", None)
else:
    sys.modules["rospy"] = _ORIG_MODULES["rospy"]


class TeensyLauncherTests(unittest.TestCase):
    def setUp(self):
        sys.modules["rospy"] = mock_rospy
        importlib.reload(teensy_launcher)
        mock_rospy.reset_mock()

    def tearDown(self):
        if _ORIG_MODULES["rospy"] is None:
            sys.modules.pop("rospy", None)
        else:
            sys.modules["rospy"] = _ORIG_MODULES["rospy"]

    def test_main_launches_rosserial_when_device_exists(self):
        params = {
            "~port": "/dev/ttyUSB-test",
            "~baud": 57600,
        }
        mock_rospy.get_param.side_effect = lambda name, default=None: params.get(name, default)

        with patch.object(teensy_launcher.os.path, "exists", return_value=True):
            with patch.object(teensy_launcher.subprocess, "call") as subprocess_call:
                teensy_launcher.main()

        subprocess_call.assert_called_once_with(
            [
                "rosrun",
                "rosserial_python",
                "serial_node.py",
                "_port:=/dev/ttyUSB-test",
                "_baud:=57600",
            ]
        )

    def test_main_warns_and_skips_when_device_is_missing(self):
        mock_rospy.get_param.side_effect = lambda name, default=None: default

        with patch.object(teensy_launcher.os.path, "exists", return_value=False):
            with patch.object(teensy_launcher.subprocess, "call") as subprocess_call:
                teensy_launcher.main()

        subprocess_call.assert_not_called()
        self.assertTrue(mock_rospy.logwarn.called)


if __name__ == "__main__":
    unittest.main()
