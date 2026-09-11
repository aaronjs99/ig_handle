#!/usr/bin/env python3
"""Send Xsens bootloader wake-up bytes for supervised hardware recovery only.

This standalone utility writes directly to the attached USB device. It is not a
normal bringup path and must not be run against an unidentified USB peripheral.
"""

import time

import usb.core
import usb.util

# Xsens vendor and MTi-30 bootloader product identifiers.
VID = 0x2639
PID = 0x0003

# Xbus WakeUp message: [Preamble, BID, MID, LEN, Checksum].
# 0xFA 0xFF 0x3E 0x00 0x03
WAKEUP = b"\xFA\xFF\x3E\x00\x03"


def main() -> int:
    """Send the recovery sequence only after explicit command invocation."""

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("Device not found! Check connection.")
        return 1

    print("Found IMU in Bootloader mode (bcdDevice 0.00)")
    if dev.is_kernel_driver_active(0):
        dev.detach_kernel_driver(0)
    dev.set_configuration()

    try:
        # Xsens bootloader devices normally expose outbound endpoint 0x02.
        dev.write(0x02, WAKEUP, 100)
        print("Sent WakeUp pulse to Endpoint 0x02...")
        time.sleep(0.5)

        # Request configuration mode after the wake-up pulse.
        dev.write(0x02, b"\xFA\xFF\x30\x00\xD1", 100)
        print("Sent GoToConfig pulse. Re-plug the device now.")
    except Exception as exc:
        print(f"Error: {exc}")
        print("If 'Resource Busy', try running with sudo.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
