"""Send Xsens bootloader wake-up bytes for supervised hardware recovery only.

This standalone utility writes directly to the attached USB device. It is not a
normal bringup path and must not be run against an unidentified USB peripheral.
"""

import usb.core
import usb.util
import time

# Xsens vendor and MTi-30 bootloader product identifiers.
VID = 0x2639
PID = 0x0003

# Locate the configured device before writing any recovery bytes.
dev = usb.core.find(idVendor=VID, idProduct=PID)

if dev is None:
    print("Device not found! Check connection.")
    exit()

print(f"Found IMU in Bootloader mode (bcdDevice 0.00)")

# Detach a bound kernel driver so the recovery interface can be opened.
if dev.is_kernel_driver_active(0):
    dev.detach_kernel_driver(0)

# Select the device configuration required by the bootloader endpoint.
dev.set_configuration()

# Xbus WakeUp message: [Preamble, BID, MID, LEN, Checksum].
# 0xFA 0xFF 0x3E 0x00 0x03
WAKEUP = b"\xFA\xFF\x3E\x00\x03"

try:
    # Xsens bootloader devices normally expose outbound endpoint 0x02.
    dev.write(0x02, WAKEUP, 100)
    print("Sent WakeUp pulse to Endpoint 0x02...")
    time.sleep(0.5)

    # Request configuration mode after the wake-up pulse.
    dev.write(0x02, b"\xFA\xFF\x30\x00\xD1", 100)
    print("Sent GoToConfig pulse. Re-plug the device now.")
except Exception as e:
    print(f"Error: {e}")
    print("If 'Resource Busy', try running with sudo.")
