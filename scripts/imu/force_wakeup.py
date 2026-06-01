import usb.core
import usb.util
import time

# Xsens Vendor and MTi-30 Bootloader Product ID
VID = 0x2639
PID = 0x0003

# Find our device
dev = usb.core.find(idVendor=VID, idProduct=PID)

if dev is None:
    print("Device not found! Check connection.")
    exit()

print(f"Found IMU in Bootloader mode (bcdDevice 0.00)")

# Detach kernel drivers if any (unlikely in class 255 but safe)
if dev.is_kernel_driver_active(0):
    dev.detach_kernel_driver(0)

# Set configuration
dev.set_configuration()

# Xbus WakeUp message: [Preamble, BID, MID, LEN, Checksum]
# 0xFA 0xFF 0x3E 0x00 0x03
WAKEUP = b"\xFA\xFF\x3E\x00\x03"

try:
    # Most Xsens devices use Endpoint 0x02 for Outbound data
    # We send the wakeup pulse
    dev.write(0x02, WAKEUP, 100)
    print("Sent WakeUp pulse to Endpoint 0x02...")
    time.sleep(0.5)

    # Send GoToConfig to force exit maintenance
    dev.write(0x02, b"\xFA\xFF\x30\x00\xD1", 100)
    print("Sent GoToConfig pulse. Re-plug the device now.")
except Exception as e:
    print(f"Error: {e}")
    print("If 'Resource Busy', try running with sudo.")
