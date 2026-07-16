# Teensy Configuration

`firmware_config.h` is the compile-time configuration consumed by
`main/main.ino`. Change pin assignments, timing rates, ROS topic names, and
the legacy NMEA payload there when the firmware is rebuilt. Measured telescope
calibration belongs there as the firmware mirror of the YAML contract.

Keep the telescope values synchronized with
`../telescope/hardware.yaml`. The telescope remains unavailable unless all
three flags are true:

- `kEnabled`
- `kConfigured`
- `kWiringVerified`

The shipped values are dummy-safe: geometry is zero, motion is disabled, and
the provisional pins are never configured by the current sketch. A future
motor/encoder module must enforce these gates before configuring any actuator
GPIO. The existing timing pins and topics are independent of telescope motion.
