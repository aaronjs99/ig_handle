# Teensy Configuration

`firmware_config.h` is the compile-time configuration consumed by
`main/main.ino`. Change pin assignments, timing rates, ROS topic names, and
the legacy NMEA payload there when the firmware is rebuilt. Measured telescope
calibration belongs there as the firmware mirror of the YAML contract.

Keep the telescope values synchronized with
`../telescope/hardware.yaml`. The telescope remains unavailable unless all
three gates are true:

- `kEnabled`
- `kConfigured`
- `kWiringVerified`

It also refuses to operate until the mirrored current-sense calibration and
motor-current trip are both nonzero.

The shipped values are dummy-safe: geometry is zero, motion is disabled, and
the provisional pins are never configured by the current sketch. The runtime
also requires calibrated current sensing before it can energize the motor.
`main/telescope_control.h` is the side-effect-free geometry layer and
`main/telescope_runtime.h` is the motor/encoder runtime. The existing timing
pins and topics are independent of telescope motion.
