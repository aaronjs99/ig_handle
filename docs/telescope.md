# Telescoping Sonar Arm

The physical design reported for the arm uses a Teensy 4.1, 12 V BRINGSMART
JGY-370 worm gearmotor, HiLetgo BTS7960 H-bridge, E38S6-600-24G incremental
encoder, and one full-retraction mechanical limit switch. Measured length is
about 0.846 m retracted and 1.380 m extended.

This is not yet a commissioned runtime contract. The checked-in
[`hardware.yaml`](../config/telescope/hardware.yaml) configuration is disabled,
contains placeholder geometry, and still
expects redundant minimum and maximum limit contacts. ORACLE correspondingly
disables automatic telescope-length commands. The one-switch physical design
and the current firmware/configuration must be reconciled before motor drive is
enabled.

## Homing and position

In the proposed one-switch design, the retraction switch is the physical
reference. A completed homing cycle retracts slowly until that switch activates,
removes motor drive, and sets the encoder position to zero. The present firmware
does not unconditionally home at startup: it homes only after a minimum-length
request unless it already observes the minimum switch. Automatic positioning
remains unavailable until the hardware contract, switch logic, geometry, and
homing behavior are commissioned together.

A one-switch implementation can omit a full-extension switch when packaging
makes it physically impractical, but that requires a deliberate firmware and
configuration change. Its extended end would rely on a calibrated maximum
encoder count, slow-down region, motor timeout, lack-of-motion detection, and a
mechanical hard stop as the final passive boundary. Repeated loaded contact with
the stop is not normal control behavior. The current checked-in contract instead
expects maximum-limit inputs and must not be represented as already compatible
with the one-switch design.

The encoder measures a constant-radius capstan rather than the changing-radius
storage spool. Nominal travel follows capstan circumference and quadrature
counts, then is replaced by measured full-travel calibration.

## Current and packaging

The present firmware, configured through
[`firmware_config.h`](../config/teensy/firmware_config.h), reads BTS7960
current-sense inputs, but the hardware configuration still marks current
feedback unavailable and no calibrated
motor-current contract exists. If those driver outputs cannot provide adequate
bidirectional accuracy and range, an external Hall-effect sensor belongs in the
motor supply path after the 12 V branch fuse and before the H-bridge. Either
method must cover startup and stall current. Current supports jam/overload
detection only after offset, polarity, normal-motion, startup, and stall-related
thresholds are measured.

High-current motor wiring stays short and separated from encoder and limit
signals. The home switch uses normally-closed logic where practical so a broken
wire fails as a fault. Connectors must preserve current rating, locking,
serviceability, contact protection, and strain relief.

If the main enclosure cannot provide volume for MCU, H-bridge, current sensor,
fusing, regulators, LiDAR circuits, bend radii, and service loops, a small sealed
arm-controller enclosure is safer and more serviceable than inaccessible
stacking.

The current ROS surface in
[`runtime_surface.yaml`](../config/runtime_surface.yaml) publishes length, motor
current, and a status string.
Homed state, extension percentage, direction, PWM, individual switch states,
faults, and operating mode remain desirable structured fields, not current
published contract fields.
