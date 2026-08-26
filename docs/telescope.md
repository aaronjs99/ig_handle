# Telescoping Sonar Arm

The physical design reported for the arm uses a Teensy 4.1, 12 V BRINGSMART
JGY-370 worm gearmotor, HiLetgo BTS7960 H-bridge, E38S6-600-24G incremental
encoder, and one full-retraction mechanical limit switch. Measured length is
about 0.846 m retracted and 1.380 m extended.

This is not yet a commissioned runtime contract. The checked-in
[`hardware.yaml`](../config/telescope/hardware.yaml) configuration is disabled,
contains zero-valued measurement-required geometry, and represents the V6
one-switch design:
the normally-closed minimum loop is monitored on D34, and no maximum switch is
invented. ORACLE correspondingly disables automatic telescope-length commands.
Motor drive remains disabled until the switch, geometry, encoder, active-high
external-field supervisor, motor polarity, and wiring are commissioned together. Once enabled in
a future measured configuration, an open minimum loop blocks retraction on the
next 10 ms control pass; invalid field power asynchronously removes bridge enable with 10 ms
polling as a backup; and invalid calibration, homing
timeout, or encoder stall latches motor shutdown until reboot or a future
explicitly reviewed reset path. Ordinary
command timeout and reaching an endpoint remain nonfatal stops.

## Homing and position

In the V6 one-switch design, the retraction switch is the physical
reference. A completed homing cycle retracts slowly until that switch activates,
removes motor drive, and sets the encoder position to zero. The present firmware
does not unconditionally home at startup: it homes only after a minimum-length
request unless it already observes the minimum switch. Automatic positioning
remains unavailable until the hardware contract, switch logic, geometry, and
homing behavior are commissioned together.

A one-switch implementation can omit a full-extension switch when packaging
makes it physically impractical. Its extended end relies on a calibrated maximum
encoder count, slow-down region, motor timeout, lack-of-motion detection, and a
mechanical hard stop as the final passive boundary. Repeated loaded contact with
the stop is not normal control behavior. An optional maximum switch could be
added only through a separately reviewed contract; the checked-in
V6 interface does not reserve or invent one.

The encoder measures a constant-radius capstan rather than the changing-radius
storage spool. Runtime position uses a measured full-travel encoder calibration,
not an unused motor-revolution proxy.

## Electrical safety and packaging

V6 does not connect the unverified BTS7960 `R_IS`/`L_IS` outputs. The retained
motor-current topic publishes `NaN`, not an invented value. If current feedback
is later required, first characterize the exact IBT-2 clone or add an external
Hall-effect sensor after the 12 V branch fuse and before the H-bridge. That is a
separate schematic, calibration, firmware, and documentation change.

High-current motor wiring stays short and separated from encoder and limit
signals. The home switch uses normally-closed logic so a pressed switch,
unplugged connector, or broken wire removes retraction permission on the next
10 ms control pass. Only the return to a healthy closed loop is debounced for 25 ms. With one NC
loop the firmware cannot distinguish a true minimum from a broken cable, so
extension away from the apparent minimum remains a separately commissioned
behavior. The encoder zero is therefore rebased only during an explicit,
controlled homing request; merely observing an open loop during normal motion
does not rewrite position. A broken loop during that homing request remains an
accepted one-contact ambiguity and must be covered by commissioning and the
mechanical hard stop. Connectors must preserve current rating, locking, serviceability,
contact protection, and strain relief.

The BTS7960 receives RPWM on D22, LPWM on D23, and one buffered D38 enable that
fans to `R_EN` and `L_EN`. Connector-side pulldowns keep all three commands low.
U12 TPS3897ADRYT supervises fused `EXT5_FIELD`, downstream of the
TPS259470-protected J1 input. J1 must remain inside a 4.95-5.05 V instantaneous
loaded envelope, including DC set-point error, ripple, overshoot, undershoot,
and operating transients. Total `EXT5_FIELD` current across the motor-driver
logic, encoder, and other field loads must remain at or below 250 mA, and TP4
must remain at or above 4.80 V under the worst simultaneous load and worst
qualified temperature. Qualification records the voltage drop across and
temperature of both U11 and F2. If measured or credible worst-case field load
exceeds 250 mA, stop and reassess the load budget, F2, U11, rail-drop, and
thermal design before enabling motion. R48 84.5 kOhm and R49
10.0 kOhm, both 0.1%, set a nominal 4.725 V rising and 4.678 V falling threshold;
the tolerance-plus-`I_SENSE` rising range is approximately 4.668-4.782 V. C17
100 nF is the supervisor's local VCC bypass; the supervisor provides approximately
40 us nominal valid qualification; this is a formula/nominal value, not a guaranteed
maximum. Its
open-drain output, R62 10 kOhm core-3.3-V pull-up, and R64 100 kOhm pulldown create
active-high `FIELD_VALID` on D27. LOW means invalid/off and remains fail-low with
the core off. Invalid propagation is about 16 us typical, but the datasheet gives
no maximum; characterize the assembled worst case and retain the independent
hard E-stop. All four U6
active-high OE pins use `FIELD_VALID` directly, so the motor/MTi driver is
disabled in hardware without adding a gate to its data paths. As a secondary
defense, the MCU falling-edge interrupt pulls the shared BTS enable low. The
ordinary 10 ms control pass polls the same input as a backup, clears motion
and homing requests, and zeroes both PWM channels. Restoring the field rail does
not resume the old request; a fresh valid command is required. USB backup is not
permitted to power BTS logic or encoder.

If the timing core or MCU is off, firmware cannot inspect D27; R64, the U6 OE
connections, and connector-side pulldowns independently hold the driver and
RPWM, LPWM, and enable paths inactive. R62 10 kOhm and R64 100 kOhm define the
`FIELD_VALID` board node; D27 is a plain MCU input and does not add another bias.
An open branch between that node and D27 is therefore indeterminate/lost firmware
evidence, not a guaranteed invalid reading. The U3/U6 hardware OE gates remain
independently connected to the board node. A D27 short to 3.3 V or a failed-open
supervisor output with R62 intact can still masquerade as valid, so route
continuity, open-branch, short-high, and stuck-output fault tests are commissioning
gates, and the hard E-stop remains independent. If the core is off while
the nominal 5 V field rail remains present, R48 bounds a hypothetical sense-pin
clamp feed to at most 59 uA; this is bounded leakage, not a literal zero-Ioff
claim. The independent hard E-stop remains in the 12 V motor-power path.
`hardware.yaml::hard_estop_installed_and_tested` and firmware
`kHardEstopVerified` are both false by default, and `canOperate()` requires the
firmware gate. Set them true only after a normally-closed hard E-stop has been
installed in the 12 V motor-power path and a physical test proves that pressing
it removes motor power independently of the Teensy, USB link, BTS7960 logic,
and software state.

If the main enclosure cannot provide volume for MCU, H-bridge, fusing,
regulators, LiDAR circuits, bend radii, and service loops, a small sealed
arm-controller enclosure is safer and more serviceable than inaccessible
stacking.

The current ROS surface in
[`runtime_surface.yaml`](../config/runtime_surface.yaml) publishes length, motor
current, and a status string. Motor current is explicitly unavailable (`NaN`)
until a real sensing contract is added.
Homed state, extension percentage, direction, PWM, individual switch states,
faults, and operating mode remain desirable structured fields, not current
published contract fields.
