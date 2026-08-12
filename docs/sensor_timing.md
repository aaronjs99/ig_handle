# Sensor Timing Firmware

The Teensy 4.1 firmware separates physical data acquisition from timing I/O.
The Linux host continues acquiring all four cameras, both VLP-16s, and the
Xsens MTi-30 continuously. The firmware can qualify a common reference edge,
schedule future trigger pulses, observe per-camera exposure and IMU sync edges,
and publish timing diagnostics. It neither transports payload data nor proves
that a host driver accepted a hardware trigger.

All new timing outputs are disabled and unassigned in
[`firmware_config.h`](../config/teensy/firmware_config.h). A build becomes
actuating only after the corresponding pins, polarities, line drivers,
configuration, and wiring-verification gates are supplied. The scheduler then
requires stable reference periods before running, stops on a missing or
out-of-range reference, rejects late trigger deadlines, requires camera
ExposureActive feedback, and latches faults rather than continuing open-loop.
Relative timing remains unqualified until the assembled board is measured with
an oscilloscope or logic analyzer. The DS3231 `INT/SQW` output is open-drain;
the assembled board must pull it up to 3.3 V or use a verified 3.3 V level
shifter. A module's 5 V pull-up must never reach the non-5-V-tolerant Teensy.
The currently available square wave is not a verified UTC/GNSS phase source.
Published firmware time references use
a monotonic epoch beginning at the first qualified edge and explicitly do not
claim UTC. On the existing `sensor_msgs/TimeReference` diagnostics,
`header.stamp` is only the ROS publication-receipt time and `time_ref` is the
unmapped relative MCU epoch. These topics are non-authoritative and are not a
ROS-time calibration contract.

## Electrical boundary

The installed-camera inventory must be checked against the physical labels.
The current Teledyne reference names the official color model
`FG-PGE-50S5C-IP`; the repository's `FG-PGE-50S5C-C-IP` spelling is not assumed
to be a second electrical variant.

For the current Forge IP67 family, the M8 eight-pin GPIO interface assigns pin
1 to 8-24 V camera input power, pin 2/Line0 to the opto-isolated input, pin 3 to
opto ground, pin 6/Line1 to the opto-isolated output, and pin 7 to camera power
ground. Published typical thresholds are 1.55 V low and 2.6 V high for Line0;
Line1 is specified as approximately 0.56 V low and 5 V high under the vendor's
measurement conditions. The Teensy accepts only 0-3.3 V inputs and its normal
output recommendation is 4 mA. Therefore neither Forge line connects directly
to the MCU. The board must provide characterized, 3.3 V-safe buffered or
isolated inputs and outputs, defined returns, and protected/fused 8-24 V camera
branches when GPIO power is used. Use IP-rated M8 cables or sealing plugs to
retain the camera's ingress rating.

Firmware provides both four independently configurable logical outputs and one
common hardware-fanout output, but sequential GPIO writes are not claimed to be
simultaneous. Select and characterize the common fanout or an output latch when
all four Line0 edges must be simultaneous. Configure each
camera separately for `FrameStart`, the selected Line0 rising edge, and Line1
`ExposureActive`; both device-side functions have separate false-by-default
firmware gates. Verify every accepted trigger and exposure edge on the
assembled harness before enabling the firmware gate. The current host profile
remains free-running and unchanged.

The VLP-16s acquire continuously. Their input is a clock/phase reference, not a
scan trigger. The official manual requires a PPS high above 3.0 V and below 5.0
V, low below 1.2 V, and at least 2 mA source current. NMEA and PPS must alternate;
PPS-to-NMEA separation is at least 50 ms and the NMEA sentence must finish at
least 300 ms before the next PPS. The current firmware has no verified UTC/GNSS
source, so it does not emit the old placeholder GPRMC sentence. Dual-VLP phase
locking additionally requires a locked PPS, an RPM multiple of 60 between 300
and 1200, and distinct measured phase offsets configured and saved on the
sensors. The MCU only distributes the shared edge. Its two GPIO writes are
sequential; use and characterize a common electrical fanout when the two LiDAR
edges must share one physical transition.

The encased MTi-30 synchronization contract accepts 0-0.8 V low and 2.5-20 V
high on SyncIn/ClockSync; SyncOut is 0-0.4 V low and above 2.9 V high. Firmware
cannot determine the selected MTi `SyncSettings`. The optional MCU output is
only a time-event marker for the continuously sampled IMU; it must not select
`StartSampling` or reduce navigation data to the camera's 10 Hz cadence. It
therefore remains disabled until polarity, voltage, cable pin, the SyncIn event,
the SyncOut feedback event, and a corresponding packet marker are independently
verified in MT Manager.

## Firmware structure and validation

`sensor_sync.h` owns the side-effect-free state machine, bounded edge
mailboxes, and continuous rollover-safe relative-epoch mapper. GPIO ISRs only
deposit timestamped edges; the timer ISR is the sole owner of scheduler,
feedback, fault, and relative-time state. Mailbox overflow latches
synchronization off. `sensor_sync_runtime.h` owns Teensy GPIO, camera
exposure midpoint capture, IMU events, feedback deadlines, and safe inactive
outputs. `firmware_pin_contract.h` rejects cross-owner aliases, RTC-I2C pin
collisions, and unsupported telescope PWM/ADC assignments before either
runtime initializes. `main.ino` owns ROS publication, RTC edge labelling, the
timer ISR, and the existing disabled telescope runtime.

The pure scheduler is suitable for non-actuating host compilation and state
transition checks. These checks are performed from an ephemeral harness rather
than a repository validator. They check logic only. A valid Teensy build
requires a Teensyduino core, rosserial headers, and RTClib. Acceptance
additionally requires a
current-limited bench supply, no motor or sensor payloads connected for initial
bring-up, safe inactive outputs during boot/reset/disconnect, instrumented
edge-to-edge skew and jitter, deliberate missing-reference and feedback-fault
injection, MTi packet-marker verification, VLP PPS-lock/phase-lock status, and
camera exposure/frame-counter correlation. Passing simulation or host tests is
not physical synchronization evidence.

The telescope motor driver must also have external hard pulldowns and a
hardware driver-disable path so reset, boot, disconnected MCU, or high-impedance
GPIO cannot energize either half-bridge. Firmware fail-closed states supplement
that electrical interlock; they do not replace it.

## Primary references

- Teledyne FLIR, [Forge 5 MP GigE camera family](https://www.flir.com/products/forge-5mp-gige/), model and acquisition-feature reference.
- Teledyne FLIR, [Forge IP67 GPIO electrical characteristics](https://softwareservices.flir.com/FG-PGE-50S5-IP/latest/Model/spec.html), M8 pinout, thresholds, delays, and line ratings.
- Teledyne FLIR, [Forge IP67 acquisition control](https://softwareservices.flir.com/FG-PGE-50S5-IP/latest/Model/public/AcquisitionControl.html), FrameStart trigger-source/mode/overlap contract.
- Ouster (Velodyne), [VLP-16 User Manual, 63-9243 Rev. F](https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf), PPS/NMEA electrical, timing, and phase-lock requirements.
- Analog Devices, [DS3231 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/DS3231.pdf), open-drain `INT/SQW` electrical contract.
- Xsens, [MTi 10-series and MTi 100-series User Manual, MT0605P](https://www.xsens.com/hubfs/Downloads/usermanual/MTi_usermanual.pdf), encased MTi connector, SyncIn/SyncOut, and ClockSync electrical characteristics.
- PJRC, [Teensy 4.1 technical specifications](https://www.pjrc.com/store/teensy41.html), MCU GPIO voltage and current limits.
