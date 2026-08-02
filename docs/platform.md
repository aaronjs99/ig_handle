# Sensor Platform

IG Handle is the physical acquisition boundary for GRANDE. It records what is
installed, how each device is reached, which frame and topic it owns, how time
is stamped, and whether the device is currently available.

The deployed inventory is defined by `config/sensors/sensor_contract.yaml`, not
by launch-file names. `sensor_network.yaml` records endpoints and interface
roles. `sensor_frames.yaml` records reviewed static transforms, while
`sensor_models.yaml` records capabilities independent of one deployed serial.

Changing a physical transform requires measurement evidence. Runtime code may
select a configured device but cannot patch its geometry or identity.

The host uses separate local and boat-facing network roles. Stable device
identity uses serials, USB attributes, udev aliases, and explicit network
endpoints. A changing `/dev/ttyACM*` number or switch port is not itself a new
identity.

Camera serial files preserve calibration for each physical camera. The sensor
contract assigns ordered inspection roles so DEFECTOR and ORACLE consume role,
topic, frame, and calibration without product-name coupling.

Device stamps are preserved when available. IG Handle publishes timing inputs
for downstream estimation and records raw sensor evidence without turning
acquisition health into mission or actuator authority. GRANDE normally owns
integrated recording; IG Handle can collect isolated raw evidence for hardware
investigation.

## Imaging Sonar

IG Handle separates physical sonar identity and acquisition from downstream
mapping and mission use. The installed Imagenex/DeltaT path and optional
Ping360 path are different providers; their protocols and geometry are not
interchangeable.

The DeltaT wrapper launches the vendor executable and forwards its raw UDP
transport; it does not currently publish the provider-neutral profile. The
Ping360 provider uses the Blue Robotics Ping protocol and publishes both raw
messages and a provider-neutral profile carrying angle, range resolution, gain,
frequency, source identity, and acquisition provenance.

Physical endpoint, identity, frame, and profile live in IG Handle. MARINER may
consume an accepted profile for mapping; ORACLE may request a sonar-relevant
mission; neither owns device commands.

Read-only identity is the lowest-risk first connection. Active transmission
requires the configured allow flag and operator authorization because a scan
changes physical device state. The provider rejects invalid message lengths,
checksums, ranges, profiles, and identity. These runtime guards remain even
without standalone validation utilities.

The simulator can publish the same canonical profile with synthetic provenance.
Replay can reproduce transport and mapping behavior. Neither establishes real
acoustic propagation, multipath, target reflectivity, beam geometry, or latency.

Field evidence retains device identity, profile, sound-speed assumption, pose
and frame relationship, environment, raw packets or profiles, and synchronized
navigation state. A sonar image is observation evidence, not a metric defect
measurement without reviewed geometry and calibration.

## Heron Propulsion

IG Handle owns the physical facts needed to inspect and commission the Heron's
propulsion hardware. MARINER owns controller mathematics and the final guarded
command path; GRANDE owns evidence lineage and run-level evaluation.

The hardware chain includes battery protection, power distribution, vehicle
controller, motor-enable/heartbeat path, Castle ESCs, motors, impellers, ducts,
wiring, and telemetry sensing. Reverse asymmetry may come from configuration,
electrical delivery, motor/ESC condition, mechanics, water loading, weight
distribution, or command processing; one observation does not identify cause.

Restrained-air current can isolate electrical or mechanical asymmetry from
water flow, but it does not identify afloat thrust. Current is not treated as
force without an independent force measurement.

Castle Link can read and compare ESC profiles, including direction, endpoints,
braking, timing, and current-related settings. The two sides should be compared
as exact exported profiles before changing parameters. Physical inspection
covers connectors, motor/shaft freedom, impeller and duct condition, water
ingress, and side-to-side wiring differences.

ROS controller gains affect software command response but do not correct an
ESC, motor, impeller, or hull-balance defect. Hardware and software changes are
isolated and recorded separately.

Useful evidence aligns requested command, accepted drive, motor-enable state,
voltage, side-specific current, RPM or PWM when available, and vehicle motion.
Forward/reverse, left/right, coast, dry, restrained, and afloat regimes remain
labelled. Replacement screening is a candidate record, not compatibility
approval.
