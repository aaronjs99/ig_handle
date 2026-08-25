# Sensor Platform

IG Handle is the physical acquisition boundary for GRANDE. It records what is
installed, how each device is reached, which frame and topic it owns, how time
is stamped, and whether the device is currently available.

The deployed inventory is defined by
[`sensor_contract.yaml`](../config/sensors/sensor_contract.yaml), not by
launch-file names. [`sensor_network.yaml`](../config/network/sensor_network.yaml)
records runtime endpoints and interface roles.
[`sensor_frames.yaml`](../config/sensors/sensor_frames.yaml) records configured
static transforms and their evidence state, while
[`sensor_models.yaml`](../config/sensors/sensor_models.yaml) records capabilities
independent of one deployed serial.

Changing a physical transform requires measurement evidence. Runtime code may
select a configured device but cannot patch its geometry or identity.

The host uses separate local and boat-facing network roles. Stable device
identity uses serials, USB attributes, udev aliases, and explicit network
endpoints. A changing `/dev/ttyACM*` number or switch port is not itself a new
identity. The runtime network contract places the sonar endpoint and host at
`192.168.2.4` and `192.168.2.10`; the checked-in `01-netplan.yaml` template now
uses that same `192.168.2.0/24` subnet. Applying netplan remains an explicit
operator action after verifying switch wiring, endpoint identity, and link state.
The Heron-facing host address is `192.168.131.10`, matching the deployed
networkd configuration; the base computer remains `192.168.131.1`.

Camera serial files reserve one calibration record per physical camera. The
current records are explicitly uncalibrated placeholders and must not be treated
as metric camera calibration. The sensor contract assigns ordered inspection
roles so DEFECTOR and ORACLE consume role, topic, frame, and calibration state
without product-name coupling.

Device stamps are preserved when available. IG Handle publishes timing inputs
for downstream estimation and records raw sensor evidence without turning
acquisition health into mission or actuator authority. GRANDE normally owns
integrated recording; IG Handle can collect isolated raw evidence for hardware
investigation.

## IMU Ownership and Recovery

GRANDE's canonical physical IMU is the serial-qualified Xsens MTi-30 on IG
Handle. Sensor contract ID 1 binds serial `0368319D` to `/dev/sensors/imu`,
publishes only below `/sensors/imu`, and is owned by the persistent
`ig-handle-xsens.service`. The service waits without opening another device
when the exact serial or physical Heron ROS master is unavailable. It restarts
the provider after USB loss, stale output, driver exit, or ROS-master
replacement and refuses duplicate publishers. `sensor_bringup` observes this
external owner for readiness but never opens or stops its serial port.

The stock Heron `/imu/*` and `/cv5/ros_mscl_node` surfaces describe an optional
onboard MicroStrain installation. They are not aliases for the IG Handle Xsens.
No Xsens topic is bridged into `/imu/data_raw`, and Xsens recovery grants no
navigation or actuation authority. A Heron without the optional onboard sensor
may retain inactive stock diagnostics, but GRANDE readiness uses the canonical
`/sensors/imu/data` contract.

The Teensy timing design and its disabled-by-default electrical gates are
specified in [`sensor_timing.md`](sensor_timing.md). Host camera, LiDAR, and IMU
acquisition remains continuous; the firmware's trigger scheduler is a future
commissioning surface rather than an enabled runtime claim.

## Battery Identity and Telemetry

The platform inventory distinguishes one fixed compute battery from two
interchangeable propulsion batteries. `IGHANDLE-01` is commissioned against the
complete live JK BMS identity: BLE address, device name, model, hardware and
software versions, serial number, and manufacturing date. Voltage, current, and
charge behavior are measurements and never serve as identity. A host-wide lock
prevents concurrent processes from owning the same BLE device, while periodic
device-information queries re-establish that identity during continued
telemetry acquisition. The `/ighandle_jk_bms` node publishes standard state on
`/sense_ighandle`; its paired `/sense_ighandle/details` record carries identity,
raw pack and cell fields, status, alarms, counters, and provenance with the same
timestamp. Fleet admission requires both synchronized messages to retain that
exact caller identity; another publisher cannot establish compute-pack identity.

`HERON-01` and `HERON-02` represent physical propulsion packs that the Heron MCU
cannot identify electrically. They remain uncommissioned until the matching
durable labels are physically present and inspected. After commissioning,
GRANDE binds a labelled installation to physical `/sense_heron` observations
through an append-only event ledger. A new logger session, publisher restart,
sequence or timestamp rollback, or source-progress gap prevents an older
selection from carrying forward without explicit reconfirmation. The resulting
effective assignment is published on `/battery/heron_identity` and recorded
beside the measurement streams. This separation allows both removable packs to
build independent histories while keeping unassigned observations visible as
`UNKNOWN`.

The installation ledger attributes selection and removal to the local
`username@hostname` caller by default. A supplied `selected_by` value is retained
as operator-entered provenance, not as authenticated user identity.

Persistent selection state, installation history, normalized samples, and
figures default to `~/.local/share/grande/battery`. The standalone IG Handle
service uses a local ROS master when the Heron is off; the combined service
uses the physical Heron master and validated `/sense_heron` ingress. Both use
the same data root and are mutually exclusive. Repository data remains retained
historical evidence rather than live mutable state.
Energy integration is available only from contiguous, identity-qualified compute
pack power. Propulsion motor-controller currents are preserved for diagnosis but
do not establish total pack current; propulsion energy therefore remains null
with an explicit unavailability reason.

## Imaging Sonar

IG Handle separates physical sonar identity and acquisition from downstream
mapping and mission use. The configured legacy Imagenex/DeltaT path and the
candidate Ping360 path are different providers; their protocols and geometry
are not interchangeable. The current sensor contract deliberately leaves
sensor 8 provider-unverified and selects passive `udp_raw`; it does not claim
that the attached head is a commissioned DeltaT or Ping360.

The DeltaT wrapper launches the vendor executable and forwards every UDP
datagram in `SonarRawPacket`, preserving receipt time, source endpoint, packet
kind, provider, model, extrinsic revision, and sequence; the low 32 sequence
bits are also placed in the ROS header for downstream trace correlation.
The physical DT100-labelled path is fail-closed: its contract keeps
`hardware_commissioned: false`, so neither the vendor process nor its raw
receiver is started. Promotion requires a verified numeric UDP source IP from
the beamforming/output computer; an optional nonzero source port makes the
admission check cover the complete endpoint. The sonar-head control address is
not substituted for this output-source identity. Packets from every other
source are dropped before they acquire DT100 provider provenance. Passive
`udp_raw` capture remains available separately and stays labelled
`unverified_udp`.
MARINER accepts only the documented 83P
profile-point format and rejects 83A, 83B, 837, malformed lengths, and invalid
headers. Its selected profile additionally requires 480 beams over a
120-degree sector, a -60-degree first beam, 0.25-degree spacing, 5000-sample
high-resolution processing, intensity output, 240 kHz, and the selected
sound-speed value. These are wire-admission settings, not proof that the vendor
process was configured correctly; a mismatch publishes no cloud. It does not
reinterpret other formats as a shared byte layout. The
checked-in Ping360 provider implements UDP transport only; USB or serial
transport is not implemented and must not be assumed. When selected, the UDP
provider uses the Blue Robotics Ping protocol and publishes both raw messages
and a provider-neutral profile carrying angle, range resolution, gain,
frequency, source identity, acquisition frame, and extrinsic revision.

Physical endpoint, identity, frame, and profile live in IG Handle. MARINER may
consume an accepted profile for mapping; ORACLE may request a sonar-relevant
mission; neither owns device commands.

DT100 and Ping360 have separate configured frames. The DT100 seed is a
down-looking 120-degree cross-track fan on `dt100_link`; Ping360 is a
horizontal mechanical scan on `ping360_link`. Their current revision tokens
(`dt100-seed-2026-08-11-v1` and `ping360-seed-2026-08-11-v1`) identify separate
configuration seeds, not measured calibration. Both transforms remain physically
unverified, so marker observations are shadow-only until the active provider,
axes, origin, and revision are measured. For a structured marker,
DT100 is the primary pose sensor when its simultaneous fan intersects the
constellation. Ping360 is a useful 360-degree discovery or planar fallback,
but its sequential sweep and broad vertical aperture do not supply an
instantaneous six-degree-of-freedom pose.

Read-only identity is the lowest-risk first connection and still exchanges Ping
protocol request and response packets. Active acoustic scanning requires both
scan mode and the configured allow flag because a scan changes physical device
state. The provider rejects invalid message lengths, checksums, ranges,
profiles, and identity. These runtime guards remain even without standalone
validation utilities.

The simulator can publish the same canonical profile with an explicit producer-
set `synthetic=true` field; the physical provider always sets it false. Replay
can reproduce transport and mapping behavior. Neither establishes real
acoustic propagation, multipath, target reflectivity, beam geometry, or latency.
Adding this producer-set field on 2026-08-11 intentionally changed the ROS 1
`SonarProfile` MD5 to `c60a9cd87d90490ea37c2ae5164e2b76`. Older profile
bags are wire-incompatible with the current message and require an explicit,
reviewed migration bridge or conversion rule before replay; they must not be
silently treated as current profiles.

Field evidence retains device identity, profile, sound-speed assumption, pose
and frame relationship, environment, raw packets or profiles, and synchronized
navigation state. ROS receipt time plus 83P latency fields can recover the
packet-reported center-ping time, but no real capture has yet established the
beamforming computer's clock basis, latency semantics during live output, or
network-delay bound. A sonar image is observation evidence, not a metric defect
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
