# Sensor Platform

IG Handle is the physical acquisition boundary for GRANDE. It records what is
installed, how each device is reached, which frame and topic it owns, how time
is stamped, and whether the device is currently available.

The deployed inventory is defined by
[`sensor_contract.yaml`](../config/sensors/platform/sensor_contract.yaml), not by
launch-file names. [`sensor_network.yaml`](../config/network/sensor_network.yaml)
records runtime endpoints and interface roles.
[`sensor_frames.yaml`](../config/sensors/platform/sensor_frames.yaml) records configured
static transforms and their evidence state, while
[`sensor_models.yaml`](../config/sensors/platform/sensor_models.yaml) records capabilities
independent of one deployed serial.

Changing a physical transform requires measurement evidence. Runtime code may
select a configured device but cannot patch its geometry or identity.

The host uses separate local and boat-facing network roles. Stable device
identity uses serials, USB attributes, udev aliases, and explicit network
endpoints. A changing `/dev/ttyACM*` number or switch port is not itself a new
identity. The runtime network contract keeps separate DT100 and Ping360 endpoints on
the sensor subnet. Both sonar device addresses are unconfigured; the host-side address is
`192.168.2.10`.
The checked-in `01-netplan.yaml` template uses the same `192.168.2.0/24`
subnet. Neither candidate address nor netplan is a commissioned wiring or
network result. Applying netplan remains an explicit operator action after
verifying switch wiring, endpoint identities, and link state.
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
publishes only below `/sensors/imu`, and is owned by exactly one persistent
Xsens service. The standalone user service uses the independently owned local
master at `http://127.0.0.1:11311` and advertises the sensor-LAN address
`192.168.50.10`; the integrated system service explicitly uses the physical
Heron master. Both values are validated from the canonical network contract.
The provider waits without opening another device when its selected master or
exact serial is unavailable, restarts after USB loss, stale output, driver exit,
or master replacement, and refuses duplicate publishers. It records immutable
process identities and revalidates current ancestry before signaling anything.
`sensor_bringup` observes this external owner for readiness but never opens or
stops its serial port.

Internally owned providers are launched only when the ROS publisher graph is
available and the required topics are unclaimed. The supervisor verifies each
publisher against the current managed process tree before reporting it alive,
uses bounded ROS XML-RPC calls, and signals only processes whose PID, immutable
start time, and current ancestry are all revalidated. Publishers on disabled
sensor topics remain untouched but are exposed explicitly in health output.
Every `sensor_bringup` instance also holds the same advisory lease at
`/run/lock/ig-handle-sensor-bringup.lock` for its full lifetime. Because both
standalone and integrated launches pass through this process, cameras and
LiDAR cannot be started on two ROS masters at once. The kernel releases the
lease on exit or crash; stale file contents never establish ownership.

## Fixed ROS Graph Profiles

The active deployment uses integrated sensing on the Heron graph:
`ig-handle-xsens-integrated.service` owns the IMU and
`ig-handle-sensors-integrated.service` owns camera/LiDAR supervision. These
transient user units are not themselves a verified reboot-persistence guarantee.
The separate `ig-handle-roscore-user.service` owns the loopback master used by
`battery-ighandle-monitor.service`; that battery-only graph does not own physical
IMU, camera, or LiDAR devices.

The maintained standalone alternative consists of the core, Xsens-user, and
sensor-bringup-user units under `systemd/`. Do not enable those sensor units while
integrated sensing owns the devices. Both profiles use the same sensor leases,
explicit master selection, publisher checks, and serial identity checks. A profile
change requires stopping the current owner first; it never happens automatically.

User services source `GRANDE_WORKSPACE_SETUP` from the optional
`~/.config/grande/environment` deployment configuration, with the conventional
workspace below the current user's home as default. Network addresses come from
`sensor_network.yaml`, accessed through `network_config.py`. The local master
must remain loopback-only and its advertised sensor address must be assigned to
a local interface. A battery service may use that core without starting the
standalone physical sensor stack.

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
battery service depends on the independently owned local ROS master when the
Heron is off; it never creates or stops that core. The combined service uses the
physical Heron master and validated `/sense_heron` ingress. Both use the same
data root and are mutually exclusive. Repository data remains retained
historical evidence rather than live mutable state.

Energy integration is available only from contiguous, identity-qualified compute
pack power. Propulsion motor-controller currents are preserved for diagnosis but
do not establish total pack current; propulsion energy therefore remains null
with an explicit unavailability reason.

## Independent Sonar Devices

IG Handle owns two separately selectable sensor identities. ID 8 is the DT100
echosounder under `/sensors/sonar/echosounder/`; ID 9 is the Ping360 imaging
sonar under `/sensors/sonar/imaging/`. Each has a separate network endpoint,
configured frame, provider process, and lifecycle state. Selecting neither,
either one, or both uses the same sensor lifecycle owner. The shared Ethernet
switch and arm cable are a proposed physical topology, not installed or
electrically qualified by this software change. The box would require
independently regulated power for the switch and each sonar. Its cable current,
voltage drop, connectors, and installed device revisions still require design
verification. The [published DT100 supply range](https://imagenex.com/assets/images/downloads/DT100_Specs_rev5.pdf) is 22–32 V; the [Ping360 Ethernet wiring guide](https://bluerobotics.com/learn/changing-communications-interface-on-the-ping360/) states 11–18 V.

ID 8 remains an uncommissioned passive UDP receiver. Its source address is
blank and `hardware_commissioned` is false. Receiving an unverified datagram
does not assign DT100 identity or make a decoded cloud. Once commissioned,
the DT100 vendor output path admits only the configured source endpoint;
MARINER's existing decoder accepts the documented 83P profile format and
rejects incompatible or malformed formats. Its downstream products are
raw packets, a decoded cloud, an accumulated cloud, and the latest slice.
The selected 83P profile settings remain in the dedicated sonar profile file;
they are admission settings, not evidence of physical configuration.

ID 9 uses the existing Ping protocol over UDP. The default mode requests
identity only; it cannot produce scan imagery and must report no recent
measurement. Active acoustic scanning requires both scan mode and the
configured transmit allowance. In scan mode, ten seconds without a valid
profile causes a one-time motor-off request and stops further scan requests.
That UDP request has no device acknowledgement here; physical motor state
must be checked separately. Profiles, a polar intensity image, raw packets,
and diagnostics are kept distinct. Intensity is not geometric range.

The sensor supervisor reports provider process operation separately from
measurement availability. Absent profiles in identity mode and old latched
diagnostics are observations, not reasons to restart the provider. The
dashboard displays both devices separately; DT100 uses the cloud viewer,
and Ping360 uses the on-demand image viewer. A positive source-data label
also requires verified publisher ownership. GRANDE's runtime and raw bag
profiles include both configured sonar streams; narrower recording profiles
have their own topic lists. MARINER can start both processing paths together.
Optional Range Aid marker processing launches one independent frontend per
enabled sonar. DT100 and Ping360 retain separate timestamps, extrinsic
revisions, observations, statuses, and diagnostic visualizations. Both can
run together without making either stream authoritative for navigation.

Configured frames `dt100_link` and `ping360_link` retain their seed
extrinsic revisions, not measured calibration. DT100's down-looking fan and
Ping360's horizontal sequential scan do not share a geometric model.
The simulator can supply explicitly synthetic packets or profiles through
the same downstream processing; simulation is not proof of physical
propagation, endpoint identity, or calibration. Physical source, timing,
frame and profile evidence must be retained before claiming a calibrated
map or marker observation.

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
