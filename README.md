# IG Handle

IG Handle is GRANDE's physical sensor and platform-interface package. It owns
device identity, sensor bringup, network endpoints, timestamps, static sensor
geometry, raw acquisition, hardware supervision, and the embedded telescope
interface.

It does not own navigation, mapping, mission planning, or simulation. MARINER
consumes canonical sensor topics; GRANDE composes and records the runtime.

## Responsibilities

| Area | IG Handle owns |
| --- | --- |
| Sensors and power | Physical inventory, battery identity, endpoint selection, drivers, raw capture, and health. |
| Frames | Measured sensor extrinsics and configured static sensor edges. |
| Network | Tracked sensor addresses and host/interface roles. |
| Timing | Device stamps, timing inputs, and embedded timing surfaces. |
| Sonar | Raw acquisition, Ping360 protocol provider, identity, and profile metadata. |
| Telescope | Hardware configuration, homing, position feedback, and guarded motor interface. |
| Heron hardware | Physical propulsion facts, inspection guidance, and commissioning boundaries. |

## Documentation

- [Sensor platform](docs/platform.md) covers inventory, network, cameras, supervision, recording, and stable device identity.
- [Sensor timing firmware](docs/sensor_timing.md) covers fail-closed Teensy reference qualification, trigger/feedback wiring contracts, and bench acceptance.
- [Telescoping arm](docs/telescope.md) covers the motor, driver, encoder, home switch, current sensing, packaging, and remaining measurements.

Each narrative document has a matching PDF. Markdown is canonical.

## Independent sonar devices

Sensor ID 8 owns DT100 echosounder UDP acquisition; sensor ID 9 owns Ping360
identity and, only when explicitly configured, imaging profiles. Both can be
selected together. Their network endpoints, frames, provider processes, raw
topics, and data-availability reports are independent. The DT100 endpoint is
unconfigured; its current passive UDP receiver does not establish that a packet
came from a commissioned DT100. The Ping360 device address is also unconfigured; the typical Ethernet UDP
port `12345` remains unverified for the installed device. Ping360 starts in identity mode, which does not produce scan
profiles or images. Missing imagery is reported without restarting the provider.
A scan that stops returning valid profiles sends one motor-off command and
remains stopped until a new explicit session.

The proposed underwater arrangement has one arm cable feeding an Ethernet
switch and separate regulated DC outputs in a junction box. At 10/100 Mbps,
two intact twisted pairs carry Ethernet; the other conductors may carry a
custom DC feed only after cable ampacity, voltage drop, connectors, supply,
switch, and installed sonar revisions are checked. Published [DT100 specifications](https://imagenex.com/assets/images/downloads/DT100_Specs_rev5.pdf)
state 22–32 V, while the [Ping360 Ethernet wiring guide](https://bluerobotics.com/learn/changing-communications-interface-on-the-ping360/)
states 11–18 V. Use separate regulated outputs after checking the installed
revisions. This is a topology proposal, not a wiring or physical-readiness claim.


Physical addresses, frames, serials, ports, polarity, geometry, and current
scales must come from reviewed hardware records. Values marked provisional,
placeholder, or unverified are not commissioned facts. A launch or session may
select a configured device; it must not invent those facts.

# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitattributes | Defines repository text and binary path handling. | Git | Repository contributors |
| .gitignore | Applies the shared GRANDE exclusions and additionally keeps locally extracted NatNet SDK trees out of source control. | Git | Repository contributors |
| CMakeLists.txt | Builds package-owned messages and installs sensor entrypoints, runtime modules, launch/configuration resources, persistent service definitions, and the pinned vendor executable across devel and install spaces. | catkin, ROS Noetic message generation, setup.py | catkin build and install spaces |
| LICENSE | Provides the repository-level MIT license terms. | None | Repository users and redistributors |
| package.xml | Declares ROS package metadata plus power, Bluetooth, and sensor dependencies. | ROS Noetic, BlueZ D-Bus, GLib | catkin and rosdep |
| setup.py | Installs the reusable sensor, sonar, power, and motion-capture packages on the standard source, devel, and install Python paths. | catkin_pkg, scripts/sensors, scripts/sonar, scripts/power, scripts/mocap | CMakeLists.txt, IG Handle, and GRANDE Python consumers |
