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
| Sensors | Physical inventory, endpoint selection, drivers, raw capture, and health. |
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

Physical addresses, frames, serials, ports, polarity, geometry, and current
scales must come from reviewed hardware records. Values marked provisional,
placeholder, or unverified are not commissioned facts. A launch or session may
select a configured device; it must not invent those facts.

# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitattributes | Defines repository text and binary path handling. | Git | Repository contributors |
| .gitignore | Excludes generated IG Handle build and sensor artifacts. | Git | Repository contributors |
| CMakeLists.txt | Builds package-owned sonar messages and installs executable sensor entrypoints, standard catkin Python runtime packages, launch/configuration resources, and the pinned vendor executable across devel and install spaces. | catkin, ROS Noetic message generation, setup.py | catkin build and install spaces |
| LICENSE | Provides the repository-level MIT license terms. | None | Repository users and redistributors |
| package.xml | Declares ROS package metadata and sensor dependencies. | ROS Noetic | catkin and rosdep |
| setup.py | Installs the reusable `ig_handle_runtime`, `ig_handle_sonar`, and mocap transport packages on the standard source/devel/install Python path. | catkin_pkg, scripts/ig_handle_runtime, scripts/ig_handle_sonar, scripts/ig_handle_mocap_natnet, scripts/ig_handle_mocap_udp | CMakeLists.txt, IG Handle and GRANDE Python consumers |
