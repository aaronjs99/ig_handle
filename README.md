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
| Heron hardware | Physical propulsion inspection, commissioning, and replacement records. |

## Documentation

- [Sensor platform](docs/platform.md) covers inventory, network, cameras, supervision, recording, and stable device identity.
- [Telescoping arm](docs/telescope.md) covers the motor, driver, encoder, home switch, current sensing, packaging, and remaining measurements.

Each narrative document has a matching PDF. Markdown is canonical.

Physical addresses, frames, serials, ports, polarity, geometry, and current
scales come from measured hardware records. A launch or session may select a
configured device; it must not invent those facts.

# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitattributes | Defines repository text and binary path handling. | Git | Repository contributors |
| .gitignore | Excludes generated IG Handle build and sensor artifacts. | Git | Repository contributors |
| CMakeLists.txt | Declares the ROS package build and installed sensor scripts. | catkin, ROS Noetic | catkin build |
| package.xml | Declares ROS package metadata and sensor dependencies. | ROS Noetic | catkin and rosdep |
