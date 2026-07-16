# IG Handle

IG Handle is the synchronized sensing and data-collection package for the
GRANDE platform. It owns real sensor bringup, hardware endpoint configuration,
timing surfaces, and raw capture support.

It does not own navigation, mapping, mission planning, or simulation. MARINER
consumes the sensor topics for localization and mapping; `grande` composes the
full runtime.

## Sensor Stack

The Heron platform profile is organized around:

- horizontal Velodyne VLP-16 LiDAR
- vertical Velodyne VLP-16 LiDAR
- Xsens IMU / AHRS
- four Forge IP67 GigE cameras
- Imagenex DT100 sonar
- optional motion-capture comparison link
- optional Teensy timing bridge
- optional telescoping sonar arm driven by the Teensy bridge

The minimal SLAM input set is:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/sensors/lidar/hori/points` | `sensor_msgs/PointCloud2` | Primary LiDAR geometry |
| `/sensors/imu/data` | `sensor_msgs/Imu` | Inertial input for LIO backends |

The telescope command and feedback names are defined in
`config/runtime_surface.yaml`. GRANDE sends a desired arm length in metres on
`/actuators/telescope/command/length`; the Teensy publishes metres on
`/actuators/telescope/state/length`, calibrated amperes on
`/actuators/telescope/state/motor_current`, and a textual state on
`/actuators/telescope/state/status`. `main.ino` owns this command path
alongside timing. It will not configure any telescope pin or energize the
motor until wiring, geometry, encoder direction, motor polarity, and current
sense calibration are measured and the three firmware enable gates are true.

The firmware-side values live in `config/teensy/firmware_config.h`, while the
human-readable hardware record lives in `config/telescope/hardware.yaml`.
`main/telescope_control.h` maps calibrated encoder counts to arm length and
length to motor revolutions. `main/telescope_runtime.h` implements the guarded
position loop: minimum-limit homing rebases the encoder zero, maximum travel
clamps extension, both NO/NC contacts are checked, stale commands and encoder
stalls stop the motor, and every command is range-checked before a PWM output.


## Network Configuration

Sensor endpoint facts live in:

```text
config/network/sensor_network.yaml
config/network/01-netplan.yaml
```

Use those files as the source of truth for IP addresses, interfaces, and launch
defaults. `/etc/hosts` entries can mirror the same values, but they are not the
package authority.

Typical host roles:

| Interface | Purpose |
| --- | --- |
| `enp2s0` | PoE switch path for Heron, LiDARs, cameras, and sonar |
| `wlo1` | Lab Wi-Fi and optional mocap link |
| `tailscale0` | Remote access |

Check the live machine against the tracked config:

```bash
ip -br addr
```

## Launch

Full-stack real runs should normally start through GRANDE:

```bash
roslaunch grande bringup.launch mode:=real
```

Raw sensor collection can be launched directly when testing the sensor layer:

```bash
roslaunch ig_handle collect_raw_data.launch
```

The Heron adapter starts the shared IG sensor suite from the sensor contract.
Use contract ids when isolating individual sensors:

```bash
roslaunch grande bringup.launch mode:=real disabled_sensor_ids:=<contract-id>
roslaunch grande bringup.launch mode:=real extra_sensor_ids:=<contract-id>
```

Contract ids are opaque inventory keys. They are not positions, counts, or a
contiguous sequence, so callers should never infer the sensor count from the id
values. Runtime packages should use deployment bindings such as `state_input`,
`mapping_primary`, `ranging_raw`, and binding groups such as
`inspection_streams`; only IG Handle should translate those bindings into the
physical devices, endpoints, frames, and topics.

## Camera Notes

Forge cameras are selected by serial and expected GigE IP so the Spinnaker SDK
does not accidentally bind to duplicate wrong-subnet entries.

The default Forge profile is conservative:

- continuous/free-run acquisition
- 10 Hz
- 10 ms exposure ceiling
- `BayerRG8`
- ISP disabled
- centered 640 x 512 ROI

Do not enable Line0 hardware trigger mode unless the trigger source is connected
and verified. Placeholder CameraInfo files keep topics present but are marked
uncalibrated.

## Sonar Notes

IG Handle publishes DT100 raw packets on:

```text
/sensors/sonar/raw
```

MARINER owns the downstream decoder that turns supported profile packets into:

```text
/sensors/sonar/scan
```

Keep both raw and decoded sonar topics in field bags. Raw packets prove device
reception; decoded scan clouds prove that the packet mode can be mapped.

The standalone IG Handle sonar default is `sonar_profile:=pool`. Top-level
GRANDE bringup resolves the sonar profile from the selected scenario.

## Motion Capture

Mocap is a lab logging and comparison path, not the production odometry source.
The bridge publishes raw comparison topics from `ig_handle/config/runtime_surface.yaml`
and leaves TF publishing disabled by default.

Run the datacollect UDP receiver when the Motive-side broadcaster is active:

```bash
roslaunch "$(rospack find ig_handle)/launch/core/natnet_bridge.launch" \
  transport:=datacollect_udp
```

Initialize odometry-vs-mocap alignment only after odometry, mocap, and IMU
samples are available:

```bash
rostopic pub -1 /mocap/initialize std_msgs/Bool "data: true"
```

## Recording

The integration recorder should capture:

- all active cameras
- IMU
- horizontal and vertical LiDAR
- DT100 sonar raw and decoded scan
- Heron base telemetry
- TF
- optional mocap comparison topics

Runtime capture topics often use compressed image transports to keep bags
manageable. Live consumers should use raw image roots unless a launch profile
explicitly says otherwise.

## Hardware Validation

Connectivity checks:

```bash
rosrun ig_handle network_config.py heron_ip
rosrun ig_handle network_config.py heron_local_ip
```

Heron base checks:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/heron_ws/devel/setup.bash
export ROS_MASTER_URI=http://$(rosrun ig_handle network_config.py heron_ip):11311
export ROS_IP=$(rosrun ig_handle network_config.py heron_local_ip)
unset ROS_HOSTNAME

rostopic list
rostopic echo -n 1 /sense
rostopic echo -n 1 /motor_enable
```

The default Heron battery floor is `14.0 V`; override with
`IG_HANDLE_HERON_MIN_BATTERY_V` only when the lab threshold intentionally
changes.

## Udev Rules

IG Handle owns hardware identity. Session scripts should never patch Xsens
ports, create ad-hoc symlinks, or guess `ttyUSB*`.

What happened on June 26, 2026: the Xsens MTi appeared as more than one USB
serial interface. A broad `/dev/imu` rule could bind the ROS driver to the
wrong interface, so the launch sometimes opened a port that was not the live
data stream. The permanent fix is one package-owned rule that binds the Xsens
VID/PID and exposes only USB interface `01` as `/dev/sensors/imu`.

```bash
sudo rm -f \
  /etc/udev/rules.d/99-udev.rules \
  /etc/udev/rules.d/99-xsens.rules \
  /etc/udev/rules.d/99-xsens-mti-ftdi.rules \
  /etc/udev/rules.d/99-ig_handle_udev.rules \
  /etc/udev/rules.d/99-ig2_udev.rules
sudo install -m 0644 config/udev/99-ig-handle.rules /etc/udev/rules.d/99-ig-handle.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo udevadm settle
udevadm info -q property -n /dev/sensors/imu | egrep 'ID_VENDOR_ID|ID_MODEL_ID|ID_USB_INTERFACE_NUM|DEVLINKS'
```

The verification should show `ID_VENDOR_ID=2639`, `ID_MODEL_ID=0003`,
`ID_USB_INTERFACE_NUM=01`, and `DEVLINKS` containing `/dev/sensors/imu`.

The canonical Xsens port is `/dev/sensors/imu`. Keep the driver binding, stable
device name, serial permissions, and latency setting together in
`config/udev/99-ig-handle.rules`. Keep `ig_handle/config/sensors/sensor_contract.yaml`
and `launch/sensors/start_imu.launch` pointed at that canonical device path.

## ROS 2

A ROS 2 version of this package exists on the `ros2` branch.
