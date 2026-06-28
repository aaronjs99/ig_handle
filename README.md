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

The minimal SLAM input set is:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/sensors/lidar/hori/points` | `sensor_msgs/PointCloud2` | Primary LiDAR geometry |
| `/sensors/imu/data` | `sensor_msgs/Imu` | Inertial input for LIO backends |

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

The Heron adapter starts the shared IG sensor suite. Use the global camera
switch or per-camera switches when isolating camera issues:

```bash
roslaunch grande bringup.launch mode:=real use_cameras:=false
roslaunch grande bringup.launch mode:=real use_camera_f2:=false
```

## Camera Notes

Forge cameras are selected by serial and expected GigE IP so the Spinnaker SDK
does not accidentally bind to duplicate wrong-subnet entries.

The default Forge profile is conservative:

- continuous/free-run acquisition
- 10 Hz
- 10 ms exposure ceiling
- `BayerRG8`
- ISP disabled
- centered 1280 x 1024 ROI

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
The bridge publishes raw comparison topics under `/mocap` and leaves TF
publishing disabled by default.

Run the datacollect UDP receiver when the Motive-side broadcaster is active:

```bash
roslaunch "$(rospack find ig_handle)/launch/core/natnet_bridge.launch" \
  transport:=datacollect_udp
```

Initialize odometry-vs-mocap alignment only after odometry, mocap, and IMU
samples are available:

```bash
rostopic pub -1 /mocap/initialize_alignment std_msgs/Bool "data: true"
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
pytest -q ig_handle/tests/test_hardware_connectivity.py
```

Heron base checks:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/heron_ws/devel/setup.bash
export ROS_MASTER_URI=http://$(rosrun ig_handle network_config.py heron_ip):11311
export ROS_IP=$(rosrun ig_handle network_config.py heron_local_ip)
unset ROS_HOSTNAME

pytest -q ig_handle/tests/test_heron_base.py
```

The default Heron battery floor is `14.0 V`; override with
`IG_HANDLE_HERON_MIN_BATTERY_V` only when the lab threshold intentionally
changes.

## Udev Rules

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

The canonical Xsens port is `/dev/sensors/imu`. Keep the driver binding, stable
device name, and serial permissions in `config/udev/99-ig-handle.rules`.

## ROS 2

A ROS 2 version of this package exists on the `ros2` branch.
