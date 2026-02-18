# IG Handle: Multi-Modal Hardware-Synchronized Orchestration Layer

## Overview

**IG-Handle** is an open-source, hardware-synchronized multi-modal orchestration layer for acquiring high-fidelity LiDAR-visual-inertial datasets with strict temporal alignment across sensor streams. Synchronization is driven by a microcontroller and hardware time references to reduce timing uncertainty in downstream perception and SLAM.

### Standard sensor stack

- **2× Velodyne VLP-16** LiDARs (spatial geometry)
- **2× FLIR Blackfly S USB3** monochrome cameras (visual context)
- **1× Xsens MTi-30 AHRS** IMU (inertial state)

## How synchronization works

Synchronous acquisition is orchestrated via a **DS3231 RTC** and a **Teensy 4.1**, enabling:

- **NMEA/PPS temporal alignment** for LiDAR timing (RTC accuracy: ±2 ppm)
- **Deterministic stereo triggering** for dual cameras at 20 Hz
- **Interrupt-driven IMU sampling** at 200 Hz

The architecture is extensible to additional LiDARs, thermal cameras, and sonar-based bathymetry.

### References

```bibtex
@article{thoms2023tightly,
  title={Tightly Coupled, Graph-Based DVL/IMU Fusion and Decoupled Mapping for SLAM-Centric Maritime Infrastructure Inspection},
  author={Thoms, Alexander and Earle, Gabriel and Charron, Nicholas and Narasimhan, Sriram},
  journal={IEEE Journal of Oceanic Engineering},
  year={2023},
  publisher={IEEE}
}
```

- Parts list, CAD, schematics, and build instructions: [Google Drive folder](https://drive.google.com/drive/folders/1DrAMQ9eQS1JjoDI4LWuoENaN7cZ9nRTC?usp=sharing)
- Reference compute: [Intel NUC kit details](https://drive.google.com/file/d/1mJj0qhpS1F2KvkGdUfzvh3qHi5qF908q/view?usp=sharing)

---

## Installation

Follow the BEAM installation guide for a clean Ubuntu 20.04 machine:  
[Beam Robotics Installation Guide](https://github.com/BEAMRobotics/beam_robotics/wiki/Beam-Robotics-Installation-Guide)

Recommended catkin workspace: `~/catkin_ws`

---

## Quickstart: collect raw data

```bash
roslaunch ig_handle collect_raw_data.launch
```

- Output is a timestamped folder containing `raw.bag`.
- Use `output` to set an alternative parent directory:

```bash
roslaunch ig_handle collect_raw_data.launch output:=~/my_folder
```

This records to: `~/my_folder/YYYY_MM_DD_HH_MM_SS/raw.bag`

> Note: after power-on, the LiDAR may take **25 to 30 seconds** to appear on the network. Wait about 30 seconds before recording.

### Robot variants

`collect_raw_data.launch` supports robot-specific bundles via `robot:=...`

**ig-heron** adds cameras `F3/F4`, LiDAR `lidar_v`, and DT100 sonar:

```bash
roslaunch ig_handle collect_raw_data.launch robot:=heron
```

**ig-husky** adds cameras `F3/F4`, LiDAR `lidar_v`, Husky base/control packages, and includes a thermal camera (not tested):

```bash
roslaunch ig_handle collect_raw_data.launch robot:=husky
```

Before using these launch files, create a bag directory:

```bash
mkdir -p bags
```

---

## Field workflow

### ig-handle only

We recommend using the touch monitor connected to the handle computer.

```bash
roslaunch ig_handle collect_raw_data.launch
```

Stop recording with `ctrl+c`.

### ig-husky and ig-heron

1. Connect your laptop to the handle computer over Ethernet and assign your laptop an IP on the LiDAR subnet (example: `192.168.1.151`).

   > If your laptop is also on Wi-Fi using `192.168.1.0/24`, you may hit a subnet collision. See **System configuration → Networking → Avoiding Wi-Fi subnet collisions**.

2. SSH into the handle computer:

   ```bash
   ssh ig-handle@192.168.1.<HANDLE_IP>
   ```

   `<HANDLE_IP>` is the static IP assigned to the ig-handle host on the LiDAR subnet (example: `192.168.1.10`).

3. Start a `screen` session:

   ```bash
   screen
   ```

   Press `enter` to start the session.

4. Start recording (uncomment the robot you are using):

   ```bash
   roslaunch ig_handle collect_raw_data.launch \
     robot:=husky  # ig-husky
   # robot:=heron  # ig-heron
   ```

5. Detach: `ctrl+a`, then `ctrl+d`

6. Disconnect the Ethernet cable and perform data collection.

7. Reconnect Ethernet and reattach to end recording:

   ```bash
   screen -r
   ```

   Stop with `ctrl+c`.

---

## Raw data description

`collect_raw_data.launch` invokes `ig_handle/scripts/pipeline/record_bag.sh` and records robot-specific topics.

### ig-handle topics

| Topic                                     | Message type                   |
|-------------------------------------------|--------------------------------|
| `/sensors/camera/f1/image_raw/compressed` | `sensor_msgs/CompressedImage`  |
| `/sensors/camera/f2/image_raw/compressed` | `sensor_msgs/CompressedImage`  |
| `/sensors/camera/time`                    | `sensor_msgs/TimeReference`    |
| `/sensors/imu/data`                       | `sensor_msgs/Imu`              |
| `/sensors/imu/time`                       | `sensor_msgs/TimeReference`    |
| `/sensors/lidar/hori/packets`             | `velodyne_msgs/VelodyneScan`   |
| `/sensors/lidar/hori/points`              | `sensor_msgs/PointCloud2`      |
| `/sensors/pps/time`                       | `sensor_msgs/TimeReference`    |

### Additional topics for ig-heron and ig-husky

| Topic                                     | Message type                   |
|-------------------------------------------|--------------------------------|
| `/sensors/camera/f3/image_raw/compressed` | `sensor_msgs/CompressedImage`  |
| `/sensors/camera/f4/image_raw/compressed` | `sensor_msgs/CompressedImage`  |
| `/sensors/lidar/vert/packets`             | `velodyne_msgs/VelodyneScan`   |
| `/sensors/lidar/vert/points`              | `sensor_msgs/PointCloud2`      |

Additional sensors:
- **ig-heron** records `/sensors/sonar/scan` as `sensor_msgs/PointCloud2`
- **ig-husky** records `/sensors/camera/thermal/image_raw/compressed` as `sensor_msgs/CompressedImage`

To add or remove topics, edit `ig_handle/scripts/pipeline/record_bag.sh`.

---

## Raw data processing

Raw data is processed using: `scripts/pipeline/process_raw_bag.py`

### What it does

1. Restamps camera and IMU messages using their time reference messages
2. Interpolates sonar messages against the PPS reference signal

Because camera and IMU data messages may serialize later than their time references, the script discards messages before the first time reference (by serialized time), then restamps using a FIFO queue.

The script throws an error when signal dropout is detected (for example, loose connections). If dropout occurs late in a long recording, process only the valid prefix using `--bag_end`.

To inspect dropouts, visualize the raw bag with:

```bash
rosrun rqt_bag rqt_bag
```

### Interface

```bash
cd ~/catkin_ws/src/ig_handle/scripts/pipeline
python3 process_raw_bag.py --help
```

Example:

```bash
cd ~/catkin_ws/src/ig_handle/scripts/pipeline
python3 process_raw_bag.py --bag ~/bags/YYYY_MM_DD_HH_MM_SS/raw.bag
```

Outputs `output.bag` in the same folder.

> Note: LiDAR time sync may require a warm-up of about 5 seconds. We recommend starting playback at t=5 s:

```bash
cd ~/bags/YYYY_MM_DD_HH_MM_SS/
rosbag play --start=5 output.bag --pause
```

Press `enter` to continue playback.

---

## Copy data from robots

After processing, copy data to your laptop over Ethernet:

```bash
scp -r ~/bags/YYYY_MM_DD_HH_MM_SS user@192.168.1.XXX:~/bags/dir
```

Where:
- `YYYY_MM_DD_HH_MM_SS` is the folder containing the bag(s)
- `user` is your laptop username
- `192.168.1.XXX` is your laptop IP on the LiDAR subnet
- `~/bags/dir` is the destination directory on your laptop

Example:

```bash
scp -r ~/bags/2023_10_15_03_56_54 alex@192.168.1.151:~/bags/ig-handle
```

---

## System configuration

### Networking

IG-Handle uses dedicated IPv4 subnets for sensors:

- **LiDAR subnet:** `192.168.1.0/24`  
  Example device IPs: `192.168.1.201` (`lidar_h`), `192.168.1.202` (`lidar_v`)
- **Sonar subnet:** `192.168.0.0/24`  
  Example device IP: `192.168.0.2` (DT100)

In our build, the LiDARs (and the outward-facing Ethernet port) connect to an internal switch. The host aggregates sensor Ethernet adapters using a Linux bridge (`br0`).

**Default host addressing:** a single bridge (`br0`) is assigned both:
- `192.168.1.10/24` (LiDAR)
- `192.168.0.3/24` (sonar)

Wi-Fi (`wlo1`) remains the default route for internet.

#### Verify interface names

```bash
ip -br link
ip -br addr
```

You should see a `br0` interface and your Ethernet adapters (often named `enx...`) as bridge ports.

#### Configure `br0` with NetworkManager (recommended)

This configuration is persistent across reboots.

```bash
# Put br0 in manual IPv4 mode with both static addresses
sudo nmcli con modify br0 ipv4.method manual
sudo nmcli con modify br0 ipv4.addresses "192.168.1.10/24"
sudo nmcli con modify br0 +ipv4.addresses "192.168.0.3/24"

# Ensure br0 never becomes a default route (Wi-Fi stays the internet path)
sudo nmcli con modify br0 ipv4.never-default yes

# De-prioritize br0 routes compared to Wi-Fi if subnets collide
sudo nmcli con modify br0 ipv4.route-metric 5000

# Pin LiDAR device IPs to br0 (important on collided Wi-Fi networks)
sudo nmcli con modify br0 +ipv4.routes "192.168.1.201/32 0.0.0.0 10"
sudo nmcli con modify br0 +ipv4.routes "192.168.1.202/32 0.0.0.0 10"

# Apply changes
sudo nmcli con down br0 || true
sudo nmcli con up br0
```

#### Avoiding Wi-Fi subnet collisions (important)

Some Wi-Fi networks also use `192.168.1.0/24`. If Wi-Fi is assigned an address like `192.168.1.x` while the LiDAR network is also `192.168.1.0/24`, Linux will have two routes to the same subnet and may send gateway traffic out the wrong interface.

**Symptom:** Wi-Fi appears connected but cannot reach the gateway or internet.

**Fix:** keep Wi-Fi as the default route, and pin the LiDAR IPs to `br0` using `/32` host routes (shown above).

#### Sanity check

After configuring networking, verify all paths:

```bash
ip route get 8.8.8.8         # via Wi-Fi gateway, dev wlo1
ip route get 192.168.1.201   # dev br0, src 192.168.1.10
ip route get 192.168.1.202   # dev br0, src 192.168.1.10
ip route get 192.168.0.2     # dev br0, src 192.168.0.3
```

You can also confirm device reachability:

```bash
ping -c 3 192.168.1.201
ping -c 3 192.168.1.202
ping -c 3 192.168.0.2
```

#### Best practice (recommended for new builds)

To avoid collisions entirely, consider using a less common private subnet for LiDAR devices (for example, `192.168.50.0/24` or `10.42.0.0/24`) and reconfigure sensor IPs accordingly.

#### Advanced: netplan (headless systems only)

An example netplan configuration is provided in `config/01-ig_handle_netplan.yaml` for systems using `systemd-networkd` (common on Ubuntu Server or headless installs).

> Do not use netplan if NetworkManager is active.

To use netplan:
1. Update interface names in the YAML (see `ip -br link`).
2. Copy into `/etc/netplan/` and apply:

```bash
sudo cp config/01-ig_handle_netplan.yaml /etc/netplan/
sudo netplan apply
```

### Udev

Udev rules create stable device aliases for USB peripherals (for example, `/dev/teensy` and `/dev/imu`) so device names do not change across reboots or replugging.

To install the provided rules:

```bash
sudo cp config/99-ig_handle_udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER
```

You must log out and back in (or reboot) for the `dialout` group change to take effect.

---

## ROS 2

This package has been ported to ROS 2 on the `ros2` branch.