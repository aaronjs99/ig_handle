#!/bin/bash

set -e # exit on first error

now=$(date +"%Y_%m_%d_%H_%M_%S")
bagDir="${1/#~/$HOME}/$now"
mkdir -p $bagDir
echo "Collecting ig_handle bag file..."
echo "Saving to: $bagDir/raw.bag"
rosbag record -O $bagDir/raw.bag \
  /sensors/camera/f1/image_raw/compressed \
  /sensors/camera/f2/image_raw/compressed \
  /sensors/camera/f3/image_raw/compressed \
  /sensors/camera/f4/image_raw/compressed \
  /sensors/camera/thermal/image_raw/compressed \
  /sensors/camera/time \
  /sensors/imu/data \
  /sensors/imu/time \
  /sensors/pps/time \
  /sensors/lidar/hori/packets \
  /sensors/lidar/hori/points \
  /sensors/lidar/vert/packets \
  /sensors/lidar/vert/points \
  /sensors/sonar/scan
