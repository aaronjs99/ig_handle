# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| start_cam.launch | Starts one serial-bound Forge GigE camera with its configured acquisition profile, CameraInfo, and optional debayered viewer stream. | spinnaker_camera_driver, image_proc, config/sensors/cameras/ | scripts/sensor_bringup.py through the sensor contract |
| start_imu.launch | Starts the Xsens driver on the stable IMU device path and remaps measurement and device-time outputs to contract topics. | xsens_mti_driver, config/udev/99-ig-handle.rules | scripts/sensor_bringup.py through the sensor contract |
| start_lidar.launch | Starts the VLP-16 packet driver and calibrated point-cloud transform, with the unused laser scan disabled by default. | velodyne_driver, velodyne_pointcloud | scripts/sensor_bringup.py through the sensor contract |
| start_sonar.launch | Selects passive unverified UDP or guarded Ping360 paths while keeping DT100 process/receiver nodes absent until explicit hardware commissioning and configured UDP source admission. | scripts/sonar/sonar.py, scripts/sensor_contract_launch_eval.py, config/sensors/sonar/profiles.yaml, optional dt100_driver | scripts/sensor_bringup.py through the sensor contract |
| start_thermal.launch | Starts the Boson USB camera and rectification nodelet on configured image topics. | flir_boson_usb, image_proc | scripts/sensor_bringup.py through the sensor contract |
