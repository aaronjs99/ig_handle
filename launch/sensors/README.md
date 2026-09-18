# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| start_cam.launch | Starts one serial-bound Forge GigE camera with its configured acquisition profile, CameraInfo, and optional debayered viewer stream; isolates the driver's unsafe optional GenICam property polling from `/diagnostics` by default. | spinnaker_camera_driver, image_proc, config/sensors/cameras/ | scripts/sensors/sensor_bringup.py through the sensor contract |
| start_imu.launch | Starts Xsens hardware on the stable device path, retains receipt-stamped IMU at `<topic>_receipt` plus the original TimeReference, and runs sample-clock normalization under the existing canonical publisher name. Simulation publishes canonical acquisition timestamps directly. | xsens_mti_driver, scripts/imu/sample_clock.py, config/udev/99-ig-handle.rules | scripts/sensors/external_sensor_provider.py through the sensor contract |
| start_lidar.launch | Starts the VLP-16 packet driver and calibrated point-cloud transform, with the unused laser scan disabled by default. | velodyne_driver, velodyne_pointcloud | scripts/sensors/sensor_bringup.py through the sensor contract |
| start_sonar.launch | Accepts one fully resolved provider bundle from the sensor lifecycle owner, selects passive unverified UDP or guarded Ping360 paths, and keeps DT100 nodes absent until explicit hardware commissioning and source admission. | scripts/sonar/provider.py, config/sensors/sonar/profiles.yaml, optional dt100_driver | scripts/sensors/sensor_bringup.py through the sensor contract |
