# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| start_cam.launch | Launches and wires the start cam ROS runtime. | nodelet | ig_handle/config/sensors/sensor_contract.yaml |
| start_imu.launch | Launches and wires the start imu ROS runtime. | xsens_mti_driver | ig_handle/config/sensors/sensor_contract.yaml |
| start_lidar.launch | Launches and wires the start lidar ROS runtime. | None | ig_handle/config/sensors/sensor_contract.yaml |
| start_sonar.launch | Launches and wires the start sonar ROS runtime. | ig_handle, dt100_driver | ig_handle/config/sensors/sensor_contract.yaml |
| start_thermal.launch | Launches and wires the start thermal ROS runtime. | nodelet | ig_handle/config/sensors/sensor_contract.yaml |
