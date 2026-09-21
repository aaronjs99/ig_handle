# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| analyze_stationary_bag.py | Produces an overwrite-protected stationary IMU characterization with timing, stationary covariance, first-difference noise proxies, drift, spectral, Allan-deviation, gravity, and one-second timeline outputs. The generated record is descriptive and cannot commission covariance publication or autonomous use. | ROS bag, NumPy, finalized bag containing `/sensors/imu/data` and optional time-reference/magnetometer topics | Offline Xsens characterization and reviewed visualization inputs |
| force_wakeup.py | Sends the Xsens bootloader wake-up sequence for supervised recovery without replacing normal driver bringup. | PyUSB, physical Xsens USB connection | CMake installation and manual IMU recovery |
| sample_clock.py | Pairs receipt-stamped IMU and TimeReference messages, anchors each monotonic device-clock epoch into ROS time, and publishes unchanged measurements with acquisition intervals. Drops older receipts and duplicate samples; reanchors on device/ROS clock reset. The fixed epoch offset includes uncalibrated receipt delay. Raw inputs remain available. | rospy, sensor_msgs/Imu, sensor_msgs/TimeReference | launch/sensors/start_imu.launch; physical canonical IMU consumers |
