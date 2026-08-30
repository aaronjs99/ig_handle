# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| analyze_stationary_bag.py | Produces an overwrite-protected, hash-recorded stationary IMU characterization with timing, stationary covariance, first-difference noise proxies, drift, spectral, Allan-deviation, gravity, and one-second timeline outputs. The generated record is descriptive and cannot commission covariance publication or autonomous use. | ROS bag, NumPy, PyYAML, finalized bag containing `/sensors/imu/data` and optional time-reference/magnetometer topics | Offline Xsens characterization and reviewed visualization inputs |
| force_wakeup.py | Sends the Xsens bootloader wake-up sequence for supervised recovery without replacing normal driver bringup. | PyUSB, physical Xsens USB connection | CMake installation and manual IMU recovery |
| validate_stationary_candidate.py | Recomputes source-bag and artifact hashes through the manifest, then fails closed on incomplete capture gates, wrong identity/frame/topic/caller/units, nonfinite values, asymmetric or invalid matrices, inconsistent proxy standard deviations, or any attempt to mark the descriptive record commissioned. Passing validation does not commission covariance. | NumPy, PyYAML, source bag, analysis manifest | Review gate for outputs from `analyze_stationary_bag.py` |
