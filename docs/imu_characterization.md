# Xsens stationary IMU characterization

IGHandle owns IMU acquisition and acquisition timestamps. DLiO estimates vehicle
motion. Stationary IMU statistics describe noise and drift; they do not establish
absolute accuracy, thrust, or vehicle-state covariance.

## Analyze a saved recording

After the recording closes, use a new output directory:

~~~bash
rosrun ig_handle analyze_stationary_bag.py INPUT.bag \
  --output-dir NEW_ANALYSIS_DIRECTORY \
  --device-id 0368319D \
  --declared-stationary \
  --mounting-state "secured on lab bench" \
  --location "Boelter Hall"
~~~

The analyzer reads /sensors/imu/data and optional time-reference and magnetometer
topics. It writes analysis JSON plus one-second, Allan-deviation, and power
spectral density CSV tables. Timing, frame, publisher, sequence gaps, covariance,
noise proxies, and drift are reported as observations. It does not generate a
calibration candidate, manifest, file hashes, or a pass/fail certificate.

Raw bags remain outside Git under /home/ig-handle/bags/sensors/imu/.
Place analysis outputs under analysis/<run-id> there. Existing output directories
are not overwritten. The source bag is read-only.

The device ID and stationarity are operator descriptions; they are not independently
established by the IMU messages. The time-reference message may use a different
epoch from ROS time. The report keeps that clock separate from the acquisition
timestamps used by estimation.

## Physical measurements still needed

| Measurement | What it identifies | Limitation |
| --- | --- | --- |
| Stationary recording | Repeatability, apparent gyro bias, noise, vibration and drift | Does not establish absolute accuracy or orientation covariance |
| Six accelerometer orientations | Bias, scale and cross-axis effects relative to gravity | Does not identify gyro scale or vehicle dynamics |
| Repeated cold starts | Startup bias and warmup repeatability | Does not establish dynamic accuracy |
| Motion against an independent reference | Rate, attitude, axes and timestamp alignment | Requires a reference independent of the IMU |
| Synchronized LiDAR/IMU and independent pose | Localization error and uncertainty | Must cover the intended operating conditions |

Keep environment, mounting, temperature when available, and capture times with
the measurements. Use separate recordings to assess fitted model accuracy.
Descriptive stationary covariance is not automatically installed as DLiO pose or
twist covariance. This offline analysis does not authorize or stop navigation.
