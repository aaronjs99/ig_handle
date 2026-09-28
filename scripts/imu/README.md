# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| analyze_stationary_bag.py | Produces an overwrite-protected stationary IMU characterization with timing, stationary covariance, first-difference noise proxies, drift, spectral, Allan-deviation, gravity, and one-second timeline outputs. The generated record is descriptive and cannot commission covariance publication or autonomous use. | ROS bag, NumPy, finalized bag containing `/sensors/imu/data` and optional time-reference/magnetometer topics | Offline Xsens characterization and reviewed visualization inputs |
| force_wakeup.py | Sends the Xsens bootloader wake-up sequence for supervised recovery without replacing normal driver bringup. | PyUSB, physical Xsens USB connection | CMake installation and manual IMU recovery |
| sample_clock.py | Consumes one typed Xsens record containing both the exact IMU measurement and its clock metadata. Hardware source sequence is read only from the typed field, never ROS Header.seq. It preserves raw intervals and source/host/device epochs; receipt-offset mapping is explicitly uncalibrated with unknown uncertainty. | rospy, sensor_msgs/Imu, xsens_mti_driver/XsensClockEvent, ig_handle/ClockedImu | launch/sensors/start_imu.launch; physical canonical IMU consumers |

| fit_clock_mapping.py | Fits an offline affine mapping from explicit one-to-one source/reference event correspondences and writes a positive mapping revision; reports scale, centered offset, residuals, and uncertainty basis without updating a running clock. | Python 3 standard library; CSV with the documented correspondence columns | Circuit timing analysis and explicitly configured camera-clock mapping |

## Offline clock-fit input

The CSV must identify established pairs with these columns:
correspondence_id, source_event_id, reference_event_id, source_clock_domain,
reference_clock_domain, source_clock_instance_id, source_clock_epoch_id,
reference_clock_instance_id, reference_clock_epoch_id, source_time_ns, and
reference_time_ns. Correspondence IDs, source event IDs, and reference event
IDs must each be unique in the fit. A fit is scoped to one source clock domain,
instance, and epoch and one reference clock domain, instance, and epoch; it
rejects rows that cross a device restart or clock reset.
Pairing is evidence supplied by the operator or an upstream association
procedure; the fitter never guesses matches from nearest receipt times. Run it
as python3 fit_clock_mapping.py pairs.csv mapping.json --revision N, where N is
a new positive revision for that exact source/reference clock scope. The
resulting JSON is the adapter input; do not reuse a revision after changing a
pair or fitted coefficient.

pair_uncertainty_ns is optional. If provided, it is the positive standard
uncertainty of the paired time difference and must be supplied for every row.
If it is unknown, leave it blank on every row. The fit is weighted by these
uncertainties when available; it reports all pair residuals and rejects no
outliers. Review correspondence identity and residuals before using a fitted
mapping in a separate, measured calibration workflow. This command itself
never changes a running sensor clock.
