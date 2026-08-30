# Xsens IMU characterization and covariance boundary

This document defines how the IG Handle Xsens MTi-30 is characterized without
changing normal sensor bringup, DLiO output, or motion authority. It is an
experiment and review procedure, not authorization to move the platform.

## Ownership and safety boundary

- IG Handle owns raw IMU acquisition, device identity, timestamps, calibration
  evidence, and optional candidate sensor metadata.
- DLiO and MARINER own vehicle-state estimation. IMU-only statistics must not be
  copied into DLiO pose or twist covariance.
- The canonical topic remains `/sensors/imu/data`. Candidate matrices remain
  offline; no runtime covariance publisher is provided.
- All existing zero-covariance control rejection remains in force.
- No step in this document actuates the Heron, IG Handle arm, or any thruster.

## Evidence levels

| Gate | Evidence produced | What it can support | What it cannot support |
| --- | --- | --- | --- |
| Stationary capture with setup and duration recorded | Timing integrity, apparent gyro bias, short-term noise, full stationary covariance, PSD, Allan deviation, gravity consistency | Descriptive stationary-noise evidence and vibration diagnosis | Absolute accuracy, scale factors, orientation covariance, vehicle-state covariance |
| Six-position accelerometer capture | Per-axis bias, scale, and cross-axis calibration checks | Accelerometer calibration candidate | Gyro scale, yaw, DLiO covariance |
| Repeated cold starts | Bias and convergence repeatability | Startup uncertainty and warmup policy | Dynamic accuracy |
| Deliberate, reference-observed motion | Rate, attitude, axis, and timestamp checks | Dynamic IMU validation | LiDAR registration uncertainty |
| Synchronized LiDAR-IMU plus independent truth | Registration residuals, observability, trajectory error, empirical coverage | Candidate DLiO error-state/measurement model | Automatic commissioning without held-out consistency review |

## Stationary capture contract

1. Confirm the exact device identity and symlink from the sensor contract.
2. Confirm a single process owns the serial device. Never start a second Xsens
   driver alongside the persistent provider.
3. Use an explicit ROS master and record a bounded bag containing:
   `/sensors/imu/data`, `/sensors/imu/time`, and `/sensors/imu/mag` when
   available.
4. Declare the mounting state and stationarity. Do not infer independent
   stationarity from the IMU under evaluation.
5. Preserve `.active` or failed bags. Never overwrite or silently delete an
   earlier run.
6. After the bag closes, verify topic counts, advancing timestamps, maximum
   gaps, frame `imu_link`, and the source bag SHA-256 before analysis.
7. Run the analyzer into a new directory, then validate the resulting candidate
   against both the finalized source bag and generated manifest:

```bash
rosrun ig_handle analyze_stationary_bag.py INPUT.bag \
  --output-dir NEW_ANALYSIS_DIRECTORY \
  --device-id 0368319D \
  --source-revision FULL_40_CHARACTER_GIT_SHA \
  --declared-stationary \
  --mounting-state "secured on lab bench" \
  --location "Boelter Hall"

rosrun ig_handle validate_stationary_candidate.py CANDIDATE.yaml \
  --source-bag INPUT.bag \
  --manifest MANIFEST.json \
  --expected-device-id 0368319D \
  --expected-frame imu_link
```

Raw calibration bags and sibling analysis directories live outside Git under
`/home/ig-handle/bags/grande/calibration/imu/`. Keep each finalized `.bag` at
that root and place its generated bundle under `analysis/<run-id>/`. Do not copy
raw bags into a package data directory.

The analyzer refuses an existing output directory and always writes an analysis
JSON, one-second timeline CSV, Allan-deviation CSV, PSD CSV, and manifest JSON.
It writes the descriptive candidate YAML only when every capture gate passes;
otherwise the analysis and manifest record that the candidate was withheld. The
external validator recomputes the source-bag hash and every bundle-artifact
hash. The analyzer also refuses to publish a bundle if the source bag changes
during analysis. A candidate is always marked `commissioned: false`.

The device ID supplied to the analyzer is an operator assertion because a
standard `sensor_msgs/Imu` bag does not contain the USB serial. Preserve
capture-time udev evidence in future manifests when independent identity binding
is required.

Treat `/sensors/imu/time` as a separate reference clock unless its epoch is
verified independently. The analyzer reports its source string, monotonicity,
rate, and interval difference from the IMU header; it does not assume that
`time_ref` is UTC. Candidate timing gates use the `/sensors/imu/data` header
because that is the timestamp consumed by downstream estimation.

## Physical follow-up gates

These require the operator to confirm the setup before data collection:

### Six-position accelerometer sequence

- Hold each sensor axis once along and once opposite gravity.
- Keep each pose mechanically stable for at least two minutes after settling.
- Record temperature, pose order, fixture, operator, start/end times, and bag
  hash.
- Fit bias, scale, and non-orthogonality on a calibration subset; evaluate on
  separate held-out samples.

### Repeat cold starts

- Use at least five power cycles on different days.
- Record the unobserved startup interval, warmup duration, room temperature,
  initial bias, and fused-attitude convergence.
- Do not tune and evaluate on the same starts.

### Deliberate dynamic sequence

- Use a reference that is independent of the Xsens estimate.
- Include rest, single-axis rotations, combined rotations, and translation.
- Verify sign, axis mapping, latency, saturation, and timestamp alignment.
- Keep the platform non-actuating unless a separate field authorization exists.

### LiDAR-IMU and DLiO qualification

- Record raw LiDAR, raw IMU, TF, DLiO registration diagnostics, and independent
  pose truth where available.
- Estimate uncertainty with an error-state propagation and calibrated LiDAR
  observation model; do not invert the GICP Hessian and call it covariance.
- Evaluate empirical error coverage and NEES/NIS-style consistency across
  normal motion, weak geometry, IMU gaps, LiDAR dropouts, and held-out runs.
- Rank deficiency, timestamp gaps, invalid matrices, or missing calibration must
  withhold estimator commissioning and suppress any future estimator output or
  increase uncertainty conservatively.
- Promotion to canonical state requires a separate reviewed configuration
  change. Until then, zero DLiO covariance must continue to block autonomous
  feedback.

## Review checklist

- Source bag finalized and hash recorded
- Device ID, frame, topic, and source revision recorded
- Sequence continuity and timing-gap checks passed
- Covariance matrices finite, symmetric, and positive semidefinite
- Spectral and Allan estimates include duration and confidence limitations
- Candidate artifact validator passed
- Canonical topics and control gates unchanged
- Raw evidence preserved outside Git; only lightweight reviewed artifacts used
  for reports
