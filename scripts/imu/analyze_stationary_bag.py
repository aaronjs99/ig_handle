#!/usr/bin/env python3
"""Produce a provenance-bound, provisional stationary IMU characterization.

The analysis is descriptive. A stationary bag without independent attitude or
rate ground truth can estimate repeatability and noise, but it cannot establish
absolute accuracy or commission a covariance model for autonomous operation.
"""

import argparse
import csv
import hashlib
import json
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rosbag
import yaml
from sensors.imu_candidate_validation import validate_artifact


AXES = ("x", "y", "z")
DEFAULT_IMU_TOPIC = "/sensors/imu/data"
DEFAULT_MAG_TOPIC = "/sensors/imu/mag"
DEFAULT_TIME_TOPIC = "/sensors/imu/time"
EXPECTED_IMU_MESSAGE_TYPE = "sensor_msgs/Imu"
PLACEHOLDERS = {"", "unknown", "unspecified", "n/a", "na", "none"}


def quaternion_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def utc_iso(epoch_seconds):
    return (
        datetime.fromtimestamp(float(epoch_seconds), timezone.utc)
        .isoformat()
        .replace("+00:00", "Z")
    )


def connection_header_text(connection_header, key):
    value = connection_header.get(key, b"")
    return (
        value.decode("utf-8", errors="replace")
        if isinstance(value, bytes)
        else str(value)
    )


def covariance(values):
    return np.cov(values, rowvar=False, ddof=1)


def correlation(values):
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.corrcoef(values, rowvar=False)


def matrix_payload(values):
    raw_covariance = covariance(values)
    differences = np.diff(values, axis=0)
    difference_covariance = (
        0.5 * covariance(differences) if len(differences) >= 2 else None
    )
    return {
        "mean": values.mean(axis=0).tolist(),
        "covariance": raw_covariance.tolist(),
        "correlation": correlation(values).tolist(),
        "stddev": np.sqrt(np.diag(raw_covariance)).tolist(),
        "difference_covariance": (
            difference_covariance.tolist()
            if difference_covariance is not None
            else None
        ),
        "difference_stddev": (
            np.sqrt(np.diag(difference_covariance)).tolist()
            if difference_covariance is not None
            else None
        ),
    }


def line_fit(times, values):
    centered = times - times.mean()
    denominator = float(np.dot(centered, centered))
    if denominator <= 0.0:
        return np.zeros(values.shape[1])
    return np.dot(centered, values - values.mean(axis=0)) / denominator


def line_fit_payload(times, values):
    """Return per-axis linear slopes and goodness of fit."""
    slopes = line_fit(times, values)
    centered = times - times.mean()
    fitted = values.mean(axis=0) + centered[:, None] * slopes[None, :]
    residual = values - fitted
    residual_sum_squares = np.sum(residual**2, axis=0)
    total_sum_squares = np.sum((values - values.mean(axis=0)) ** 2, axis=0)
    r_squared = np.zeros_like(total_sum_squares, dtype=float)
    np.divide(
        residual_sum_squares,
        total_sum_squares,
        out=r_squared,
        where=total_sum_squares > 0.0,
    )
    r_squared = np.where(total_sum_squares > 0.0, 1.0 - r_squared, 0.0)
    return {"slope": slopes.tolist(), "r_squared": r_squared.tolist()}


def endpoint_delta(times, values, window_seconds=5.0):
    start_mask = times <= times[0] + window_seconds
    end_mask = times >= times[-1] - window_seconds
    return (values[end_mask].mean(axis=0) - values[start_mask].mean(axis=0)).tolist()


def allan_deviation(values, sample_rate_hz):
    """Compute overlapping Allan deviation of cluster-averaged samples."""
    sample_count = len(values)
    maximum_cluster = max(1, sample_count // 10)
    cluster_sizes = np.unique(
        np.maximum(
            1,
            np.round(np.logspace(0.0, math.log10(maximum_cluster), num=32)).astype(int),
        )
    )
    records = []
    for cluster_size in cluster_sizes:
        if sample_count < 2 * cluster_size + 1:
            continue
        cumulative = np.vstack(
            (np.zeros((1, values.shape[1])), np.cumsum(values, axis=0))
        )
        cluster_means = (
            cumulative[cluster_size:] - cumulative[:-cluster_size]
        ) / cluster_size
        separated_differences = (
            cluster_means[cluster_size:] - cluster_means[:-cluster_size]
        )
        deviation = np.sqrt(0.5 * np.mean(separated_differences**2, axis=0))
        records.append(
            {
                "tau_seconds": float(cluster_size / sample_rate_hz),
                "cluster_size": int(cluster_size),
                "overlapping_pair_count": int(len(separated_differences)),
                "disjoint_pair_count": int(sample_count // (2 * cluster_size)),
                "deviation": deviation.tolist(),
            }
        )
    return records


def allan_summary(records):
    if not records:
        return {}
    taus = np.asarray([record["tau_seconds"] for record in records], dtype=float)
    deviations = np.asarray([record["deviation"] for record in records], dtype=float)
    fit_mask = (taus >= 0.03) & (taus <= min(1.0, taus[-1]))
    if np.count_nonzero(fit_mask) < 2:
        fit_mask = np.arange(len(taus)) < min(5, len(taus))
    fit_records = []
    for axis_index, axis in enumerate(AXES):
        axis_fit_mask = (
            fit_mask
            & np.isfinite(deviations[:, axis_index])
            & (deviations[:, axis_index] > 0.0)
        )
        if np.count_nonzero(axis_fit_mask) < 2:
            fit_records.append(
                {
                    "axis": axis,
                    "available": False,
                    "reason": "fewer_than_two_positive_finite_points",
                }
            )
            continue
        log_tau = np.log10(taus[axis_fit_mask])
        log_deviation = np.log10(deviations[axis_fit_mask, axis_index])
        slope, intercept = np.polyfit(log_tau, log_deviation, 1)
        predicted = slope * log_tau + intercept
        residual_sum_squares = float(np.sum((log_deviation - predicted) ** 2))
        total_sum_squares = float(np.sum((log_deviation - log_deviation.mean()) ** 2))
        fit_records.append(
            {
                "axis": axis,
                "available": True,
                "slope": float(slope),
                "coefficient_at_one_second": float(10.0**intercept),
                "r_squared": (
                    float(1.0 - residual_sum_squares / total_sum_squares)
                    if total_sum_squares > 0.0
                    else 0.0
                ),
                "fit_tau_seconds": [
                    float(taus[axis_fit_mask][0]),
                    float(taus[axis_fit_mask][-1]),
                ],
            }
        )
    minima = np.argmin(deviations, axis=0)
    bias_instability = []
    for axis_index, minimum_index in enumerate(minima):
        bias_instability.append(
            {
                "value": float(deviations[minimum_index, axis_index] / 0.664),
                "tau_seconds": float(taus[minimum_index]),
                "boundary_limited": bool(
                    minimum_index == 0 or minimum_index == len(records) - 1
                ),
            }
        )
    return {
        "short_noise_log_log_fit": fit_records,
        "bias_instability_heuristic": bias_instability,
    }


def welch_psd(times, values, sample_rate_hz):
    """Welch PSD after uniform interpolation and per-segment linear detrending."""
    sample_count = len(values)
    maximum_segment = min(sample_count, 4096)
    segment_size = 2 ** int(math.floor(math.log(maximum_segment, 2)))
    if segment_size < 64:
        return None, None
    step = segment_size // 2
    window = np.hanning(segment_size)
    scale = sample_rate_hz * float(np.sum(window**2))
    spectra = []
    uniform_times = np.arange(sample_count, dtype=float) / sample_rate_hz
    uniform_values = np.column_stack(
        [np.interp(uniform_times, times, values[:, axis]) for axis in range(3)]
    )
    sample_axis = np.arange(segment_size, dtype=float)
    sample_axis_centered = sample_axis - sample_axis.mean()
    sample_axis_energy = float(np.dot(sample_axis_centered, sample_axis_centered))
    for start in range(0, sample_count - segment_size + 1, step):
        segment = uniform_values[start : start + segment_size]
        segment_centered = segment - segment.mean(axis=0)
        slopes = np.dot(sample_axis_centered, segment_centered) / sample_axis_energy
        segment = segment_centered - sample_axis_centered[:, None] * slopes[None, :]
        transform = np.fft.rfft(segment * window[:, None], axis=0)
        power = np.abs(transform) ** 2 / scale
        if segment_size % 2 == 0:
            power[1:-1] *= 2.0
        else:
            power[1:] *= 2.0
        spectra.append(power)
    if not spectra:
        return None, None
    return (
        np.fft.rfftfreq(segment_size, d=1.0 / sample_rate_hz),
        np.mean(spectra, axis=0),
        len(spectra),
    )


def spectral_summary(times, values, sample_rate_hz):
    result = welch_psd(times, values, sample_rate_hz)
    frequencies, power, segment_count = (
        result if result[0] is not None else (None, None, 0)
    )
    if frequencies is None:
        return {"available": False, "reason": "insufficient_samples"}
    asd = np.sqrt(power)
    upper_floor_hz = min(40.0, sample_rate_hz * 0.45)
    floor_mask = (frequencies >= 5.0) & (frequencies <= upper_floor_hz)
    bands = ((0.1, 1.0), (1.0, 10.0), (10.0, 40.0))
    band_records = []
    for low_hz, requested_high_hz in bands:
        high_hz = min(requested_high_hz, frequencies[-1])
        mask = (frequencies >= low_hz) & (frequencies <= high_hz)
        if np.count_nonzero(mask) < 2:
            continue
        variance = np.trapz(power[mask], frequencies[mask], axis=0)
        band_records.append(
            {
                "low_hz": low_hz,
                "high_hz": float(high_hz),
                "rms": np.sqrt(np.maximum(variance, 0.0)).tolist(),
            }
        )

    peaks = []
    candidate_mask = (frequencies >= 0.2) & (
        frequencies <= min(45.0, sample_rate_hz * 0.45)
    )
    candidate_indices = np.flatnonzero(candidate_mask)
    for axis_index, axis in enumerate(AXES):
        axis_power = power[:, axis_index]
        local = [
            index
            for index in candidate_indices
            if 0 < index < len(axis_power) - 1
            and axis_power[index] > axis_power[index - 1]
            and axis_power[index] >= axis_power[index + 1]
        ]
        baseline = float(np.median(axis_power[candidate_mask]))
        ranked = sorted(local, key=lambda index: axis_power[index], reverse=True)
        accepted = []
        for index in ranked:
            ratio = float(axis_power[index] / baseline) if baseline > 0.0 else None
            if ratio is not None and ratio < 5.0:
                continue
            if any(abs(frequencies[index] - prior) < 0.25 for prior in accepted):
                continue
            accepted.append(float(frequencies[index]))
            peaks.append(
                {
                    "axis": axis,
                    "frequency_hz": float(frequencies[index]),
                    "asd": float(asd[index, axis_index]),
                    "power_to_median_ratio": ratio,
                }
            )
            if len(accepted) == 5:
                break
    return {
        "available": True,
        "segment_samples": int((len(frequencies) - 1) * 2),
        "segment_count": int(segment_count),
        "resolution_hz": float(frequencies[1] - frequencies[0]),
        "noise_floor_band_hz": [5.0, float(upper_floor_hz)],
        "median_asd_noise_floor": (
            np.median(asd[floor_mask], axis=0).tolist()
            if np.any(floor_mask)
            else [None, None, None]
        ),
        "band_rms": band_records,
        "dominant_local_peaks": peaks,
        "curve": {
            "frequency_hz": frequencies.tolist(),
            "asd": asd.tolist(),
        },
    }


def covariance_is_zero(messages, field):
    return all(
        all(abs(float(value)) <= 0.0 for value in getattr(message, field))
        for message in messages
    )


def sample_std(values, axis=0):
    if len(values) < 2:
        shape = np.asarray(values).shape[1:]
        return np.full(shape, np.nan)
    return np.std(values, axis=axis, ddof=1)


def one_second_windows(times, gyro, accel, rpy, magnetometer):
    start = math.floor(float(times[0]))
    stop = math.ceil(float(times[-1]))
    rows = []
    for window_start in np.arange(start, stop, 1.0):
        mask = (times >= window_start) & (times < window_start + 1.0)
        if not np.any(mask):
            continue
        row = {
            "time_seconds": float((window_start + 0.5) - times[0]),
            "sample_count": int(np.count_nonzero(mask)),
        }
        for prefix, block in (("gyro", gyro), ("accel", accel), ("rpy", rpy)):
            means = block[mask].mean(axis=0)
            deviations = sample_std(block[mask])
            for axis_index, axis in enumerate(AXES):
                row[f"{prefix}_{axis}_mean"] = float(means[axis_index])
                row[f"{prefix}_{axis}_stddev"] = float(deviations[axis_index])
        if magnetometer is not None:
            nearest = magnetometer[
                (magnetometer[:, 0] >= window_start)
                & (magnetometer[:, 0] < window_start + 1.0)
            ]
            if len(nearest):
                mag_means = nearest[:, 1:4].mean(axis=0)
                mag_deviations = sample_std(nearest[:, 1:4])
                for axis_index, axis in enumerate(AXES):
                    row[f"mag_{axis}_mean"] = float(mag_means[axis_index])
                    row[f"mag_{axis}_stddev"] = float(mag_deviations[axis_index])
        rows.append(row)
    return rows


def maximum_one_second_mean_norm(rows, prefix):
    norms = []
    for row in rows:
        keys = [f"{prefix}_{axis}_mean" for axis in AXES]
        if all(key in row for key in keys):
            norms.append(math.sqrt(sum(float(row[key]) ** 2 for key in keys)))
    return float(max(norms)) if norms else None


def maximum_one_second_norm_departure(rows, prefix, reference):
    norms = []
    for row in rows:
        keys = [f"{prefix}_{axis}_mean" for axis in AXES]
        if all(key in row for key in keys):
            norms.append(math.sqrt(sum(float(row[key]) ** 2 for key in keys)))
    return float(max(abs(value - reference) for value in norms)) if norms else None


def write_csv(path, rows):
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(
            {
                key: (
                    None
                    if isinstance(value, (float, np.floating))
                    and not math.isfinite(float(value))
                    else value
                )
                for key, value in row.items()
            }
            for row in rows
        )


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return None
    return value


def allan_rows(signal, records):
    rows = []
    for record in records:
        for axis_index, axis in enumerate(AXES):
            rows.append(
                {
                    "signal": signal,
                    "axis": axis,
                    "tau_seconds": record["tau_seconds"],
                    "cluster_size": record["cluster_size"],
                    "overlapping_pair_count": record["overlapping_pair_count"],
                    "disjoint_pair_count": record["disjoint_pair_count"],
                    "deviation": record["deviation"][axis_index],
                }
            )
    return rows


def spectral_rows(signal, spectral):
    if not spectral.get("available"):
        return []
    frequencies = spectral["curve"]["frequency_hz"]
    asd = spectral["curve"]["asd"]
    return [
        {
            "signal": signal,
            "axis": axis,
            "frequency_hz": frequency,
            "asd": asd[index][axis_index],
        }
        for index, frequency in enumerate(frequencies)
        for axis_index, axis in enumerate(AXES)
    ]


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--mag-topic", default=DEFAULT_MAG_TOPIC)
    parser.add_argument("--time-topic", default=DEFAULT_TIME_TOPIC)
    parser.add_argument("--device-id", default="")
    parser.add_argument("--expected-frame", default="imu_link")
    parser.add_argument("--expected-caller-id", default="/sensors/imu/driver")
    parser.add_argument("--source-revision", default="")
    parser.add_argument("--declared-stationary", action="store_true")
    parser.add_argument("--mounting-state", default="unspecified")
    parser.add_argument("--location", default="unspecified")
    parser.add_argument("--operator-notes", default="")
    return parser.parse_args()


def main():
    args = parse_arguments()
    bag_path = args.bag.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    device_id = args.device_id.strip()
    source_revision = args.source_revision.strip().lower()
    mounting_state = args.mounting_state.strip()
    location = args.location.strip()
    if not bag_path.is_file():
        raise FileNotFoundError(f"source bag does not exist: {bag_path}")
    if output_dir.exists():
        raise FileExistsError(f"output directory already exists: {output_dir}")
    bag_stat_before = bag_path.stat()
    bag_sha256_before = sha256_file(bag_path)
    stem = bag_path.stem
    analysis_path = output_dir / f"{stem}_analysis.json"
    timeline_path = output_dir / f"{stem}_one_second.csv"
    allan_path = output_dir / f"{stem}_allan_deviation.csv"
    spectral_path = output_dir / f"{stem}_power_spectral_density.csv"
    candidate_path = output_dir / f"{stem}_stationary_candidate.yaml"
    manifest_path = output_dir / f"{stem}_manifest.json"

    imu_messages = []
    magnetometer_rows = []
    time_reference_rows = []
    connection_contract = {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, receipt_time, connection_header in bag.read_messages(
            topics=[args.imu_topic, args.mag_topic, args.time_topic],
            return_connection_header=True,
        ):
            contract = connection_contract.setdefault(
                topic, {"caller_ids": set(), "message_types": set()}
            )
            contract["caller_ids"].add(
                connection_header_text(connection_header, "callerid")
            )
            contract["message_types"].add(
                connection_header_text(connection_header, "type")
            )
            if topic == args.imu_topic:
                raw_header_stamp = message.header.stamp.to_sec()
                used_receipt_fallback = raw_header_stamp <= 0.0
                stamp = (
                    receipt_time.to_sec() if used_receipt_fallback else raw_header_stamp
                )
                rpy = quaternion_to_rpy(
                    message.orientation.x,
                    message.orientation.y,
                    message.orientation.z,
                    message.orientation.w,
                )
                imu_messages.append(
                    (
                        message,
                        receipt_time.to_sec(),
                        stamp,
                        used_receipt_fallback,
                        (
                            message.angular_velocity.x,
                            message.angular_velocity.y,
                            message.angular_velocity.z,
                        ),
                        (
                            message.linear_acceleration.x,
                            message.linear_acceleration.y,
                            message.linear_acceleration.z,
                        ),
                        rpy,
                    )
                )
            elif topic == args.mag_topic:
                stamp = message.header.stamp.to_sec() or receipt_time.to_sec()
                magnetic_vector = getattr(message, "magnetic_field", None)
                if magnetic_vector is None:
                    magnetic_vector = getattr(message, "vector", None)
                if magnetic_vector is None:
                    raise RuntimeError(
                        f"{args.mag_topic} message lacks magnetic_field or vector"
                    )
                magnetometer_rows.append(
                    (
                        stamp,
                        magnetic_vector.x,
                        magnetic_vector.y,
                        magnetic_vector.z,
                    )
                )
            elif topic == args.time_topic:
                header_stamp = message.header.stamp.to_sec() or receipt_time.to_sec()
                time_reference_rows.append(
                    (
                        header_stamp,
                        message.time_ref.to_sec(),
                        receipt_time.to_sec(),
                        str(message.source),
                    )
                )

    if len(imu_messages) < 3:
        raise RuntimeError(f"{args.imu_topic} contains fewer than three messages")

    messages = [row[0] for row in imu_messages]
    receipt_times = np.asarray([row[1] for row in imu_messages], dtype=float)
    header_times = np.asarray([row[2] for row in imu_messages], dtype=float)
    header_stamp_fallbacks = np.asarray([row[3] for row in imu_messages], dtype=bool)
    elapsed = header_times - header_times[0]
    gyro = np.asarray([row[4] for row in imu_messages], dtype=float)
    accel = np.asarray([row[5] for row in imu_messages], dtype=float)
    rpy = np.unwrap(np.asarray([row[6] for row in imu_messages], dtype=float), axis=0)
    frame_ids = sorted({str(message.header.frame_id) for message in messages})
    imu_caller_ids = sorted(connection_contract[args.imu_topic]["caller_ids"])
    imu_message_types = sorted(connection_contract[args.imu_topic]["message_types"])
    sequences = np.asarray(
        [int(message.header.seq) for message in messages], dtype=np.int64
    )
    quaternion_norms = np.asarray(
        [
            math.sqrt(
                message.orientation.x**2
                + message.orientation.y**2
                + message.orientation.z**2
                + message.orientation.w**2
            )
            for message in messages
        ],
        dtype=float,
    )

    if not (
        np.all(np.isfinite(receipt_times))
        and np.all(np.isfinite(header_times))
        and np.all(np.isfinite(gyro))
        and np.all(np.isfinite(accel))
        and np.all(np.isfinite(rpy))
        and np.all(np.isfinite(quaternion_norms))
    ):
        raise RuntimeError("IMU stream contains nonfinite timestamps or measurements")

    delta_time = np.diff(header_times)
    if not np.all(delta_time > 0.0):
        raise RuntimeError("IMU header timestamps are not strictly increasing")
    positive_delta_time = delta_time[delta_time > 0.0]
    duration = float(elapsed[-1])
    if duration <= 0.0:
        raise RuntimeError("IMU capture duration must be positive")
    effective_rate = float((len(messages) - 1) / duration)
    expected_period = float(duration / (len(messages) - 1))
    sequence_differences = np.diff(sequences)
    gap_threshold = expected_period * 1.5
    ideal_header_times = header_times[0] + np.arange(len(messages)) * expected_period
    timestamp_phase_residual = header_times - ideal_header_times

    gyro_payload = matrix_payload(gyro)
    accel_payload = matrix_payload(accel)
    rpy_payload = matrix_payload(rpy)
    gyro_allan = allan_deviation(gyro, effective_rate)
    accel_allan = allan_deviation(accel, effective_rate)
    rpy_degrees = np.degrees(rpy)

    accel_mean = np.asarray(accel_payload["mean"], dtype=float)
    accel_norm = np.linalg.norm(accel, axis=1)
    mean_accel_norm = float(np.linalg.norm(accel_mean))
    gravity_direction = accel_mean / mean_accel_norm
    inclination = math.degrees(
        math.acos(float(np.clip(-gravity_direction[2], -1.0, 1.0)))
    )

    magnetometer = (
        np.asarray(magnetometer_rows, dtype=float) if magnetometer_rows else None
    )
    magnetometer_elapsed = None
    magnetometer_payload = {"available": False}
    if magnetometer is not None and len(magnetometer) >= 2:
        if not np.all(np.isfinite(magnetometer)):
            raise RuntimeError("magnetometer stream contains nonfinite values")
        magnetometer_elapsed = magnetometer.copy()
        magnetometer_elapsed[:, 0] -= header_times[0]
        mag_values = magnetometer[:, 1:4]
        mag_norm = np.linalg.norm(mag_values, axis=1)
        mag_intervals = np.diff(magnetometer[:, 0])
        mag_duration = float(magnetometer[-1, 0] - magnetometer[0, 0])
        mag_timing_valid = bool(mag_duration > 0.0 and np.all(mag_intervals > 0.0))
        mag_rate = (
            float((len(magnetometer) - 1) / mag_duration)
            if mag_duration > 0.0
            else None
        )
        magnetometer_payload = {
            "available": True,
            "samples": int(len(magnetometer)),
            "effective_rate_hz": mag_rate,
            "timestamps_strictly_increasing": mag_timing_valid,
            "nonpositive_intervals": int(np.count_nonzero(mag_intervals <= 0.0)),
            **matrix_payload(mag_values),
            "norm_mean": float(mag_norm.mean()),
            "norm_stddev": float(mag_norm.std(ddof=1)),
            "spectral": (
                spectral_summary(magnetometer_elapsed[:, 0], mag_values, mag_rate)
                if mag_timing_valid
                else {
                    "available": False,
                    "reason": "magnetometer_timestamps_not_strictly_increasing",
                }
            ),
        }

    time_reference_payload = {"available": False}
    if len(time_reference_rows) >= 2:
        time_reference = np.asarray(
            [row[:3] for row in time_reference_rows], dtype=float
        )
        if not np.all(np.isfinite(time_reference)):
            raise RuntimeError("time-reference stream contains nonfinite values")
        time_reference_sources = sorted({row[3] for row in time_reference_rows})
        reference_offset = time_reference[:, 1] - time_reference[:, 0]
        receipt_offset = time_reference[:, 2] - time_reference[:, 0]
        reference_intervals = np.diff(time_reference[:, 1])
        header_intervals = np.diff(time_reference[:, 0])
        interval_residual = reference_intervals - header_intervals
        reference_duration = float(time_reference[-1, 1] - time_reference[0, 1])
        time_reference_payload = {
            "available": True,
            "samples": int(len(time_reference)),
            "sources": time_reference_sources,
            "header_time_first_seconds": float(time_reference[0, 0]),
            "header_time_last_seconds": float(time_reference[-1, 0]),
            "reference_time_first_seconds": float(time_reference[0, 1]),
            "reference_time_last_seconds": float(time_reference[-1, 1]),
            "reference_duration_seconds": reference_duration,
            "reference_effective_rate_hz": (
                float((len(time_reference) - 1) / reference_duration)
                if reference_duration > 0.0
                else None
            ),
            "reference_nonpositive_intervals": int(
                np.count_nonzero(reference_intervals <= 0.0)
            ),
            "reference_interval_seconds": {
                "mean": float(reference_intervals.mean()),
                "stddev": (
                    float(reference_intervals.std(ddof=1))
                    if len(reference_intervals) >= 2
                    else None
                ),
                "minimum": float(reference_intervals.min()),
                "maximum": float(reference_intervals.max()),
            },
            "reference_interval_minus_header_interval_seconds": {
                "mean": float(interval_residual.mean()),
                "stddev": (
                    float(interval_residual.std(ddof=1))
                    if len(interval_residual) >= 2
                    else None
                ),
                "minimum": float(interval_residual.min()),
                "maximum": float(interval_residual.max()),
            },
            "reference_minus_header_seconds": {
                "mean": float(reference_offset.mean()),
                "stddev": float(reference_offset.std(ddof=1)),
                "minimum": float(reference_offset.min()),
                "maximum": float(reference_offset.max()),
            },
            "receipt_minus_header_seconds": {
                "mean": float(receipt_offset.mean()),
                "stddev": float(receipt_offset.std(ddof=1)),
                "minimum": float(receipt_offset.min()),
                "maximum": float(receipt_offset.max()),
            },
        }

    windows = one_second_windows(
        elapsed, gyro, accel, rpy_degrees, magnetometer_elapsed
    )
    derived_covariances = [
        np.asarray(gyro_payload["covariance"], dtype=float),
        np.asarray(gyro_payload["difference_covariance"], dtype=float),
        np.asarray(accel_payload["covariance"], dtype=float),
        np.asarray(accel_payload["difference_covariance"], dtype=float),
    ]
    derived_covariance_finite = bool(
        all(np.all(np.isfinite(matrix)) for matrix in derived_covariances)
    )
    first_difference_proxy_positive_definite = False
    if derived_covariance_finite:
        first_difference_proxy_positive_definite = bool(
            np.min(np.linalg.eigvalsh(derived_covariances[1])) > 1e-15
            and np.min(np.linalg.eigvalsh(derived_covariances[3])) > 1e-15
        )
    candidate_checks = {
        "declared_stationary": bool(args.declared_stationary),
        "minimum_duration_60_seconds": duration >= 60.0,
        "minimum_sample_count_6000": len(messages) >= 6000,
        "effective_rate_80_to_120_hz": 80.0 <= effective_rate <= 120.0,
        "device_id_recorded": device_id.lower() not in PLACEHOLDERS,
        "source_revision_is_full_git_sha": bool(
            re.fullmatch(r"[0-9a-f]{40}", source_revision)
        ),
        "mounting_state_recorded": mounting_state.lower() not in PLACEHOLDERS,
        "location_recorded": location.lower() not in PLACEHOLDERS,
        "frame_matches": frame_ids == [args.expected_frame],
        "publisher_identity_matches": imu_caller_ids == [args.expected_caller_id],
        "message_type_matches": imu_message_types == [EXPECTED_IMU_MESSAGE_TYPE],
        "sequence_continuous": bool(np.all(sequence_differences == 1)),
        "timestamps_strictly_increasing": True,
        "maximum_interval_below_1p5_expected": bool(
            np.max(delta_time) <= gap_threshold
        ),
        "imu_header_stamps_nonzero": not bool(np.any(header_stamp_fallbacks)),
        "measurements_finite": True,
        "quaternions_normalized": bool(np.max(np.abs(quaternion_norms - 1.0)) <= 1e-6),
        "derived_covariance_finite": derived_covariance_finite,
        "first_difference_noise_proxy_positive_definite": (
            first_difference_proxy_positive_definite
        ),
    }
    candidate_eligible = all(candidate_checks.values())

    analysis = {
        "schema_version": "1.0",
        "review_status": "provisional",
        "candidate_gate": {
            "eligible": candidate_eligible,
            "checks": candidate_checks,
        },
        "provenance": {
            "bag_path": str(bag_path),
            "bag_sha256": bag_sha256_before,
            "bag_size_bytes": bag_stat_before.st_size,
            "bag_start_utc": utc_iso(bag_start),
            "bag_end_utc": utc_iso(bag_end),
            "source_revision": source_revision or None,
            "device_id": device_id or None,
            "device_identity_basis": "operator_asserted_cli_not_observable_in_bag",
            "imu_topic": args.imu_topic,
            "mag_topic": args.mag_topic,
            "time_topic": args.time_topic,
        },
        "sampling": {
            "samples": int(len(messages)),
            "duration_seconds": duration,
            "effective_rate_hz": effective_rate,
            "sequence_first": int(sequences[0]),
            "sequence_last": int(sequences[-1]),
            "period_seconds": {
                "mean": float(delta_time.mean()),
                "median": float(np.median(positive_delta_time)),
                "stddev": float(delta_time.std(ddof=1)),
                "minimum": float(delta_time.min()),
                "p01": float(np.percentile(delta_time, 1)),
                "p99": float(np.percentile(delta_time, 99)),
                "maximum": float(delta_time.max()),
            },
            "intervals_below_0p8_expected": int(
                np.count_nonzero(delta_time < expected_period * 0.8)
            ),
            "intervals_above_1p2_expected": int(
                np.count_nonzero(delta_time > expected_period * 1.2)
            ),
            "nonpositive_intervals": int(np.count_nonzero(delta_time <= 0.0)),
            "intervals_over_1p5_expected": int(
                np.count_nonzero(delta_time > gap_threshold)
            ),
            "sequence_discontinuities": int(
                np.count_nonzero(sequence_differences != 1)
            ),
            "missing_sequence_estimate": int(
                np.sum(np.maximum(sequence_differences - 1, 0))
            ),
            "header_stamp_receipt_fallback_count": int(
                np.count_nonzero(header_stamp_fallbacks)
            ),
            "timestamp_phase_residual_seconds": {
                "mean": float(timestamp_phase_residual.mean()),
                "stddev": float(timestamp_phase_residual.std(ddof=1)),
                "minimum": float(timestamp_phase_residual.min()),
                "maximum": float(timestamp_phase_residual.max()),
            },
            "receipt_minus_header_seconds": {
                "mean": float((receipt_times - header_times).mean()),
                "stddev": float((receipt_times - header_times).std(ddof=1)),
                "minimum": float((receipt_times - header_times).min()),
                "maximum": float((receipt_times - header_times).max()),
            },
        },
        "message_contract": {
            "frame_ids": frame_ids,
            "expected_frame": args.expected_frame,
            "frame_matches": frame_ids == [args.expected_frame],
            "caller_ids": imu_caller_ids,
            "expected_caller_id": args.expected_caller_id,
            "caller_matches": imu_caller_ids == [args.expected_caller_id],
            "message_types": imu_message_types,
            "expected_message_type": EXPECTED_IMU_MESSAGE_TYPE,
            "message_type_matches": imu_message_types == [EXPECTED_IMU_MESSAGE_TYPE],
            "connections": {
                topic: {
                    "caller_ids": sorted(contract["caller_ids"]),
                    "message_types": sorted(contract["message_types"]),
                }
                for topic, contract in sorted(connection_contract.items())
            },
            "quaternion_norm_error": {
                "mean": float(np.mean(np.abs(quaternion_norms - 1.0))),
                "maximum": float(np.max(np.abs(quaternion_norms - 1.0))),
            },
            "input_covariance_all_zero": {
                "orientation": covariance_is_zero(messages, "orientation_covariance"),
                "angular_velocity": covariance_is_zero(
                    messages, "angular_velocity_covariance"
                ),
                "linear_acceleration": covariance_is_zero(
                    messages, "linear_acceleration_covariance"
                ),
            },
        },
        "gyroscope": {
            **gyro_payload,
            "linear_drift": line_fit_payload(elapsed, gyro),
            "five_second_endpoint_delta": endpoint_delta(elapsed, gyro),
            "norm_percentiles": {
                "p95": float(np.percentile(np.linalg.norm(gyro, axis=1), 95)),
                "p99": float(np.percentile(np.linalg.norm(gyro, axis=1), 99)),
                "maximum": float(np.linalg.norm(gyro, axis=1).max()),
            },
            "maximum_one_second_mean_norm": maximum_one_second_mean_norm(
                windows, "gyro"
            ),
            "allan": gyro_allan,
            "allan_summary": allan_summary(gyro_allan),
            "spectral": spectral_summary(elapsed, gyro, effective_rate),
        },
        "accelerometer": {
            **accel_payload,
            "linear_drift": line_fit_payload(elapsed, accel),
            "five_second_endpoint_delta": endpoint_delta(elapsed, accel),
            "norm_mean": float(accel_norm.mean()),
            "norm_stddev": float(accel_norm.std(ddof=1)),
            "gravity_reference_mps2": 9.80665,
            "mean_norm_minus_reference_mps2": mean_accel_norm - 9.80665,
            "mean_direction_unit": gravity_direction.tolist(),
            "inclination_from_sensor_minus_z_degrees": inclination,
            "accel_derived_roll_degrees": math.degrees(
                math.atan2(accel_mean[1], accel_mean[2])
            ),
            "accel_derived_pitch_degrees": math.degrees(
                math.atan2(
                    -accel_mean[0],
                    math.sqrt(accel_mean[1] ** 2 + accel_mean[2] ** 2),
                )
            ),
            "maximum_one_second_mean_norm_departure": (
                maximum_one_second_norm_departure(
                    windows, "accel", float(accel_norm.mean())
                )
            ),
            "allan": accel_allan,
            "allan_summary": allan_summary(accel_allan),
            "spectral": spectral_summary(elapsed, accel, effective_rate),
        },
        "orientation": {
            **rpy_payload,
            "units": "radians",
            "mean_degrees": np.degrees(rpy.mean(axis=0)).tolist(),
            "stddev_degrees": np.degrees(rpy.std(axis=0, ddof=1)).tolist(),
            "range_degrees": np.ptp(rpy_degrees, axis=0).tolist(),
            "linear_drift_degrees": line_fit_payload(elapsed, rpy_degrees),
            "five_second_endpoint_delta_degrees": endpoint_delta(elapsed, rpy_degrees),
        },
        "magnetometer": magnetometer_payload,
        "time_reference": time_reference_payload,
        "limitations": [
            "Stationarity was assumed from test setup and was not independently observed.",
            "Stationary repeatability does not establish absolute orientation, rate, or acceleration accuracy.",
            "Orientation covariance requires an independent reference and is intentionally omitted from the candidate artifact.",
            "First-difference covariance is a descriptive noise proxy, not a commissioned sensor or estimator measurement covariance.",
            "The device ID was asserted by the operator at analysis time and is not independently observable from the IMU bag.",
            "LiDAR registration and vehicle-state uncertainty are not observable from this IMU-only bag.",
            "Bias-instability values are heuristic; boundary-limited Allan minima require a longer record.",
        ],
    }

    candidate = {
        "schema_version": "1.0",
        "artifact_type": "stationary_imu_characterization_candidate",
        "qualification": "candidate",
        "commissioned": False,
        "device": {
            "id": device_id or None,
            "frame_id": args.expected_frame,
            "identity_basis": "operator_asserted_cli_not_observable_in_bag",
        },
        "source": {
            "bag_sha256": analysis["provenance"]["bag_sha256"],
            "bag_size_bytes": analysis["provenance"]["bag_size_bytes"],
            "bag_start_utc": analysis["provenance"]["bag_start_utc"],
            "bag_end_utc": analysis["provenance"]["bag_end_utc"],
            "source_revision": source_revision or None,
            "imu_topic": args.imu_topic,
            "imu_caller_ids": imu_caller_ids,
            "expected_imu_caller_id": args.expected_caller_id,
            "imu_message_types": imu_message_types,
            "expected_imu_message_type": EXPECTED_IMU_MESSAGE_TYPE,
            "sample_count": len(messages),
            "duration_seconds": duration,
            "effective_rate_hz": effective_rate,
        },
        "test": {
            "declared_stationary": bool(args.declared_stationary),
            "mounting_state": mounting_state,
            "location": location,
            "operator_notes": args.operator_notes,
        },
        "capture_checks": dict(candidate_checks),
        "measurements": {
            "angular_velocity": {
                "units": "rad/s",
                "mean": gyro_payload["mean"],
                "stationary_covariance": gyro_payload["covariance"],
                "first_difference_noise_proxy_covariance": gyro_payload[
                    "difference_covariance"
                ],
                "first_difference_noise_proxy_stddev": gyro_payload[
                    "difference_stddev"
                ],
            },
            "linear_acceleration": {
                "units": "m/s^2",
                "mean": accel_payload["mean"],
                "stationary_covariance": accel_payload["covariance"],
                "first_difference_noise_proxy_covariance": accel_payload[
                    "difference_covariance"
                ],
                "first_difference_noise_proxy_stddev": accel_payload[
                    "difference_stddev"
                ],
            },
            "orientation": {"status": "unavailable_without_independent_reference"},
        },
        "limitations": analysis["limitations"],
    }
    candidate_validation_error = None
    if candidate_eligible:
        try:
            validate_artifact(candidate, device_id, args.expected_frame)
        except (KeyError, TypeError, ValueError) as error:
            candidate_eligible = False
            candidate_validation_error = str(error)
    candidate_checks["artifact_validator_passes"] = candidate_eligible
    analysis["candidate_gate"] = {
        "eligible": candidate_eligible,
        "checks": candidate_checks,
        "validation_error": candidate_validation_error,
    }
    analysis = json_safe(analysis)
    bag_stat_after = bag_path.stat()
    bag_sha256_after = sha256_file(bag_path)
    if (
        bag_stat_after.st_size != bag_stat_before.st_size
        or bag_stat_after.st_mtime_ns != bag_stat_before.st_mtime_ns
        or bag_sha256_after != bag_sha256_before
    ):
        raise RuntimeError("source bag changed while it was being analyzed")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    output_dir.mkdir(exist_ok=False)
    analysis_path.write_text(
        json.dumps(analysis, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_csv(timeline_path, windows)
    write_csv(
        allan_path,
        allan_rows("gyroscope", gyro_allan) + allan_rows("accelerometer", accel_allan),
    )
    spectral_output_rows = spectral_rows(
        "gyroscope", analysis["gyroscope"]["spectral"]
    ) + spectral_rows("accelerometer", analysis["accelerometer"]["spectral"])
    if magnetometer_payload.get("available"):
        spectral_output_rows += spectral_rows(
            "magnetometer", magnetometer_payload["spectral"]
        )
    write_csv(spectral_path, spectral_output_rows)
    if candidate_eligible:
        candidate_path.write_text(
            yaml.safe_dump(candidate, sort_keys=False), encoding="utf-8"
        )
    artifact_paths = [analysis_path, timeline_path, allan_path, spectral_path]
    if candidate_eligible:
        artifact_paths.append(candidate_path)
    manifest = {
        "schema_version": "1.0",
        "run_id": stem,
        "created_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "status": (
            "analysis_complete_candidate_only"
            if candidate_eligible
            else "analysis_complete_candidate_withheld"
        ),
        "command": [str(value) for value in sys.argv],
        "provenance": {
            "source_bag": str(bag_path),
            "source_bag_sha256": analysis["provenance"]["bag_sha256"],
            "analyzer": str(Path(__file__).resolve()),
            "analyzer_sha256": sha256_file(Path(__file__).resolve()),
            "source_revision": source_revision or None,
            "device_id": device_id or None,
            "frame_id": args.expected_frame,
            "publisher_caller_ids": imu_caller_ids,
            "expected_publisher_caller_id": args.expected_caller_id,
        },
        "artifacts": [
            {
                "path": path.name,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
            for path in artifact_paths
        ],
        "gates": {
            "stationarity_independently_verified": False,
            "publisher_identity_matches": imu_caller_ids == [args.expected_caller_id],
            "candidate_emitted": candidate_eligible,
            "orientation_reference_available": False,
            "lidar_covariance_available": False,
            "candidate_commissioned": False,
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(analysis_path)
    print(timeline_path)
    print(allan_path)
    print(spectral_path)
    if candidate_eligible:
        print(candidate_path)
    else:
        print("candidate withheld; see candidate_gate in analysis JSON")
    print(manifest_path)


if __name__ == "__main__":
    main()
