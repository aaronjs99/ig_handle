"""Structural and numerical validation for stationary IMU candidate artifacts."""

import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np
import yaml


SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
GIT_SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")
EXPECTED_IMU_TOPIC = "/sensors/imu/data"
EXPECTED_CALLER_ID = "/sensors/imu/driver"
EXPECTED_IMU_MESSAGE_TYPE = "sensor_msgs/Imu"
PLACEHOLDERS = {"", "unknown", "unspecified", "n/a", "na", "none"}
REQUIRED_CAPTURE_CHECKS = {
    "declared_stationary",
    "minimum_duration_60_seconds",
    "minimum_sample_count_6000",
    "effective_rate_80_to_120_hz",
    "device_id_recorded",
    "source_revision_is_full_git_sha",
    "mounting_state_recorded",
    "location_recorded",
    "frame_matches",
    "publisher_identity_matches",
    "message_type_matches",
    "sequence_continuous",
    "timestamps_strictly_increasing",
    "maximum_interval_below_1p5_expected",
    "imu_header_stamps_nonzero",
    "measurements_finite",
    "quaternions_normalized",
    "derived_covariance_finite",
    "first_difference_noise_proxy_positive_definite",
}
MEASUREMENT_CONTRACT = {
    "angular_velocity": "rad/s",
    "linear_acceleration": "m/s^2",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_vector(value, field, length=3):
    array = np.asarray(value, dtype=float)
    require(array.shape == (length,), f"{field} must contain {length} values")
    require(np.all(np.isfinite(array)), f"{field} must contain finite values")
    return array


def covariance_matrix(value, field, require_positive_definite=False):
    matrix = np.asarray(value, dtype=float)
    require(matrix.shape == (3, 3), f"{field} must be a 3 x 3 matrix")
    require(np.all(np.isfinite(matrix)), f"{field} must contain finite values")
    require(
        np.allclose(matrix, matrix.T, rtol=1e-8, atol=1e-12),
        f"{field} must be symmetric",
    )
    eigenvalues = np.linalg.eigvalsh((matrix + matrix.T) / 2.0)
    require(
        float(np.min(eigenvalues)) >= -1e-12,
        f"{field} must be positive semidefinite; eigenvalues={eigenvalues.tolist()}",
    )
    if require_positive_definite:
        require(
            bool(np.all(np.diag(matrix) > 0.0)) and float(np.min(eigenvalues)) > 1e-15,
            f"{field} must be strictly positive definite",
        )
    return matrix


def validate_artifact(artifact, expected_device_id="", expected_frame="imu_link"):
    expected_device_id = str(expected_device_id).strip()
    expected_frame = str(expected_frame).strip()
    require(isinstance(artifact, dict), "artifact root must be a mapping")
    require(artifact.get("schema_version") == "1.0", "unsupported schema_version")
    require(
        artifact.get("artifact_type") == "stationary_imu_characterization_candidate",
        "unexpected artifact_type",
    )
    require(
        artifact.get("qualification") == "candidate",
        "qualification must be candidate",
    )
    require(artifact.get("commissioned") is False, "commissioned must remain false")

    device = artifact.get("device") or {}
    require(
        expected_device_id.lower() not in PLACEHOLDERS,
        "expected_device_id must be explicit and non-placeholder",
    )
    require(expected_frame.lower() not in PLACEHOLDERS, "expected_frame is required")
    require(
        str(device.get("id", "")).strip().lower() not in PLACEHOLDERS,
        "device.id must be explicit and non-placeholder",
    )
    require(bool(device.get("frame_id")), "device.frame_id is required")
    require(
        device.get("identity_basis") == "operator_asserted_cli_not_observable_in_bag",
        "device.identity_basis must disclose operator assertion",
    )
    require(
        str(device["id"]).lower() == expected_device_id.lower(),
        "device.id does not match --expected-device-id",
    )
    require(
        device["frame_id"] == expected_frame,
        "device.frame_id does not match --expected-frame",
    )

    source = artifact.get("source") or {}
    require(
        SHA256_PATTERN.fullmatch(str(source.get("bag_sha256", ""))) is not None,
        "source.bag_sha256 must be a lowercase SHA-256 digest",
    )
    require(
        int(source.get("bag_size_bytes", 0)) > 0,
        "source.bag_size_bytes must be positive",
    )
    require(
        int(source.get("sample_count", 0)) >= 6000,
        "source.sample_count must be at least 6000",
    )
    require(
        math.isfinite(float(source.get("duration_seconds", 0.0)))
        and float(source["duration_seconds"]) >= 60.0,
        "source.duration_seconds must be at least 60 seconds",
    )
    require(
        math.isfinite(float(source.get("effective_rate_hz", 0.0)))
        and 80.0 <= float(source["effective_rate_hz"]) <= 120.0,
        "source.effective_rate_hz must be finite and between 80 and 120 Hz",
    )
    expected_rate = (int(source["sample_count"]) - 1) / float(
        source["duration_seconds"]
    )
    require(
        math.isclose(
            float(source["effective_rate_hz"]),
            expected_rate,
            rel_tol=1e-9,
            abs_tol=1e-9,
        ),
        "source.effective_rate_hz is inconsistent with sample count and duration",
    )
    require(
        GIT_SHA_PATTERN.fullmatch(str(source.get("source_revision", ""))) is not None,
        "source.source_revision must be a 40-character lowercase Git SHA",
    )
    require(
        source.get("imu_topic") == EXPECTED_IMU_TOPIC,
        f"source.imu_topic must be {EXPECTED_IMU_TOPIC}",
    )
    caller_ids = source.get("imu_caller_ids")
    require(
        caller_ids == [EXPECTED_CALLER_ID]
        and source.get("expected_imu_caller_id") == EXPECTED_CALLER_ID,
        "source IMU caller identity must exactly match the expected caller",
    )
    message_types = source.get("imu_message_types")
    require(
        message_types == [EXPECTED_IMU_MESSAGE_TYPE]
        and source.get("expected_imu_message_type") == EXPECTED_IMU_MESSAGE_TYPE,
        "source IMU message type must exactly match sensor_msgs/Imu",
    )

    test = artifact.get("test") or {}
    require(test.get("declared_stationary") is True, "test must be declared stationary")
    require(
        str(test.get("mounting_state", "")).strip().lower() not in PLACEHOLDERS,
        "test.mounting_state must be explicit and non-placeholder",
    )
    require(
        str(test.get("location", "")).strip().lower() not in PLACEHOLDERS,
        "test.location must be explicit and non-placeholder",
    )

    capture_checks = artifact.get("capture_checks") or {}
    require(
        REQUIRED_CAPTURE_CHECKS.issubset(capture_checks),
        "artifact is missing required capture checks",
    )
    require(
        all(capture_checks.get(name) is True for name in REQUIRED_CAPTURE_CHECKS),
        "every required capture check must be true",
    )

    measurements = artifact.get("measurements") or {}
    for name, units in MEASUREMENT_CONTRACT.items():
        measurement = measurements.get(name) or {}
        require(measurement.get("units") == units, f"{name}.units must be {units}")
        finite_vector(measurement.get("mean"), f"{name}.mean")
        covariance_matrix(
            measurement.get("stationary_covariance"),
            f"{name}.stationary_covariance",
        )
        noise_proxy = covariance_matrix(
            measurement.get("first_difference_noise_proxy_covariance"),
            f"{name}.first_difference_noise_proxy_covariance",
            require_positive_definite=True,
        )
        reported_proxy_stddev = finite_vector(
            measurement.get("first_difference_noise_proxy_stddev"),
            f"{name}.first_difference_noise_proxy_stddev",
        )
        require(
            np.allclose(
                reported_proxy_stddev,
                np.sqrt(np.maximum(np.diag(noise_proxy), 0.0)),
                rtol=1e-7,
                atol=1e-12,
            ),
            f"{name}.first_difference_noise_proxy_stddev does not match covariance diagonal",
        )

    orientation = measurements.get("orientation") or {}
    require(
        orientation.get("status") == "unavailable_without_independent_reference",
        "orientation must remain unavailable without an independent reference",
    )
    limitations = artifact.get("limitations")
    require(
        isinstance(limitations, list) and len(limitations) > 0,
        "at least one limitation is required",
    )
    return artifact


def validate_provenance_files(artifact, artifact_path, source_bag_path, manifest_path):
    source_bag_path = Path(source_bag_path).expanduser().resolve()
    manifest_path = Path(manifest_path).expanduser().resolve()
    source_hash = sha256_file(source_bag_path)
    require(
        source_hash == artifact["source"]["bag_sha256"],
        "source bag SHA-256 does not match artifact provenance",
    )
    require(
        source_bag_path.stat().st_size == int(artifact["source"]["bag_size_bytes"]),
        "source bag size does not match artifact provenance",
    )
    with manifest_path.open("r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    require(manifest.get("schema_version") == "1.0", "unsupported manifest schema")
    require(
        manifest.get("status") == "analysis_complete_candidate_only",
        "manifest status does not describe a completed candidate analysis",
    )
    require(
        (manifest.get("gates") or {}).get("candidate_emitted") is True,
        "manifest does not record candidate emission",
    )
    require(
        (manifest.get("provenance") or {}).get("source_bag_sha256") == source_hash,
        "manifest source bag SHA-256 does not match",
    )
    run_id = str(manifest.get("run_id", ""))
    require(
        run_id and Path(run_id).name == run_id,
        "manifest run_id must be a nonempty filename-safe value",
    )
    expected_artifact_names = {
        f"{run_id}_analysis.json",
        f"{run_id}_one_second.csv",
        f"{run_id}_allan_deviation.csv",
        f"{run_id}_power_spectral_density.csv",
        f"{run_id}_stationary_candidate.yaml",
    }
    artifact_entries_all = manifest.get("artifacts")
    require(
        isinstance(artifact_entries_all, list) and artifact_entries_all,
        "manifest artifacts must be a nonempty list",
    )
    manifest_directory = manifest_path.parent.resolve()
    resolved_entries = []
    seen_names = set()
    for entry in artifact_entries_all:
        require(isinstance(entry, dict), "each manifest artifact must be a mapping")
        relative_name = str(entry.get("path", ""))
        require(
            relative_name and Path(relative_name).name == relative_name,
            "manifest artifact paths must be sibling filenames",
        )
        require(
            relative_name not in seen_names, "manifest artifact paths must be unique"
        )
        seen_names.add(relative_name)
        resolved_path = (manifest_directory / relative_name).resolve()
        require(
            resolved_path.parent == manifest_directory,
            "manifest artifact path escapes the bundle directory",
        )
        require(
            resolved_path.is_file(), f"manifest artifact is missing: {relative_name}"
        )
        require(
            SHA256_PATTERN.fullmatch(str(entry.get("sha256", ""))) is not None,
            f"manifest artifact has an invalid SHA-256: {relative_name}",
        )
        require(
            sha256_file(resolved_path) == entry["sha256"],
            f"manifest artifact hash mismatch: {relative_name}",
        )
        require(
            resolved_path.stat().st_size == int(entry.get("size_bytes", -1)),
            f"manifest artifact size mismatch: {relative_name}",
        )
        resolved_entries.append((entry, resolved_path))

    require(
        expected_artifact_names.issubset(seen_names),
        "manifest omits one or more required candidate-bundle artifacts",
    )

    artifact_entries = [
        entry
        for entry, resolved_path in resolved_entries
        if resolved_path == artifact_path
    ]
    require(
        len(artifact_entries) == 1,
        "manifest must contain exactly one entry for the candidate artifact",
    )
    entry = artifact_entries[0]
    require(
        entry.get("sha256") == sha256_file(artifact_path),
        "candidate artifact hash mismatch",
    )


def load_and_validate(
    artifact_path,
    expected_device_id="",
    expected_frame="imu_link",
    source_bag_path=None,
    manifest_path=None,
):
    artifact_path = Path(artifact_path).expanduser().resolve()
    with artifact_path.open("r", encoding="utf-8") as handle:
        artifact = yaml.safe_load(handle)
    validate_artifact(artifact, expected_device_id, expected_frame)
    if source_bag_path is not None or manifest_path is not None:
        require(
            source_bag_path is not None and manifest_path is not None,
            "source_bag_path and manifest_path must be provided together",
        )
        validate_provenance_files(
            artifact, artifact_path, source_bag_path, manifest_path
        )
    return artifact
