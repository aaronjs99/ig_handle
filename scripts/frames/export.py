#!/usr/bin/env python3
"""Export canonical Heron sensor extrinsics for launch-time consumers."""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Mapping, Optional, Sequence

import yaml


def _repo_default_path() -> Path:
    try:
        package_dir = subprocess.check_output(
            ["rospack", "find", "ig_handle"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        package_dir = str(Path(__file__).resolve().parents[2])
    return Path(package_dir) / "config" / "sensors" / "sensor_frames.yaml"


def _load(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}
    sensors = payload.get("sensor_frames")
    if not isinstance(sensors, dict):
        raise ValueError(f"{path} does not contain sensor_frames")
    return sensors


def _fmt(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"


def _transform(sensors: Mapping[str, object], key: str) -> Mapping[str, object]:
    transforms = sensors.get("transforms", {})
    if not isinstance(transforms, dict) or key not in transforms:
        raise KeyError(f"missing transform {key!r}")
    transform = transforms[key]
    if not isinstance(transform, dict):
        raise TypeError(f"transform {key!r} is not a mapping")
    return transform


def _vec(mapping: Mapping[str, object], key: str) -> Sequence[float]:
    values = mapping.get(key)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{key} must be a 3-vector")
    return [float(value) for value in values]


def _urdf_body(sensors: Mapping[str, object], key: str) -> Mapping[str, object]:
    reference = sensors.get("geometry_reference", {})
    body_transforms = reference.get("urdf_body_transforms", {})
    if not isinstance(body_transforms, dict) or key not in body_transforms:
        raise KeyError(f"missing urdf body transform {key!r}")
    transform = body_transforms[key]
    if not isinstance(transform, dict):
        raise TypeError(f"urdf body transform {key!r} is not a mapping")
    return transform


def _emit_shell(sensors: Mapping[str, object]) -> None:
    optical_rpy = _urdf_body(sensors, "optical_joint_rpy")
    exports = {
        "HERON_IMU_XYZ": _fmt(_vec(_transform(sensors, "imu"), "translation")),
        "HERON_IMU_RPY": _fmt(_vec(_transform(sensors, "imu"), "rotation_rpy")),
        "HERON_LIDAR_H_XYZ": _fmt(_vec(_transform(sensors, "lidar_h"), "translation")),
        "HERON_LIDAR_H_RPY": _fmt(_vec(_transform(sensors, "lidar_h"), "rotation_rpy")),
        "HERON_LIDAR_H_MOUNT_XYZ": _fmt(
            _vec(_urdf_body(sensors, "lidar_h_mount"), "translation")
        ),
        "HERON_LIDAR_H_MOUNT_RPY": _fmt(
            _vec(_urdf_body(sensors, "lidar_h_mount"), "rotation_rpy")
        ),
        "HERON_LIDAR_V_XYZ": _fmt(_vec(_transform(sensors, "lidar_v"), "translation")),
        "HERON_LIDAR_V_RPY": _fmt(_vec(_transform(sensors, "lidar_v"), "rotation_rpy")),
        "HERON_LIDAR_V_MOUNT_XYZ": _fmt(
            _vec(_urdf_body(sensors, "lidar_v_mount"), "translation")
        ),
        "HERON_LIDAR_V_MOUNT_RPY": _fmt(
            _vec(_urdf_body(sensors, "lidar_v_mount"), "rotation_rpy")
        ),
        "HERON_SONAR_XYZ": _fmt(_vec(_transform(sensors, "sonar"), "translation")),
        "HERON_SONAR_RPY": _fmt(_vec(_transform(sensors, "sonar"), "rotation_rpy")),
        "HERON_CAMERA_MOUNT_XYZ": _fmt(
            _vec(_urdf_body(sensors, "camera_mount"), "translation")
        ),
        "HERON_CAMERA_MOUNT_RPY": _fmt(
            _vec(_urdf_body(sensors, "camera_mount"), "rotation_rpy")
        ),
        "HERON_CAMERA_FORWARD_OPTICAL_RPY": _fmt(_vec(optical_rpy, "forward")),
        "HERON_CAMERA_UPWARD_OPTICAL_RPY": _fmt(_vec(optical_rpy, "upward")),
    }

    for camera_id in ("F1", "F2", "F3", "F4"):
        transform = _urdf_body(sensors, camera_id)
        exports[f"HERON_CAMERA_{camera_id}_XYZ"] = _fmt(_vec(transform, "translation"))
        exports[f"HERON_CAMERA_{camera_id}_RPY"] = _fmt(_vec(transform, "rotation_rpy"))

    for key, value in exports.items():
        print(f"export {key}={_shell_quote(value)}")


def _emit_dlio_geometry(sensors: Mapping[str, object]) -> None:
    frames = sensors.get("frames", {})
    if not isinstance(frames, dict):
        raise ValueError("sensor_frames.frames must be a mapping")
    imu = _transform(sensors, "imu")
    lidar = _transform(sensors, "lidar_h")
    payload = {
        "dlio": {
            "frames": {
                "map": frames.get("map", "map"),
                "odom": frames.get("odom", "odom"),
                "baselink": frames.get("base", "base_link"),
                "lidar": frames.get("lidar_h", "lidar_h_link"),
                "imu": frames.get("imu", "imu_link"),
            },
            "extrinsics": {
                "baselink2imu": {
                    "t": _vec(imu, "translation"),
                    "R": _rpy_to_matrix(_vec(imu, "rotation_rpy")),
                },
                "baselink2lidar": {
                    "t": _vec(lidar, "translation"),
                    "R": _rpy_to_matrix(_vec(lidar, "rotation_rpy")),
                },
            },
        }
    }
    yaml.safe_dump(payload, sys.stdout, sort_keys=False)


def _rpy_to_matrix(rpy: Sequence[float]) -> list[float]:
    roll, pitch, yaw = [float(value) for value in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    ]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sensor-frames",
        dest="sensor_frames",
        type=Path,
        default=Path(os.environ.get("HERON_SENSOR_FRAMES_FILE", _repo_default_path())),
    )
    parser.add_argument(
        "--format",
        choices=("shell", "dlio-geometry"),
        default="shell",
    )
    args = parser.parse_args(argv)

    sensors = _load(args.sensor_frames)
    if args.format == "shell":
        _emit_shell(sensors)
    elif args.format == "dlio-geometry":
        _emit_dlio_geometry(sensors)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
