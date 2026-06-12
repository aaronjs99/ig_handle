#!/usr/bin/env python3
"""Export canonical Heron sensor extrinsics for launch-time consumers."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

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


def _emit_lio_geometry(sensors: Mapping[str, object]) -> None:
    frames = sensors.get("frames", {})
    if not isinstance(frames, dict):
        raise ValueError("sensor_frames.frames must be a mapping")
    imu = _transform(sensors, "imu")
    lidar = _transform(sensors, "lidar_h")
    payload = {
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
    yaml.safe_dump(payload, sys.stdout, sort_keys=False)


def _identity_matrix() -> list[float]:
    return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def _matmul(left: Sequence[float], right: Sequence[float]) -> list[float]:
    return [
        sum(float(left[row * 3 + k]) * float(right[k * 3 + col]) for k in range(3))
        for row in range(3)
        for col in range(3)
    ]


def _matvec(matrix: Sequence[float], vector: Sequence[float]) -> list[float]:
    return [
        sum(float(matrix[row * 3 + col]) * float(vector[col]) for col in range(3))
        for row in range(3)
    ]


def _vec_add(left: Sequence[float], right: Sequence[float]) -> list[float]:
    return [float(left[i]) + float(right[i]) for i in range(3)]


def _compose_pose(
    parent_position: Sequence[float],
    parent_rotation: Sequence[float],
    translation: Sequence[float],
    rotation_rpy: Sequence[float],
) -> tuple[list[float], list[float]]:
    local_position = _matvec(parent_rotation, translation)
    return _vec_add(parent_position, local_position), _matmul(
        parent_rotation, _rpy_to_matrix(rotation_rpy)
    )


def _edge_trace(name: str, color: str, records: Sequence[Mapping[str, Any]]) -> dict:
    x_values: list[Optional[float]] = []
    y_values: list[Optional[float]] = []
    z_values: list[Optional[float]] = []
    labels: list[Optional[str]] = []
    for record in records:
        parent = record["parent_position"]
        child = record["child_position"]
        label = (
            f"{record['parent']} -> {record['child']}<br>"
            f"{record['name']}<br>{record['group']}"
        )
        x_values.extend([parent[0], child[0], None])
        y_values.extend([parent[1], child[1], None])
        z_values.extend([parent[2], child[2], None])
        labels.extend([label, label, None])
    return {
        "type": "scatter3d",
        "mode": "lines",
        "name": name,
        "x": x_values,
        "y": y_values,
        "z": z_values,
        "text": labels,
        "hoverinfo": "text",
        "line": {"color": color, "width": 5},
    }


def _marker_trace(
    name: str,
    color: str,
    frame_names: Sequence[str],
    positions: Mapping[str, Sequence[float]],
) -> dict:
    names = [frame for frame in frame_names if frame in positions]
    return {
        "type": "scatter3d",
        "mode": "markers+text",
        "name": name,
        "x": [positions[frame][0] for frame in names],
        "y": [positions[frame][1] for frame in names],
        "z": [positions[frame][2] for frame in names],
        "text": names,
        "hovertext": [
            (
                f"{frame}<br>"
                f"xyz: [{positions[frame][0]:.5f}, "
                f"{positions[frame][1]:.5f}, {positions[frame][2]:.5f}]"
            )
            for frame in names
        ],
        "hoverinfo": "text",
        "textposition": "top center",
        "marker": {"size": 5, "color": color},
    }


def _axis_trace(
    axis_name: str,
    color: str,
    axis_vector: Sequence[float],
    positions: Mapping[str, Sequence[float]],
    rotations: Mapping[str, Sequence[float]],
) -> dict:
    x_values: list[Optional[float]] = []
    y_values: list[Optional[float]] = []
    z_values: list[Optional[float]] = []
    labels: list[Optional[str]] = []
    for frame, position in positions.items():
        scale = 0.08 if frame == "base_link" else 0.045
        direction = [
            float(value) * scale for value in _matvec(rotations[frame], axis_vector)
        ]
        endpoint = _vec_add(position, direction)
        label = f"{frame} {axis_name}"
        x_values.extend([position[0], endpoint[0], None])
        y_values.extend([position[1], endpoint[1], None])
        z_values.extend([position[2], endpoint[2], None])
        labels.extend([label, label, None])
    return {
        "type": "scatter3d",
        "mode": "lines",
        "name": f"frame {axis_name.lower()} axes",
        "x": x_values,
        "y": y_values,
        "z": z_values,
        "text": labels,
        "hoverinfo": "text",
        "line": {"color": color, "width": 3},
    }


def _emit_frame_html(sensors: Mapping[str, object]) -> None:
    excluded_body_frames = {"lidar_h_mount", "lidar_v_mount", "camera_mount"}
    positions: dict[str, list[float]] = {
        "map": [0.0, 0.0, 0.0],
        "odom": [0.0, 0.0, 0.0],
        "base_link": [0.0, 0.0, 0.0],
    }
    rotations: dict[str, list[float]] = {
        "map": _identity_matrix(),
        "odom": _identity_matrix(),
        "base_link": _identity_matrix(),
    }
    edge_groups: dict[str, list[dict[str, Any]]] = {
        "navigation reference": [],
        "runtime sensor": [],
        "body reference": [],
    }

    def add_edge(name: str, group: str, transform: Mapping[str, object]) -> None:
        parent = str(transform.get("parent", "") or "").strip()
        child = str(transform.get("child", "") or "").strip()
        if not parent or not child:
            raise ValueError(f"{name} must include parent and child")
        if parent not in positions:
            positions[parent] = [0.0, 0.0, 0.0]
            rotations[parent] = _identity_matrix()
        child_position, child_rotation = _compose_pose(
            positions[parent],
            rotations[parent],
            _vec(transform, "translation"),
            _vec(transform, "rotation_rpy"),
        )
        positions[child] = child_position
        rotations[child] = child_rotation
        edge_groups[group].append(
            {
                "name": name,
                "group": group,
                "parent": parent,
                "child": child,
                "parent_position": list(positions[parent]),
                "child_position": list(child_position),
            }
        )

    add_edge(
        "map_to_odom_reference",
        "navigation reference",
        {
            "parent": "map",
            "child": "odom",
            "translation": [0.0, 0.0, 0.0],
            "rotation_rpy": [0.0, 0.0, 0.0],
        },
    )
    add_edge(
        "odom_to_base_link_reference",
        "navigation reference",
        {
            "parent": "odom",
            "child": "base_link",
            "translation": [0.0, 0.0, 0.0],
            "rotation_rpy": [0.0, 0.0, 0.0],
        },
    )

    transforms = sensors.get("transforms", {})
    if not isinstance(transforms, dict):
        raise ValueError("sensor_frames.transforms must be a mapping")
    for name, transform in transforms.items():
        if isinstance(transform, dict):
            add_edge(str(name), "runtime sensor", transform)

    body_transforms = sensors.get("geometry_reference", {}).get(
        "urdf_body_transforms", {}
    )
    if not isinstance(body_transforms, dict):
        body_transforms = {}
    for name, transform in body_transforms.items():
        if (
            name == "optical_joint_rpy"
            or name in excluded_body_frames
            or not isinstance(transform, dict)
        ):
            continue
        add_edge(str(name), "body reference", transform)

    nav_frames = ["map", "odom", "base_link"]
    sensor_frames = [
        frame
        for frame in positions
        if frame not in set(nav_frames) and frame not in excluded_body_frames
    ]

    traces = [
        _edge_trace(
            "navigation reference edges", "#636efa", edge_groups["navigation reference"]
        ),
        _edge_trace("runtime sensor edges", "#00a67d", edge_groups["runtime sensor"]),
        _edge_trace("body reference edges", "#2a3f5f", edge_groups["body reference"]),
        _marker_trace("navigation frames", "#636efa", nav_frames, positions),
        _marker_trace("sensor frames", "#00a67d", sorted(sensor_frames), positions),
        _axis_trace("X", "#ef553b", [1.0, 0.0, 0.0], positions, rotations),
        _axis_trace("Y", "#00cc96", [0.0, 1.0, 0.0], positions, rotations),
        _axis_trace("Z", "#636efa", [0.0, 0.0, 1.0], positions, rotations),
    ]
    layout = {
        "title": {"text": "Heron Sensor Frames"},
        "template": "plotly_white",
        "width": 1200,
        "height": 800,
        "margin": {"l": 0, "r": 0, "t": 50, "b": 0},
        "legend": {"orientation": "h", "y": 1.02, "x": 0},
        "scene": {
            "aspectmode": "data",
            "xaxis": {"title": "X forward (m)", "showbackground": True},
            "yaxis": {"title": "Y left / port (m)", "showbackground": True},
            "zaxis": {"title": "Z up (m)", "showbackground": True},
            "camera": {"eye": {"x": 1.55, "y": -1.85, "z": 1.05}},
        },
    }
    print(
        "\n".join(
            [
                "<html>",
                '<head><meta charset="utf-8" /></head>',
                "<body>",
                '<div id="heron-sensor-frames" style="height:800px; width:1200px;"></div>',
                '<script src="https://cdn.plot.ly/plotly-3.3.1.min.js"></script>',
                "<script>",
                "const traces = " + json.dumps(traces, sort_keys=True) + ";",
                "const layout = " + json.dumps(layout, sort_keys=True) + ";",
                "Plotly.newPlot('heron-sensor-frames', traces, layout, {responsive: true, displaylogo: false});",
                "</script>",
                "</body>",
                "</html>",
            ]
        )
    )


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
        choices=("shell", "lio-geometry", "html"),
        default="shell",
    )
    args = parser.parse_args(argv)

    sensors = _load(args.sensor_frames)
    if args.format == "shell":
        _emit_shell(sensors)
    elif args.format == "lio-geometry":
        _emit_lio_geometry(sensors)
    elif args.format == "html":
        _emit_frame_html(sensors)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
