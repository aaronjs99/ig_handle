#!/usr/bin/env python3
"""Apply a vetted LiDAR-pair calibration report to IG Handle extrinsics config."""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from scipy.spatial.transform import Rotation


TRANSFORM_KEYS = ("initial_target_from_source", "refined_target_from_source")


@dataclass(frozen=True)
class CalibrationReport:
    path: Path
    data: dict[str, Any]

    @classmethod
    def load(cls, path: Path) -> "CalibrationReport":
        data = yaml.safe_load(path.expanduser().read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"{path} does not contain a YAML mapping")
        return cls(path=path.expanduser(), data=data)

    @property
    def apply_recommended(self) -> bool:
        return bool(self.data.get("quality_gate", {}).get("apply_recommended", False))

    @property
    def reject_reasons(self) -> list[str]:
        return [
            str(reason)
            for reason in self.data.get("quality_gate", {}).get("reject_reasons", [])
        ]

    def transform(self, key: str) -> dict[str, Any]:
        if key not in TRANSFORM_KEYS:
            raise ValueError(f"unknown transform key {key!r}")
        transform = self.data.get(key)
        if not isinstance(transform, dict):
            raise ValueError(f"report is missing {key}")
        for required in (
            "translation_xyz_m",
            "rotation_quat_xyzw",
            "rotation_rpy_rad",
            "matrix_row_major",
        ):
            if required not in transform:
                raise ValueError(f"{key} is missing {required}")
        return transform

    def correction_metrics(self) -> dict[str, float]:
        correction = self.data.get("correction_target_frame", {})
        matrix = correction.get("matrix_row_major")
        if not isinstance(matrix, list):
            return {
                "translation_correction_norm_m": float("nan"),
                "rotation_correction_angle_deg": float("nan"),
            }
        translation = correction.get("translation_xyz_m", [float("nan")] * 3)
        rotation = Rotation.from_matrix(
            [[float(v) for v in row[:3]] for row in matrix[:3]]
        )
        return {
            "translation_correction_norm_m": math.sqrt(
                sum(float(v) * float(v) for v in translation[:3])
            ),
            "rotation_correction_angle_deg": float(
                rotation.magnitude() * 180.0 / math.pi
            ),
        }

    def metadata(self, transform_key: str) -> dict[str, Any]:
        return {
            "report_path": str(self.path),
            "bag_path": str(self.data.get("bag", "")),
            "applied_at_utc": dt.datetime.now(dt.timezone.utc)
            .replace(microsecond=0)
            .isoformat()
            .replace("+00:00", "Z"),
            "transform_key": transform_key,
            "quality_gate": copy.deepcopy(self.data.get("quality_gate", {})),
            "icp": copy.deepcopy(self.data.get("icp", {})),
            "pairs_used": self.data.get("pairs_used"),
        }


class LidarPairExtrinsicsStore:
    def __init__(self, path: Path) -> None:
        self.path = path.expanduser()

    def load(self) -> dict[str, Any]:
        if not self.path.exists():
            return {"schema_version": 1, "lidar_pair": {"history": []}}
        data = yaml.safe_load(self.path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError(f"{self.path} does not contain a YAML mapping")
        return data

    def build_updated_config(
        self,
        report: CalibrationReport,
        transform_key: str,
    ) -> dict[str, Any]:
        config = self.load()
        pair = config.setdefault("lidar_pair", {})
        previous = copy.deepcopy(pair.get("last_calibration"))
        history = pair.setdefault("history", [])
        if previous:
            history.append(previous)

        transform = report.transform(transform_key)
        previous_active = copy.deepcopy(pair.get("active_transform"))
        pair["active_transform"] = {
            "parent": str(report.data.get("target_frame", "lidar_h_link")),
            "child": str(report.data.get("source_frame", "lidar_v_link")),
            "expected_mount_relation": str(
                previous_active.get(
                    "expected_mount_relation", "approximately_90_deg_pitch"
                )
                if isinstance(previous_active, dict)
                else "approximately_90_deg_pitch"
            ),
            "translation_xyz_m": [float(v) for v in transform["translation_xyz_m"]],
            "rotation_quat_xyzw": [float(v) for v in transform["rotation_quat_xyzw"]],
            "rotation_rpy_rad": [float(v) for v in transform["rotation_rpy_rad"]],
            "matrix_row_major": [
                [float(v) for v in row] for row in transform["matrix_row_major"]
            ],
        }
        pair["last_calibration"] = report.metadata(transform_key)
        return config

    def write(self, config: dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = yaml.safe_dump(config, sort_keys=False)
        self.path.write_text(payload, encoding="utf-8")


def default_config_path() -> Path:
    try:
        import rospkg

        return (
            Path(rospkg.RosPack().get_path("ig_handle"))
            / "config"
            / ("lidar_pair_extrinsics.yaml")
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[2]
            / "config"
            / "lidar_pair_extrinsics.yaml"
        )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument(
        "--transform-key",
        choices=TRANSFORM_KEYS,
        default="refined_target_from_source",
        help="Report transform to apply when the quality gate passes.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Apply even when report.quality_gate.apply_recommended is false.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print the resulting config without writing it.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(sys.argv[1:] if argv is None else argv))
    report = CalibrationReport.load(args.report)
    if not report.apply_recommended and not args.force:
        reasons = ", ".join(report.reject_reasons) or "quality gate rejected report"
        metrics = report.correction_metrics()
        print(
            "Refusing to update LiDAR extrinsics because calibration quality is not acceptable: "
            + reasons,
            file=sys.stderr,
        )
        print(
            "The current seed already encodes the expected approximately 90 degree pitch between "
            "the horizontal and vertical lidars; this report moved too far from that seed "
            f"(translation correction {metrics['translation_correction_norm_m']:.3f} m, "
            f"rotation correction {metrics['rotation_correction_angle_deg']:.3f} deg).",
            file=sys.stderr,
        )
        print("Re-run with --force only after RViz/manual review.", file=sys.stderr)
        return 3

    store = LidarPairExtrinsicsStore(args.config)
    updated = store.build_updated_config(report, args.transform_key)
    if args.dry_run:
        print(yaml.safe_dump(updated, sort_keys=False))
        return 0

    store.write(updated)
    print(f"updated {args.config} from {args.report} ({args.transform_key})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
