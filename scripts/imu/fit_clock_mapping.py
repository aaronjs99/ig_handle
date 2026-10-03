#!/usr/bin/env python3
"""Fit an affine clock map from explicitly paired acquisition events.

This offline tool never changes the live sample-clock mapping. It uses every
identified correspondence and reports residuals instead of rejecting data by
an unrecorded acceptance threshold.
"""

import argparse
import csv
import json
import math
from pathlib import Path


_REQUIRED = (
    "correspondence_id",
    "source_event_id",
    "reference_event_id",
    "source_clock_domain",
    "reference_clock_domain",
    "source_clock_instance_id",
    "source_clock_epoch_id",
    "reference_clock_instance_id",
    "reference_clock_epoch_id",
    "source_time_ns",
    "reference_time_ns",
)
_OPTIONAL = ("pair_uncertainty_ns",)


def fit_correspondences(rows, clock_mapping_revision=1):
    """Fit reference_ns = anchor_ns + offset_ns + rate*(source_ns-pivot_ns)."""
    try:
        clock_mapping_revision = int(clock_mapping_revision)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("clock mapping revision must be a positive integer") from error
    if clock_mapping_revision <= 0:
        raise ValueError("clock mapping revision must be a positive integer")
    pairs = []
    for row in rows:
        pair = {key: row.get(key) for key in _REQUIRED + _OPTIONAL}
        if any(pair[key] in (None, "") for key in _REQUIRED):
            raise ValueError(
                "each row needs every required identity, domain, and timestamp"
            )
        for key in (
            "correspondence_id",
            "source_event_id",
            "reference_event_id",
            "source_clock_domain",
            "reference_clock_domain",
            "source_clock_instance_id",
            "source_clock_epoch_id",
            "reference_clock_instance_id",
            "reference_clock_epoch_id",
        ):
            pair[key] = str(pair[key]).strip()
            if not pair[key]:
                raise ValueError("{} cannot be empty".format(key))
        try:
            pair["source_time_ns"] = int(pair["source_time_ns"])
            pair["reference_time_ns"] = int(pair["reference_time_ns"])
        except (TypeError, ValueError):
            raise ValueError("event times must be integer nanoseconds")
        uncertainty = pair["pair_uncertainty_ns"]
        if uncertainty in (None, ""):
            pair["pair_uncertainty_ns"] = None
        else:
            try:
                uncertainty = float(uncertainty)
            except (TypeError, ValueError):
                raise ValueError("pair_uncertainty_ns must be positive and finite")
            if not math.isfinite(uncertainty) or uncertainty <= 0.0:
                raise ValueError("pair_uncertainty_ns must be positive and finite")
            pair["pair_uncertainty_ns"] = uncertainty
        pairs.append(pair)

    if len(pairs) < 2:
        raise ValueError("at least two identified correspondences are needed")
    if len({p["correspondence_id"] for p in pairs}) != len(pairs):
        raise ValueError("correspondence_id values must be unique")
    if len({p["source_event_id"] for p in pairs}) != len(pairs):
        raise ValueError("a source event cannot be counted twice")
    if len({p["reference_event_id"] for p in pairs}) != len(pairs):
        raise ValueError("a reference event cannot be counted twice")
    source_domains = {p["source_clock_domain"] for p in pairs}
    reference_domains = {p["reference_clock_domain"] for p in pairs}
    source_scopes = {
        (p["source_clock_instance_id"], p["source_clock_epoch_id"]) for p in pairs
    }
    reference_scopes = {
        (p["reference_clock_instance_id"], p["reference_clock_epoch_id"]) for p in pairs
    }
    if len(source_domains) != 1 or len(reference_domains) != 1:
        raise ValueError(
            "one fit can contain only one source and one reference clock domain"
        )
    if len(source_scopes) != 1:
        raise ValueError("one fit cannot cross source clock instances or epochs")
    if len(reference_scopes) != 1:
        raise ValueError("one fit cannot cross reference clock instances or epochs")
    if len({p["source_time_ns"] for p in pairs}) < 2:
        raise ValueError("source clock times must span at least two distinct values")

    known_uncertainty = all(p["pair_uncertainty_ns"] is not None for p in pairs)
    if not known_uncertainty and any(
        p["pair_uncertainty_ns"] is not None for p in pairs
    ):
        raise ValueError(
            "provide pair uncertainty for every row or leave it blank for every row"
        )

    source_times = sorted(p["source_time_ns"] for p in pairs)
    pivot_ns = source_times[len(source_times) // 2]
    reference_anchor_ns = pairs[0]["reference_time_ns"]
    dx = [float(p["source_time_ns"] - pivot_ns) for p in pairs]
    dy = [float(p["reference_time_ns"] - reference_anchor_ns) for p in pairs]
    if known_uncertainty:
        weights = [1.0 / (p["pair_uncertainty_ns"] ** 2) for p in pairs]
    else:
        weights = [1.0] * len(pairs)

    sum_w = sum(weights)
    mean_x = sum(w * x for w, x in zip(weights, dx)) / sum_w
    mean_y = sum(w * y for w, y in zip(weights, dy)) / sum_w
    centered_xx = sum(w * (x - mean_x) ** 2 for w, x in zip(weights, dx))
    centered_xy = sum(
        w * (x - mean_x) * (y - mean_y) for w, x, y in zip(weights, dx, dy)
    )
    if not math.isfinite(centered_xx) or centered_xx <= 0.0:
        raise ValueError("source clock span is numerically degenerate")
    rate = centered_xy / centered_xx
    offset_at_pivot_ns = mean_y - rate * mean_x
    if not math.isfinite(rate) or not math.isfinite(offset_at_pivot_ns):
        raise ValueError("clock fit produced a non-finite model")

    residuals = []
    for pair, x, y in zip(pairs, dx, dy):
        residual = y - (offset_at_pivot_ns + rate * x)
        residuals.append(residual)
        pair["residual_ns"] = residual

    absolute = sorted(abs(value) for value in residuals)
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    p95_index = max(0, int(math.ceil(0.95 * len(absolute))) - 1)

    # Covariance is conditional on supplied pair uncertainties. Without them,
    # estimate residual variance only when there are residual degrees of freedom.
    covariance = None
    if known_uncertainty or len(pairs) > 2:
        sum_x = sum(w * x for w, x in zip(weights, dx))
        sum_xx = sum(w * x * x for w, x in zip(weights, dx))
        determinant = sum_w * sum_xx - sum_x * sum_x
        if determinant > 0.0 and math.isfinite(determinant):
            scale = 1.0
            if not known_uncertainty:
                scale = sum(value * value for value in residuals) / (len(pairs) - 2)
            covariance = [
                [scale * sum_xx / determinant, -scale * sum_x / determinant],
                [-scale * sum_x / determinant, scale * sum_w / determinant],
            ]

    return {
        "model": "affine_clock_map",
        "clock_mapping_revision": clock_mapping_revision,
        "source_clock_domain": next(iter(source_domains)),
        "reference_clock_domain": next(iter(reference_domains)),
        "source_clock_instance_id": next(iter(source_scopes))[0],
        "source_clock_epoch_id": next(iter(source_scopes))[1],
        "reference_clock_instance_id": next(iter(reference_scopes))[0],
        "reference_clock_epoch_id": next(iter(reference_scopes))[1],
        "source_pivot_ns": pivot_ns,
        "reference_anchor_ns": reference_anchor_ns,
        "reference_offset_at_source_pivot_ns": offset_at_pivot_ns,
        "reference_ns_per_source_ns": rate,
        "scale_ppm_from_unity": (rate - 1.0) * 1e6,
        "pair_count": len(pairs),
        "uncertainty_basis": (
            "provided_pair_standard_uncertainties"
            if known_uncertainty
            else "unknown_equal_weight"
        ),
        "parameter_covariance": covariance,
        "residual_rms_ns": rms,
        "residual_p95_abs_ns": absolute[p95_index],
        "residual_max_abs_ns": max(absolute),
        "outlier_rejection": "none; every identified correspondence is included",
        "correspondences": pairs,
    }


def read_pairs(path, clock_mapping_revision=1):
    with Path(path).open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = [
            field for field in _REQUIRED if field not in (reader.fieldnames or [])
        ]
        if missing:
            raise ValueError("CSV is missing columns: " + ", ".join(missing))
        return fit_correspondences(reader, clock_mapping_revision)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", help="CSV of already identified event pairs")
    parser.add_argument("output_json", help="path for the fitted model and residuals")
    parser.add_argument(
        "--revision",
        type=int,
        required=True,
        help="positive revision for this immutable source/reference clock scope",
    )
    args = parser.parse_args()
    fit = read_pairs(args.input_csv, args.revision)
    output = Path(args.output_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(fit, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        "wrote {} ({} pairs; RMS residual {:.3f} ns)".format(
            output, fit["pair_count"], fit["residual_rms_ns"]
        )
    )


if __name__ == "__main__":
    main()
