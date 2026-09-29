"""Shared measured affine-clock mapping; no ROS transport or receipt-time inference."""

import math


def parse_clock_mappings(document):
    """Parse explicit fits scoped to one source and ROS clock epoch."""
    records = (
        document.get("mappings", [document]) if isinstance(document, dict) else None
    )
    if not isinstance(records, list):
        raise ValueError("clock mapping file must contain one fit or a mappings list")
    mappings = {}
    for record in records:
        if not isinstance(record, dict) or record.get("model") != "affine_clock_map":
            raise ValueError("clock mapping entries must be affine_clock_map fits")
        source_domain = str(record.get("source_clock_domain", "")).strip()
        source_instance = str(record.get("source_clock_instance_id", "")).strip()
        source_epoch = str(record.get("source_clock_epoch_id", "")).strip()
        reference_domain = str(record.get("reference_clock_domain", "")).strip()
        reference_instance = str(record.get("reference_clock_instance_id", "")).strip()
        reference_epoch = str(record.get("reference_clock_epoch_id", "")).strip()
        try:
            revision = int(record["clock_mapping_revision"])
            pivot_ns = int(record["source_pivot_ns"])
            anchor_ns = int(record["reference_anchor_ns"])
            offset_ns = float(record["reference_offset_at_source_pivot_ns"])
            rate = float(record["reference_ns_per_source_ns"])
            residual_rms_ns = float(record["residual_rms_ns"])
            uncertainty_basis = str(record.get("uncertainty_basis", ""))
            raw_covariance = record.get("parameter_covariance")
            if raw_covariance is None:
                if uncertainty_basis == "provided_pair_standard_uncertainties":
                    raise ValueError(
                        "a calibrated clock mapping needs parameter covariance"
                    )
                covariance = [[0.0, 0.0], [0.0, 0.0]]
            else:
                covariance = [[float(value) for value in row] for row in raw_covariance]
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "clock mapping is missing finite affine-fit parameters"
            ) from error
        if (
            not all(
                (
                    source_domain,
                    source_instance,
                    source_epoch,
                    reference_domain,
                    reference_instance,
                    reference_epoch,
                )
            )
            or revision <= 0
            or not math.isfinite(offset_ns)
            or not math.isfinite(rate)
            or rate <= 0.0
            or not math.isfinite(residual_rms_ns)
            or residual_rms_ns < 0.0
            or len(covariance) != 2
            or any(len(row) != 2 for row in covariance)
            or not all(math.isfinite(value) for row in covariance for value in row)
        ):
            raise ValueError(
                "clock mapping has invalid identity, revision, or fit values"
            )
        covariance_scale = max(
            1.0, *(abs(value) for row in covariance for value in row)
        )
        determinant = (
            covariance[0][0] * covariance[1][1]
            - (0.5 * (covariance[0][1] + covariance[1][0])) ** 2
        )
        if (
            covariance[0][0] < 0.0
            or covariance[1][1] < 0.0
            or abs(covariance[0][1] - covariance[1][0]) > 1e-10 * covariance_scale
            or determinant
            < -1e-12
            * max(
                1.0,
                abs(covariance[0][0] * covariance[1][1]),
                covariance[0][1] ** 2,
            )
        ):
            raise ValueError("clock mapping covariance must be positive semidefinite")
        covariance[0][1] = covariance[1][0] = 0.5 * (
            covariance[0][1] + covariance[1][0]
        )
        key = (
            source_domain,
            source_instance,
            source_epoch,
            reference_domain,
            reference_instance,
            reference_epoch,
        )
        if key in mappings:
            raise ValueError("clock mapping file repeats a source/reference scope")
        mappings[key] = {
            "revision": revision,
            "pivot_ns": pivot_ns,
            "anchor_ns": anchor_ns,
            "offset_ns": offset_ns,
            "rate": rate,
            "residual_rms_ns": residual_rms_ns,
            "covariance": covariance,
            "uncertainty_known": uncertainty_basis
            == "provided_pair_standard_uncertainties",
        }
    return mappings


def apply_clock_mapping(mapping, source_time_ns):
    """Map one source event without converting large absolute times to float."""
    delta_ns = int(source_time_ns) - mapping["pivot_ns"]
    relative_ns = mapping["offset_ns"] + mapping["rate"] * delta_ns
    if not math.isfinite(relative_ns):
        return None
    mapped_ns = mapping["anchor_ns"] + int(round(relative_ns))
    if mapped_ns <= 0 or mapped_ns >= (1 << 32) * 1000000000:
        return None
    uncertainty_sec = math.nan
    calibrated = False
    if mapping["uncertainty_known"]:
        covariance = mapping["covariance"]
        variance_ns2 = (
            covariance[0][0]
            + 2.0 * delta_ns * covariance[0][1]
            + delta_ns * delta_ns * covariance[1][1]
            + mapping["residual_rms_ns"] ** 2
        )
        if math.isfinite(variance_ns2) and variance_ns2 >= 0.0:
            uncertainty_sec = math.sqrt(variance_ns2) * 1e-9
            calibrated = math.isfinite(uncertainty_sec)
    return mapped_ns, uncertainty_sec, calibrated
