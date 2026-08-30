#!/usr/bin/env python3
"""Validate a hash-linked stationary IMU characterization candidate artifact.

Passing validation confirms structure and numerical sanity only. It does not
commission the candidate for navigation, control, or automatic covariance
publication.
"""

import argparse
from pathlib import Path

from sensors.imu_candidate_validation import load_and_validate


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--source-bag", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--expected-device-id", required=True)
    parser.add_argument("--expected-frame", default="imu_link")
    return parser.parse_args()


def main():
    args = parse_arguments()
    artifact_path = args.artifact.expanduser().resolve()
    load_and_validate(
        artifact_path,
        expected_device_id=args.expected_device_id,
        expected_frame=args.expected_frame,
        source_bag_path=args.source_bag,
        manifest_path=args.manifest,
    )

    print(f"STRUCTURALLY VALID descriptive artifact: {artifact_path}")
    print("Source bag and manifest hashes match. This does not commission covariance.")


if __name__ == "__main__":
    main()
