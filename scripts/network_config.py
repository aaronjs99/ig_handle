#!/usr/bin/env python3
"""Read the IG Handle network endpoints."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NETWORK_CONFIG = PACKAGE_ROOT / "config" / "network" / "sensor_network.yaml"

KEY_PATHS: Dict[str, Tuple[str, ...]] = {
    "camera_f1_ip": ("endpoints", "camera_f1", "host"),
    "camera_f2_ip": ("endpoints", "camera_f2", "host"),
    "camera_f3_ip": ("endpoints", "camera_f3", "host"),
    "camera_f4_ip": ("endpoints", "camera_f4", "host"),
    "lidar_h_ip": ("endpoints", "lidar_h", "host"),
    "lidar_v_ip": ("endpoints", "lidar_v", "host"),
    "sonar_ip": ("endpoints", "sonar", "host"),
    "heron_ip": ("endpoints", "heron", "host"),
    "heron_local_ip": ("ros", "heron_local_ip"),
    "local_master_uri": ("ros", "local_master_uri"),
    "local_ros_ip": ("ros", "local_ros_ip"),
    "mocap_natnet_server_ip": ("mocap", "natnet_server_ip"),
    "mocap_natnet_client_ip": ("mocap", "natnet_client_ip"),
    "mocap_natnet_multicast_address": ("mocap", "natnet_multicast_address"),
    "mocap_udp_bind_ip": ("mocap", "udp_bind_ip"),
    "mocap_datacollect_source_ip": ("mocap", "datacollect_source_ip"),
    "sensor_lan_ip": ("local_interfaces", "sensor_lan_ip"),
    "sonar_lan_ip": ("local_interfaces", "sonar_lan_ip"),
}


def load_network_config(package_root: Path | str = PACKAGE_ROOT) -> Dict[str, Any]:
    path = Path(package_root) / "config" / "network" / "sensor_network.yaml"
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return dict(data.get("sensor_network", {}) or {})


def network_value(
    key: str,
    *,
    package_root: Path | str = PACKAGE_ROOT,
    default: str = "",
) -> str:
    path = KEY_PATHS.get(str(key or "").strip())
    if not path:
        return str(default)
    value: Any = load_network_config(package_root)
    for part in path:
        if not isinstance(value, dict) or part not in value:
            return str(default)
        value = value[part]
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("key", choices=sorted(KEY_PATHS))
    parser.add_argument("--package-root", default=str(PACKAGE_ROOT))
    parser.add_argument("--default", default="")
    args = parser.parse_args()
    print(
        network_value(
            args.key,
            package_root=args.package_root,
            default=args.default,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
