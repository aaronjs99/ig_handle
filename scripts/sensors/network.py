#!/usr/bin/env python3
"""Read the IG Handle network endpoints."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, MutableMapping, Optional, Tuple
import os
import socket
import subprocess

import rospkg
import yaml


def _package_root() -> Path:
    for source_root in Path(__file__).resolve().parents:
        if (source_root / "package.xml").is_file() and (
            source_root / "config" / "network" / "sensor_network.yaml"
        ).is_file():
            return source_root
    return Path(rospkg.RosPack().get_path("ig_handle")).resolve()


PACKAGE_ROOT = _package_root()
NETWORK_CONFIG = PACKAGE_ROOT / "config" / "network" / "sensor_network.yaml"

KEY_PATHS: Dict[str, Tuple[str, ...]] = {
    "camera_f1_ip": ("endpoints", "camera_f1", "host"),
    "camera_f2_ip": ("endpoints", "camera_f2", "host"),
    "camera_f3_ip": ("endpoints", "camera_f3", "host"),
    "camera_f4_ip": ("endpoints", "camera_f4", "host"),
    "lidar_h_ip": ("endpoints", "lidar_h", "host"),
    "lidar_v_ip": ("endpoints", "lidar_v", "host"),
    "dt100_ip": ("endpoints", "dt100", "host"),
    "ping360_ip": ("endpoints", "ping360", "host"),
    "heron_ip": ("endpoints", "heron", "host"),
    "heron_local_ip": ("ros", "heron_local_ip"),
    "standalone_master_uri": ("ros", "standalone_master_uri"),
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


def route_source_ip(target_ip: str) -> Optional[str]:
    """Return the local source address selected by the kernel for a target."""
    try:
        output = subprocess.check_output(
            ["ip", "-o", "route", "get", target_ip],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        return None
    parts = output.split()
    for index, part in enumerate(parts):
        if part == "src" and index + 1 < len(parts):
            return parts[index + 1]
    return None


def configure_heron_ros_environment(
    enabled: bool,
    host: str,
    host_ip: str,
    local_ip: str,
    *,
    environ: Optional[MutableMapping[str, str]] = None,
) -> None:
    """Select the Heron ROS master and local route when automatic setup is enabled."""
    if not enabled:
        return
    try:
        master_ip = socket.gethostbyname(host)
    except OSError:
        master_ip = host_ip
    env = os.environ if environ is None else environ
    env["ROS_MASTER_URI"] = "http://{}:11311".format(master_ip)
    env["ROS_IP"] = route_source_ip(master_ip) or local_ip
    env.pop("ROS_HOSTNAME", None)
