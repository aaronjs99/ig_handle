#!/usr/bin/env python3
"""Load configured sonar runtime profiles."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from sensors.network import network_value


@dataclass(frozen=True)
class SonarProfile:
    name: str
    range_m: float
    gain: int
    sonar_ip: str
    udp_dest_ip: str
    udp_port: int
    sound_velocity_m_per_s: float


def load_sonar_profile(
    config_path: str,
    profile_name: Optional[str] = None,
    *,
    sonar_ip: Optional[str] = None,
    udp_dest_ip: Optional[str] = None,
    udp_port: Optional[str] = None,
    sound_velocity_m_per_s: Optional[str] = None,
) -> SonarProfile:
    with Path(config_path).open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    defaults = _mapping(config.get("defaults", {}), "defaults")
    profiles = _mapping(config.get("profiles", {}), "profiles")
    selected_name = str(profile_name or defaults.get("profile", "pool")).strip()
    if selected_name not in profiles:
        available = ", ".join(sorted(str(name) for name in profiles)) or "<none>"
        raise ValueError(
            "unknown sonar profile %r in %s; available profiles: %s"
            % (selected_name, config_path, available)
        )

    profile = _mapping(profiles[selected_name], "profiles.%s" % selected_name)
    range_m = _required(profile, "range_m", selected_name)
    gain = _required(profile, "gain", selected_name)

    return SonarProfile(
        name=selected_name,
        range_m=float(range_m),
        gain=int(gain),
        sonar_ip=str(
            _override_or_profile_or_default(sonar_ip, profile, defaults, "sonar_ip")
        ),
        udp_dest_ip=str(
            _override_or_profile_or_default(
                udp_dest_ip, profile, defaults, "udp_dest_ip"
            )
        ),
        udp_port=int(
            _override_or_profile_or_default(udp_port, profile, defaults, "udp_port")
        ),
        sound_velocity_m_per_s=float(
            _override_or_profile_or_default(
                sound_velocity_m_per_s,
                profile,
                defaults,
                "sound_velocity_m_per_s",
            )
        ),
    )


def _mapping(value, label: str) -> Mapping:
    if not isinstance(value, Mapping):
        raise ValueError("%s must be a mapping" % label)
    return value


def _required(profile: Mapping, key: str, profile_name: str):
    if key not in profile:
        raise ValueError("sonar profile %r is missing %s" % (profile_name, key))
    return profile[key]


def _override_or_profile_or_default(
    override: Optional[str], profile: Mapping, defaults: Mapping, key: str
):
    if override not in (None, ""):
        return override
    if key in profile:
        return profile[key]
    network_key = {"sonar_ip": "sonar_ip", "udp_dest_ip": "sonar_lan_ip"}.get(key)
    if network_key:
        value = network_value(network_key)
        if value:
            return value
    if key not in defaults:
        raise ValueError("sonar profile defaults are missing %s" % key)
    return defaults[key]
