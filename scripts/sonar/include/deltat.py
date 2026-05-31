#!/usr/bin/env python3
"""DeltaT binary launcher and runtime INI generation."""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from .profiles import SonarProfile, load_sonar_profile


BINARY_NAME = "Linux_DeltaT_v1023_x86_64"
RUNTIME_SUBDIR = Path("ig_handle") / "deltat"
RUNTIME_INI_NAME = "Linux_DeltaT.INI"


@dataclass(frozen=True)
class DeltaTLaunchRequest:
    profile_name: Optional[str] = None
    profile_config: Optional[str] = None
    sonar_ip: Optional[str] = None
    udp_dest_ip: Optional[str] = None
    udp_port: Optional[str] = None
    sound_velocity_m_per_s: Optional[str] = None
    verbose: bool = False


class DeltaTLauncher:
    """Prepare the generated INI and exec the vendor DeltaT binary."""

    def __init__(
        self,
        *,
        package_dir: Path,
        runtime_dir: Optional[Path] = None,
        binary_path: Optional[Path] = None,
    ) -> None:
        self.package_dir = Path(package_dir)
        self.runtime_dir = runtime_dir or _default_runtime_dir()
        self.binary_path = binary_path or (
            self.package_dir / "scripts" / "sonar" / BINARY_NAME
        )
        self.runtime_binary = self.runtime_dir / BINARY_NAME
        self.runtime_ini = self.runtime_dir / RUNTIME_INI_NAME
        self.last_profile: Optional[SonarProfile] = None
        self.last_ini_text = ""

    def profile_config_path(self, request: DeltaTLaunchRequest) -> Path:
        if request.profile_config:
            return Path(request.profile_config)
        return self.package_dir / "config" / "sonar" / "profiles.yaml"

    def load_profile(self, request: DeltaTLaunchRequest) -> SonarProfile:
        profile = load_sonar_profile(
            str(self.profile_config_path(request)),
            request.profile_name,
            sonar_ip=request.sonar_ip,
            udp_dest_ip=request.udp_dest_ip,
            udp_port=request.udp_port,
            sound_velocity_m_per_s=request.sound_velocity_m_per_s,
        )
        self.last_profile = profile
        return profile

    def ini_text(self, profile: SonarProfile) -> str:
        self.last_ini_text = "\n".join(
            [
                "IPAddress:",
                profile.sonar_ip,
                "Range:",
                _format_number(profile.range_m),
                "Gain:",
                str(profile.gain),
                "UDPAddress:",
                profile.udp_dest_ip,
                "UDPPort:",
                str(profile.udp_port),
                "ExitOnKeyStroke:",
                "0",
                "SoundVelocity:",
                _format_number(profile.sound_velocity_m_per_s),
                "",
            ]
        )
        return self.last_ini_text

    def prepare(self, request: DeltaTLaunchRequest) -> Path:
        self._check_binary()
        config_path = self.profile_config_path(request)
        if not config_path.is_file():
            raise RuntimeError("sonar profile config not found: %s" % config_path)

        profile = self.load_profile(request)
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.runtime_ini.write_text(self.ini_text(profile), encoding="utf-8")
        if self.runtime_binary.exists() or self.runtime_binary.is_symlink():
            self.runtime_binary.unlink()
        self.runtime_binary.symlink_to(self.binary_path)
        self._log_start(request, profile, config_path)
        return self.runtime_binary

    def exec(self, request: DeltaTLaunchRequest) -> None:
        runtime_binary = self.prepare(request)
        sys.stdout.flush()
        os.chdir(str(self.runtime_dir))
        os.execv(str(runtime_binary), [str(runtime_binary)])

    def _check_binary(self) -> None:
        if not self.binary_path.is_file():
            raise RuntimeError("DeltaT binary not found: %s" % self.binary_path)
        if not os.access(str(self.binary_path), os.X_OK):
            raise RuntimeError("DeltaT binary is not executable: %s" % self.binary_path)

    def _log_start(
        self, request: DeltaTLaunchRequest, profile: SonarProfile, config_path: Path
    ) -> None:
        print("Starting DeltaT with runtime INI: %s" % self.runtime_ini)
        print("Sonar profile:            %s" % profile.name)
        print("Profile config:           %s" % config_path)
        print("Sonar IP:                 %s" % profile.sonar_ip)
        print(
            "Range/Gain:               %s / %s"
            % (_format_number(profile.range_m), profile.gain)
        )
        print(
            "UDP destination:          %s:%s" % (profile.udp_dest_ip, profile.udp_port)
        )
        if request.verbose:
            print("----- INI just before exec -----")
            for number, line in enumerate(self.last_ini_text.splitlines(), start=1):
                print("%6d\t%s" % (number, line))
            print("--------------------------------")


def run_cli(argv: Sequence[str], *, package_dir: Path) -> None:
    args = _parser().parse_args(list(argv))
    request = DeltaTLaunchRequest(
        profile_name=args.profile or None,
        profile_config=args.profile_config or None,
        sonar_ip=args.sonar_ip or None,
        udp_dest_ip=args.udp_ip or None,
        udp_port=args.udp_port or None,
        sound_velocity_m_per_s=args.sound_velocity or None,
        verbose=args.verbose or _env_true("VERBOSE"),
    )
    DeltaTLauncher(package_dir=package_dir).exec(request)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="sonar.py deltat",
        description="Generate Linux_DeltaT.INI from sonar config and exec DeltaT.",
    )
    parser.add_argument("--profile-config", default="")
    parser.add_argument("--profile", default="")
    parser.add_argument("--sonar-ip", default="")
    parser.add_argument("--udp-ip", "--udp-dest-ip", dest="udp_ip", default="")
    parser.add_argument("--udp-port", default="")
    parser.add_argument("--sound-velocity", default="")
    parser.add_argument("--verbose", action="store_true")
    return parser


def _default_runtime_dir() -> Path:
    ros_home = os.environ.get("ROS_HOME")
    if ros_home:
        return Path(ros_home) / RUNTIME_SUBDIR
    return Path.home() / ".ros" / RUNTIME_SUBDIR


def _env_true(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


def _format_number(value: float) -> str:
    if float(value).is_integer():
        return str(int(value))
    return str(value)
