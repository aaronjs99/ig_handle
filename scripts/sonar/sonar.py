#!/usr/bin/env python3
"""Single entrypoint for the Imagenex sonar runtime tools."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional, Sequence


COMMAND_ALIASES = {
    "receiver": "receiver",
    "raw": "receiver",
    "deltat": "deltat",
    "tx": "deltat",
}


def main(argv: Optional[Sequence[str]] = None) -> None:
    original_argv = list(sys.argv if argv is None else argv)
    app_args = _app_args(original_argv)
    if not app_args or app_args[0] in {"-h", "--help"}:
        _print_usage()
        return

    package_dir = _package_dir()

    command = COMMAND_ALIASES.get(app_args[0])
    scripts_dir = package_dir / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))
    sonar_script_dir = package_dir / "scripts" / "sonar"
    if str(sonar_script_dir) not in sys.path:
        sys.path.insert(0, str(sonar_script_dir))

    if command == "receiver":
        from include.receiver import run

        run()
        return
    if command == "deltat":
        from include.deltat_runner import run_cli

        run_cli(app_args[1:], package_dir=package_dir)
        return

    print("Unknown sonar command: %s" % app_args[0], file=sys.stderr)
    _print_usage()
    raise SystemExit(2)


def _app_args(argv: Sequence[str]) -> list[str]:
    try:
        import rospy

        return list(rospy.myargv(argv=list(argv))[1:])
    except Exception:
        return [arg for arg in argv[1:] if ":=" not in arg]


def _package_dir() -> Path:
    try:
        import rospkg

        return Path(rospkg.RosPack().get_path("ig_handle"))
    except Exception:
        return Path(__file__).resolve().parents[2]


def _print_usage() -> None:
    print(
        "Usage: sonar.py {receiver|deltat} [args]\n"
        "\n"
        "Commands:\n"
        "  receiver  Publish raw vendor UDP datagrams on /sensors/sonar/raw.\n"
        "  deltat    Generate Linux_DeltaT.INI from config and exec the DeltaT binary.",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
