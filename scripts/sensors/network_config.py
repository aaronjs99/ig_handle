#!/usr/bin/env python3
"""Print one value from IG Handle's canonical network contract."""

import argparse

from sensors.network import KEY_PATHS, PACKAGE_ROOT, network_value


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
