#!/usr/bin/env python3
"""Install IG Handle's reusable contract helpers through catkin."""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        "mocap",
        "mocap.natnet",
        "mocap.udp",
        "power",
        "sensors",
        "sonar",
    ],
    package_dir={"": "scripts"},
)
setup(**setup_args)
