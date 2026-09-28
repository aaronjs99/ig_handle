"""Launch standalone, read-only JK BMS telemetry acquisition."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context, *args, **kwargs):
    del args, kwargs
    share = Path(get_package_share_directory("ig_handle"))
    config_path = Path(LaunchConfiguration("bms_config_file").perform(context))
    registry_path = Path(
        LaunchConfiguration("battery_registry_file").perform(context)
    )
    with config_path.open("r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream) or {}
    parameters.pop("schema_version", None)
    parameters.update(
        {
            "enabled": True,
            "battery_registry_file": str(registry_path),
            "battery_topic": LaunchConfiguration("battery_topic").perform(context),
            "details_topic": LaunchConfiguration("details_topic").perform(context),
        }
    )
    return [
        Node(
            package="ig_handle",
            executable="jk_bms_node.py",
            name="ighandle_jk_bms",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("ig_handle"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "battery_registry_file",
                default_value=str(share / "config/sensors/power/battery_registry.yaml"),
            ),
            DeclareLaunchArgument(
                "bms_config_file",
                default_value=str(share / "config/sensors/power/jk_bms.yaml"),
            ),
            DeclareLaunchArgument("battery_topic", default_value="/sense_ighandle"),
            DeclareLaunchArgument(
                "details_topic", default_value="/sense_ighandle/details"
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
