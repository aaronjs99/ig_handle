"""Publish selected, configured static sensor transforms."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    share = Path(get_package_share_directory("ig_handle"))
    config_path = Path(LaunchConfiguration("sensor_frames_file").perform(context))
    if not config_path.is_absolute():
        config_path = share / "config" / "sensors" / "platform" / config_path

    return [
        Node(
            package="ig_handle",
            executable="broadcast.py",
            name="sensor_tf_broadcaster",
            output="screen",
            parameters=[
                {
                    "sensor_frames_file": str(config_path),
                    "allowed_transform_names": LaunchConfiguration(
                        "allowed_transform_names"
                    ),
                }
            ],
        )
    ]


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("ig_handle"))
        / "config"
        / "sensors"
        / "platform"
        / "sensor_frames.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_frames_file", default_value=str(default_config)
            ),
            DeclareLaunchArgument(
                "allowed_transform_names", default_value="lidar_h,lidar_v"
            ),
            OpaqueFunction(function=_node),
        ]
    )
