"""Launch the acquisition timing adapter on ROS 2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("timing_status_topic", default_value=""),
        DeclareLaunchArgument("firmware_event_topic", default_value=""),
        DeclareLaunchArgument(
            "timing_events_topic", default_value="/sensors/timing/events"
        ),
        DeclareLaunchArgument(
            "camera_frame_timing_topics",
            default_value="",
            description=(
                "Comma-separated ROS 2 CameraFrameTiming topics; leave empty when "
                "the ROS 2 camera driver is not installed."
            ),
        ),
        DeclareLaunchArgument("clock_mappings_file", default_value=""),
    ]
    node = Node(
        package="ig_handle",
        executable="timing_event_adapter.py",
        name="acquisition_timing_event_adapter",
        output="screen",
        parameters=[
            {
                "timing_status_topic": LaunchConfiguration("timing_status_topic"),
                "firmware_event_topic": LaunchConfiguration("firmware_event_topic"),
                "timing_events_topic": LaunchConfiguration("timing_events_topic"),
                "camera_frame_timing_topics": LaunchConfiguration(
                    "camera_frame_timing_topics"
                ),
                "clock_mappings_file": LaunchConfiguration("clock_mappings_file"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
