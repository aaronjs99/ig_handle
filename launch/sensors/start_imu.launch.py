from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def _launch_nodes(context):
    topic = LaunchConfiguration("topic").perform(context)
    receipt_topic = LaunchConfiguration("receipt_topic").perform(context)
    if not receipt_topic:
        receipt_topic = f"{topic}_receipt"
    time_topic = LaunchConfiguration("sample_time_topic").perform(context)
    timed_output_topic = LaunchConfiguration("timed_output_topic").perform(context)
    if not timed_output_topic:
        timed_output_topic = f"{topic}_timed"
    namespace = LaunchConfiguration("node_namespace").perform(context)
    node_name = LaunchConfiguration("node_name").perform(context)
    parameters_file = (
        Path(get_package_share_directory("xsens_mti_ros2_driver"))
        / "param"
        / "xsens_mti_node.yaml"
    )

    driver = Node(
        package="xsens_mti_ros2_driver",
        executable="xsens_mti_node",
        name="xsens_mti_hardware",
        namespace=namespace,
        output="screen",
        parameters=[
            str(parameters_file),
            {
                "port": LaunchConfiguration("port"),
                "baudrate": ParameterValue(
                    LaunchConfiguration("baudrate"), value_type=int
                ),
                "scan_for_devices": ParameterValue(
                    LaunchConfiguration("scan_for_devices"), value_type=bool
                ),
                "frame_id": LaunchConfiguration("frame_id"),
                "pub_imu": True,
                "pub_sampletime": True,
                "pub_transform": False,
            },
        ],
        remappings=[
            ("/imu/data", receipt_topic),
            ("/imu/time_ref", time_topic),
            ("/imu/mag", "/sensors/imu/mag"),
            ("/imu/acceleration", "/sensors/imu/acceleration"),
            ("/imu/angular_velocity", "/sensors/imu/angular_velocity"),
            ("/filter/quaternion", "/sensors/imu/filter/quaternion"),
        ],
    )
    sample_clock = Node(
        package="ig_handle",
        executable="sample_clock.py",
        name=node_name,
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "input_topic": receipt_topic,
                "time_topic": time_topic,
                "output_topic": topic,
                "timed_output_topic": timed_output_topic,
            }
        ],
    )
    return [driver, sample_clock]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("topic", default_value="/sensors/imu/data"),
            DeclareLaunchArgument("sample_time_topic", default_value="/sensors/imu/sample_time"),
            DeclareLaunchArgument("timed_output_topic", default_value=""),
            DeclareLaunchArgument("receipt_topic", default_value=""),
            DeclareLaunchArgument("port", default_value="/dev/sensors/imu"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("scan_for_devices", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="imu_link"),
            DeclareLaunchArgument("node_name", default_value="driver"),
            DeclareLaunchArgument("node_namespace", default_value="/sensors/imu"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
