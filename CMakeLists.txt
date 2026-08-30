cmake_minimum_required(VERSION 3.0.2)
project(ig_handle)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)
catkin_python_setup()

add_message_files(
  FILES
    JkBmsDetails.msg
    Ping360RawPacket.msg
    SonarDiagnostics.msg
    SonarProfile.msg
    SonarRawPacket.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/sonar/provider.py
    scripts/mocap/bridge.py
    scripts/frames/export.py
    scripts/frames/broadcast.py
    scripts/imu/analyze_stationary_bag.py
    scripts/imu/force_wakeup.py
    scripts/imu/validate_stationary_candidate.py
    scripts/external_sensor_provider.py
    scripts/network_config.py
    scripts/sensor_bringup.py
    scripts/teensy/teensy_rosserial_launcher.py
    scripts/power/heron_sense_ingress.py
    scripts/power/jk_bms_node.py
    scripts/power/validate_battery_registry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/sonar/Linux_DeltaT_v1023_x86_64
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch systemd
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
