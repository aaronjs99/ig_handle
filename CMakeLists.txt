cmake_minimum_required(VERSION 3.0.2)
project(ig_handle)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
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
    scripts/sonar/sonar.py
    scripts/mocap/mocap.py
    scripts/frames/export.py
    scripts/frames/broadcast.py
    scripts/imu/force_wakeup.py
    scripts/network_config.py
    scripts/sensor_bringup.py
    scripts/sensor_contract.py
    scripts/teensy/teensy_rosserial_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/sonar/Linux_DeltaT_v1023_x86_64
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    scripts/network_config.py
    scripts/sensor_contract.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  FILES
    scripts/network_launch_eval.py
    scripts/sensor_contract_launch_eval.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY scripts/sonar/include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/sonar/include
)

install(
  DIRECTORY
    scripts/mocap/natnet
    scripts/mocap/udp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/mocap
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
