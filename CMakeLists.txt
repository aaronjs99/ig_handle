cmake_minimum_required(VERSION 3.0.2)
project(ig_handle)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/sonar/sonar.py
    scripts/mocap/mocap.py
    scripts/teensy/teensy_rosserial_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/sonar/Linux_DeltaT_v1023_x86_64
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/sonar
)

install(
  DIRECTORY scripts/sonar/include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/sonar
  USE_SOURCE_PERMISSIONS
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
