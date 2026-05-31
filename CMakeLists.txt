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
    scripts/sonar/cloud_generator.py
    scripts/sonar/decoder.py
    scripts/sonar/profiles.py
    scripts/sonar/receiver.py
    scripts/mocap/mocap.py
    scripts/teensy/teensy_rosserial_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/sonar/run_deltat.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY scripts/sonar/deltat/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/sonar/deltat
  USE_SOURCE_PERMISSIONS
)

install(
  PROGRAMS
    scripts/sonar/decoder.py
    scripts/sonar/profiles.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/sonar
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
