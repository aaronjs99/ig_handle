cmake_minimum_required(VERSION 3.0.2)
project(ig_handle)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

install(
  PROGRAMS 
    scripts/run_deltat.sh
    scripts/dt100_rx.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY scripts/deltat/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/deltat
  USE_SOURCE_PERMISSIONS
)
