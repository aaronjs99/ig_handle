cmake_minimum_required(VERSION 3.0.2)
project(ig_handle)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

install(
  PROGRAMS 
    scripts/sonar/run_deltat.sh
    scripts/sonar/dt100_rx.py
    scripts/sonar/dt100_profile_to_cloud.py
    scripts/mocap/natnet_bridge.py
    scripts/pipeline/process_raw_bag.py
    scripts/pipeline/plot_odom.py
    scripts/pipeline/record_bag.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY scripts/sonar/deltat/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/deltat
  USE_SOURCE_PERMISSIONS
)
