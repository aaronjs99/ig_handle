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

install(
  PROGRAMS 
    scripts/sonar/run_deltat.sh
    scripts/sonar/dt100_rx.py
    scripts/sonar/dt100_profile_to_cloud.py
    scripts/mocap/natnet_pose_bridge.py
    scripts/pipeline/process_raw_bag.py
    scripts/pipeline/plot_mocap_odom.py
    scripts/pipeline/record_bag.sh
    scripts/teensy_rosserial_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY scripts/sonar/deltat/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/deltat
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
