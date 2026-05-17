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
    scripts/sonar/dt100_rx.py
    scripts/sonar/dt100_profile_to_cloud.py
    scripts/mocap/natnet_pose_bridge.py
    scripts/calibration/apply_lidar_pair_calibration.py
    scripts/calibration/lidar_pair_tf_broadcaster.py
    scripts/pipeline/process_raw_bag.py
    scripts/mocap/plot_mocap_odom.py
    scripts/teensy/teensy_rosserial_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/sonar/run_deltat.sh
    scripts/pipeline/record_bag.sh
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
