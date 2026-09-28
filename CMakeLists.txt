cmake_minimum_required(VERSION 3.8)
project(ig_handle)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AcquisitionTimingEvent.msg
  msg/ClockedImu.msg
  msg/FirmwareTimingEvent.msg
  msg/JkBmsDetails.msg
  msg/Ping360RawPacket.msg
  msg/SonarDiagnostics.msg
  msg/SonarPlanarProfile.msg
  msg/SonarProfile.msg
  msg/SonarRawPacket.msg
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_python_install_package(ig_handle_runtime
  PACKAGE_DIR scripts/ig_handle_runtime
)
foreach(module IN ITEMS sensors teensy mocap sonar power)
  ament_python_install_package(${module}
    PACKAGE_DIR scripts/${module}
  )
endforeach()
ament_python_install_package(mocap.udp
  PACKAGE_DIR scripts/mocap/udp
)

install(DIRECTORY config systemd
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.launch.py"
    PATTERN "README.md"
)
install(PROGRAMS
  scripts/frames/broadcast.py
  scripts/frames/export.py
  scripts/imu/sample_clock.py
  scripts/imu/fit_clock_mapping.py
  scripts/mocap/bridge.py
  scripts/power/jk_bms_node.py
  scripts/power/heron_sense_ingress.py
  scripts/sonar/receiver.py
  scripts/sonar/ping360_provider.py
  scripts/teensy/timing_event_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
