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
    scripts/sensor_bringup.py
    scripts/teensy/teensy_rosserial_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(IG_HANDLE_PYTHON_MODULES
  scripts/network_config.py
  scripts/sensor_contract.py
)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
foreach(IG_HANDLE_PYTHON_MODULE ${IG_HANDLE_PYTHON_MODULES})
  get_filename_component(IG_HANDLE_PYTHON_MODULE_FILE
                         ${IG_HANDLE_PYTHON_MODULE} NAME)
  get_filename_component(IG_HANDLE_PYTHON_MODULE_NAME
                         ${IG_HANDLE_PYTHON_MODULE} NAME_WE)
  set(IG_HANDLE_PYTHON_MODULE_DEVEL
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${IG_HANDLE_PYTHON_MODULE_FILE})
  add_custom_target(${IG_HANDLE_PYTHON_MODULE_NAME}_devel ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/${IG_HANDLE_PYTHON_MODULE}
            ${IG_HANDLE_PYTHON_MODULE_DEVEL}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${IG_HANDLE_PYTHON_MODULE}
    VERBATIM
  )
endforeach()

install(
  PROGRAMS scripts/network_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES scripts/sensor_contract.py
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
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
