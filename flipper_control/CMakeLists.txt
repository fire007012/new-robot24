cmake_minimum_required(VERSION 3.0.2)
project(flipper_control)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager_msgs
  control_msgs
  diagnostic_msgs
  Eyou_Canopen_Master
  Eyou_ROS1_Master
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  urdf
)

add_message_files(
  FILES
    FlipperControlState.msg
)

add_service_files(
  FILES
    SetControlProfile.srv
    SetLinkageMode.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flipper_control_core flipper_control_controllers
  CATKIN_DEPENDS
    controller_interface
    controller_manager_msgs
    control_msgs
    diagnostic_msgs
    Eyou_Canopen_Master
    Eyou_ROS1_Master
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    trajectory_msgs
    urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/flipper_motor_debug_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(flipper_control_core
  src/flipper_reference_generator.cpp
)
add_dependencies(flipper_control_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(flipper_control_core
  ${catkin_LIBRARIES}
)

add_library(flipper_control_controllers
  src/joint_group_velocity_controller.cpp
)
add_dependencies(flipper_control_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(flipper_control_controllers
  ${catkin_LIBRARIES}
)

add_executable(flipper_manager_node
  src/flipper_manager_node.cpp
)
add_dependencies(flipper_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(flipper_manager_node
  flipper_control_core
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_flipper_reference_generator
    test/test_flipper_reference_generator.cpp
  )
  if(TARGET test_flipper_reference_generator)
    add_dependencies(test_flipper_reference_generator
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(test_flipper_reference_generator
      flipper_control_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS flipper_control_core flipper_control_controllers flipper_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES flipper_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
