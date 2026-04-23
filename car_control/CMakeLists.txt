cmake_minimum_required(VERSION 3.0.2)
project(car_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf2_ros
  trajectory_msgs
  control_msgs
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs gazebo_msgs tf2_ros trajectory_msgs control_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_executable(tracked_cmd_bridge_node
  src/tracked_cmd_bridge_node.cpp
)
target_link_libraries(tracked_cmd_bridge_node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(tracked_cmd_bridge_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/base_cmd_node.py
  scripts/gripper_cmd_node.py
  scripts/ds5_teleop_node.py
  scripts/gazebo_model_odom_node.py
  scripts/servo_twist_frame_bridge_node.py
  scripts/keyboard_teleop_node.py
  scripts/keyboard_teleop_gui.py
  scripts/flipper_diagnostic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS tracked_cmd_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
