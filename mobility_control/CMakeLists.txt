cmake_minimum_required(VERSION 3.0.2)
project(mobility_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf2_ros
)

find_path(RUCKIG_INCLUDE_DIR
  NAMES ruckig/ruckig.hpp
  PATHS /usr/local/include
  REQUIRED
)

find_library(RUCKIG_LIBRARY
  NAMES ruckig
  PATHS /usr/local/lib
  REQUIRED
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RUCKIG_INCLUDE_DIR}
)

add_executable(base_cmd_node
  src/base_cmd_node.cpp
)

target_link_libraries(base_cmd_node
  ${catkin_LIBRARIES}
  ${RUCKIG_LIBRARY}
)

catkin_install_python(PROGRAMS
  scripts/base_cmd_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS base_cmd_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
