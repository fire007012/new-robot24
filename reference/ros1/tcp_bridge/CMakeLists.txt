cmake_minimum_required(VERSION 3.0.2)
project(tcp_bridge)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system thread)

# JSON library (header-only)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann)

add_message_files(
  FILES
  MotorCommand.msg
  ExecutorState.msg
  CameraInfo.msg
  JointData.msg
  JointCommand.msg
  CartesianCommand.msg
  ControlCommand.msg
  SystemStatus.msg
  SystemCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tcp_bridge
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs message_runtime
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(tcp_bridge_node
  src/main.cpp
  src/tcp_server.cpp
  src/ros_handler.cpp
  src/json_converter.cpp
)

add_dependencies(tcp_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(tcp_bridge_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)
