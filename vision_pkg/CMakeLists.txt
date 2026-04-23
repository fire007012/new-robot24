cmake_minimum_required(VERSION 3.0.2)
project(vision_pkg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall)

set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS
  core
  dnn
  highgui
  imgcodecs
  imgproc
)

add_message_files(FILES
  Detection.msg
  ObstacleWarning.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${THIRD_PARTY_DIR}/include
)

add_executable(yolov8_node
  src/yolov8.cpp
  src/yolov8_node.cpp
)
target_link_libraries(yolov8_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(yolov8_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(panorama_node
  src/fisheye.cpp
  src/panorama.cpp
  src/panorama_node.cpp
)
target_link_libraries(panorama_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(panorama_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(object_pose_node
  src/object_pose_node.cpp
)
target_link_libraries(object_pose_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(object_pose_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(vision_display_node
  src/yolov8.cpp
  src/vision_display_node.cpp
)
target_link_libraries(vision_display_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(vision_display_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(distance_display_node
  src/distance_display_node.cpp
)
target_link_libraries(distance_display_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(distance_display_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(obstacle_warning_node
  src/obstacle_warning_node.cpp
)
target_link_libraries(obstacle_warning_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(obstacle_warning_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

catkin_install_python(PROGRAMS
  scripts/detect_behind_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
