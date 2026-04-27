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

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER QUIET
  gstreamer-1.0
  gstreamer-app-1.0
)

if(NOT GSTREAMER_FOUND)
  message(WARNING "GStreamer development packages not found; h265_encoder_node and h265_decoder_node will not be built. Install libgstreamer1.0-dev and libgstreamer-plugins-base1.0-dev to enable H.265 support.")
endif()

add_message_files(
  FILES
  Detection.msg
  DetectedObject3D.msg
  DetectedObject3DArray.msg
  ObstacleWarning.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
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
  ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
  ${GSTREAMER_LIBRARY_DIRS}
)

add_definitions(
  ${GSTREAMER_CFLAGS_OTHER}
)

add_library(vision_yolo
  src/yolov8.cpp
)
add_dependencies(vision_yolo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_yolo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(vision_panorama
  src/fisheye.cpp
  src/panorama.cpp
)
add_dependencies(vision_panorama ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_panorama
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(vision_display_node
  src/vision_display_node.cpp
)
add_dependencies(vision_display_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_display_node
  vision_yolo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(panorama_node
  src/panorama_node.cpp
)
add_dependencies(panorama_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(panorama_node
  vision_panorama
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if(GSTREAMER_FOUND)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/h265_encoder_node.cpp)
    add_executable(h265_encoder_node
      src/h265_encoder_node.cpp
    )
    add_dependencies(h265_encoder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(h265_encoder_node
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${GSTREAMER_LIBRARIES}
    )
  endif()

  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/h265_decoder_node.cpp)
    add_executable(h265_decoder_node
      src/h265_decoder_node.cpp
    )
    add_dependencies(h265_decoder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(h265_decoder_node
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${GSTREAMER_LIBRARIES}
    )
  endif()
endif()

add_executable(object_pose_node
  src/object_pose_node.cpp
)
add_dependencies(object_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_pose_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(distance_display_node
  src/distance_display_node.cpp
)
add_dependencies(distance_display_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(distance_display_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(obstacle_warning_node
  src/obstacle_warning_node.cpp
)
add_dependencies(obstacle_warning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_warning_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(yolov8_node
  src/yolov8_node.cpp
)
add_dependencies(yolov8_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yolov8_node
  vision_yolo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/detect_behind_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

