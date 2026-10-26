cmake_minimum_required(VERSION 3.10)
project(Eyou_Canopen_Master LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── catkin ─────────────────────────────��────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  control_msgs
  sensor_msgs
  diagnostic_msgs
  hardware_interface
  controller_manager
  std_srvs
  urdf
  message_generation
  diagnostic_updater
  trajectory_msgs
)

add_service_files(FILES
  ApplyLimits.srv
  SetMode.srv
  SetZero.srv
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES canopen_core canopen_ros_adapter
  CATKIN_DEPENDS roscpp rospy actionlib control_msgs sensor_msgs diagnostic_msgs hardware_interface controller_manager std_srvs urdf message_runtime diagnostic_updater trajectory_msgs
)

# ── 第三方依赖 ─────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(LELY REQUIRED liblely-coapp liblely-io2 liblely-ev liblely-co)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
find_library(RUCKIG_LIBRARY NAMES ruckig PATHS /usr/local/lib REQUIRED)
include(GoogleTest)

option(CANOPEN_FETCH_SPDLOG
  "Allow downloading spdlog via FetchContent when system package is unavailable"
  ON)
find_package(spdlog QUIET CONFIG)
if(NOT spdlog_FOUND)
  if(NOT CANOPEN_FETCH_SPDLOG)
    message(FATAL_ERROR
      "spdlog not found. Install system spdlog or configure with -DCANOPEN_FETCH_SPDLOG=ON.")
  endif()
  include(FetchContent)
  FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.12.0
  )
  FetchContent_MakeAvailable(spdlog)
endif()

# ── canopen_core 库（不依赖 ROS） ──────────────────────
add_library(canopen_core
  src/cia402_protocol.cpp
  src/cia402_state_machine.cpp
  src/shared_state.cpp
  src/axis_driver.cpp
  src/canopen_master.cpp
  src/dcf_path_utils.cpp
  src/canopen_robot_hw.cpp
  src/joints_config.cpp
  src/pdo_mapping.cpp
  src/lifecycle_manager.cpp
  src/operational_coordinator.cpp
  src/sdo_accessor.cpp
  src/diagnostics_collector.cpp
  src/realtime_loop.cpp
  src/axis_logic.cpp
  src/boot_identity_diag.cpp
  src/zero_soft_limit_executor.cpp
)

target_include_directories(canopen_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  SYSTEM PUBLIC
    ${LELY_INCLUDE_DIRS}
)

target_link_libraries(canopen_core
  PUBLIC
    ${LELY_LIBRARIES}
    Threads::Threads
    yaml-cpp
    spdlog::spdlog
)

target_compile_options(canopen_core
  PRIVATE
    -Wall -Wextra -Werror
    ${LELY_CFLAGS_OTHER}
)

# ── 独立可执行（不依赖 ROS） ──────────────────────────
add_executable(canopen_hw_node
  src/main.cpp
)

target_link_libraries(canopen_hw_node
  PRIVATE
    canopen_core
)

# ── ROS1 适配层 ───────────────────────────────────────
add_library(canopen_ros_adapter
  src/canopen_robot_hw_ros.cpp
  src/urdf_joint_limits.cpp
  src/canopen_aux_services.cpp
  src/controllers/ip_follow_joint_trajectory_executor_validation.cpp
  src/controllers/ip_follow_joint_trajectory_executor_runtime.cpp
  src/controllers/ip_follow_joint_trajectory_executor_ros.cpp
  src/service_gateway.cpp
)
target_include_directories(canopen_ros_adapter
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(canopen_ros_adapter
  PUBLIC
    canopen_core
    ${RUCKIG_LIBRARY}
    ${catkin_LIBRARIES}
)
target_compile_options(canopen_ros_adapter
  PRIVATE
    -Wall -Wextra -Werror
)

add_executable(canopen_hw_ros_node
  src/canopen_hw_ros_node.cpp
)
target_link_libraries(canopen_hw_ros_node
  PRIVATE
    canopen_ros_adapter
)
# This node is sometimes granted file capabilities (NET_RAW / NET_ADMIN) for SocketCAN.
# In that secure-exec mode the dynamic loader ignores LD_LIBRARY_PATH, so we need a
# transitive embedded search path for indirect ROS dependencies such as
# libcontroller_manager.so -> libclass_loader.so. Emitting legacy RPATH instead of
# RUNPATH keeps the ROS library directories visible to those indirect dependencies.
target_link_options(canopen_hw_ros_node
  PRIVATE
    -Wl,--disable-new-dtags
)
add_dependencies(canopen_hw_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ── Install ───────────────────────────────────────────
install(TARGETS canopen_core canopen_ros_adapter canopen_hw_node canopen_hw_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/canopen_hw/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/joint_action_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/generate_dcf.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ── Tests ─────────────────────────────────────────────
enable_testing()

add_executable(test_state_machine
  test/test_state_machine.cpp
)
target_link_libraries(test_state_machine PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_state_machine)

add_executable(test_shared_state
  test/test_shared_state.cpp
)
target_link_libraries(test_shared_state PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_shared_state)

add_executable(test_canopen_robot_hw
  test/test_canopen_robot_hw.cpp
)
target_link_libraries(test_canopen_robot_hw PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_canopen_robot_hw)

add_executable(test_canopen_robot_hw_ros
  test/test_canopen_robot_hw_ros.cpp
)
target_link_libraries(test_canopen_robot_hw_ros PRIVATE canopen_ros_adapter GTest::gtest_main)
gtest_discover_tests(test_canopen_robot_hw_ros)

add_executable(test_ip_follow_joint_trajectory_executor
  test/test_ip_follow_joint_trajectory_executor.cpp
)
target_link_libraries(test_ip_follow_joint_trajectory_executor PRIVATE canopen_ros_adapter GTest::gtest_main)
gtest_discover_tests(test_ip_follow_joint_trajectory_executor)

add_executable(test_position_channel_routing
  test/test_position_channel_routing.cpp
)
target_link_libraries(test_position_channel_routing PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_position_channel_routing)

add_executable(test_unit_conversion
  test/test_unit_conversion.cpp
)
target_link_libraries(test_unit_conversion PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_unit_conversion)

add_executable(test_joints_config
  test/test_joints_config.cpp
)
target_link_libraries(test_joints_config PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_joints_config)

add_executable(test_zero_soft_limit_executor
  test/test_zero_soft_limit_executor.cpp
)
target_link_libraries(test_zero_soft_limit_executor PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_zero_soft_limit_executor)

add_executable(test_dcf_path_utils
  test/test_dcf_path_utils.cpp
)
target_link_libraries(test_dcf_path_utils PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_dcf_path_utils)

add_executable(test_urdf_joint_limits
  test/test_urdf_joint_limits.cpp
)
target_link_libraries(test_urdf_joint_limits PRIVATE canopen_ros_adapter GTest::gtest_main)
gtest_discover_tests(test_urdf_joint_limits)

add_executable(test_canopen_master
  test/test_canopen_master.cpp
)
target_link_libraries(test_canopen_master PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_canopen_master)

add_executable(test_startup_integration
  test/test_startup_integration.cpp
)
target_link_libraries(test_startup_integration PRIVATE canopen_core GTest::gtest_main)
target_compile_definitions(test_startup_integration PRIVATE
  PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  PROJECT_BINARY_DIR="${CMAKE_BINARY_DIR}"
)
gtest_discover_tests(test_startup_integration)

add_executable(test_shared_state_concurrent
  test/test_shared_state_concurrent.cpp
)
target_link_libraries(test_shared_state_concurrent PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_shared_state_concurrent)

add_executable(test_boundary_cases
  test/test_boundary_cases.cpp
)
target_link_libraries(test_boundary_cases PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_boundary_cases)

add_executable(test_lifecycle_manager
  test/test_lifecycle_manager.cpp
)
target_link_libraries(test_lifecycle_manager PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_lifecycle_manager)

add_executable(test_manual_control
  test/test_manual_control.cpp
)
target_link_libraries(test_manual_control PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_manual_control)

add_executable(test_sdo_accessor
  test/test_sdo_accessor.cpp
)
target_link_libraries(test_sdo_accessor PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_sdo_accessor)

add_executable(test_sdo_single_flight
  test/test_sdo_single_flight.cpp
)
target_link_libraries(test_sdo_single_flight PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_sdo_single_flight)

add_executable(test_pdo_mapping
  test/test_pdo_mapping.cpp
)
target_link_libraries(test_pdo_mapping PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_pdo_mapping)

add_executable(test_diagnostics_collector
  test/test_diagnostics_collector.cpp
)
target_link_libraries(test_diagnostics_collector PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_diagnostics_collector)

add_executable(test_multi_mode
  test/test_multi_mode.cpp
)
target_link_libraries(test_multi_mode PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_multi_mode)

add_executable(test_realtime_loop
  test/test_realtime_loop.cpp
)
target_link_libraries(test_realtime_loop PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_realtime_loop)

add_executable(test_axis_logic
  test/test_axis_logic.cpp
)
target_link_libraries(test_axis_logic PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_axis_logic)

add_executable(test_boot_identity_diag
  test/test_boot_identity_diag.cpp
)
target_link_libraries(test_boot_identity_diag PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_boot_identity_diag)

add_executable(test_operational_coordinator
  test/test_operational_coordinator.cpp
)
target_link_libraries(test_operational_coordinator PRIVATE canopen_core GTest::gtest_main)
gtest_discover_tests(test_operational_coordinator)
