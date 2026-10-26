cmake_minimum_required(VERSION 3.10)
project(Eyou_ROS1_Master LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  hardware_interface
  controller_manager
  controller_manager_msgs
  std_msgs
  std_srvs
  can_driver
  Eyou_Canopen_Master
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LELY REQUIRED liblely-coapp liblely-io2 liblely-ev liblely-co)

add_message_files(
  FILES
  JointRuntimeState.msg
  JointRuntimeStateArray.msg
)

add_service_files(
  FILES
  ApplyJointLimits.srv
  SetJointZero.srv
  SetJointMode.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp rospy hardware_interface controller_manager controller_manager_msgs std_msgs std_srvs can_driver Eyou_Canopen_Master
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/hybrid_joint_action_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ── 统一外观节点 ─────────────────────────────────────────
add_executable(hybrid_motor_hw_node
  src/hybrid_auto_startup.cpp
  src/hybrid_mode_router.cpp
  src/hybrid_motor_hw_node.cpp
  src/hybrid_robot_hw.cpp
  src/hybrid_operational_coordinator.cpp
  src/hybrid_service_gateway.cpp
)
add_dependencies(hybrid_motor_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_motor_hw_node
  ${catkin_LIBRARIES}
  stdc++fs
  ${LELY_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)

  catkin_add_gtest(test_hybrid_operational_coordinator
    tests/test_hybrid_operational_coordinator.cpp
  )
  if(TARGET test_hybrid_operational_coordinator)
    target_include_directories(test_hybrid_operational_coordinator PRIVATE include ${GTEST_INCLUDE_DIRS})
    target_sources(test_hybrid_operational_coordinator PRIVATE
      src/hybrid_operational_coordinator.cpp
    )
    target_link_libraries(test_hybrid_operational_coordinator
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
  endif()

  catkin_add_gtest(test_hybrid_service_gateway
    tests/test_hybrid_service_gateway.cpp
  )
  if(TARGET test_hybrid_service_gateway)
    target_include_directories(test_hybrid_service_gateway PRIVATE include ${GTEST_INCLUDE_DIRS})
    target_sources(test_hybrid_service_gateway PRIVATE
      src/hybrid_operational_coordinator.cpp
      src/hybrid_service_gateway.cpp
    )
    target_link_libraries(test_hybrid_service_gateway
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
  endif()

  catkin_add_gtest(test_hybrid_auto_startup
    tests/test_hybrid_auto_startup.cpp
  )
  if(TARGET test_hybrid_auto_startup)
    target_include_directories(test_hybrid_auto_startup PRIVATE include ${GTEST_INCLUDE_DIRS})
    target_sources(test_hybrid_auto_startup PRIVATE
      src/hybrid_auto_startup.cpp
      src/hybrid_operational_coordinator.cpp
      src/hybrid_service_gateway.cpp
    )
    target_link_libraries(test_hybrid_auto_startup
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
  endif()

  catkin_add_gtest(test_hybrid_mode_router
    tests/test_hybrid_mode_router.cpp
  )
  if(TARGET test_hybrid_mode_router)
    target_include_directories(test_hybrid_mode_router PRIVATE include ${GTEST_INCLUDE_DIRS})
    target_sources(test_hybrid_mode_router PRIVATE
      src/hybrid_mode_router.cpp
    )
    target_link_libraries(test_hybrid_mode_router
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
  endif()

  add_rostest_gtest(test_hybrid_launch_smoke
    tests/test_hybrid_launch_smoke.test
    tests/test_hybrid_launch_smoke.cpp
  )
  if(TARGET test_hybrid_launch_smoke)
    target_include_directories(test_hybrid_launch_smoke PRIVATE include ${GTEST_INCLUDE_DIRS})
    target_link_libraries(test_hybrid_launch_smoke
      ${catkin_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
  endif()

endif()
