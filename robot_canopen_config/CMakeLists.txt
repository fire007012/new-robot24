cmake_minimum_required(VERSION 3.8)
project(robot_canopen_config)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(DCFGEN_EXECUTABLE dcfgen REQUIRED)
find_program(DCFCHK_EXECUTABLE dcfchk REQUIRED)

set(CONFIG_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/config)
set(CONFIG_BUILD ${CMAKE_CURRENT_BINARY_DIR}/config)
set(CONFIG_INSTALL ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config)

add_custom_command(
  OUTPUT ${CONFIG_BUILD}/master.dcf ${CONFIG_BUILD}/bus.yml
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CONFIG_BUILD}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_bus.py
          ${CONFIG_SOURCE}/legacy_master.yaml ${CONFIG_BUILD}/bus.yml
          --joints ${CONFIG_SOURCE}/legacy_joints.yaml --dcf-path .
  COMMAND ${DCFGEN_EXECUTABLE} -v -d ${CONFIG_BUILD} -rS ${CONFIG_BUILD}/bus.yml
  COMMAND ${DCFCHK_EXECUTABLE} ${CONFIG_BUILD}/master.dcf
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_generated.py
          ${CONFIG_SOURCE}/legacy_master.dcf ${CONFIG_BUILD}/master.dcf ${CONFIG_BUILD}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_bus.py
          ${CONFIG_SOURCE}/legacy_master.yaml ${CONFIG_BUILD}/bus.yml
          --joints ${CONFIG_SOURCE}/legacy_joints.yaml --dcf-path .
  DEPENDS ${CONFIG_SOURCE}/legacy_master.yaml
          ${CONFIG_SOURCE}/legacy_joints.yaml
          ${CONFIG_SOURCE}/legacy_master.dcf
          ${CONFIG_SOURCE}/YiyouServo_V1.4.dcfgen.eds
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_bus.py
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_generated.py
  WORKING_DIRECTORY ${CONFIG_SOURCE}
  VERBATIM
)
add_custom_target(generate_robot_canopen_dcf ALL DEPENDS ${CONFIG_BUILD}/master.dcf)


set(AXIS_NAMES left_front_arm_joint right_front_arm_joint left_rear_arm_joint right_rear_arm_joint shoulder_yaw_joint shoulder_pitch_joint)
set(AXIS_IDS 1 2 3 4 6 5)
foreach(i RANGE 0 5)
  list(GET AXIS_NAMES ${i} AXIS_NAME)
  list(GET AXIS_IDS ${i} AXIS_ID)
  set(AXIS_BUILD ${CONFIG_BUILD}/axis_${AXIS_ID})
  set(AXIS_INSTALL ${CONFIG_INSTALL}/axis_${AXIS_ID})
  add_custom_command(OUTPUT ${AXIS_BUILD}/master.dcf ${AXIS_BUILD}/bus.yml
    COMMAND ${CMAKE_COMMAND} -E make_directory ${AXIS_BUILD}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_bus.py
      ${CONFIG_SOURCE}/legacy_master.yaml ${AXIS_BUILD}/bus.yml --joints ${CONFIG_SOURCE}/legacy_joints.yaml --dcf-path . --axes ${AXIS_NAME}
    COMMAND ${DCFGEN_EXECUTABLE} -v -d ${AXIS_BUILD} -rS ${AXIS_BUILD}/bus.yml
    COMMAND ${DCFCHK_EXECUTABLE} ${AXIS_BUILD}/master.dcf
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_bus.py
      ${CONFIG_SOURCE}/legacy_master.yaml ${AXIS_BUILD}/bus.yml --joints ${CONFIG_SOURCE}/legacy_joints.yaml --dcf-path . --axes ${AXIS_NAME}
    DEPENDS ${CONFIG_SOURCE}/legacy_master.yaml ${CONFIG_SOURCE}/legacy_joints.yaml ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_bus.py
    WORKING_DIRECTORY ${CONFIG_SOURCE} VERBATIM)
  add_custom_target(generate_axis_${AXIS_ID} ALL DEPENDS ${AXIS_BUILD}/master.dcf)
  install(FILES ${CONFIG_SOURCE}/YiyouServo_V1.4.dcfgen.eds DESTINATION share/${PROJECT_NAME}/config/axis_${AXIS_ID})
endforeach()
install(DIRECTORY ${CONFIG_SOURCE}/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY ${CONFIG_BUILD}/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(canopen_base_driver REQUIRED)
  find_package(canopen_core REQUIRED)
  ament_add_gtest(test_configuration_path test/test_configuration_path.cpp)
  ament_target_dependencies(test_configuration_path canopen_core)
  target_link_libraries(test_configuration_path canopen_core::device_container)
  ament_add_gtest(test_virtual_can test/test_virtual_can.cpp)
  ament_target_dependencies(test_virtual_can canopen_base_driver)
  target_compile_definitions(test_virtual_can PRIVATE
    TEST_MASTER_DCF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/virtual_master.dcf"
    TEST_DEVICE_EDS="${CONFIG_SOURCE}/YiyouServo_V1.4.dcfgen.eds")
  find_package(pluginlib REQUIRED)
  find_package(hardware_interface REQUIRED)
  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  ament_target_dependencies(test_plugin_load pluginlib hardware_interface)
  add_test(
    NAME test_generate_bus
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
            -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_generate_bus.py -v
  )
endif()

ament_package()
