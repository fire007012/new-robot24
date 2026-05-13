cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  controller_manager_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

add_action_files(
  FILES
  ExecuteArmGoal.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    control_msgs
    controller_manager_msgs
    geometry_msgs
    message_runtime
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/arm_goal_executor_node.py
  scripts/gripper_cmd_node.py
  scripts/send_execute_arm_goal.py
  scripts/servo_twist_frame_bridge_node.py
  scripts/wrist_roll_service_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_execute_arm_goal_contract.py)
endif()

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
