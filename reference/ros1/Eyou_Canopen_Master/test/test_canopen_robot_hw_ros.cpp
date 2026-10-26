#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <ros/time.h>

#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {
namespace {

TEST(CanopenRobotHwRos, CommandSyncSequenceForcesGuardResetAndPositionResync) {
  SharedState shared(1);
  CanopenRobotHw hw(&shared);
  const std::vector<std::string> joint_names{"joint_0"};
  CanopenRobotHwRos hw_ros(&hw, joint_names);

  AxisFeedback fb;
  fb.actual_position = 1000;
  fb.is_operational = true;
  fb.is_fault = false;
  fb.heartbeat_lost = false;
  fb.arm_epoch = 1;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();

  const ros::Time now(0.0);
  const ros::Duration period(0.01);

  for (int i = 0; i < 25; ++i) {
    hw_ros.read(now, period);
    hw_ros.write(now, period);
  }
  const SharedSnapshot ready = shared.Snapshot();
  ASSERT_TRUE(ready.commands[0].valid);

  fb.actual_position = 2222;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();
  shared.AdvanceCommandSyncSequence();

  hw_ros.read(now, period);
  hw_ros.write(now, period);

  const SharedSnapshot after_sync = shared.Snapshot();
  EXPECT_FALSE(after_sync.commands[0].valid);
  const long long diff =
      static_cast<long long>(after_sync.commands[0].target_position) -
      static_cast<long long>(fb.actual_position);
  EXPECT_LE(std::llabs(diff), 1LL);
}

TEST(CanopenRobotHwRos, StaleControllerZeroDoesNotBecomeReadyAfterGuardExpires) {
  SharedState shared(1);
  CanopenRobotHw hw(&shared);
  const std::vector<std::string> joint_names{"joint_0"};
  CanopenRobotHwRos hw_ros(&hw, joint_names);

  auto* pos_iface = hw_ros.get<hardware_interface::PositionJointInterface>();
  ASSERT_NE(pos_iface, nullptr);
  auto pos_handle = pos_iface->getHandle("joint_0");

  AxisFeedback fb;
  fb.actual_position = 200000;
  fb.is_operational = true;
  fb.is_fault = false;
  fb.heartbeat_lost = false;
  fb.arm_epoch = 1;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();

  const ros::Time now(0.0);
  const ros::Duration period(0.01);

  for (int i = 0; i < 30; ++i) {
    hw_ros.read(now, period);
    pos_handle.setCommand(0.0);
    hw_ros.write(now, period);
  }

  const SharedSnapshot snap = shared.Snapshot();
  EXPECT_FALSE(snap.commands[0].valid);
  const long long diff =
      static_cast<long long>(snap.commands[0].target_position) -
      static_cast<long long>(fb.actual_position);
  EXPECT_LE(std::llabs(diff), 1LL);
}

TEST(CanopenRobotHwRos, ControllerMustAlignBeforeCommandBecomesReady) {
  SharedState shared(1);
  CanopenRobotHw hw(&shared);
  const std::vector<std::string> joint_names{"joint_0"};
  CanopenRobotHwRos hw_ros(&hw, joint_names);

  auto* pos_iface = hw_ros.get<hardware_interface::PositionJointInterface>();
  ASSERT_NE(pos_iface, nullptr);
  auto pos_handle = pos_iface->getHandle("joint_0");

  AxisFeedback fb;
  fb.actual_position = 200000;
  fb.is_operational = true;
  fb.is_fault = false;
  fb.heartbeat_lost = false;
  fb.arm_epoch = 1;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();

  const ros::Time now(0.0);
  const ros::Duration period(0.01);

  for (int i = 0; i < 30; ++i) {
    hw_ros.read(now, period);
    pos_handle.setCommand(0.0);
    hw_ros.write(now, period);
  }
  ASSERT_FALSE(shared.Snapshot().commands[0].valid);

  hw_ros.read(now, period);
  pos_handle.setCommand(hw.joint_position(0));
  hw_ros.write(now, period);

  hw_ros.read(now, period);
  pos_handle.setCommand(hw.joint_position(0));
  hw_ros.write(now, period);

  EXPECT_TRUE(shared.Snapshot().commands[0].valid);
}

}  // namespace
}  // namespace canopen_hw
