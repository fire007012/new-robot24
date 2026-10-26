#include <gtest/gtest.h>

#include "canopen_hw/canopen_robot_hw.hpp"

TEST(RobotHw, ReadTicksToRad) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenRobotHw hw(&shared);

  canopen_hw::AxisFeedback fb;
  fb.actual_position = 5308416;  // 1 rev
  fb.actual_velocity = 5308416;  // 1 rev/s
  fb.actual_torque = 500;        // 50%
  fb.is_operational = true;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();

  hw.ReadFromSharedState();
  const double pos = hw.joint_position(0);
  EXPECT_GT(pos, 6.27);
  EXPECT_LT(pos, 6.29);
  EXPECT_TRUE(hw.all_operational());
}

TEST(RobotHw, WriteRadToTicks) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenRobotHw hw(&shared);

  canopen_hw::AxisFeedback fb;
  fb.is_operational = true;
  shared.UpdateFeedback(0, fb);
  shared.RecomputeAllOperational();
  hw.ReadFromSharedState();

  hw.SetJointCommand(0, 3.14159265358979323846);
  hw.WriteToSharedState();
  const canopen_hw::SharedSnapshot snap = shared.Snapshot();
  EXPECT_GT(snap.commands[0].target_position, 2600000);
  EXPECT_LT(snap.commands[0].target_position, 2700000);
}

TEST(RobotHw, NullSharedStateUsesZeroAxis) {
  canopen_hw::CanopenRobotHw hw(nullptr);
  EXPECT_EQ(hw.axis_count(), 0u);

  // 空 shared_state 下 read/write 应为空操作，不发生崩溃。
  hw.ReadFromSharedState();
  hw.WriteToSharedState();
  EXPECT_FALSE(hw.all_operational());
}

TEST(RobotHw, WritePositionWhileNotOperationalZerosVelocityAndTorque) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenRobotHw hw(&shared);

  // 默认 all_operational=false（尚未进入可运行态）。
  hw.SetJointCommand(0, 1.0);
  hw.SetJointVelocityCommand(0, 2.0);
  hw.SetJointTorqueCommand(0, 3.0);
  hw.WriteToSharedState();

  const canopen_hw::SharedSnapshot snap = shared.Snapshot();
  // 未就绪阶段位置指令应锁定到反馈位置（默认 0）。
  EXPECT_EQ(snap.commands[0].target_position, 0);
  EXPECT_EQ(snap.commands[0].target_velocity, 0);
  EXPECT_EQ(snap.commands[0].target_torque, 0);
}

TEST(RobotHw, CommandSyncSequenceMirrorsSharedStateSnapshot) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenRobotHw hw(&shared);
  EXPECT_EQ(hw.command_sync_sequence(), 0u);

  shared.AdvanceCommandSyncSequence();
  hw.ReadFromSharedState();
  EXPECT_EQ(hw.command_sync_sequence(), 1u);

  shared.AdvanceCommandSyncSequence();
  shared.AdvanceCommandSyncSequence();
  hw.ReadFromSharedState();
  EXPECT_EQ(hw.command_sync_sequence(), 3u);
}
