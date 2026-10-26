#include <gtest/gtest.h>

#include "canopen_hw/canopen_robot_hw.hpp"

TEST(UnitConversion, TicksToRadDefaultAxis) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenRobotHw hw(&shared);

  canopen_hw::AxisFeedback fb0;
  fb0.actual_position = 5308416;
  fb0.is_operational = true;
  shared.UpdateFeedback(0, fb0);
  shared.RecomputeAllOperational();
  hw.ReadFromSharedState();
  EXPECT_GT(hw.joint_position(0), 6.27);
  EXPECT_LT(hw.joint_position(0), 6.29);
}

TEST(UnitConversion, CustomAxisConversion) {
  canopen_hw::SharedState shared(2);
  canopen_hw::CanopenRobotHw hw(&shared);

  canopen_hw::CanopenRobotHw::AxisConversion conv1;
  conv1.counts_per_rev = 1000.0;
  conv1.rated_torque_nm = 10.0;
  conv1.velocity_scale = 2.0;
  conv1.torque_scale = 0.5;
  hw.ConfigureAxisConversion(1, conv1);

  canopen_hw::AxisFeedback fb1;
  fb1.actual_position = 500;   // 0.5 rev -> pi
  fb1.actual_velocity = 500;   // 0.5 rev/s -> pi rad/s, *2 -> 2*pi
  fb1.actual_torque = 1000;    // 100% -> 10Nm, *0.5 -> 5Nm
  shared.UpdateFeedback(1, fb1);
  hw.ReadFromSharedState();

  EXPECT_GT(hw.joint_position(1), 3.13);
  EXPECT_LT(hw.joint_position(1), 3.15);
  EXPECT_GT(hw.joint_velocity(1), 6.27);
  EXPECT_LT(hw.joint_velocity(1), 6.29);
  EXPECT_GT(hw.joint_effort(1), 4.99);
  EXPECT_LT(hw.joint_effort(1), 5.01);
}

TEST(UnitConversion, NotOperationalStillWritesPosition) {
  canopen_hw::SharedState shared(3);
  canopen_hw::CanopenRobotHw hw(&shared);

  // all_operational=false 时仍应写入位置目标，速度/力矩保持 0。
  canopen_hw::AxisCommand cmd_before;
  cmd_before.target_position = 777;
  cmd_before.target_velocity = 123;
  cmd_before.target_torque = 456;
  shared.UpdateCommand(2, cmd_before);

  canopen_hw::AxisFeedback fb0;
  fb0.is_operational = false;
  fb0.is_fault = true;
  shared.UpdateFeedback(0, fb0);
  shared.RecomputeAllOperational();
  hw.ReadFromSharedState();
  hw.SetJointCommand(2, 1.23);
  hw.WriteToSharedState();
  const auto snap = shared.Snapshot();
  EXPECT_NE(snap.commands[2].target_position, 777);
  EXPECT_EQ(snap.commands[2].target_velocity, 0);
  EXPECT_EQ(snap.commands[2].target_torque, 0);
}
