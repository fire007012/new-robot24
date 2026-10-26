#include <gtest/gtest.h>

#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/cia402_state_machine.hpp"
#include "canopen_hw/shared_state.hpp"

using namespace canopen_hw;

// --- 状态机多模式测试 ---

class StateMachineMultiMode : public ::testing::Test {
 protected:
  CiA402StateMachine sm;

  void DriveToOperational(int8_t mode) {
    sm.set_target_mode(mode);
    sm.request_enable();
    sm.set_position_lock_threshold(50000);

    sm.Update(kState_SwitchOnDisabled, mode, 1000);
    sm.Update(kState_ReadyToSwitchOn | (1u << 9), mode, 1000);
    sm.Update(kState_OperationEnabled, mode, 1000);  // 首帧，epoch 产生

    if (mode == kMode_CSP) {
      sm.SetExternalPositionCommand(1000, true, sm.arm_epoch());
    } else if (mode == kMode_CSV) {
      sm.set_ros_target_velocity(500);
    } else if (mode == kMode_CST) {
      sm.set_ros_target_torque(200);
    }

    sm.Update(kState_OperationEnabled, mode, 1000);  // 次帧门控放行
  }
};

TEST_F(StateMachineMultiMode, CSVModeEnablesCorrectly) {
  sm.set_target_mode(kMode_CSV);
  sm.request_enable();

  sm.Update(kState_SwitchOnDisabled, kMode_CSV, 1000);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CSV, 1000);
  sm.Update(kState_OperationEnabled, kMode_CSV, 1000);
  EXPECT_FALSE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 0);

  sm.set_ros_target_velocity(500);
  sm.Update(kState_OperationEnabled, kMode_CSV, 1000);
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 500);
}

TEST_F(StateMachineMultiMode, CSVModePassesVelocityAfterUnlock) {
  sm.set_target_mode(kMode_CSV);
  sm.request_enable();

  sm.Update(kState_SwitchOnDisabled, kMode_CSV, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CSV, 0);
  sm.Update(kState_OperationEnabled, kMode_CSV, 0);  // 首帧
  EXPECT_EQ(sm.safe_target_velocity(), 0);

  sm.set_ros_target_velocity(500);
  sm.Update(kState_OperationEnabled, kMode_CSV, 0);  // 次帧
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 500);
}

TEST_F(StateMachineMultiMode, CSTModePassesTorqueAfterUnlock) {
  sm.set_target_mode(kMode_CST);
  sm.request_enable();

  sm.Update(kState_SwitchOnDisabled, kMode_CST, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CST, 0);
  sm.Update(kState_OperationEnabled, kMode_CST, 0);  // 首帧
  EXPECT_EQ(sm.safe_target_torque(), 0);

  sm.set_ros_target_torque(200);
  sm.Update(kState_OperationEnabled, kMode_CST, 0);  // 次帧
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_torque(), 200);
}

TEST_F(StateMachineMultiMode, VelocityZeroedOnFault) {
  sm.set_target_mode(kMode_CSV);
  sm.request_enable();

  sm.Update(kState_SwitchOnDisabled, kMode_CSV, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CSV, 0);
  sm.Update(kState_OperationEnabled, kMode_CSV, 0);
  sm.set_ros_target_velocity(500);
  sm.Update(kState_OperationEnabled, kMode_CSV, 0);
  EXPECT_EQ(sm.safe_target_velocity(), 500);

  sm.Update(kState_Fault, kMode_CSV, 0);
  EXPECT_EQ(sm.safe_target_velocity(), 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);
  EXPECT_FALSE(sm.is_operational());
}

TEST_F(StateMachineMultiMode, TorqueZeroedOnDisable) {
  sm.set_target_mode(kMode_CST);
  sm.request_enable();

  sm.Update(kState_SwitchOnDisabled, kMode_CST, 0);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CST, 0);
  sm.Update(kState_OperationEnabled, kMode_CST, 0);
  sm.set_ros_target_torque(300);
  sm.Update(kState_OperationEnabled, kMode_CST, 0);
  EXPECT_EQ(sm.safe_target_torque(), 300);

  sm.request_disable();
  sm.Update(kState_SwitchedOn, kMode_CST, 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);
  EXPECT_EQ(sm.safe_target_velocity(), 0);
}

TEST_F(StateMachineMultiMode, SafeModeReflectsTargetMode) {
  sm.set_target_mode(kMode_IP);
  EXPECT_EQ(sm.safe_mode_of_operation(), kMode_IP);
  sm.set_target_mode(kMode_CSV);
  EXPECT_EQ(sm.safe_mode_of_operation(), kMode_CSV);
  sm.set_target_mode(kMode_CST);
  EXPECT_EQ(sm.safe_mode_of_operation(), kMode_CST);
}

TEST_F(StateMachineMultiMode, CSVUnlocksAtNonZeroPosition) {
  sm.set_target_mode(kMode_CSV);
  sm.request_enable();
  sm.set_position_lock_threshold(500);

  sm.Update(kState_SwitchOnDisabled, kMode_CSV, 100000);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CSV, 100000);
  sm.Update(kState_OperationEnabled, kMode_CSV, 100000);
  sm.set_ros_target_velocity(100);
  sm.Update(kState_OperationEnabled, kMode_CSV, 100000);
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 100);
}

TEST_F(StateMachineMultiMode, CSTUnlocksAtNonZeroPosition) {
  sm.set_target_mode(kMode_CST);
  sm.request_enable();
  sm.set_position_lock_threshold(500);

  sm.Update(kState_SwitchOnDisabled, kMode_CST, 80000);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CST, 80000);
  sm.Update(kState_OperationEnabled, kMode_CST, 80000);
  sm.set_ros_target_torque(50);
  sm.Update(kState_OperationEnabled, kMode_CST, 80000);
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_torque(), 50);
}

TEST_F(StateMachineMultiMode, EpochMismatchBlocksCSPUnlock) {
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();
  sm.Update(kState_SwitchOnDisabled, kMode_CSP, 1000);
  sm.Update(kState_ReadyToSwitchOn | (1u << 9), kMode_CSP, 1000);
  sm.Update(kState_OperationEnabled, kMode_CSP, 1000);

  sm.SetExternalPositionCommand(1000, true, sm.arm_epoch() + 1);
  sm.Update(kState_OperationEnabled, kMode_CSP, 1000);

  EXPECT_FALSE(sm.is_operational());
  EXPECT_TRUE(sm.is_position_locked());
}

// --- RobotHw 多模式命令流测试 ---

class RobotHwMultiMode : public ::testing::Test {
 protected:
  SharedState state{2};
  CanopenRobotHw hw{&state};

  void MakeAllOperational() {
    for (std::size_t i = 0; i < 2; ++i) {
      AxisFeedback fb;
      fb.is_operational = true;
      fb.state = CiA402State::OperationEnabled;
      fb.arm_epoch = 3;
      state.UpdateFeedback(i, fb);
    }
    state.RecomputeAllOperational();
    hw.ReadFromSharedState();
  }
};

TEST_F(RobotHwMultiMode, VelocityCommandWrittenToSharedState) {
  MakeAllOperational();
  hw.SetJointMode(0, kMode_CSV);
  hw.SetJointVelocityCommand(0, 1.0);
  hw.SetCommandReady(0, true);
  hw.SetCommandEpoch(0, 3);
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(0, &cmd));
  EXPECT_EQ(cmd.mode_of_operation, kMode_CSV);
  EXPECT_NE(cmd.target_velocity, 0);
  EXPECT_TRUE(cmd.valid);
  EXPECT_EQ(cmd.arm_epoch, 3u);
}

TEST_F(RobotHwMultiMode, VelocityCommandUsesPerAxisOperationalGate) {
  AxisFeedback fb0;
  fb0.is_operational = true;
  fb0.state = CiA402State::OperationEnabled;
  fb0.arm_epoch = 3;
  state.UpdateFeedback(0, fb0);

  AxisFeedback fb1;
  fb1.is_operational = false;
  fb1.state = CiA402State::OperationEnabled;
  fb1.arm_epoch = 3;
  state.UpdateFeedback(1, fb1);
  state.RecomputeAllOperational();
  hw.ReadFromSharedState();
  ASSERT_FALSE(hw.all_operational());

  hw.SetJointMode(0, kMode_CSV);
  hw.SetJointMode(1, kMode_CSV);
  hw.SetJointVelocityCommand(0, 1.0);
  hw.SetJointVelocityCommand(1, 1.0);
  hw.SetCommandReady(0, true);
  hw.SetCommandReady(1, true);
  hw.SetCommandEpoch(0, 3);
  hw.SetCommandEpoch(1, 3);
  hw.WriteToSharedState();

  AxisCommand cmd0;
  AxisCommand cmd1;
  ASSERT_TRUE(state.GetCommand(0, &cmd0));
  ASSERT_TRUE(state.GetCommand(1, &cmd1));
  EXPECT_NE(cmd0.target_velocity, 0);
  EXPECT_EQ(cmd1.target_velocity, 0);
}

TEST_F(RobotHwMultiMode, TorqueCommandWrittenToSharedState) {
  MakeAllOperational();
  hw.SetJointMode(1, kMode_CST);
  hw.SetJointTorqueCommand(1, 3.0);
  hw.SetCommandReady(1, true);
  hw.SetCommandEpoch(1, 3);
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(1, &cmd));
  EXPECT_EQ(cmd.mode_of_operation, kMode_CST);
  EXPECT_NE(cmd.target_torque, 0);
  EXPECT_TRUE(cmd.valid);
  EXPECT_EQ(cmd.arm_epoch, 3u);
}

TEST_F(RobotHwMultiMode, DefaultModeIsCSP) {
  MakeAllOperational();
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(0, &cmd));
  EXPECT_EQ(cmd.mode_of_operation, kMode_CSP);
}

TEST_F(RobotHwMultiMode, NotOperationalZerosVelocityButKeepsMode) {
  // Don't call MakeAllOperational.
  hw.SetJointMode(0, kMode_CSV);
  hw.SetJointVelocityCommand(0, 1.0);
  hw.WriteToSharedState();

  AxisCommand cmd;
  ASSERT_TRUE(state.GetCommand(0, &cmd));
  EXPECT_EQ(cmd.mode_of_operation, kMode_CSV);
  EXPECT_EQ(cmd.target_velocity, 0);
  EXPECT_EQ(cmd.target_torque, 0);
  EXPECT_FALSE(cmd.valid);
}

TEST_F(RobotHwMultiMode, InvalidAxisIgnored) {
  hw.SetJointMode(99, kMode_CSV);
  hw.SetJointVelocityCommand(99, 1.0);
  hw.SetJointTorqueCommand(99, 1.0);
  hw.SetCommandReady(99, true);
  hw.SetCommandEpoch(99, 9);
  // No crash expected.
}
