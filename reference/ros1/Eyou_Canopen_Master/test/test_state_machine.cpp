#include <gtest/gtest.h>

#include "canopen_hw/cia402_protocol.hpp"
#include "canopen_hw/cia402_state_machine.hpp"

using canopen_hw::CiA402State;
using canopen_hw::CiA402Protocol;
using canopen_hw::CiA402StateMachine;
using canopen_hw::AxisIntent;
using canopen_hw::kCtrl_DisableOperation;
using canopen_hw::kCtrl_EnableOperation;
using canopen_hw::kCtrl_Bit_Halt;
using canopen_hw::kCtrl_Bit_InterpolationEnable;
using canopen_hw::kCtrl_FaultReset;
using canopen_hw::kCtrl_Shutdown;
using canopen_hw::kMode_IP;
using canopen_hw::kMode_CSP;
using canopen_hw::kMode_CSV;

namespace {

void EnterOperationEnabledAndUnlockCsp(CiA402StateMachine* sm,
                                       int32_t actual,
                                       int32_t target) {
  sm->set_target_mode(kMode_CSP);
  sm->request_enable();
  sm->Update(0x0040, kMode_CSP, actual);
  sm->Update(0x0021, kMode_CSP, actual);
  // 首帧进入 OP：epoch 递增，默认不解锁。
  sm->Update(0x0027, kMode_CSP, actual);
  const uint32_t epoch = sm->arm_epoch();
  sm->SetExternalPositionCommand(target, true, epoch);
  sm->Update(0x0027, kMode_CSP, actual);
}

}  // namespace

TEST(CiA402SM, SwitchOnDisabledSendsShutdown) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();

  sm.Update(0x0040, kMode_CSP, 100);
  EXPECT_EQ(sm.state(), CiA402State::SwitchOnDisabled);
  EXPECT_EQ(sm.controlword(), kCtrl_Shutdown);
}

TEST(CiA402SM, SwitchOnDisabledKeepsDisableVoltageUntilEnabled) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  sm.Update(0x0040, kMode_CSP, 100);
  EXPECT_EQ(sm.state(), CiA402State::SwitchOnDisabled);
  EXPECT_EQ(sm.controlword(), canopen_hw::kCtrl_DisableVoltage);
}

TEST(CiA402SM, ReadyToSwitchOnJumpEnable) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);
  EXPECT_EQ(sm.state(), CiA402State::ReadyToSwitchOn);
  EXPECT_EQ(sm.controlword(), kCtrl_EnableOperation);
}

// 回归测试：Recover 后 tpdo 初始为零，驱动器可能短暂收到 mode=0 导致
// mode_display 归零。状态机不应因此卡死在 ReadyToSwitchOn。
TEST(CiA402SM, ReadyToSwitchOnEnablesEvenIfModeDisplayIsZero) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();

  sm.Update(0x0040, kMode_CSP, 100);
  // mode_display=0（驱动器因收到 mode=0 而清除了模式）
  sm.Update(0x0021, /*mode_display=*/0, 100);
  EXPECT_EQ(sm.state(), CiA402State::ReadyToSwitchOn);
  EXPECT_EQ(sm.controlword(), kCtrl_EnableOperation);
}

TEST(CiA402SM, FirstOperationEnabledStaysLockedUntilValidEpochCommand) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);
  sm.Update(0x0027, kMode_CSP, 12345);

  EXPECT_EQ(sm.state(), CiA402State::OperationEnabled);
  EXPECT_TRUE(sm.is_position_locked());
  EXPECT_FALSE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), 12345);
}

TEST(CiA402SM, EpochMismatchKeepsLocked) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);
  sm.Update(0x0027, kMode_CSP, 100);

  const uint32_t bad_epoch = sm.arm_epoch() + 1;
  sm.SetExternalPositionCommand(100, true, bad_epoch);
  sm.Update(0x0027, kMode_CSP, 100);

  EXPECT_TRUE(sm.is_position_locked());
  EXPECT_FALSE(sm.is_operational());
}

TEST(CiA402SM, RosTargetCloseUnlocksWithValidEpoch) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();

  sm.Update(0x0040, kMode_CSP, 100);
  sm.Update(0x0021, kMode_CSP, 100);
  sm.Update(0x0027, kMode_CSP, 12348);

  sm.SetExternalPositionCommand(12350, true, sm.arm_epoch());
  sm.Update(0x0027, kMode_CSP, 12348);

  EXPECT_FALSE(sm.is_position_locked());
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), 12350);
}

TEST(CiA402SM, GlobalFaultBlocksPreEnableChain) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();
  sm.set_global_fault(true);

  sm.Update(0x0040, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), canopen_hw::kCtrl_DisableVoltage);

  sm.Update(0x0021, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_Shutdown);

  sm.Update(0x0023, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_Shutdown);
}

TEST(CiA402SM, FaultLatchClearStillRequiresExplicitEnableRequest) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();
  sm.set_global_fault(true);

  sm.Update(0x0040, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), canopen_hw::kCtrl_DisableVoltage);

  // 清除故障闩锁前先撤销 enable 请求，验证不会隐式恢复。
  sm.request_disable();
  sm.set_global_fault(false);
  sm.Update(0x0040, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), canopen_hw::kCtrl_DisableVoltage);

  // 只有再次显式 enable 才会推进使能链。
  sm.request_enable();
  sm.Update(0x0040, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_Shutdown);
}

TEST(CiA402SM, FaultResetThreePhaseFlow) {
  CiA402StateMachine sm;
  EnterOperationEnabledAndUnlockCsp(&sm, 12348, 12350);

  sm.set_fault_reset_policy(2, 5, 2);

  // FaultReactionActive: 不应触发复位边沿
  sm.Update(0x000F, kMode_CSP, 12348);
  EXPECT_EQ(sm.state(), CiA402State::FaultReactionActive);
  EXPECT_NE(sm.controlword(), kCtrl_FaultReset);

  // Hold 阶段
  sm.Update(0x0008, kMode_CSP, 12348);  // Hold 1
  sm.Update(0x0008, kMode_CSP, 12348);  // Hold 2
  // SendEdge
  sm.Update(0x0008, kMode_CSP, 12348);
  EXPECT_EQ(sm.controlword(), kCtrl_FaultReset);
  EXPECT_EQ(sm.fault_reset_count(), 1);
}

TEST(CiA402SM, DisableRequestDropsOperationalInOperationEnabled) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSV);

  sm.request_enable();
  sm.Update(0x0040, kMode_CSV, 1000);
  sm.Update(0x0021, kMode_CSV, 1000);
  sm.Update(0x0027, kMode_CSV, 1000);  // 首帧，epoch 产生

  sm.set_ros_target_velocity(500);     // 绑定当前 epoch
  sm.Update(0x0027, kMode_CSV, 1000);  // 次帧解锁
  ASSERT_TRUE(sm.is_operational());
  ASSERT_EQ(sm.safe_target_velocity(), 500);

  // 即使状态字暂时仍是 OperationEnabled，收到 disable 请求后也应立即回落。
  sm.request_disable();
  sm.Update(0x0027, kMode_CSV, 1000);

  EXPECT_EQ(sm.controlword(), kCtrl_DisableOperation);
  EXPECT_FALSE(sm.is_operational());
  EXPECT_EQ(sm.safe_target_velocity(), 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);
}

TEST(CiA402SM, HaltBitFreezesTargetsAndResumeClearsBit) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSV);

  sm.request_enable();
  sm.Update(0x0040, kMode_CSV, 1000);
  sm.Update(0x0021, kMode_CSV, 1000);
  sm.Update(0x0027, kMode_CSV, 1000);  // 首帧

  sm.set_ros_target_velocity(500);
  sm.Update(0x0027, kMode_CSV, 1000);  // 次帧解锁
  ASSERT_TRUE(sm.is_operational());
  ASSERT_EQ(sm.safe_target_velocity(), 500);

  sm.request_halt();
  sm.Update(0x0027, kMode_CSV, 1012);
  EXPECT_EQ(sm.controlword(), static_cast<uint16_t>(kCtrl_EnableOperation | kCtrl_Bit_Halt));
  EXPECT_TRUE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), 1012);
  EXPECT_EQ(sm.safe_target_velocity(), 0);
  EXPECT_EQ(sm.safe_target_torque(), 0);

  sm.set_ros_target_velocity(900);
  sm.Update(0x0027, kMode_CSV, 1020);
  EXPECT_EQ(sm.safe_target(), 1020);
  EXPECT_EQ(sm.safe_target_velocity(), 0);

  sm.request_resume();
  sm.Update(0x0027, kMode_CSV, 1020);
  EXPECT_EQ(sm.controlword(), kCtrl_EnableOperation);
  EXPECT_EQ(sm.safe_target_velocity(), 900);
}

TEST(CiA402SM, IpModeSetsInterpolationEnableBitInOperationEnabled) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_IP);
  sm.request_enable();

  sm.Update(0x0040, kMode_IP, 1000);
  sm.Update(0x0021, kMode_IP, 1000);
  sm.Update(0x0027, kMode_IP, 1000);

  const auto expected =
      static_cast<uint16_t>(kCtrl_EnableOperation | kCtrl_Bit_InterpolationEnable);
  EXPECT_EQ(sm.controlword(), expected);
}

TEST(CiA402Protocol, EnableChainFollowsIntentLevel) {
  CiA402Protocol protocol;
  CiA402Protocol::Input in{};
  in.target_mode = kMode_CSP;
  in.intent = AxisIntent::Run;
  in.actual_position = 100;

  in.statusword = 0x0040;
  auto out = protocol.Process(in);
  EXPECT_EQ(out.controlword, kCtrl_Shutdown);
  EXPECT_FALSE(out.is_operational);

  in.statusword = 0x0021;
  out = protocol.Process(in);
  EXPECT_EQ(out.controlword, kCtrl_EnableOperation);
  EXPECT_FALSE(out.is_operational);
}

TEST(CiA402Protocol, FaultStateNeverSendsFaultReset) {
  CiA402Protocol protocol;
  CiA402Protocol::Input in{};
  in.target_mode = kMode_CSP;
  in.intent = AxisIntent::Run;
  in.statusword = 0x0008;
  in.actual_position = 321;

  const auto out = protocol.Process(in);
  EXPECT_EQ(out.decoded_state, CiA402State::Fault);
  EXPECT_TRUE(out.is_fault);
  EXPECT_EQ(out.controlword, canopen_hw::kCtrl_DisableVoltage);
  EXPECT_EQ(out.safe_target_position, 321);
}

TEST(CiA402Protocol, HaltToRunAdvancesEpochAndRelocks) {
  CiA402Protocol protocol;
  CiA402Protocol::Input in{};
  in.target_mode = kMode_CSP;
  in.statusword = 0x0027;
  in.actual_position = 5000;
  in.intent = AxisIntent::Halt;

  auto out = protocol.Process(in);
  ASSERT_TRUE(out.arm_epoch_advanced);
  const uint32_t first_epoch = out.arm_epoch;
  EXPECT_EQ(out.controlword,
            static_cast<uint16_t>(kCtrl_EnableOperation | kCtrl_Bit_Halt));
  EXPECT_FALSE(out.is_operational);

  in.intent = AxisIntent::Run;
  in.cmd_valid = true;
  in.cmd_arm_epoch = first_epoch;
  in.ros_target_position = 5000;
  out = protocol.Process(in);

  EXPECT_TRUE(out.arm_epoch_advanced);
  EXPECT_GT(out.arm_epoch, first_epoch);
  EXPECT_FALSE(out.is_operational);
  EXPECT_EQ(out.safe_target_position, 5000);
}

TEST(CiA402Protocol, RunRequiresValidEpochBeforeUnlock) {
  CiA402Protocol protocol;
  CiA402Protocol::Input in{};
  in.target_mode = kMode_CSP;
  in.statusword = 0x0027;
  in.actual_position = 1200;
  in.intent = AxisIntent::Run;
  in.ros_target_position = 1200;

  auto out = protocol.Process(in);
  const uint32_t epoch = out.arm_epoch;
  ASSERT_TRUE(out.arm_epoch_advanced);
  EXPECT_FALSE(out.is_operational);

  // cmd_epoch 不匹配时保持锁定。
  in.cmd_valid = true;
  in.cmd_arm_epoch = epoch + 1;
  out = protocol.Process(in);
  EXPECT_FALSE(out.is_operational);
  EXPECT_EQ(out.safe_target_position, 1200);

  // epoch 匹配后允许透传并进入 operational。
  in.cmd_arm_epoch = epoch;
  out = protocol.Process(in);
  EXPECT_TRUE(out.is_operational);
  EXPECT_EQ(out.safe_target_position, 1200);
}

TEST(CiA402Protocol, StaleIntentSequenceFallsBackToDisable) {
  CiA402Protocol protocol;
  protocol.set_max_stale_intent_frames(2);

  CiA402Protocol::Input in{};
  in.target_mode = kMode_CSP;
  in.intent = AxisIntent::Run;
  in.intent_sequence = 42;
  in.actual_position = 10;
  in.statusword = 0x0040;

  auto out = protocol.Process(in);
  EXPECT_EQ(out.controlword, kCtrl_Shutdown);

  out = protocol.Process(in);
  EXPECT_EQ(out.controlword, kCtrl_Shutdown);

  out = protocol.Process(in);
  EXPECT_EQ(out.controlword, canopen_hw::kCtrl_DisableVoltage);

  // intent 序列推进后恢复为新鲜意图。
  in.intent_sequence = 43;
  out = protocol.Process(in);
  EXPECT_EQ(out.controlword, kCtrl_Shutdown);
}
