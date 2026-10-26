#include <gtest/gtest.h>

#include "canopen_hw/cia402_state_machine.hpp"

using canopen_hw::CiA402State;
using canopen_hw::CiA402StateMachine;
using canopen_hw::kCtrl_DisableVoltage;
using canopen_hw::kCtrl_FaultReset;
using canopen_hw::kCtrl_Shutdown;
using canopen_hw::kMode_CSP;

// 验证 ResetFaultCounter 可以从 PermanentFault 恢复。
TEST(ManualControl, ResetFaultCounterRecoversPermanentFault) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.set_fault_reset_policy(1, 1, 1);  // 1 次即进入 PermanentFault

  // 进入 FAULT 状态。
  sm.Update(0x0008, kMode_CSP, 0);
  EXPECT_EQ(sm.state(), CiA402State::Fault);

  // HoldLow 阶段。
  sm.Update(0x0008, kMode_CSP, 0);
  // SendEdge 阶段: fault_reset_count 变为 1。
  sm.Update(0x0008, kMode_CSP, 0);
  // WaitRecovery 超时后回到 HoldLow，但 count 已达上限。
  sm.Update(0x0008, kMode_CSP, 0);
  // 再次进入 HoldLow → 检测到 count >= max → PermanentFault。
  sm.Update(0x0008, kMode_CSP, 0);

  // 此时 controlword 应为 DisableVoltage（PermanentFault 不再自动复位）。
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);
  EXPECT_GE(sm.fault_reset_count(), 1);

  // 手动重置故障计数器。
  sm.ResetFaultCounter();
  EXPECT_EQ(sm.fault_reset_count(), 0);

  // 下一次 Update 应重新进入自动复位流程。
  sm.Update(0x0008, kMode_CSP, 0);
  EXPECT_EQ(sm.state(), CiA402State::Fault);
  // 不再是 PermanentFault，应该在 HoldLow 阶段。
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);

  // 继续推进到 SendEdge。
  sm.Update(0x0008, kMode_CSP, 0);
  EXPECT_EQ(sm.controlword(), kCtrl_FaultReset);
}

// 验证 request_disable 阻止自动使能推进。
TEST(ManualControl, DisableRequestPreventsEnable) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);

  // 默认 enable_requested_ = false，SwitchOnDisabled 发 DisableVoltage。
  sm.Update(0x0040, kMode_CSP, 0);
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);

  // 请求 disable 后，SwitchOnDisabled 应发 DisableVoltage。
  sm.request_disable();
  sm.Update(0x0040, kMode_CSP, 0);
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);

  // 重新 enable 后恢复。
  sm.request_enable();
  sm.Update(0x0040, kMode_CSP, 0);
  EXPECT_EQ(sm.controlword(), kCtrl_Shutdown);
}
