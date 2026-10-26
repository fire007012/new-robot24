#include <gtest/gtest.h>

#include <climits>
#include <fstream>
#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/cia402_state_machine.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/shared_state.hpp"

using canopen_hw::CiA402State;
using canopen_hw::CiA402StateMachine;
using canopen_hw::kCtrl_DisableVoltage;
using canopen_hw::kCtrl_FaultReset;
using canopen_hw::kMode_CSP;

// --- axis_count 边界 ---

TEST(Boundary, AxisCountZeroNormalizesToOne) {
  canopen_hw::SharedState shared(6);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 0;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";

  canopen_hw::CanopenMaster master(cfg, &shared);
  EXPECT_EQ(master.config().axis_count, 1u);
}

TEST(Boundary, AxisCountExceedsMaxClamped) {
  // 构造时传入超大值，内部应限制在 kMaxAxisCount
  canopen_hw::SharedState shared(100);
  EXPECT_EQ(shared.axis_count(), canopen_hw::SharedState::kMaxAxisCount);

  // 验证 RecomputeAllOperational 不越界
  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  // all_operational 应为 false (默认 feedback 未设置 is_operational)
  EXPECT_FALSE(snap.all_operational);
}

TEST(Boundary, AxisCountZeroClampedToOne) {
  canopen_hw::SharedState shared(0);
  EXPECT_EQ(shared.axis_count(), 1u);
}

// --- node_id 边界 ---

TEST(Boundary, NodeIdZeroDefaultFill) {
  canopen_hw::SharedState shared(6);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 3;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";
  cfg.joints.assign(3, canopen_hw::CanopenMasterConfig::JointConfig{});

  canopen_hw::CanopenMaster master(cfg, &shared);
  ASSERT_EQ(master.config().joints.size(), 3u);
  EXPECT_EQ(master.config().joints[0].node_id, 1u);
  EXPECT_EQ(master.config().joints[1].node_id, 2u);
  EXPECT_EQ(master.config().joints[2].node_id, 3u);
}

TEST(Boundary, NodeIdOutOfRange) {
  // node_id: 0 应被拒绝
  {
    const std::string path = "/tmp/joints_test_node0.yaml";
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 0\n"
           "    counts_per_rev: 1000\n";
    ofs.close();

    std::string error;
    canopen_hw::CanopenMasterConfig cfg;
    const bool ok =
        canopen_hw::LoadJointsYaml(path, &error, &cfg);
    EXPECT_FALSE(ok);
    EXPECT_NE(error.find("invalid node_id"), std::string::npos);
  }

  // node_id: 128 应被拒绝
  {
    const std::string path = "/tmp/joints_test_node128.yaml";
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 128\n"
           "    counts_per_rev: 1000\n";
    ofs.close();

    std::string error;
    canopen_hw::CanopenMasterConfig cfg;
    const bool ok =
        canopen_hw::LoadJointsYaml(path, &error, &cfg);
    EXPECT_FALSE(ok);
    EXPECT_NE(error.find("invalid node_id"), std::string::npos);
  }
}

// --- int32 极值位置 ---

TEST(Boundary, Int32ExtremePositionSameValueUnlocksWithValidEpoch) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();

  sm.Update(0x0040, kMode_CSP, INT32_MAX);
  sm.Update(0x0021, kMode_CSP, INT32_MAX);
  sm.Update(0x0027, kMode_CSP, INT32_MAX);  // 首帧，epoch 产生
  sm.SetExternalPositionCommand(INT32_MAX, true, sm.arm_epoch());
  sm.Update(0x0027, kMode_CSP, INT32_MAX);

  EXPECT_EQ(sm.safe_target(), INT32_MAX);
  EXPECT_FALSE(sm.is_position_locked());
  EXPECT_TRUE(sm.is_operational());
}

TEST(Boundary, Int32ExtremeAbsDiffStaysLockedWithoutReasonableTarget) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();
  sm.Update(0x0040, kMode_CSP, INT32_MIN);
  sm.Update(0x0021, kMode_CSP, INT32_MIN);
  sm.Update(0x0027, kMode_CSP, INT32_MIN);  // 首帧，epoch 产生
  sm.SetExternalPositionCommand(INT32_MAX, true, sm.arm_epoch());
  sm.Update(0x0027, kMode_CSP, INT32_MIN);

  EXPECT_TRUE(sm.is_position_locked());
  EXPECT_FALSE(sm.is_operational());
  EXPECT_EQ(sm.safe_target(), INT32_MIN);
}

// --- 故障复位次数耗尽 ---

TEST(Boundary, FaultResetMaxAttemptsReached) {
  CiA402StateMachine sm;
  sm.set_target_mode(kMode_CSP);
  sm.request_enable();
  sm.set_fault_reset_policy(1, 2, 2);  // hold=1, wait=2, max=2

  // 推进到 OperationEnabled 并解锁
  sm.Update(0x0040, kMode_CSP, 100);  // SwitchOnDisabled
  sm.Update(0x0021, kMode_CSP, 100);  // ReadyToSwitchOn
  sm.Update(0x0027, kMode_CSP, 100);  // OperationEnabled first frame
  sm.SetExternalPositionCommand(100, true, sm.arm_epoch());
  sm.Update(0x0027, kMode_CSP, 100);
  ASSERT_TRUE(sm.is_operational());

  // 进入 Fault (statusword=0x0008)
  sm.Update(0x0008, kMode_CSP, 100);
  ASSERT_EQ(sm.state(), CiA402State::Fault);

  // 第一次复位尝试
  sm.Update(0x0008, kMode_CSP, 100);  // SendEdge
  EXPECT_EQ(sm.controlword(), kCtrl_FaultReset);
  EXPECT_EQ(sm.fault_reset_count(), 1);

  // WaitRecovery: wait=2 周期
  sm.Update(0x0008, kMode_CSP, 100);
  sm.Update(0x0008, kMode_CSP, 100);

  // 第二次复位尝试
  sm.Update(0x0008, kMode_CSP, 100);
  sm.Update(0x0008, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_FaultReset);
  EXPECT_EQ(sm.fault_reset_count(), 2);

  sm.Update(0x0008, kMode_CSP, 100);
  sm.Update(0x0008, kMode_CSP, 100);

  sm.Update(0x0008, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);
  EXPECT_EQ(sm.fault_reset_count(), 2);

  sm.Update(0x0008, kMode_CSP, 100);
  sm.Update(0x0008, kMode_CSP, 100);
  EXPECT_EQ(sm.controlword(), kCtrl_DisableVoltage);
  EXPECT_EQ(sm.fault_reset_count(), 2);
}
