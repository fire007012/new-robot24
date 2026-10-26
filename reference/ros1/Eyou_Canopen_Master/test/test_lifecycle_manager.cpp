#include <gtest/gtest.h>

#include <string>

#include "canopen_hw/lifecycle_manager.hpp"

using canopen_hw::CanopenMasterConfig;
using canopen_hw::LifecycleManager;
using canopen_hw::LifecycleState;

namespace {

CanopenMasterConfig MakeMinimalConfig() {
  CanopenMasterConfig config;
  config.axis_count = 1;
  config.can_interface = "canable0";
  config.master_dcf_path = "/tmp/definitely_missing_master_for_lifecycle_test.dcf";
  config.joints.resize(1);
  config.joints[0].name = "joint_1";
  config.joints[0].node_id = 1;
  return config;
}

}  // namespace

// LifecycleManager 的状态跃迁测试。
// 不依赖真实 CAN 总线：InitMotors/Init 在当前环境应失败，
// 重点验证 guard 与失败语义。

TEST(LifecycleManager, StartsUnconfigured) {
  LifecycleManager lm;
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_EQ(lm.master(), nullptr);
  EXPECT_EQ(lm.robot_hw(), nullptr);
  EXPECT_EQ(lm.shared_state(), nullptr);
  EXPECT_FALSE(lm.ever_initialized());
  EXPECT_FALSE(lm.require_init());
  EXPECT_FALSE(lm.halted());
}

TEST(LifecycleManager, HaltRejectsWhenUnconfigured) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Halt());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, ResumeRejectsWhenUnconfigured) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Resume());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, RecoverRejectsWhenUnconfigured) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Recover());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, StopCommunicationRejectsWhenUnconfigured) {
  LifecycleManager lm;
  std::string detail;
  EXPECT_FALSE(lm.StopCommunication(&detail));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_FALSE(lm.require_init());
  EXPECT_EQ(detail, "invalid lifecycle state");
}

TEST(LifecycleManager, ShutdownFromUnconfiguredIsNoop) {
  LifecycleManager lm;
  EXPECT_TRUE(lm.Shutdown());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_FALSE(lm.require_init());
  EXPECT_FALSE(lm.halted());
}

TEST(LifecycleManager, ConfigureRejectsInvalidAxisCount) {
  LifecycleManager lm;
  CanopenMasterConfig config;
  config.axis_count = 0;
  EXPECT_FALSE(lm.Configure(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, ConfigureRejectsExcessiveAxisCount) {
  LifecycleManager lm;
  CanopenMasterConfig config;
  config.axis_count = canopen_hw::SharedState::kMaxAxisCount + 1;
  EXPECT_FALSE(lm.Configure(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, ConfigureValidConfigEntersConfigured) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  EXPECT_TRUE(lm.Configure(config));
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
  EXPECT_NE(lm.master(), nullptr);
  EXPECT_NE(lm.robot_hw(), nullptr);
  EXPECT_NE(lm.shared_state(), nullptr);
  EXPECT_FALSE(lm.ever_initialized());
  EXPECT_FALSE(lm.require_init());
  EXPECT_FALSE(lm.halted());
}

TEST(LifecycleManager, StopCommunicationFromConfiguredMarksRequireInit) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();
  std::string detail;

  ASSERT_TRUE(lm.Configure(config));
  EXPECT_TRUE(lm.StopCommunication(&detail));
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
  EXPECT_FALSE(lm.require_init());
  EXPECT_FALSE(lm.halted());
  EXPECT_TRUE(detail.empty());
}

TEST(LifecycleManager, RecoverRejectsBeforeFirstInitMotors) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  ASSERT_TRUE(lm.Configure(config));
  EXPECT_FALSE(lm.Recover());
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
}

TEST(LifecycleManager, ResumeRejectsInConfigured) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  ASSERT_TRUE(lm.Configure(config));
  EXPECT_FALSE(lm.Resume());
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
}

TEST(LifecycleManager, RecoverRejectsAfterStopCommunicationUntilInit) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();
  std::string detail;

  ASSERT_TRUE(lm.Configure(config));
  ASSERT_TRUE(lm.StopCommunication(&detail));
  ASSERT_FALSE(lm.require_init());

  EXPECT_FALSE(lm.Recover());
  EXPECT_EQ(lm.state(), LifecycleState::Configured);

  // 由于测试配置使用不存在的 DCF，InitMotors 失败且状态保持 Configured。
  EXPECT_FALSE(lm.InitMotors());
  EXPECT_FALSE(lm.require_init());
}

TEST(LifecycleManager, InitMotorsFailureKeepsConfigured) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  ASSERT_TRUE(lm.Configure(config));
  EXPECT_FALSE(lm.InitMotors());
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
  EXPECT_FALSE(lm.ever_initialized());
  EXPECT_NE(lm.master(), nullptr);
  EXPECT_FALSE(lm.require_init());
  EXPECT_FALSE(lm.halted());
}

TEST(LifecycleManager, InitCompatibilityFailureRollsBackToUnconfigured) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();

  EXPECT_FALSE(lm.Init(config));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_EQ(lm.master(), nullptr);
  EXPECT_EQ(lm.robot_hw(), nullptr);
  EXPECT_EQ(lm.shared_state(), nullptr);
  EXPECT_FALSE(lm.ever_initialized());
  EXPECT_FALSE(lm.require_init());
  EXPECT_FALSE(lm.halted());
}

TEST(LifecycleManager, ShutdownClearsRequireInitFlag) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();
  std::string detail;

  ASSERT_TRUE(lm.Configure(config));
  ASSERT_TRUE(lm.StopCommunication(&detail));
  ASSERT_FALSE(lm.require_init());

  EXPECT_TRUE(lm.Shutdown());
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
  EXPECT_FALSE(lm.require_init());
  EXPECT_FALSE(lm.halted());
}

TEST(LifecycleManager, InitRejectsMissingJointsFile) {
  LifecycleManager lm;
  EXPECT_FALSE(lm.Init("/nonexistent/master.dcf", "/nonexistent/joints.yaml"));
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);
}

TEST(LifecycleManager, TransitionMatrixGuardsWithoutMotorInit) {
  LifecycleManager lm;
  auto config = MakeMinimalConfig();
  std::string detail;

  // Unconfigured: 全部运行态相关操作应拒绝。
  EXPECT_FALSE(lm.InitMotors());
  EXPECT_FALSE(lm.Halt());
  EXPECT_FALSE(lm.Resume());
  EXPECT_FALSE(lm.Recover(&detail));
  EXPECT_EQ(detail, "recover requires Active state");

  detail.clear();
  EXPECT_FALSE(lm.StopCommunication(&detail));
  EXPECT_EQ(detail, "invalid lifecycle state");
  EXPECT_EQ(lm.state(), LifecycleState::Unconfigured);

  // Configured: 仍未进入 Active，运行态操作应继续拒绝。
  ASSERT_TRUE(lm.Configure(config));
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
  EXPECT_FALSE(lm.Halt());
  EXPECT_FALSE(lm.Resume());

  detail.clear();
  EXPECT_FALSE(lm.Recover(&detail));
  EXPECT_EQ(detail, "recover requires Active state");

  // Configured 下允许 stop communication，状态仍保持 Configured。
  detail.clear();
  EXPECT_TRUE(lm.StopCommunication(&detail));
  EXPECT_TRUE(detail.empty());
  EXPECT_EQ(lm.state(), LifecycleState::Configured);
  EXPECT_FALSE(lm.require_init());
}
