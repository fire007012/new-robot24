#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "canopen_hw/canopen_master.hpp"

TEST(MasterConfig, ZeroAxisNormalized) {
  canopen_hw::SharedState shared(6);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 0;  // 构造时会被归一化为至少 1 轴。
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";
  cfg.joints.clear();

  canopen_hw::CanopenMaster master(cfg, &shared);

  const auto& normalized = master.config();
  EXPECT_EQ(normalized.axis_count, 1u);
  ASSERT_EQ(normalized.joints.size(), 1u);
  EXPECT_EQ(normalized.joints[0].node_id, 1u);
  EXPECT_FALSE(normalized.joints[0].verify_pdo_mapping);
  EXPECT_EQ(normalized.joints[0].position_lock_threshold, 15000);
  EXPECT_EQ(normalized.joints[0].max_fault_resets, 3);
  EXPECT_EQ(normalized.joints[0].fault_reset_hold_cycles, 5);
}

TEST(MasterConfig, NotRunningAfterConstruction) {
  canopen_hw::SharedState shared(6);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 0;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";
  cfg.joints.clear();

  canopen_hw::CanopenMaster master(cfg, &shared);

  EXPECT_FALSE(master.running());
  EXPECT_EQ(master.axis_count(), 0u);
}

TEST(MasterConfig, ResetAllFaultsRejectsWhenNotRunning) {
  canopen_hw::SharedState shared(2);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 2;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";
  cfg.joints.clear();

  canopen_hw::CanopenMaster master(cfg, &shared);

  std::string detail;
  EXPECT_FALSE(master.ResetAllFaults(&detail));
  EXPECT_EQ(detail, "master not running");
}

TEST(MasterConfig, WaitForAllSdoIdleSucceedsWithoutDrivers) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 1;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";

  canopen_hw::CanopenMaster master(cfg, &shared);

  std::vector<std::size_t> pending;
  EXPECT_TRUE(master.WaitForAllSdoIdle(std::chrono::milliseconds(10), &pending));
  EXPECT_TRUE(pending.empty());
}

TEST(MasterConfig, WaitForSdoIdleRejectsMissingAxisDriver) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 1;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";

  canopen_hw::CanopenMaster master(cfg, &shared);

  EXPECT_FALSE(master.WaitForSdoIdle(0, std::chrono::milliseconds(10)));
}

TEST(MasterConfig, WaitForAllStartupCompleteSucceedsWithoutDrivers) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 1;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";

  canopen_hw::CanopenMaster master(cfg, &shared);

  std::vector<std::size_t> pending;
  EXPECT_TRUE(
      master.WaitForAllStartupComplete(std::chrono::milliseconds(10), &pending));
  EXPECT_TRUE(pending.empty());
}

TEST(MasterConfig, WaitForStartupCompleteRejectsMissingAxisDriver) {
  canopen_hw::SharedState shared(1);
  canopen_hw::CanopenMasterConfig cfg;
  cfg.axis_count = 1;
  cfg.master_node_id = 127;
  cfg.can_interface = "canable0";

  canopen_hw::CanopenMaster master(cfg, &shared);

  EXPECT_FALSE(master.WaitForStartupComplete(0, std::chrono::milliseconds(10)));
}
