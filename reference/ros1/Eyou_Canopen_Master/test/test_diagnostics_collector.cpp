#include <gtest/gtest.h>

#include "canopen_hw/diagnostics_collector.hpp"
#include "canopen_hw/shared_state.hpp"

using canopen_hw::DiagnosticsCollector;
using canopen_hw::SystemDiagnostics;

TEST(DiagnosticsCollector, NullMasterReturnsEmpty) {
  DiagnosticsCollector collector(nullptr);
  SystemDiagnostics diag = collector.Collect();
  EXPECT_FALSE(diag.master_running);
  EXPECT_EQ(diag.axis_count, 0u);
  EXPECT_FALSE(diag.all_operational);
  EXPECT_TRUE(diag.axes.empty());
}

// 通过 SharedState 直接注入反馈来测试诊断收集。
// 不需要真实 CAN 硬件或 CanopenMaster。

TEST(DiagnosticsCollector, SharedStateDirectFeedback) {
  canopen_hw::SharedState state(2);

  canopen_hw::AxisFeedback fb0;
  fb0.state = canopen_hw::CiA402State::OperationEnabled;
  fb0.is_operational = true;
  fb0.is_fault = false;
  fb0.heartbeat_lost = false;
  fb0.last_emcy_eec = 0;
  state.UpdateFeedback(0, fb0);

  canopen_hw::AxisFeedback fb1;
  fb1.state = canopen_hw::CiA402State::Fault;
  fb1.is_operational = false;
  fb1.is_fault = true;
  fb1.heartbeat_lost = true;
  fb1.last_emcy_eec = 0x1234;
  state.UpdateFeedback(1, fb1);

  state.RecomputeAllOperational();

  auto snap = state.Snapshot();
  EXPECT_EQ(snap.feedback.size(), 2u);
  EXPECT_TRUE(snap.feedback[0].is_operational);
  EXPECT_FALSE(snap.feedback[1].is_operational);
  EXPECT_TRUE(snap.feedback[1].is_fault);
  EXPECT_EQ(snap.feedback[1].last_emcy_eec, 0x1234);
  EXPECT_FALSE(snap.all_operational);
}

TEST(AxisDiagnostics, DefaultValues) {
  canopen_hw::AxisDiagnostics ad;
  EXPECT_TRUE(ad.name.empty());
  EXPECT_EQ(ad.node_id, 0);
  EXPECT_EQ(ad.state, canopen_hw::CiA402State::NotReadyToSwitchOn);
  EXPECT_FALSE(ad.is_operational);
  EXPECT_FALSE(ad.is_fault);
  EXPECT_FALSE(ad.heartbeat_lost);
  EXPECT_EQ(ad.last_emcy_eec, 0);
  EXPECT_EQ(ad.emcy_count, 0u);
  EXPECT_EQ(ad.heartbeat_lost_count, 0u);
  EXPECT_EQ(ad.fault_reset_attempts, 0u);
}
