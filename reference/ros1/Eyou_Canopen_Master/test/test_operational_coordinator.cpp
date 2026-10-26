#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "canopen_hw/operational_coordinator.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {
namespace {

struct FakeMaster {
  bool start_ok = true;
  bool running = false;
  bool reset_ok = true;
  bool graceful_ok = true;
  int start_calls = 0;
  int reset_calls = 0;
  int graceful_calls = 0;
  int stop_calls = 0;
};

OperationalCoordinator::MasterOps MakeMasterOps(FakeMaster* master) {
  OperationalCoordinator::MasterOps ops;
  ops.start = [master]() {
    ++master->start_calls;
    if (!master->start_ok) {
      return false;
    }
    master->running = true;
    return true;
  };
  ops.running = [master]() { return master->running; };
  ops.reset_all_faults = [master](std::string* detail) {
    ++master->reset_calls;
    if (!master->reset_ok) {
      if (detail) {
        *detail = "reset failed";
      }
      return false;
    }
    return true;
  };
  ops.graceful_shutdown = [master](std::string* detail) {
    ++master->graceful_calls;
    if (!master->graceful_ok) {
      if (detail) {
        *detail = "graceful timeout";
      }
      return false;
    }
    return true;
  };
  ops.stop = [master]() {
    ++master->stop_calls;
    master->running = false;
  };
  return ops;
}

TEST(OperationalCoordinator, TransitionMatrixFollows0324ImportPath) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();

  auto r = coordinator.RequestInit();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Armed);
  EXPECT_EQ(shared.Snapshot().command_sync_sequence, 1u);

  coordinator.ComputeIntents();
  EXPECT_EQ(shared.GetAxisIntent(0), AxisIntent::Halt);

  r = coordinator.RequestEnable();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Armed);

  r = coordinator.RequestRelease();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Running);

  coordinator.ComputeIntents();
  EXPECT_EQ(shared.GetAxisIntent(0), AxisIntent::Run);

  r = coordinator.RequestHalt();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Armed);

  r = coordinator.RequestDisable();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Standby);

  r = coordinator.RequestEnable();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Armed);

  r = coordinator.RequestShutdown();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Configured);
  EXPECT_EQ(shared.Snapshot().command_sync_sequence, 2u);
  EXPECT_EQ(fake.stop_calls, 1);
}

TEST(OperationalCoordinator, InvalidTransitionsAreRejected) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();

  auto r = coordinator.RequestRelease();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Configured);

  r = coordinator.RequestRecover();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Configured);

  r = coordinator.RequestHalt();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Configured);

  r = coordinator.RequestDisable();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Configured);
}

TEST(OperationalCoordinator, DisableFromRunningAndArmedToStandby) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();
  ASSERT_TRUE(coordinator.RequestInit().ok);
  ASSERT_TRUE(coordinator.RequestRelease().ok);
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Running);

  auto r = coordinator.RequestDisable();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Standby);

  ASSERT_TRUE(coordinator.RequestEnable().ok);
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Armed);
  r = coordinator.RequestDisable();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Standby);
}

TEST(OperationalCoordinator, AutoFaultDowngradeAndRecoverToStandby) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();
  ASSERT_TRUE(coordinator.RequestInit().ok);
  ASSERT_TRUE(coordinator.RequestRelease().ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Running);

  AxisFeedback fb;
  fb.is_fault = true;
  shared.UpdateFeedback(0, fb);
  coordinator.UpdateFromFeedback();
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Faulted);
  EXPECT_TRUE(shared.GetGlobalFault());
  EXPECT_TRUE(shared.GetAllAxesHaltedByFault());

  // 模拟底层 fault 已清除，再发 recover。
  fb.is_fault = false;
  shared.UpdateFeedback(0, fb);

  auto r = coordinator.RequestRecover();
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Standby);
  EXPECT_FALSE(shared.GetGlobalFault());
  EXPECT_FALSE(shared.GetAllAxesHaltedByFault());
  EXPECT_EQ(shared.Snapshot().command_sync_sequence, 2u);
}

TEST(OperationalCoordinator, RecoverFailureKeepsFaulted) {
  SharedState shared(1);
  FakeMaster fake;
  fake.reset_ok = false;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();
  ASSERT_TRUE(coordinator.RequestInit().ok);
  ASSERT_TRUE(coordinator.RequestRelease().ok);

  AxisFeedback fb;
  fb.heartbeat_lost = true;
  shared.UpdateFeedback(0, fb);
  coordinator.UpdateFromFeedback();
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Faulted);

  const auto r = coordinator.RequestRecover();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.message, "reset failed");
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Faulted);
}

TEST(OperationalCoordinator, RecoverFailsWhenFeedbackStaysFaulted) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();
  ASSERT_TRUE(coordinator.RequestInit().ok);
  ASSERT_TRUE(coordinator.RequestRelease().ok);

  AxisFeedback fb;
  fb.is_fault = true;
  shared.UpdateFeedback(0, fb);
  coordinator.UpdateFromFeedback();
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Faulted);

  const auto r = coordinator.RequestRecover();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Faulted);
  EXPECT_NE(r.message.find("recover timeout"), std::string::npos);
}

TEST(OperationalCoordinator, ReleaseRejectedWhenAxisUnhealthy) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();

  ASSERT_TRUE(coordinator.RequestInit().ok);
  ASSERT_TRUE(coordinator.RequestEnable().ok);
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Armed);

  AxisFeedback fb;
  fb.is_fault = true;
  shared.UpdateFeedback(0, fb);

  const auto r = coordinator.RequestRelease();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Armed);
  EXPECT_NE(r.message.find("fault"), std::string::npos);
}

TEST(OperationalCoordinator, EnableRejectedWhenGlobalFaultLatched) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();

  ASSERT_TRUE(coordinator.RequestInit().ok);
  ASSERT_TRUE(coordinator.RequestDisable().ok);
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Standby);

  shared.SetGlobalFault(true);
  const auto r = coordinator.RequestEnable();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Standby);
  EXPECT_NE(r.message.find("global fault"), std::string::npos);
}

TEST(OperationalCoordinator, RecoverThenEnableReleaseSucceedsAfterFeedbackHealthy) {
  SharedState shared(1);
  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();
  ASSERT_TRUE(coordinator.RequestInit().ok);
  ASSERT_TRUE(coordinator.RequestRelease().ok);
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Running);

  AxisFeedback fb;
  fb.is_fault = true;
  shared.UpdateFeedback(0, fb);
  coordinator.UpdateFromFeedback();
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Faulted);

  std::thread clear_fault([&shared]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    AxisFeedback healthy;
    healthy.is_fault = false;
    healthy.heartbeat_lost = false;
    shared.UpdateFeedback(0, healthy);
  });

  const auto recover = coordinator.RequestRecover();
  clear_fault.join();
  ASSERT_TRUE(recover.ok) << recover.message;
  ASSERT_EQ(coordinator.mode(), SystemOpMode::Standby);

  auto enable = coordinator.RequestEnable();
  EXPECT_TRUE(enable.ok) << enable.message;
  auto release = coordinator.RequestRelease();
  EXPECT_TRUE(release.ok) << release.message;
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Running);
}

TEST(OperationalCoordinator, InitSafetyCheckFaultFallbackToFaulted) {
  SharedState shared(1);
  AxisFeedback fb;
  fb.is_fault = true;
  shared.UpdateFeedback(0, fb);

  FakeMaster fake;
  OperationalCoordinator coordinator(MakeMasterOps(&fake), &shared, 1);
  coordinator.SetConfigured();

  const auto r = coordinator.RequestInit();
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(coordinator.mode(), SystemOpMode::Faulted);
  EXPECT_EQ(r.message, "safety check failed: fault present after init");
}

}  // namespace
}  // namespace canopen_hw
