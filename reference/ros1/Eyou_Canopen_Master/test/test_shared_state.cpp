#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "canopen_hw/shared_state.hpp"

TEST(SharedState, BasicUpdateAndSnapshot) {
  canopen_hw::SharedState shared(1);

  canopen_hw::AxisFeedback fb;
  fb.actual_position = 123456;
  fb.actual_velocity = -345;
  fb.is_operational = true;
  fb.arm_epoch = 9;

  canopen_hw::AxisCommand cmd;
  cmd.target_position = 223344;
  cmd.valid = true;
  cmd.arm_epoch = 9;

  shared.UpdateFeedback(0, fb);
  shared.UpdateCommand(0, cmd);
  shared.SetGlobalFault(true);
  shared.SetAllAxesHaltedByFault(true);
  shared.RecomputeAllOperational();

  const canopen_hw::SharedSnapshot snap = shared.Snapshot();
  EXPECT_EQ(snap.feedback[0].actual_position, 123456);
  EXPECT_EQ(snap.feedback[0].actual_velocity, -345);
  EXPECT_TRUE(snap.feedback[0].is_operational);
  EXPECT_EQ(snap.feedback[0].arm_epoch, 9u);
  EXPECT_EQ(snap.commands[0].target_position, 223344);
  EXPECT_TRUE(snap.commands[0].valid);
  EXPECT_EQ(snap.commands[0].arm_epoch, 9u);
  EXPECT_TRUE(snap.all_operational);
  EXPECT_TRUE(snap.global_fault);
  EXPECT_TRUE(snap.all_axes_halted_by_fault);
}

TEST(SharedState, OutOfRangeIgnored) {
  canopen_hw::SharedState shared(1);

  canopen_hw::AxisCommand cmd;
  cmd.target_position = 223344;
  cmd.valid = true;
  cmd.arm_epoch = 5;
  shared.UpdateCommand(0, cmd);

  // 越界写入应被静默忽略, 不影响已有数据。
  cmd.target_position = 999;
  cmd.valid = false;
  cmd.arm_epoch = 0;
  shared.UpdateCommand(99, cmd);
  const canopen_hw::SharedSnapshot snap2 = shared.Snapshot();
  EXPECT_EQ(snap2.commands[0].target_position, 223344);
  EXPECT_TRUE(snap2.commands[0].valid);
  EXPECT_EQ(snap2.commands[0].arm_epoch, 5u);
}

TEST(SharedState, RecomputeAllOperationalTrueWhenAllOperationalAndNoFault) {
  canopen_hw::SharedState shared(6);

  for (std::size_t i = 0; i < shared.axis_count(); ++i) {
    canopen_hw::AxisFeedback fb;
    fb.is_operational = true;
    fb.is_fault = false;
    shared.UpdateFeedback(i, fb);
  }

  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  EXPECT_TRUE(snap.all_operational);
}

TEST(SharedState, RecomputeAllOperationalFalseWhenAnyAxisFault) {
  canopen_hw::SharedState shared(6);

  for (std::size_t i = 0; i < shared.axis_count(); ++i) {
    canopen_hw::AxisFeedback fb;
    fb.is_operational = true;
    fb.is_fault = false;
    shared.UpdateFeedback(i, fb);
  }

  canopen_hw::AxisFeedback fault_fb;
  fault_fb.is_operational = true;
  fault_fb.is_fault = true;
  fault_fb.heartbeat_lost = true;
  shared.UpdateFeedback(3, fault_fb);

  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  EXPECT_FALSE(snap.all_operational);
  EXPECT_TRUE(snap.feedback[3].heartbeat_lost);
}

TEST(SharedState, RecomputeUsesConfiguredAxisCount) {
  canopen_hw::SharedState shared(1);

  canopen_hw::AxisFeedback axis0;
  axis0.is_operational = true;
  axis0.is_fault = false;
  shared.UpdateFeedback(0, axis0);

  // 其余轴不存在，不应影响只配置 1 轴时的汇总结果。
  shared.RecomputeAllOperational();
  const auto snap = shared.Snapshot();
  EXPECT_TRUE(snap.all_operational);
}

TEST(SharedState, FaultLatchSetAndClear) {
  canopen_hw::SharedState shared(2);

  EXPECT_FALSE(shared.GetGlobalFault());
  EXPECT_FALSE(shared.GetAllAxesHaltedByFault());

  shared.SetGlobalFault(true);
  shared.SetAllAxesHaltedByFault(true);
  EXPECT_TRUE(shared.GetGlobalFault());
  EXPECT_TRUE(shared.GetAllAxesHaltedByFault());

  shared.SetGlobalFault(false);
  shared.SetAllAxesHaltedByFault(false);
  EXPECT_FALSE(shared.GetGlobalFault());
  EXPECT_FALSE(shared.GetAllAxesHaltedByFault());
}

TEST(SharedState, WaitForStateChangeTimeout) {
  canopen_hw::SharedState shared(1);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  EXPECT_FALSE(shared.WaitForStateChange(deadline));
}

TEST(SharedState, WaitForStateChangeNotifiedByFeedbackUpdate) {
  canopen_hw::SharedState shared(1);
  std::thread notifier([&shared]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    canopen_hw::AxisFeedback fb;
    fb.actual_position = 42;
    shared.UpdateFeedback(0, fb);
  });

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  EXPECT_TRUE(shared.WaitForStateChange(deadline));
  notifier.join();
}

TEST(SharedState, IntentChannelSetGetAndSnapshot) {
  canopen_hw::SharedState shared(2);
  EXPECT_EQ(shared.GetAxisIntent(0), canopen_hw::AxisIntent::Disable);
  EXPECT_EQ(shared.GetAxisIntent(1), canopen_hw::AxisIntent::Disable);
  EXPECT_EQ(shared.intent_sequence(), 0u);
  EXPECT_EQ(shared.command_sync_sequence(), 0u);

  shared.SetAxisIntent(0, canopen_hw::AxisIntent::Run);
  shared.SetAxisIntent(1, canopen_hw::AxisIntent::Halt);
  shared.AdvanceIntentSequence();

  const auto snap = shared.Snapshot();
  ASSERT_EQ(snap.intents.size(), 2u);
  EXPECT_EQ(snap.intents[0], canopen_hw::AxisIntent::Run);
  EXPECT_EQ(snap.intents[1], canopen_hw::AxisIntent::Halt);
  EXPECT_EQ(snap.intent_sequence, 1u);
  EXPECT_EQ(snap.command_sync_sequence, 0u);
}

TEST(SharedState, IntentOutOfRangeFallsBackToDisable) {
  canopen_hw::SharedState shared(1);
  shared.SetAxisIntent(99, canopen_hw::AxisIntent::Run);
  EXPECT_EQ(shared.GetAxisIntent(99), canopen_hw::AxisIntent::Disable);
}

TEST(SharedState, CommandSyncSequenceAdvanceAndSnapshot) {
  canopen_hw::SharedState shared(1);
  EXPECT_EQ(shared.command_sync_sequence(), 0u);

  shared.AdvanceCommandSyncSequence();
  shared.AdvanceCommandSyncSequence();

  EXPECT_EQ(shared.command_sync_sequence(), 2u);
  const auto snap = shared.Snapshot();
  EXPECT_EQ(snap.command_sync_sequence, 2u);
}
