#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include "canopen_hw/shared_state.hpp"

TEST(SharedStateConcurrent, MultiThreadReadWriteStress) {
  constexpr std::size_t kTestAxisCount = 6;
  canopen_hw::SharedState shared(kTestAxisCount);

  constexpr int kWriterThreads = 4;
  constexpr int kReaderThreads = 2;
  constexpr int kIterations = 100000;

  std::atomic<bool> start{false};
  std::vector<std::thread> threads;
  threads.reserve(kWriterThreads + kReaderThreads);

  for (int t = 0; t < kWriterThreads; ++t) {
    threads.emplace_back([&shared, &start, t]() {
      while (start.load(std::memory_order_acquire) == false) {
      }
      for (int i = 0; i < kIterations; ++i) {
        const std::size_t axis =
            static_cast<std::size_t>((i + t) % kTestAxisCount);

        canopen_hw::AxisFeedback fb;
        fb.actual_position = static_cast<int32_t>(i + t * 1000);
        fb.actual_velocity = static_cast<int32_t>(-i);
        fb.actual_torque = static_cast<int16_t>((i + t) % 1000);
        fb.is_operational = ((i % 3) > 0);
        fb.is_fault = ((i % 11) == 0);
        fb.arm_epoch = static_cast<uint32_t>((i % 13) + 1);
        shared.UpdateFeedback(axis, fb);

        canopen_hw::AxisCommand cmd;
        cmd.target_position = static_cast<int32_t>(i * 2 + t);
        cmd.valid = ((i % 7) > 0);
        cmd.arm_epoch = static_cast<uint32_t>((i % 13) + 1);
        shared.UpdateCommand(axis, cmd);

        if ((i % 64) == 0) {
          shared.RecomputeAllOperational();
          shared.SetGlobalFault((i % 128) == 0);
          shared.SetAllAxesHaltedByFault((i % 256) == 0);
        }
      }
    });
  }

  for (int t = 0; t < kReaderThreads; ++t) {
    threads.emplace_back([&shared, &start]() {
      while (start.load(std::memory_order_acquire) == false) {
      }
      for (int i = 0; i < kIterations; ++i) {
        const auto snap = shared.Snapshot();
        for (std::size_t axis = 0; axis < kTestAxisCount; ++axis) {
          // 基础不变量：读取结果应保持可解释范围，避免“撕裂式”异常值。
          EXPECT_LE(snap.feedback[axis].actual_torque, 1000);
          EXPECT_GE(snap.feedback[axis].actual_torque, 0);
          EXPECT_LE(snap.commands[axis].arm_epoch, 13u);
        }
      }
    });
  }

  start.store(true, std::memory_order_release);
  for (auto& th : threads) {
    th.join();
  }

  const auto final_snap = shared.Snapshot();
  EXPECT_EQ(final_snap.feedback.size(), kTestAxisCount);
  EXPECT_EQ(final_snap.commands.size(), kTestAxisCount);
}
