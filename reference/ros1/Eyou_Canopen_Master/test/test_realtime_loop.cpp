#include "canopen_hw/realtime_loop.hpp"

#include <atomic>

#include <gtest/gtest.h>

namespace canopen_hw {
namespace {

TEST(RealtimeLoop, TickCalledExpectedTimes) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  int count = 0;
  loop.Run([&]() -> bool {
    return ++count < 10;
  });

  EXPECT_EQ(count, 10);
}

TEST(RealtimeLoop, TickReturnFalseStopsLoop) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  loop.Run([&]() -> bool { return false; });

  // tick 先执行后立即退出，不经过 sleep，iterations 为 0
  EXPECT_EQ(loop.stats().iterations, 0u);
}

TEST(RealtimeLoop, StatsReportReasonableValues) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  int count = 0;
  loop.Run([&]() -> bool { return ++count < 5; });

  // tick 执行 5 次（count 1..4 返回 true，count 5 返回 false）
  // iterations 在 sleep 后递增，最后一次 tick 返回 false 不经过 sleep
  auto s = loop.stats();
  EXPECT_EQ(s.iterations, 4u);
  EXPECT_GE(s.avg_jitter_us, 0);
  EXPECT_GE(s.max_jitter_us, 0);
}

TEST(RealtimeLoop, NullTickReturnsImmediately) {
  RealtimeLoop::Config cfg;
  cfg.period = std::chrono::milliseconds(1);

  RealtimeLoop loop(cfg);
  loop.Run(nullptr);

  EXPECT_EQ(loop.stats().iterations, 0u);
}

}  // namespace
}  // namespace canopen_hw
