#pragma once

#include <chrono>
#include <cstdint>
#include <functional>

namespace canopen_hw {

// 周期循环统计。
struct LoopStats {
  int64_t max_jitter_us = 0;    // 最大抖动（微秒）
  int64_t avg_jitter_us = 0;    // 平均抖动（微秒）
  int64_t last_jitter_us = 0;   // 最近一次抖动（微秒）
  uint64_t iterations = 0;      // 已执行周期数
};

// 周期性实时循环。
//
// 默认使用 clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) 做绝对时间等待，
// 消除 sleep_for 的累积漂移。可选启用 SCHED_FIFO 降低调度抖动。
// 非 Linux 平台回退到 sleep_for。
class RealtimeLoop {
 public:
  struct Config {
    std::chrono::nanoseconds period{std::chrono::milliseconds(10)};
    bool use_fifo = false;
    int fifo_priority = 49;
  };

  explicit RealtimeLoop(const Config& config);

  // 执行循环：每周期调用 tick()，tick 返回 false 时退出。
  void Run(std::function<bool()> tick);

  LoopStats stats() const;

 private:
  Config config_;
  LoopStats stats_;
};

}  // namespace canopen_hw
