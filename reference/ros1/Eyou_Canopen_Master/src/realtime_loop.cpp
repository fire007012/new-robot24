#include "canopen_hw/realtime_loop.hpp"

#include <cmath>
#include <thread>

#include "canopen_hw/logging.hpp"

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <time.h>
#endif

namespace canopen_hw {

RealtimeLoop::RealtimeLoop(const Config& config) : config_(config) {}

void RealtimeLoop::Run(std::function<bool()> tick) {
  if (!tick) return;

#ifdef __linux__
  if (config_.use_fifo) {
    struct sched_param sp{};
    sp.sched_priority = config_.fifo_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      CANOPEN_LOG_WARN("RealtimeLoop: failed to set SCHED_FIFO (need root or CAP_SYS_NICE), falling back to CFS");
    }
  }

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);

  const int64_t period_ns = config_.period.count();

  while (true) {
    // 1. 先执行业务逻辑
    if (!tick()) break;

    // 2. 计算下一次绝对唤醒时间（uint64_t 防止加法溢出）
    uint64_t total_ns = static_cast<uint64_t>(next.tv_nsec) + period_ns;
    next.tv_sec += total_ns / 1000000000ULL;
    next.tv_nsec = total_ns % 1000000000ULL;

    // 3. 绝对时间休眠（若 tick 超时，next 已是过去时间，立即返回实现追赶）
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    // 4. 计算 Jitter
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    const int64_t wake_ns = (now.tv_sec - next.tv_sec) * 1000000000LL +
                            (now.tv_nsec - next.tv_nsec);
    const int64_t jitter_us = std::abs(wake_ns) / 1000;

    // 5. 更新统计（增量法计算平均值，防止长时间运行 sum 溢出）
    stats_.iterations++;
    stats_.last_jitter_us = jitter_us;
    if (jitter_us > stats_.max_jitter_us) {
      stats_.max_jitter_us = jitter_us;
    }
    stats_.avg_jitter_us += (jitter_us - stats_.avg_jitter_us) /
                            static_cast<int64_t>(stats_.iterations);
  }

#else
  // 非 Linux：用 sleep_until 保证绝对时间，消除累积漂移
  auto next = std::chrono::steady_clock::now();

  while (true) {
    // 1. 先执行业务逻辑
    if (!tick()) break;

    // 2. 推进绝对时间
    next += config_.period;

    // 3. 绝对时间休眠
    std::this_thread::sleep_until(next);

    // 4. 计算 Jitter
    auto now = std::chrono::steady_clock::now();
    const auto wake_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now - next).count();
    const int64_t jitter_us = std::abs(wake_ns) / 1000;

    // 5. 更新统计
    stats_.iterations++;
    stats_.last_jitter_us = jitter_us;
    if (jitter_us > stats_.max_jitter_us) {
      stats_.max_jitter_us = jitter_us;
    }
    stats_.avg_jitter_us += (jitter_us - stats_.avg_jitter_us) /
                            static_cast<int64_t>(stats_.iterations);
  }
#endif
}

LoopStats RealtimeLoop::stats() const {
  return stats_;
}

}  // namespace canopen_hw
