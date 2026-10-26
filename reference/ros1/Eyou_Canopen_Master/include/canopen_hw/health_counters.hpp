#pragma once

#include <atomic>
#include <cstdint>

namespace canopen_hw {

// 每轴运行时健康计数器。
// 所有字段使用 atomic，允许 Lely 回调线程递增、ROS/监控线程读取。
struct HealthCounters {
  std::atomic<uint32_t> pdo_verify_ok{0};
  std::atomic<uint32_t> pdo_verify_fail{0};
  std::atomic<uint32_t> pdo_verify_timeout{0};
  std::atomic<uint32_t> heartbeat_lost{0};
  std::atomic<uint32_t> heartbeat_recovered{0};
  std::atomic<uint32_t> fault_reset_attempts{0};
  std::atomic<uint32_t> emcy_count{0};
  std::atomic<uint32_t> boot_retries{0};

  // 不可拷贝（atomic 成员），但可 reset。
  HealthCounters() = default;
  HealthCounters(const HealthCounters&) = delete;
  HealthCounters& operator=(const HealthCounters&) = delete;

  void Reset() {
    pdo_verify_ok.store(0);
    pdo_verify_fail.store(0);
    pdo_verify_timeout.store(0);
    heartbeat_lost.store(0);
    heartbeat_recovered.store(0);
    fault_reset_attempts.store(0);
    emcy_count.store(0);
    boot_retries.store(0);
  }
};

}  // namespace canopen_hw
