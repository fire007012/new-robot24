#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

class CanopenMaster;

// 单轴诊断快照。
struct AxisDiagnostics {
  std::string name;
  uint8_t node_id = 0;
  CiA402State state = CiA402State::NotReadyToSwitchOn;
  bool is_operational = false;
  bool is_fault = false;
  bool heartbeat_lost = false;
  uint16_t last_emcy_eec = 0;

  // 健康计数。
  uint32_t emcy_count = 0;
  uint32_t heartbeat_lost_count = 0;
  uint32_t heartbeat_recovered_count = 0;
  uint32_t fault_reset_attempts = 0;
  uint32_t boot_retries = 0;
};

// 整机诊断快照。
struct SystemDiagnostics {
  bool master_running = false;
  std::size_t axis_count = 0;
  bool all_operational = false;
  std::vector<AxisDiagnostics> axes;
};

// 诊断数据收集器。
// 纯数据收集，不依赖 ROS。ROS 适配层可将 SystemDiagnostics 转为
// diagnostic_msgs/DiagnosticArray。
class DiagnosticsCollector {
 public:
  explicit DiagnosticsCollector(CanopenMaster* master);

  // 收集一次完整的诊断快照。
  SystemDiagnostics Collect() const;

 private:
  CanopenMaster* master_ = nullptr;
};

}  // namespace canopen_hw
