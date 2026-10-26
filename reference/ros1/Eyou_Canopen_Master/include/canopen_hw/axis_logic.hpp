#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>

#include "canopen_hw/bus_io.hpp"
#include "canopen_hw/cia402_protocol.hpp"
#include "canopen_hw/health_counters.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// 单轴核心逻辑，与 Lely 解耦。
// 持有状态机、反馈缓存、健康计数器，通过 BusIO 接口写总线。
class AxisLogic {
 public:
  AxisLogic(std::size_t axis_index, BusIO* bus_io, SharedState* shared_state);

  // RPDO 周期处理入口：读取反馈 → 推进状态机 → 写安全目标到总线。
  void ProcessRpdo(uint16_t statusword, int32_t actual_position,
                   int32_t actual_velocity, int16_t actual_torque,
                   int8_t mode_display);

  // EMCY 处理。
  void ProcessEmcy(uint16_t eec, uint8_t er);

  // 心跳丢失/恢复处理。
  void ProcessHeartbeat(bool lost);

  // 状态机配置。
  void Configure(int32_t position_lock_threshold, int max_fault_resets,
                 int fault_reset_hold_cycles);

  // 命令接口。
  void SetRosTarget(int32_t target_position);
  void SetRosTargetVelocity(int32_t target_velocity);
  void SetRosTargetTorque(int16_t target_torque);
  void SetTargetMode(int8_t mode);
  void SetExternalCommand(const AxisCommand& command);
  void SetGlobalFault(bool global_fault);
  void RequestEnable();
  void RequestDisable();
  void RequestHalt();
  void RequestResume();
  void ResetFault();

  // 查询。
  CiA402State feedback_state() const;
  int8_t target_mode() const;
  const HealthCounters& health() const { return health_; }
  HealthCounters& mutable_health() { return health_; }

 private:
  void SetIntent(AxisIntent intent);
  void PublishSnapshot();

  std::size_t axis_index_;
  BusIO* bus_io_;
  SharedState* shared_state_;
  uint64_t last_intent_sequence_ = 0;
  AxisIntent current_intent_ = AxisIntent::Disable;
  bool global_fault_ = false;
  int8_t target_mode_ = kMode_CSP;
  int32_t ros_target_position_ = 0;
  int32_t ros_target_velocity_ = 0;
  int16_t ros_target_torque_ = 0;
  bool cmd_valid_ = false;
  uint32_t cmd_arm_epoch_ = 0;
  uint32_t arm_epoch_cache_ = 0;

  mutable std::mutex mtx_;
  CiA402Protocol protocol_;
  AxisFeedback feedback_cache_{};
  AxisSafeCommand safe_command_cache_{};
  HealthCounters health_;
};

}  // namespace canopen_hw
