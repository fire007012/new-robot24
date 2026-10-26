#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <vector>

#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

// 线程间共享的单轴反馈快照(由 Lely 线程刷新)。
struct AxisFeedback {
  int32_t actual_position = 0;
  int32_t actual_velocity = 0;
  int16_t actual_torque = 0;
  uint16_t statusword = 0;
  int8_t mode_display = 0;
  CiA402State state = CiA402State::NotReadyToSwitchOn;
  bool is_operational = false;
  bool is_fault = false;
  bool heartbeat_lost = false;
  uint16_t last_emcy_eec = 0;
  uint32_t arm_epoch = 0;  // 当前使能会话号（0 为无效）。
};

// 线程间共享的单轴命令(由上层线程写入)。
struct AxisCommand {
  int32_t target_position = 0;
  int32_t target_velocity = 0;
  int16_t target_torque = 0;
  int8_t mode_of_operation = kMode_CSP;  // 默认 CSP。
  bool valid = false;                     // 命令源是否已完成重同步。
  uint32_t arm_epoch = 0;                 // 目标所属使能会话号（0 为无效）。
};

// 状态机过滤后的安全目标(由 Lely 线程写入)。
// 与 AxisCommand 分离，避免 Lely 线程覆盖上层线程写入的用户期望位置。
struct AxisSafeCommand {
  int32_t safe_target_position = 0;
  int32_t safe_target_velocity = 0;
  int16_t safe_target_torque = 0;
  int8_t safe_mode_of_operation = kMode_CSP;
};

// 协调层下发到每轴状态机的运行意图（电平语义）。
enum class AxisIntent : uint8_t {
  Disable,
  Enable,
  Halt,
  Run,
};

// 给调用者返回的快照结构:
// - read() 一次锁内拷贝即可拿到全轴一致视图
// - 调用方后续使用不需要持锁
struct SharedSnapshot {
  std::vector<AxisFeedback> feedback;
  std::vector<AxisCommand> commands;
  std::vector<AxisSafeCommand> safe_commands;
  std::vector<AxisIntent> intents;
  uint64_t intent_sequence = 0;
  uint64_t command_sync_sequence = 0;
  bool all_operational = false;
  bool global_fault = false;
  bool all_axes_halted_by_fault = false;
};

class SharedState {
 public:
  // 安全上限：构造时 axis_count 不得超过此值。
  static constexpr std::size_t kMaxAxisCount = 16;

  // 构造时确定轴数，内部 vector 一次性分配，运行期大小不变。
  // axis_count 会被限制在 [1, kMaxAxisCount]。
  explicit SharedState(std::size_t axis_count = 6);

  // 轴数（构造后不变）。
  std::size_t axis_count() const { return axis_count_; }

  // Lely 线程: 更新某轴反馈信息。
  void UpdateFeedback(std::size_t axis_index, const AxisFeedback& feedback);

  // 上层线程: 更新某轴命令。
  void UpdateCommand(std::size_t axis_index, const AxisCommand& command);

  // 任意线程: 读取某轴命令。越界返回 false。
  bool GetCommand(std::size_t axis_index, AxisCommand* out) const;

  // Lely 线程: 更新某轴状态机过滤后的安全目标位置。
  void UpdateSafeCommand(std::size_t axis_index,
                         const AxisSafeCommand& safe_command);

  // 协调层: 下发每轴 intent（电平语义）。
  void SetAxisIntent(std::size_t axis_index, AxisIntent intent);
  AxisIntent GetAxisIntent(std::size_t axis_index) const;
  void AdvanceIntentSequence();
  uint64_t intent_sequence() const;
  // 命令重同步序列：用于显式通知上层“必须重新对齐命令源”。
  void AdvanceCommandSyncSequence();
  uint64_t command_sync_sequence() const;

  // 由 Lely 线程在每个 SYNC/RPDO 更新后调用，汇总全轴状态。
  void RecomputeAllOperational();

  // 全局故障闩锁（任一轴 fault 可置位，统一恢复后清除）。
  void SetGlobalFault(bool fault);
  bool GetGlobalFault() const;

  // 全轴因故障被连带停机标志。
  void SetAllAxesHaltedByFault(bool halted);
  bool GetAllAxesHaltedByFault() const;

  // 任意线程: 获取完整快照。
  SharedSnapshot Snapshot() const;

  // 阻塞等待反馈状态发生变化（由 UpdateFeedback 通知唤醒）。
  // 返回 true 表示在 deadline 前被唤醒，false 表示超时。
  bool WaitForStateChange(std::chrono::steady_clock::time_point deadline);

 private:
  bool IsValidAxis(std::size_t axis_index) const;

  const std::size_t axis_count_;

  mutable std::mutex mtx_;
  std::condition_variable state_cv_;  // 由 UpdateFeedback() 通知。
  std::vector<AxisFeedback> feedback_;
  std::vector<AxisCommand> commands_;
  std::vector<AxisSafeCommand> safe_commands_;
  std::vector<AxisIntent> intents_;
  bool all_operational_ = false;
  bool global_fault_ = false;
  bool all_axes_halted_by_fault_ = false;
  uint64_t intent_sequence_ = 0;
  uint64_t command_sync_sequence_ = 0;
  uint64_t state_change_seq_ = 0;
};

}  // namespace canopen_hw
