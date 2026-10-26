#pragma once

#include <cstdint>

#include "canopen_hw/cia402_defs.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// 纯协议翻译器：
// - 输入: 反馈状态 + 轴意图 + 上层命令
// - 输出: 当前周期控制字 + 过滤后的安全目标
// - 不持有策略闩锁（enable/halt/fault reset 由协调层处理）
class CiA402Protocol {
 public:
  struct Input {
    uint16_t statusword = 0;
    int8_t mode_display = 0;
    int32_t actual_position = 0;
    int32_t actual_velocity = 0;
    int16_t actual_torque = 0;

    AxisIntent intent = AxisIntent::Disable;
    uint64_t intent_sequence = 0;
    int8_t target_mode = kMode_CSP;
    int32_t ros_target_position = 0;
    int32_t ros_target_velocity = 0;
    int16_t ros_target_torque = 0;

    bool cmd_valid = false;
    uint32_t cmd_arm_epoch = 0;
  };

  struct Output {
    uint16_t controlword = kCtrl_DisableVoltage;
    int32_t safe_target_position = 0;
    int32_t safe_target_velocity = 0;
    int16_t safe_target_torque = 0;
    int8_t safe_mode = kMode_CSP;
    CiA402State decoded_state = CiA402State::NotReadyToSwitchOn;
    bool is_fault = false;
    bool is_operational = false;
    bool arm_epoch_advanced = false;
    uint32_t arm_epoch = 0;
  };

  CiA402Protocol();

  Output Process(const Input& input);

  void set_position_lock_threshold(int32_t threshold_counts);
  void set_max_delta_per_cycle(int32_t delta_counts);
  void set_max_stale_intent_frames(uint32_t frames);

  static CiA402State DecodeState(uint16_t statusword);

 private:
  void AdvanceArmEpoch();
  void LockPosition(int32_t actual_position, Output* out);
  uint16_t ComposeEnableOperationControlword(int8_t target_mode) const;
  bool CommandReady(const Input& input) const;
  void StepPositionLock(const Input& input, Output* out);

  CiA402State state_ = CiA402State::NotReadyToSwitchOn;
  bool was_operation_enabled_ = false;
  bool prev_was_halt_ = false;
  uint32_t arm_epoch_ = 0;
  bool position_locked_ = true;
  int32_t position_lock_threshold_ = 15000;
  int32_t max_delta_per_cycle_ = 2147483647;
  int32_t last_safe_target_position_ = 0;
  uint64_t last_intent_sequence_ = 0;
  uint32_t stale_intent_frames_ = 0;
  uint32_t max_stale_intent_frames_ = 20;
};

}  // namespace canopen_hw
