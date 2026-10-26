#pragma once

#include <cstdint>

#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

// 纯逻辑 CiA402 状态机:
// - 不依赖系统时钟, 所有“时间”都由 update() 调用次数表示
// - 不依赖总线/线程/ROS, 仅处理状态输入与控制输出
class CiA402StateMachine {
 public:
  CiA402StateMachine();

  // 每个 SYNC 周期调用一次。
  // statusword/mode_display 来自驱动反馈, actual_position 用于无扰切换锁定逻辑。
  void Update(uint16_t statusword, int8_t mode_display, int32_t actual_position);

  // 输出: 当前应发送的 controlword。
  uint16_t controlword() const { return controlword_; }

  // 输出: 当前解析出的 CiA402 状态。
  CiA402State state() const { return state_; }

  // 输出: 轴是否可认为已安全投入闭环(= OPERATION_ENABLED 且锁定已释放)。
  bool is_operational() const { return is_operational_; }

  // 输出: 是否处于故障态(FAULT 或 FAULT_REACTION_ACTIVE)。
  bool is_fault() const { return is_fault_; }

  // 输出: 已发起故障复位的尝试次数(每发送一次 0x0080 计数 +1)。
  int fault_reset_count() const { return fault_reset_count_; }

  // 配置目标模式, CSP 默认 8。
  void set_target_mode(int8_t mode) { target_mode_ = mode; }

  // 请求进入使能流程。
  void request_enable() { enable_requested_ = true; }

  // 请求退出使能流程。
  void request_disable() { enable_requested_ = false; }
  bool enable_requested() const { return enable_requested_; }

  // 请求轻量级 halt（仅置控制字 bit8，不降级 402 状态机）。
  void request_halt() { halt_requested_ = true; }

  // 清除轻量级 halt。
  void request_resume() { halt_requested_ = false; }

  bool halt_requested() const { return halt_requested_; }

  // 新协议：写入上层位置命令及命令元数据。
  void SetExternalPositionCommand(int32_t target, bool valid, uint32_t arm_epoch) {
    ros_target_ = target;
    cmd_valid_ = valid;
    cmd_epoch_ = arm_epoch;
  }

  void SetExternalVelocityCommand(int32_t target) {
    ros_target_velocity_ = target;
  }

  void SetExternalTorqueCommand(int16_t target) {
    ros_target_torque_ = target;
  }

  // 兼容接口：旧调用路径保持可编译。
  void set_ros_target(int32_t target) {
    ros_target_ = target;
    cmd_valid_ = true;
    cmd_epoch_ = arm_epoch_;
  }

  // 设置上层(ROS)期望速度。
  void set_ros_target_velocity(int32_t target) {
    ros_target_velocity_ = target;
    cmd_valid_ = true;
    cmd_epoch_ = arm_epoch_;
  }

  // 设置上层(ROS)期望力矩。
  void set_ros_target_torque(int16_t target) {
    ros_target_torque_ = target;
    cmd_valid_ = true;
    cmd_epoch_ = arm_epoch_;
  }

  // 全局故障闩锁输入（由外部协调层/AxisLogic 注入）。
  void set_global_fault(bool fault) {
    global_fault_ = fault;
    if (fault == false) {
      forced_halt_by_fault_ = false;
    }
  }

  void set_forced_halt_by_fault(bool forced) { forced_halt_by_fault_ = forced; }
  bool forced_halt_by_fault() const { return forced_halt_by_fault_; }

  // 获取经过”无扰锁定”后的安全目标。
  int32_t safe_target() const { return safe_target_; }
  int32_t safe_target_velocity() const { return safe_target_velocity_; }
  int16_t safe_target_torque() const { return safe_target_torque_; }
  int8_t safe_mode_of_operation() const { return target_mode_; }

  uint32_t arm_epoch() const { return arm_epoch_; }

  // 是否仍在位置锁定阶段。
  bool is_position_locked() const { return position_locked_; }

  // 配置解锁阈值(计数单位), 默认 15000。
  void set_position_lock_threshold(int32_t threshold_counts) {
    position_lock_threshold_ = threshold_counts;
  }

  // 配置每周期位置限幅（计数单位），默认不限制。
  void set_max_delta_per_cycle(int32_t delta_counts) {
    if (delta_counts > 0) {
      max_delta_per_cycle_ = delta_counts;
    }
  }

  // 故障复位节流参数:
  // hold_cycles: 低电平保持周期数
  // wait_cycles: 发复位沿后等待恢复的最大周期数
  void set_fault_reset_policy(int hold_cycles, int wait_cycles, int max_attempts);

  // 手动重置故障计数器，使 PermanentFault 状态可以重新进入自动复位流程。
  void ResetFaultCounter();

 private:
  enum class FaultResetPhase {
    Idle,
    HoldLow,
    SendEdge,
    WaitRecovery,
    PermanentFault,
  };

  static CiA402State DecodeState(uint16_t statusword);

  // 在 FAULT 状态下执行三阶段复位流程。
  void StepFaultReset();

  // 处理 OPERATION_ENABLED 期间的无扰切换。
  void StepOperationEnabled(int32_t actual_position);

  // 清理一次复位流程上下文(用于成功恢复或退出故障态)。
  void ResetFaultFlowContext();

  void AdvanceArmEpoch();

  uint16_t controlword_ = kCtrl_DisableVoltage;
  CiA402State state_ = CiA402State::NotReadyToSwitchOn;

  // 请求与模式。
  // 安全默认值：上层需显式 request_enable() 才推进使能链路。
  bool enable_requested_ = false;
  bool halt_requested_ = false;
  bool prev_halt_requested_ = false;
  int8_t target_mode_ = kMode_CSP;

  // 运行状态输出。
  bool is_operational_ = false;
  bool is_fault_ = false;

  // 无扰切换相关。
  bool was_operation_enabled_ = false;
  bool position_locked_ = true;
  int32_t position_lock_threshold_ = 15000;
  int32_t ros_target_ = 0;
  int32_t safe_target_ = 0;
  int32_t last_actual_position_ = 0;
  bool cmd_valid_ = false;
  uint32_t cmd_epoch_ = 0;
  uint32_t arm_epoch_ = 0;
  int32_t max_delta_per_cycle_ = 2147483647;
  bool global_fault_ = false;
  bool forced_halt_by_fault_ = false;

  // 速度/力矩目标。
  int32_t ros_target_velocity_ = 0;
  int16_t ros_target_torque_ = 0;
  int32_t safe_target_velocity_ = 0;
  int16_t safe_target_torque_ = 0;

  // 故障复位相关。
  FaultResetPhase fault_phase_ = FaultResetPhase::Idle;
  int phase_cycles_ = 0;
  int wait_cycles_ = 0;
  int fault_reset_count_ = 0;
  int max_fault_resets_ = 3;
  int hold_low_cycles_ = 5;
  int max_wait_recovery_cycles_ = 100;
};

}  // namespace canopen_hw
