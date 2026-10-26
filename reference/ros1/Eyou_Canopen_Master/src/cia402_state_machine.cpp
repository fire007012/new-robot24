#include "canopen_hw/cia402_state_machine.hpp"

#include <cstdlib>

namespace canopen_hw {

namespace {

int64_t AbsDiff(int32_t a, int32_t b) {
  const int64_t d = static_cast<int64_t>(a) - static_cast<int64_t>(b);
  return d < 0 ? -d : d;
}

}  // namespace

CiA402StateMachine::CiA402StateMachine() = default;

void CiA402StateMachine::set_fault_reset_policy(int hold_cycles, int wait_cycles,
                                                int max_attempts) {
  if (hold_cycles > 0) {
    hold_low_cycles_ = hold_cycles;
  }
  if (wait_cycles > 0) {
    max_wait_recovery_cycles_ = wait_cycles;
  }
  if (max_attempts > 0) {
    max_fault_resets_ = max_attempts;
  }
}

CiA402State CiA402StateMachine::DecodeState(uint16_t statusword) {
  // 注意顺序: 先判断 FAULT_REACTION_ACTIVE/FAULT, 避免与其它掩码冲突。
  if ((statusword & kStateMask_Basic) == kState_FaultReactionActive) {
    return CiA402State::FaultReactionActive;
  }
  if ((statusword & kStateMask_Basic) == kState_Fault) {
    return CiA402State::Fault;
  }
  if ((statusword & kStateMask_Basic) == kState_NotReadyToSwitchOn) {
    return CiA402State::NotReadyToSwitchOn;
  }
  if ((statusword & kStateMask_Basic) == kState_SwitchOnDisabled) {
    return CiA402State::SwitchOnDisabled;
  }
  if ((statusword & kStateMask_ReadySwitchOn) == kState_ReadyToSwitchOn) {
    return CiA402State::ReadyToSwitchOn;
  }
  if ((statusword & kStateMask_ReadySwitchOn) == kState_SwitchedOn) {
    return CiA402State::SwitchedOn;
  }
  if ((statusword & kStateMask_ReadySwitchOn) == kState_OperationEnabled) {
    return CiA402State::OperationEnabled;
  }
  if ((statusword & kStateMask_ReadySwitchOn) == kState_QuickStopActive) {
    return CiA402State::QuickStopActive;
  }

  // 未知组合时做保守处理: 视作 NotReady。
  return CiA402State::NotReadyToSwitchOn;
}

void CiA402StateMachine::AdvanceArmEpoch() {
  ++arm_epoch_;
  if (arm_epoch_ == 0) {
    ++arm_epoch_;
  }
}

void CiA402StateMachine::Update(uint16_t statusword, int8_t mode_display,
                                int32_t actual_position) {
  // mode_display 不再用于状态机内部门控逻辑（见 ReadyToSwitchOn 分支注释），
  // 由调用层（AxisLogic/AxisDriver）负责存储和上报。
  (void)mode_display;
  last_actual_position_ = actual_position;

  const CiA402State prev_state = state_;
  state_ = DecodeState(statusword);

  is_fault_ = (state_ == CiA402State::Fault ||
               state_ == CiA402State::FaultReactionActive);

  const bool in_fault_related_state =
      (state_ == CiA402State::Fault ||
       state_ == CiA402State::FaultReactionActive);

  // 仅在离开“故障相关状态”后清理上下文:
  // - FAULT_REACTION_ACTIVE -> FAULT 的内部跃迁不应清理
  // - 否则会导致刚进入 FAULT 时复位流程重复从 Phase1 开始
  if (in_fault_related_state == false) {
    ResetFaultFlowContext();
  }

  const bool halt_released =
      (prev_halt_requested_ && (halt_requested_ == false));
  const bool enable_chain_allowed =
      enable_requested_ && !global_fault_ && !forced_halt_by_fault_;

  switch (state_) {
    case CiA402State::NotReadyToSwitchOn:
      // 自检阶段: 不下发控制命令, 避免与驱动内部流程冲突。
      controlword_ = kCtrl_DisableVoltage;
      is_operational_ = false;
      position_locked_ = true;
      safe_target_ = actual_position;
      safe_target_velocity_ = 0;
      safe_target_torque_ = 0;
      break;

    case CiA402State::SwitchOnDisabled:
      // 进入可使能起点, 默认发 Shutdown 推进到 ReadyToSwitchOn。
      controlword_ = enable_chain_allowed ? kCtrl_Shutdown : kCtrl_DisableVoltage;
      is_operational_ = false;
      position_locked_ = true;
      safe_target_ = actual_position;
      safe_target_velocity_ = 0;
      safe_target_torque_ = 0;
      break;

    case CiA402State::ReadyToSwitchOn:
      // mode_of_operation 由主站每帧写出（tpdo_mapped[0x6060]），
      // 不用 mode_display 来门控使能序列。
      controlword_ = enable_chain_allowed ? kCtrl_EnableOperation : kCtrl_Shutdown;
      is_operational_ = false;
      position_locked_ = true;
      safe_target_ = actual_position;
      safe_target_velocity_ = 0;
      safe_target_torque_ = 0;
      break;

    case CiA402State::SwitchedOn:
      // 防御分支: 若未跳级, 在此继续推到 OPERATION_ENABLED。
      controlword_ = enable_chain_allowed ? kCtrl_EnableOperation : kCtrl_Shutdown;
      is_operational_ = false;
      position_locked_ = true;
      safe_target_ = actual_position;
      safe_target_velocity_ = 0;
      safe_target_torque_ = 0;
      break;

    case CiA402State::OperationEnabled:
      if (was_operation_enabled_ == false) {
        AdvanceArmEpoch();
        position_locked_ = true;
        cmd_valid_ = false;
        cmd_epoch_ = 0;
        safe_target_ = actual_position;
        safe_target_velocity_ = 0;
        safe_target_torque_ = 0;
      }

      controlword_ = kCtrl_EnableOperation;
      if (target_mode_ == kMode_IP) {
        // IP 模式要求插补使能位（bit4）与 EnableOperation 同时置位。
        controlword_ |= kCtrl_Bit_InterpolationEnable;
      }
      if (halt_requested_) {
        controlword_ |= kCtrl_Bit_Halt;
      }
      if (enable_requested_ == false) {
        // 收到 disable 请求后，立即退出可运行态并回收安全目标。
        controlword_ = kCtrl_DisableOperation;
        is_operational_ = false;
        position_locked_ = true;
        safe_target_ = actual_position;
        safe_target_velocity_ = 0;
        safe_target_torque_ = 0;
        break;
      }

      if (halt_requested_) {
        // 轻量级 halt：保持 402 在 OperationEnabled，仅冻结安全目标。
        is_operational_ = true;
        position_locked_ = true;
        safe_target_ = actual_position;
        safe_target_velocity_ = 0;
        safe_target_torque_ = 0;
        break;
      }

      if (halt_released) {
        // 从 halt 恢复后重新锁定，避免直接透传旧目标导致跳变。
        position_locked_ = true;
        safe_target_ = actual_position;
        safe_target_velocity_ = 0;
        safe_target_torque_ = 0;
      }

      StepOperationEnabled(actual_position);
      break;

    case CiA402State::QuickStopActive:
      // 某些驱动器在 QuickStopActive 下需要明确 EnableVoltage(0x0002)
      // 才会回到 SwitchOnDisabled 路径。
      controlword_ = kCtrl_EnableVoltage;
      is_operational_ = false;
      position_locked_ = true;
      safe_target_ = actual_position;
      safe_target_velocity_ = 0;
      safe_target_torque_ = 0;
      break;

    case CiA402State::FaultReactionActive:
      // 故障制动阶段: 明确保持 0x0000, 禁止抢发 reset。
      controlword_ = kCtrl_DisableVoltage;
      is_operational_ = false;
      position_locked_ = true;
      safe_target_ = actual_position;
      safe_target_velocity_ = 0;
      safe_target_torque_ = 0;
      break;

    case CiA402State::Fault:
      // FAULT 才进入受控复位流程, 避免在反应态误复位。
      StepFaultReset();
      is_operational_ = false;
      position_locked_ = true;
      safe_target_ = actual_position;
      safe_target_velocity_ = 0;
      safe_target_torque_ = 0;
      break;
  }

  // 用于检测首次进入 OPERATION_ENABLED 的边沿。
  was_operation_enabled_ = (state_ == CiA402State::OperationEnabled);

  // 若状态从运行态退出, 立即锁住安全目标。
  if (prev_state == CiA402State::OperationEnabled &&
      (state_ == CiA402State::OperationEnabled) == false) {
    is_operational_ = false;
    position_locked_ = true;
    safe_target_ = actual_position;
    safe_target_velocity_ = 0;
    safe_target_torque_ = 0;
    cmd_valid_ = false;
    cmd_epoch_ = 0;
    forced_halt_by_fault_ = false;
  }

  prev_halt_requested_ = halt_requested_;
}

void CiA402StateMachine::StepFaultReset() {
  // 超过最大重试次数后, 不再自动复位, 维持 0x0000 等人工介入。
  if (fault_reset_count_ >= max_fault_resets_) {
    fault_phase_ = FaultResetPhase::PermanentFault;
    controlword_ = kCtrl_DisableVoltage;
    return;
  }

  // 首次进入 FAULT 时初始化流程。
  if (fault_phase_ == FaultResetPhase::Idle) {
    fault_phase_ = FaultResetPhase::HoldLow;
    phase_cycles_ = 0;
    wait_cycles_ = 0;
  }

  switch (fault_phase_) {
    case FaultResetPhase::HoldLow:
      // Phase 1: 保持 0x0000 若干周期, 保证复位上升沿清晰。
      controlword_ = kCtrl_DisableVoltage;
      ++phase_cycles_;
      if (phase_cycles_ >= hold_low_cycles_) {
        fault_phase_ = FaultResetPhase::SendEdge;
        phase_cycles_ = 0;
      }
      break;

    case FaultResetPhase::SendEdge:
      // Phase 2: 仅发送一个周期的 0x0080 上升沿。
      ++fault_reset_count_;
      controlword_ = kCtrl_FaultReset;
      fault_phase_ = FaultResetPhase::WaitRecovery;
      wait_cycles_ = 0;
      break;

    case FaultResetPhase::WaitRecovery:
      // Phase 3: 回到 0x0000 等状态离开 FAULT。
      controlword_ = kCtrl_DisableVoltage;
      ++wait_cycles_;
      if (wait_cycles_ >= max_wait_recovery_cycles_) {
        // 本次尝试已计数, 超时后重新回到 Phase 1 发起下一次尝试。
        fault_phase_ = FaultResetPhase::HoldLow;
        phase_cycles_ = 0;
        wait_cycles_ = 0;
      }
      break;

    case FaultResetPhase::PermanentFault:
      controlword_ = kCtrl_DisableVoltage;
      break;

    case FaultResetPhase::Idle:
      // 已在函数开头处理。
      controlword_ = kCtrl_DisableVoltage;
      break;
  }
}

void CiA402StateMachine::StepOperationEnabled(int32_t actual_position) {
  // 第零层：全局故障冻结。
  if (global_fault_ || forced_halt_by_fault_) {
    position_locked_ = true;
    safe_target_ = actual_position;
    safe_target_velocity_ = 0;
    safe_target_torque_ = 0;
    is_operational_ = false;
    return;
  }

  // 第一层：用户 halt 冻结。
  if (halt_requested_) {
    position_locked_ = true;
    safe_target_ = actual_position;
    safe_target_velocity_ = 0;
    safe_target_torque_ = 0;
    is_operational_ = true;
    return;
  }

  // 第二层：命令源就绪与会话匹配门控。
  if (cmd_valid_ == false || cmd_epoch_ == 0 || cmd_epoch_ != arm_epoch_) {
    position_locked_ = true;
    safe_target_ = actual_position;
    safe_target_velocity_ = 0;
    safe_target_torque_ = 0;
    is_operational_ = false;
    return;
  }

  // CSV/CST 模式: 首帧由上层 valid/epoch gate 保证，后续直接透传速度/力矩。
  if (target_mode_ == kMode_CSV || target_mode_ == kMode_CST) {
    position_locked_ = false;
    safe_target_ = actual_position;  // 切回 CSP 时避免位置跳变。
    safe_target_velocity_ = ros_target_velocity_;
    safe_target_torque_ = ros_target_torque_;
    is_operational_ = true;
    return;
  }

  // CSP 模式: 位置锁定，上层目标接近实际位置后才放行。
  if (position_locked_) {
    safe_target_ = actual_position;
    safe_target_velocity_ = 0;
    safe_target_torque_ = 0;

    if (AbsDiff(ros_target_, actual_position) <=
        static_cast<int64_t>(position_lock_threshold_)) {
      position_locked_ = false;
      safe_target_ = ros_target_;
      safe_target_velocity_ = ros_target_velocity_;
      safe_target_torque_ = ros_target_torque_;
    }
  } else {
    const int64_t diff = static_cast<int64_t>(ros_target_) -
                         static_cast<int64_t>(safe_target_);
    const int64_t max_delta = static_cast<int64_t>(max_delta_per_cycle_);

    if (diff > max_delta) {
      safe_target_ = static_cast<int32_t>(
          static_cast<int64_t>(safe_target_) + max_delta);
    } else if (diff < -max_delta) {
      safe_target_ = static_cast<int32_t>(
          static_cast<int64_t>(safe_target_) - max_delta);
    } else {
      safe_target_ = ros_target_;
    }

    safe_target_velocity_ = ros_target_velocity_;
    safe_target_torque_ = ros_target_torque_;
  }

  is_operational_ = (position_locked_ == false);
}

void CiA402StateMachine::ResetFaultFlowContext() {
  fault_phase_ = FaultResetPhase::Idle;
  phase_cycles_ = 0;
  wait_cycles_ = 0;

  // 复位尝试次数保持累计值，用于故障诊断与限次保护。
}

void CiA402StateMachine::ResetFaultCounter() {
  fault_reset_count_ = 0;
  ResetFaultFlowContext();
}

}  // namespace canopen_hw
