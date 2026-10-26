#include "canopen_hw/cia402_protocol.hpp"

#include <algorithm>
#include <cstdint>

namespace canopen_hw {

namespace {

int64_t AbsDiff(int32_t a, int32_t b) {
  const int64_t d = static_cast<int64_t>(a) - static_cast<int64_t>(b);
  return d < 0 ? -d : d;
}

}  // namespace

CiA402Protocol::CiA402Protocol() = default;

CiA402State CiA402Protocol::DecodeState(uint16_t statusword) {
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
  return CiA402State::NotReadyToSwitchOn;
}

void CiA402Protocol::set_position_lock_threshold(int32_t threshold_counts) {
  if (threshold_counts > 0) {
    position_lock_threshold_ = threshold_counts;
  }
}

void CiA402Protocol::set_max_delta_per_cycle(int32_t delta_counts) {
  if (delta_counts > 0) {
    max_delta_per_cycle_ = delta_counts;
  }
}

void CiA402Protocol::set_max_stale_intent_frames(uint32_t frames) {
  max_stale_intent_frames_ = frames;
}

void CiA402Protocol::AdvanceArmEpoch() {
  ++arm_epoch_;
  if (arm_epoch_ == 0) {
    ++arm_epoch_;
  }
}

void CiA402Protocol::LockPosition(int32_t actual_position, Output* out) {
  position_locked_ = true;
  last_safe_target_position_ = actual_position;
  out->safe_target_position = actual_position;
  out->safe_target_velocity = 0;
  out->safe_target_torque = 0;
}

uint16_t CiA402Protocol::ComposeEnableOperationControlword(
    int8_t target_mode) const {
  uint16_t cw = kCtrl_EnableOperation;
  if (target_mode == kMode_IP) {
    cw |= kCtrl_Bit_InterpolationEnable;
  }
  return cw;
}

bool CiA402Protocol::CommandReady(const Input& input) const {
  return input.cmd_valid && input.cmd_arm_epoch != 0 &&
         input.cmd_arm_epoch == arm_epoch_;
}

void CiA402Protocol::StepPositionLock(const Input& input, Output* out) {
  const bool cmd_ready = CommandReady(input);

  // CSV/CST 不做位置收敛判定，仅校验命令会话有效性。
  if (input.target_mode == kMode_CSV || input.target_mode == kMode_CST) {
    if (!cmd_ready) {
      LockPosition(input.actual_position, out);
      out->is_operational = false;
      return;
    }
    position_locked_ = false;
    last_safe_target_position_ = input.actual_position;
    out->safe_target_position = input.actual_position;
    out->safe_target_velocity = input.ros_target_velocity;
    out->safe_target_torque = input.ros_target_torque;
    out->is_operational = true;
    return;
  }

  if (position_locked_) {
    if (cmd_ready &&
        AbsDiff(input.ros_target_position, input.actual_position) <=
            static_cast<int64_t>(position_lock_threshold_)) {
      position_locked_ = false;
      last_safe_target_position_ = input.ros_target_position;
    }
  }

  if (position_locked_) {
    LockPosition(input.actual_position, out);
    out->is_operational = false;
    return;
  }

  int32_t clamped = input.ros_target_position;
  if (max_delta_per_cycle_ > 0) {
    const int64_t delta = static_cast<int64_t>(clamped) -
                          static_cast<int64_t>(last_safe_target_position_);
    const int64_t max_delta = static_cast<int64_t>(max_delta_per_cycle_);
    if (delta > max_delta) {
      clamped = static_cast<int32_t>(
          static_cast<int64_t>(last_safe_target_position_) + max_delta);
    } else if (delta < -max_delta) {
      clamped = static_cast<int32_t>(
          static_cast<int64_t>(last_safe_target_position_) - max_delta);
    }
  }

  last_safe_target_position_ = clamped;
  out->safe_target_position = clamped;
  out->safe_target_velocity = input.ros_target_velocity;
  out->safe_target_torque = input.ros_target_torque;
  out->is_operational = true;
}

CiA402Protocol::Output CiA402Protocol::Process(const Input& input) {
  Output out{};
  state_ = DecodeState(input.statusword);
  out.decoded_state = state_;
  out.is_fault = (state_ == CiA402State::Fault ||
                  state_ == CiA402State::FaultReactionActive);
  out.arm_epoch = arm_epoch_;
  out.safe_mode = input.target_mode;

  if (input.intent_sequence != last_intent_sequence_) {
    last_intent_sequence_ = input.intent_sequence;
    stale_intent_frames_ = 0;
  } else {
    if (stale_intent_frames_ != UINT32_MAX) {
      ++stale_intent_frames_;
    }
  }

  AxisIntent effective_intent = input.intent;
  if (stale_intent_frames_ >= max_stale_intent_frames_) {
    effective_intent = AxisIntent::Disable;
  }

  const bool want_enable = (effective_intent == AxisIntent::Enable ||
                            effective_intent == AxisIntent::Halt ||
                            effective_intent == AxisIntent::Run);
  const bool want_halt = (effective_intent == AxisIntent::Halt);
  const bool want_run = (effective_intent == AxisIntent::Run);

  switch (state_) {
    case CiA402State::NotReadyToSwitchOn:
      out.controlword = kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      prev_was_halt_ = false;
      break;

    case CiA402State::SwitchOnDisabled:
      out.controlword = want_enable ? kCtrl_Shutdown : kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      prev_was_halt_ = false;
      break;

    case CiA402State::ReadyToSwitchOn:
      out.controlword = want_enable ? kCtrl_EnableOperation : kCtrl_Shutdown;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      prev_was_halt_ = false;
      break;

    case CiA402State::SwitchedOn:
      out.controlword = want_enable ? kCtrl_EnableOperation : kCtrl_Shutdown;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      prev_was_halt_ = false;
      break;

    case CiA402State::OperationEnabled: {
      const bool first_frame_in_op = !was_operation_enabled_;
      if (first_frame_in_op) {
        AdvanceArmEpoch();
        out.arm_epoch_advanced = true;
        out.arm_epoch = arm_epoch_;
        out.controlword = ComposeEnableOperationControlword(input.target_mode);
        if (want_halt) {
          out.controlword |= kCtrl_Bit_Halt;
          prev_was_halt_ = true;
        } else {
          prev_was_halt_ = false;
        }
        out.is_operational = false;
        LockPosition(input.actual_position, &out);
        break;
      }

      if (!want_enable) {
        out.controlword = kCtrl_DisableOperation;
        out.is_operational = false;
        LockPosition(input.actual_position, &out);
        prev_was_halt_ = false;
        break;
      }

      if (want_halt || !want_run) {
        out.controlword = ComposeEnableOperationControlword(input.target_mode);
        if (want_halt) {
          out.controlword |= kCtrl_Bit_Halt;
        }
        out.is_operational = false;
        LockPosition(input.actual_position, &out);
        prev_was_halt_ = want_halt;
        break;
      }

      if (prev_was_halt_) {
        AdvanceArmEpoch();
        out.arm_epoch_advanced = true;
        out.arm_epoch = arm_epoch_;
        out.controlword = ComposeEnableOperationControlword(input.target_mode);
        out.is_operational = false;
        LockPosition(input.actual_position, &out);
        prev_was_halt_ = false;
        break;
      }

      out.controlword = ComposeEnableOperationControlword(input.target_mode);
      StepPositionLock(input, &out);
      prev_was_halt_ = false;
      break;
    }

    case CiA402State::Fault:
    case CiA402State::FaultReactionActive:
      out.controlword = kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      prev_was_halt_ = false;
      break;

    case CiA402State::QuickStopActive:
      out.controlword = kCtrl_DisableVoltage;
      out.is_operational = false;
      LockPosition(input.actual_position, &out);
      prev_was_halt_ = false;
      break;
  }

  was_operation_enabled_ = (state_ == CiA402State::OperationEnabled);
  return out;
}

}  // namespace canopen_hw
