#include "canopen_hw/axis_logic.hpp"

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

AxisLogic::AxisLogic(std::size_t axis_index, BusIO* bus_io,
                     SharedState* shared_state)
    : axis_index_(axis_index),
      bus_io_(bus_io),
      shared_state_(shared_state) {}

void AxisLogic::SetIntent(AxisIntent intent) {
  current_intent_ = intent;
}

void AxisLogic::ProcessRpdo(uint16_t statusword, int32_t actual_position,
                            int32_t actual_velocity, int16_t actual_torque,
                            int8_t mode_display) {
  int32_t safe_target_ticks = 0;
  int32_t safe_target_velocity = 0;
  int16_t safe_target_torque = 0;
  int8_t safe_mode = kMode_CSP;
  uint16_t controlword = 0;
  bool fault_detected = false;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    uint64_t observed_intent_sequence = 0;

    if (shared_state_) {
      global_fault_ = shared_state_->GetGlobalFault();

      const uint64_t seq = shared_state_->intent_sequence();
      observed_intent_sequence = seq;
      if (seq != last_intent_sequence_) {
        last_intent_sequence_ = seq;
        current_intent_ = shared_state_->GetAxisIntent(axis_index_);
      }
    }

    AxisIntent effective_intent = current_intent_;
    if (global_fault_ && effective_intent == AxisIntent::Run) {
      effective_intent = AxisIntent::Halt;
    }

    CiA402Protocol::Input input{};
    input.statusword = statusword;
    input.mode_display = mode_display;
    input.actual_position = actual_position;
    input.actual_velocity = actual_velocity;
    input.actual_torque = actual_torque;
    input.intent = effective_intent;
    input.intent_sequence = observed_intent_sequence;
    input.target_mode = target_mode_;
    input.ros_target_position = ros_target_position_;
    input.ros_target_velocity = ros_target_velocity_;
    input.ros_target_torque = ros_target_torque_;
    input.cmd_valid = cmd_valid_;
    input.cmd_arm_epoch = cmd_arm_epoch_;

    const CiA402Protocol::Output out = protocol_.Process(input);

    if (cmd_valid_ && cmd_arm_epoch_ == 0 && out.arm_epoch != 0) {
      // 兼容旧接口：在首次拿到有效 arm_epoch 后补齐命令会话号。
      cmd_arm_epoch_ = out.arm_epoch;
    }
    arm_epoch_cache_ = out.arm_epoch;

    feedback_cache_.actual_position = actual_position;
    feedback_cache_.actual_velocity = actual_velocity;
    feedback_cache_.actual_torque = actual_torque;
    feedback_cache_.statusword = statusword;
    feedback_cache_.mode_display = mode_display;
    feedback_cache_.state = out.decoded_state;
    feedback_cache_.is_operational = out.is_operational;
    if (out.is_fault) {
      feedback_cache_.is_fault = true;  // 只允许置位，清除需通过 recover。
    }
    feedback_cache_.arm_epoch = out.arm_epoch;
    fault_detected = feedback_cache_.is_fault;

    safe_command_cache_.safe_target_position = out.safe_target_position;
    safe_command_cache_.safe_target_velocity = out.safe_target_velocity;
    safe_command_cache_.safe_target_torque = out.safe_target_torque;
    safe_command_cache_.safe_mode_of_operation = out.safe_mode;

    controlword = out.controlword;
    safe_target_ticks = out.safe_target_position;
    safe_target_velocity = out.safe_target_velocity;
    safe_target_torque = out.safe_target_torque;
    safe_mode = out.safe_mode;
  }

  PublishSnapshot();

  if (shared_state_) {
    if (fault_detected) {
      shared_state_->SetGlobalFault(true);
      shared_state_->SetAllAxesHaltedByFault(true);
    }
    shared_state_->RecomputeAllOperational();
  }

  if (bus_io_) {
    bus_io_->WriteControlword(controlword);
    bus_io_->WriteModeOfOperation(safe_mode);
    bus_io_->WriteTargetPosition(safe_target_ticks);
    if (safe_mode == kMode_CSV) {
      bus_io_->WriteTargetVelocity(safe_target_velocity);
    }
    if (safe_mode == kMode_CST) {
      bus_io_->WriteTargetTorque(safe_target_torque);
    }
  }
}

void AxisLogic::ProcessEmcy(uint16_t eec, uint8_t er) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_cache_.last_emcy_eec = eec;
  }
  health_.emcy_count.fetch_add(1, std::memory_order_relaxed);
  CANOPEN_LOG_ERROR("axis={} EMCY eec=0x{:04x} er=0x{:02x}",
                    axis_index_, static_cast<unsigned int>(eec),
                    static_cast<unsigned int>(er));
  PublishSnapshot();
}

void AxisLogic::ProcessHeartbeat(bool lost) {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_cache_.heartbeat_lost = lost;
    if (lost) {
      // 心跳丢失后立即撤销运行意图，避免复电重连后自动再使能。
      SetIntent(AxisIntent::Disable);
      feedback_cache_.is_fault = true;
      feedback_cache_.is_operational = false;
    } else {
      // 心跳恢复只表示链路恢复，不代表故障已清除；等待 RPDO 刷新真实 fault 位。
      feedback_cache_.is_operational = false;
    }
  }
  if (lost) {
    health_.heartbeat_lost.fetch_add(1, std::memory_order_relaxed);
    CANOPEN_LOG_WARN("axis={}: heartbeat lost", axis_index_);
  } else {
    health_.heartbeat_recovered.fetch_add(1, std::memory_order_relaxed);
    CANOPEN_LOG_INFO("axis={}: heartbeat recovered", axis_index_);
  }
  PublishSnapshot();
  if (shared_state_) {
    if (lost) {
      shared_state_->SetGlobalFault(true);
      shared_state_->SetAllAxesHaltedByFault(true);
    }
    shared_state_->RecomputeAllOperational();
  }
}

void AxisLogic::Configure(int32_t position_lock_threshold,
                          int max_fault_resets,
                          int fault_reset_hold_cycles) {
  (void)max_fault_resets;
  (void)fault_reset_hold_cycles;
  std::lock_guard<std::mutex> lk(mtx_);
  protocol_.set_position_lock_threshold(position_lock_threshold);
}

void AxisLogic::SetRosTarget(int32_t target_position) {
  std::lock_guard<std::mutex> lk(mtx_);
  ros_target_position_ = target_position;
  cmd_valid_ = true;
  if (cmd_arm_epoch_ == 0) {
    cmd_arm_epoch_ = arm_epoch_cache_;
  }
}

void AxisLogic::SetRosTargetVelocity(int32_t target_velocity) {
  std::lock_guard<std::mutex> lk(mtx_);
  ros_target_velocity_ = target_velocity;
  cmd_valid_ = true;
  if (cmd_arm_epoch_ == 0) {
    cmd_arm_epoch_ = arm_epoch_cache_;
  }
}

void AxisLogic::SetRosTargetTorque(int16_t target_torque) {
  std::lock_guard<std::mutex> lk(mtx_);
  ros_target_torque_ = target_torque;
  cmd_valid_ = true;
  if (cmd_arm_epoch_ == 0) {
    cmd_arm_epoch_ = arm_epoch_cache_;
  }
}

void AxisLogic::SetTargetMode(int8_t mode) {
  std::lock_guard<std::mutex> lk(mtx_);
  target_mode_ = mode;
}

void AxisLogic::SetExternalCommand(const AxisCommand& command) {
  std::lock_guard<std::mutex> lk(mtx_);
  target_mode_ = command.mode_of_operation;
  ros_target_position_ = command.target_position;
  ros_target_velocity_ = command.target_velocity;
  ros_target_torque_ = command.target_torque;
  cmd_valid_ = command.valid;
  cmd_arm_epoch_ = command.arm_epoch;
}

void AxisLogic::SetGlobalFault(bool global_fault) {
  std::lock_guard<std::mutex> lk(mtx_);
  global_fault_ = global_fault;
}

void AxisLogic::RequestEnable() {
  std::lock_guard<std::mutex> lk(mtx_);
  SetIntent(AxisIntent::Run);
}

void AxisLogic::RequestDisable() {
  {
    std::lock_guard<std::mutex> lk(mtx_);
    SetIntent(AxisIntent::Disable);
    cmd_valid_ = false;
    cmd_arm_epoch_ = 0;
    // 关机路径期望“去使能请求”立即可见，避免等待下一帧 RPDO 才回落。
    feedback_cache_.is_operational = false;
  }
  PublishSnapshot();
  if (shared_state_) {
    shared_state_->RecomputeAllOperational();
  }
}

void AxisLogic::RequestHalt() {
  std::lock_guard<std::mutex> lk(mtx_);
  SetIntent(AxisIntent::Halt);
}

void AxisLogic::RequestResume() {
  std::lock_guard<std::mutex> lk(mtx_);
  SetIntent(AxisIntent::Run);
}

void AxisLogic::ResetFault() {
  std::lock_guard<std::mutex> lk(mtx_);
  cmd_valid_ = false;
  cmd_arm_epoch_ = 0;
  health_.fault_reset_attempts.store(0, std::memory_order_relaxed);
  feedback_cache_.is_fault = false;  // 显式清除，不依赖 RPDO 自动刷新。
}

CiA402State AxisLogic::feedback_state() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return feedback_cache_.state;
}

int8_t AxisLogic::target_mode() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return target_mode_;
}

void AxisLogic::PublishSnapshot() {
  if (shared_state_ == nullptr) return;

  AxisFeedback feedback_snapshot;
  AxisSafeCommand safe_cmd;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_snapshot = feedback_cache_;
    safe_cmd = safe_command_cache_;
  }

  shared_state_->UpdateFeedback(axis_index_, feedback_snapshot);
  shared_state_->UpdateSafeCommand(axis_index_, safe_cmd);
}

}  // namespace canopen_hw
