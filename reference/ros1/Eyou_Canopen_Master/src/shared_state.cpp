#include "canopen_hw/shared_state.hpp"

#include <algorithm>

namespace canopen_hw {

SharedState::SharedState(std::size_t axis_count)
    : axis_count_(std::clamp(axis_count,
                             static_cast<std::size_t>(1),
                             kMaxAxisCount)),
      feedback_(axis_count_),
      commands_(axis_count_),
      safe_commands_(axis_count_),
      intents_(axis_count_, AxisIntent::Disable) {}

void SharedState::UpdateFeedback(std::size_t axis_index,
                                 const AxisFeedback& feedback) {
  if (IsValidAxis(axis_index) == false) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    feedback_[axis_index] = feedback;
    ++state_change_seq_;
  }
  // 在锁外通知，避免被唤醒线程立即阻塞在 mtx_ 上。
  state_cv_.notify_all();
}

void SharedState::RecomputeAllOperational() {
  std::lock_guard<std::mutex> lk(mtx_);
  bool all_ok = true;
  for (std::size_t i = 0; i < axis_count_; ++i) {
    const auto& axis_feedback = feedback_[i];
    if (axis_feedback.is_operational == false || axis_feedback.is_fault) {
      all_ok = false;
      break;
    }
  }
  all_operational_ = all_ok;
}

void SharedState::UpdateCommand(std::size_t axis_index,
                                const AxisCommand& command) {
  if (IsValidAxis(axis_index) == false) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  commands_[axis_index] = command;
}

bool SharedState::GetCommand(std::size_t axis_index, AxisCommand* out) const {
  if (out == nullptr || IsValidAxis(axis_index) == false) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  *out = commands_[axis_index];
  return true;
}

void SharedState::UpdateSafeCommand(std::size_t axis_index,
                                    const AxisSafeCommand& safe_command) {
  if (IsValidAxis(axis_index) == false) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  safe_commands_[axis_index] = safe_command;
}

void SharedState::SetAxisIntent(std::size_t axis_index, AxisIntent intent) {
  if (IsValidAxis(axis_index) == false) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  intents_[axis_index] = intent;
}

AxisIntent SharedState::GetAxisIntent(std::size_t axis_index) const {
  if (IsValidAxis(axis_index) == false) {
    return AxisIntent::Disable;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  return intents_[axis_index];
}

void SharedState::AdvanceIntentSequence() {
  std::lock_guard<std::mutex> lk(mtx_);
  ++intent_sequence_;
}

uint64_t SharedState::intent_sequence() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return intent_sequence_;
}

void SharedState::AdvanceCommandSyncSequence() {
  std::lock_guard<std::mutex> lk(mtx_);
  ++command_sync_sequence_;
}

uint64_t SharedState::command_sync_sequence() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return command_sync_sequence_;
}

void SharedState::SetGlobalFault(bool fault) {
  std::lock_guard<std::mutex> lk(mtx_);
  global_fault_ = fault;
}

bool SharedState::GetGlobalFault() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return global_fault_;
}

void SharedState::SetAllAxesHaltedByFault(bool halted) {
  std::lock_guard<std::mutex> lk(mtx_);
  all_axes_halted_by_fault_ = halted;
}

bool SharedState::GetAllAxesHaltedByFault() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return all_axes_halted_by_fault_;
}

SharedSnapshot SharedState::Snapshot() const {
  std::lock_guard<std::mutex> lk(mtx_);
  SharedSnapshot s;
  s.feedback = feedback_;
  s.commands = commands_;
  s.safe_commands = safe_commands_;
  s.intents = intents_;
  s.intent_sequence = intent_sequence_;
  s.command_sync_sequence = command_sync_sequence_;
  s.all_operational = all_operational_;
  s.global_fault = global_fault_;
  s.all_axes_halted_by_fault = all_axes_halted_by_fault_;
  return s;
}

bool SharedState::IsValidAxis(std::size_t axis_index) const {
  return axis_index < axis_count_;
}

bool SharedState::WaitForStateChange(
    std::chrono::steady_clock::time_point deadline) {
  std::unique_lock<std::mutex> lk(mtx_);
  const uint64_t observed_seq = state_change_seq_;
  return state_cv_.wait_until(
      lk, deadline, [&]() { return state_change_seq_ != observed_seq; });
}

}  // namespace canopen_hw
