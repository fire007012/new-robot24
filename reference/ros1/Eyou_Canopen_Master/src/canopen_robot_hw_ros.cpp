#include "canopen_hw/canopen_robot_hw_ros.hpp"

#include <cassert>
#include <cmath>

#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

CanopenRobotHwRos::CanopenRobotHwRos(
    CanopenRobotHw* hw, const std::vector<std::string>& joint_names)
    : hw_(hw),
      pos_(joint_names.size(), 0.0),
      vel_(joint_names.size(), 0.0),
      eff_(joint_names.size(), 0.0),
      pos_cmd_(joint_names.size(), 0.0),
      vel_cmd_(joint_names.size(), 0.0),
      active_mode_(joint_names.size(), kMode_CSP),
      cmd_ready_(joint_names.size(), false),
      cmd_ready_guard_(joint_names.size(), cmd_ready_guard_frames_),
      arm_epoch_cache_(joint_names.size(), 0u),
      prev_arm_epoch_(joint_names.size(), 0u) {
  assert(hw_ != nullptr);
  assert(joint_names.size() == hw_->axis_count());

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    hardware_interface::JointStateHandle state_handle(
        joint_names[i], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_iface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(
        jnt_state_iface_.getHandle(joint_names[i]), &pos_cmd_[i]);
    pos_cmd_iface_.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(
        jnt_state_iface_.getHandle(joint_names[i]), &vel_cmd_[i]);
    vel_cmd_iface_.registerHandle(vel_handle);
  }

  registerInterface(&jnt_state_iface_);
  registerInterface(&pos_cmd_iface_);
  registerInterface(&vel_cmd_iface_);
}

void CanopenRobotHwRos::SetMode(std::size_t axis_index, int8_t mode) {
  if (axis_index >= active_mode_.size()) {
    return;
  }
  active_mode_[axis_index] = mode;
  hw_->SetJointMode(axis_index, mode);
}

void CanopenRobotHwRos::SetExternalPositionCommand(std::size_t axis_index,
                                                   double pos_rad) {
  if (axis_index >= pos_cmd_.size()) {
    return;
  }
  pos_cmd_[axis_index] = pos_rad;
}

double CanopenRobotHwRos::joint_position(std::size_t axis_index) const {
  return axis_index < pos_.size() ? pos_[axis_index] : 0.0;
}

double CanopenRobotHwRos::joint_velocity(std::size_t axis_index) const {
  return axis_index < vel_.size() ? vel_[axis_index] : 0.0;
}

double CanopenRobotHwRos::joint_effort(std::size_t axis_index) const {
  return axis_index < eff_.size() ? eff_[axis_index] : 0.0;
}

void CanopenRobotHwRos::read(const ros::Time& /*time*/,
                              const ros::Duration& /*period*/) {
  hw_->ReadFromSharedState();

  const uint64_t command_sync_sequence = hw_->command_sync_sequence();
  const bool command_sync_changed =
      (command_sync_sequence != prev_command_sync_sequence_);
  const bool now_all_axes_halted_by_fault = hw_->all_axes_halted_by_fault();
  const bool fault_halt_rising =
      (!prev_all_axes_halted_by_fault_ && now_all_axes_halted_by_fault);

  for (std::size_t i = 0; i < pos_.size(); ++i) {
    pos_[i] = hw_->joint_position(i);
    vel_[i] = hw_->joint_velocity(i);
    eff_[i] = hw_->joint_effort(i);
    arm_epoch_cache_[i] = hw_->arm_epoch(i);

    // arm_epoch 变化沿（非零且与上帧不同）是驱动器进入 OperationEnabled 的直接信号。
    // 用它替代 all_operational_rising 触发重同步，打破循环依赖：
    //   旧：need_resync ← all_operational ← is_operational ← cmd_valid ← need_resync（死锁）
    //   新：need_resync ← arm_epoch 变化（状态机 AdvanceArmEpoch，不依赖 is_operational）
    if (command_sync_changed) {
      // 新的命令重同步序列出现时，强制重新观察 arm_epoch 上升沿。
      prev_arm_epoch_[i] = 0u;
    }
    const bool epoch_changed =
        (arm_epoch_cache_[i] != 0u) && (arm_epoch_cache_[i] != prev_arm_epoch_[i]);

    if (command_sync_changed || epoch_changed || fault_halt_rising) {
      // 对齐命令缓冲到当前反馈，进入 guard 倒数。
      pos_cmd_[i] = pos_[i];
      cmd_ready_[i] = false;
      cmd_ready_guard_[i] = cmd_ready_guard_frames_;
    }
    prev_arm_epoch_[i] = arm_epoch_cache_[i];

    if (now_all_axes_halted_by_fault) {
      cmd_ready_[i] = false;
      continue;
    }

    if (!cmd_ready_[i] && cmd_ready_guard_[i] > 0) {
      --cmd_ready_guard_[i];
    }
    const bool position_aligned =
        std::abs(pos_cmd_[i] - pos_[i]) <= cmd_ready_position_threshold_rad_;
    if (!cmd_ready_[i] && cmd_ready_guard_[i] == 0 && position_aligned) {
      cmd_ready_[i] = true;
    }
  }

  prev_all_axes_halted_by_fault_ = now_all_axes_halted_by_fault;
  prev_command_sync_sequence_ = command_sync_sequence;
}

void CanopenRobotHwRos::write(const ros::Time& /*time*/,
                               const ros::Duration& /*period*/) {
  const bool fault_halted = hw_->all_axes_halted_by_fault();
  for (std::size_t i = 0; i < pos_cmd_.size(); ++i) {
    const bool ready =
        cmd_ready_[i] && !fault_halted && (arm_epoch_cache_[i] != 0u);
    switch (active_mode_[i]) {
      case kMode_CSV:
        hw_->SetJointVelocityCommand(i, vel_cmd_[i]);
        break;
      case kMode_IP:
      case kMode_CSP:
      default:
        // 在 controller 完成位置对齐前，持续向下层发送当前位置，
        // 避免 ros_control 持有的旧 setpoint（常见为 0）在 guard 结束后被放行。
        hw_->SetJointCommand(i, ready ? pos_cmd_[i] : pos_[i]);
        break;
    }

    hw_->SetCommandReady(i, ready);
    hw_->SetCommandEpoch(i, arm_epoch_cache_[i]);
  }
  hw_->WriteToSharedState();
}

}  // namespace canopen_hw
