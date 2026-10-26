#include "canopen_hw/canopen_robot_hw.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/logging.hpp"

namespace canopen_hw {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

CanopenRobotHw::CanopenRobotHw(SharedState* shared_state)
    : shared_state_(shared_state),
      axis_count_(shared_state ? shared_state->axis_count() : 0),
      joint_pos_(axis_count_, 0.0),
      joint_vel_(axis_count_, 0.0),
      joint_eff_(axis_count_, 0.0),
      joint_cmd_(axis_count_, 0.0),
      joint_vel_cmd_(axis_count_, 0.0),
      joint_torque_cmd_(axis_count_, 0.0),
      joint_mode_(axis_count_, kMode_CSP),
      axis_conv_(axis_count_),
      axis_arm_epoch_(axis_count_, 0),
      axis_cmd_ready_(axis_count_, false),
      axis_cmd_epoch_(axis_count_, 0),
      axis_operational_(axis_count_, false) {}

void CanopenRobotHw::ReadFromSharedState() {
  if (!shared_state_) {
    return;
  }

  const SharedSnapshot snap = shared_state_->Snapshot();
  all_operational_ = snap.all_operational;
  all_axes_halted_by_fault_ = snap.all_axes_halted_by_fault;
  command_sync_sequence_ = snap.command_sync_sequence;

  for (std::size_t i = 0; i < axis_count_; ++i) {
    joint_pos_[i] = TicksToRad(i, snap.feedback[i].actual_position);
    joint_vel_[i] = TicksPerSecToRadPerSec(i, snap.feedback[i].actual_velocity);
    joint_eff_[i] = TorquePermilleToNm(i, snap.feedback[i].actual_torque);
    axis_arm_epoch_[i] = snap.feedback[i].arm_epoch;

    const bool now_operational = snap.feedback[i].is_operational;
    if (!axis_operational_[i] && now_operational) {
      // 单轴 operational 上升沿：将指令同步为当前实际位置。
      // 作用：使 ros_control 控制器在接管前持有正确的初始指令，
      // 避免控制器以 0（初始化时反馈为 0）作为期望位置。
      joint_cmd_[i] = joint_pos_[i];
      if (snap.feedback[i].actual_position == 0) {
        CANOPEN_LOG_WARN(
            "joint {}: operational rising edge but actual_position=0, "
            "TPDO may not have arrived yet; joint_cmd_ set to 0.", i);
      }
    }
    axis_operational_[i] = now_operational;
  }
}

void CanopenRobotHw::WriteToSharedState() {
  if (!shared_state_) {
    return;
  }

  for (std::size_t i = 0; i < axis_count_; ++i) {
    AxisCommand cmd;
    if (axis_operational_[i]) {
      // 轴已就绪：透传 ROS 控制器指令。
      cmd.target_position = RadToTicks(i, joint_cmd_[i]);
    } else {
      // 轴未就绪：目标位置锁定到当前实际位置，绕过 ROS 控制器
      // 可能持有的旧指令（如初始化时反馈为 0 导致的零位指令），
      // 防止驱动器在就绪后立即跳向错误位置。
      // 使用 per-axis axis_operational_ 而非全局 all_operational_，
      // 避免单轴未就绪时影响其他已就绪轴的目标位置。
      cmd.target_position = RadToTicks(i, joint_pos_[i]);
    }
    if (axis_operational_[i]) {
      cmd.target_velocity = RadPerSecToTicksPerSec(i, joint_vel_cmd_[i]);
      cmd.target_torque = NmToTorquePermille(i, joint_torque_cmd_[i]);
    } else {
      cmd.target_velocity = 0;
      cmd.target_torque = 0;
    }
    cmd.mode_of_operation = joint_mode_[i];
    cmd.valid = axis_cmd_ready_[i];
    cmd.arm_epoch = axis_cmd_epoch_[i];
    shared_state_->UpdateCommand(i, cmd);
  }
}

void CanopenRobotHw::SetJointCommand(std::size_t axis_index, double pos_rad) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_cmd_[axis_index] = pos_rad;
}

void CanopenRobotHw::SetJointVelocityCommand(std::size_t axis_index,
                                              double vel_rad_s) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_vel_cmd_[axis_index] = vel_rad_s;
}

void CanopenRobotHw::SetJointTorqueCommand(std::size_t axis_index,
                                            double torque_nm) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_torque_cmd_[axis_index] = torque_nm;
}

void CanopenRobotHw::SetJointMode(std::size_t axis_index, int8_t mode) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  joint_mode_[axis_index] = mode;
}

double CanopenRobotHw::joint_position(std::size_t axis_index) const {
  return IsValidAxis(axis_index) ? joint_pos_[axis_index] : 0.0;
}

double CanopenRobotHw::joint_velocity(std::size_t axis_index) const {
  return IsValidAxis(axis_index) ? joint_vel_[axis_index] : 0.0;
}

double CanopenRobotHw::joint_effort(std::size_t axis_index) const {
  return IsValidAxis(axis_index) ? joint_eff_[axis_index] : 0.0;
}

uint32_t CanopenRobotHw::arm_epoch(std::size_t axis_index) const {
  return IsValidAxis(axis_index) ? axis_arm_epoch_[axis_index] : 0u;
}

void CanopenRobotHw::SetCommandReady(std::size_t axis_index, bool ready) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  axis_cmd_ready_[axis_index] = ready;
}

void CanopenRobotHw::SetCommandEpoch(std::size_t axis_index, uint32_t epoch) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  axis_cmd_epoch_[axis_index] = epoch;
}

bool CanopenRobotHw::IsValidAxis(std::size_t axis_index) const {
  return axis_index < axis_count_;
}

void CanopenRobotHw::ConfigureAxisConversion(
    std::size_t axis_index, const AxisConversion& conversion) {
  if (!IsValidAxis(axis_index)) {
    return;
  }
  axis_conv_[axis_index] = conversion;
}

void CanopenRobotHw::ApplyConfig(const CanopenMasterConfig& config) {
  const std::size_t n = std::min(axis_count_, config.joints.size());
  for (std::size_t i = 0; i < n; ++i) {
    joint_mode_[i] = config.joints[i].default_mode;

    AxisConversion conv;
    conv.counts_per_rev = config.joints[i].counts_per_rev;
    conv.rated_torque_nm = config.joints[i].rated_torque_nm;
    conv.velocity_scale = config.joints[i].velocity_scale;
    conv.torque_scale = config.joints[i].torque_scale;
    ConfigureAxisConversion(i, conv);

    if (shared_state_) {
      AxisCommand cmd{};
      shared_state_->GetCommand(i, &cmd);
      cmd.mode_of_operation = config.joints[i].default_mode;
      shared_state_->UpdateCommand(i, cmd);
    }
  }
}

double CanopenRobotHw::TicksToRad(std::size_t axis_index, int32_t ticks) const {
  const double counts_per_rev =
      std::max(1.0, axis_conv_[axis_index].counts_per_rev);
  return static_cast<double>(ticks) * (2.0 * kPi / counts_per_rev);
}

int32_t CanopenRobotHw::RadToTicks(std::size_t axis_index, double rad) const {
  const double counts_per_rev =
      std::max(1.0, axis_conv_[axis_index].counts_per_rev);
  return static_cast<int32_t>(
      std::llround(rad * counts_per_rev / (2.0 * kPi)));
}

double CanopenRobotHw::TicksPerSecToRadPerSec(std::size_t axis_index,
                                              int32_t ticks_per_sec) const {
  return TicksToRad(axis_index, ticks_per_sec) *
         axis_conv_[axis_index].velocity_scale;
}

int32_t CanopenRobotHw::RadPerSecToTicksPerSec(std::size_t axis_index,
                                                double rad_per_sec) const {
  const double counts_per_rev =
      std::max(1.0, axis_conv_[axis_index].counts_per_rev);
  const double scale = std::max(1e-9, axis_conv_[axis_index].velocity_scale);
  const long long raw =
      std::llround(rad_per_sec / scale * counts_per_rev / (2.0 * kPi));
  return static_cast<int32_t>(std::clamp(raw,
      static_cast<long long>(INT32_MIN), static_cast<long long>(INT32_MAX)));
}

double CanopenRobotHw::TorquePermilleToNm(std::size_t axis_index,
                                          int16_t permille) const {
  const double nm = static_cast<double>(permille) / 1000.0 *
                    axis_conv_[axis_index].rated_torque_nm;
  return nm * axis_conv_[axis_index].torque_scale;
}

int16_t CanopenRobotHw::NmToTorquePermille(std::size_t axis_index,
                                            double nm) const {
  const double rated = std::max(1e-9, axis_conv_[axis_index].rated_torque_nm);
  const double scale = std::max(1e-9, axis_conv_[axis_index].torque_scale);
  const long long raw = std::llround(nm / scale / rated * 1000.0);
  return static_cast<int16_t>(std::clamp(raw,
      static_cast<long long>(INT16_MIN), static_cast<long long>(INT16_MAX)));
}

}  // namespace canopen_hw
