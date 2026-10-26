#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// ROS1 ros_control 适配层。
// 将 CanopenRobotHw 的 read/write 语义桥接到 hardware_interface::RobotHW，
// 使 controller_manager 和 MoveIt 可以直接驱动。
class CanopenRobotHwRos : public hardware_interface::RobotHW {
 public:
  // joint_names 顺序必须与 CanopenRobotHw 的轴索引一一对应。
  CanopenRobotHwRos(CanopenRobotHw* hw,
                    const std::vector<std::string>& joint_names);

  // hardware_interface::RobotHW 接口。
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  // 设置某轴运动模式（kMode_IP=7, kMode_CSP=8, kMode_CSV=9）。
  void SetMode(std::size_t axis_index, int8_t mode);
  void SetExternalPositionCommand(std::size_t axis_index, double pos_rad);

  double joint_position(std::size_t axis_index) const;
  double joint_velocity(std::size_t axis_index) const;
  double joint_effort(std::size_t axis_index) const;

  bool all_operational() const { return hw_->all_operational(); }
  std::size_t axis_count() const { return pos_.size(); }

 private:
  CanopenRobotHw* hw_;  // 非拥有指针。

  // ros_control 要求这些数组在注册后地址不变，
  // 所以用 vector 在构造时一次性分配。
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> pos_cmd_;
  std::vector<double> vel_cmd_;
  std::vector<int8_t> active_mode_;  // 每轴当前模式，默认 CSP。
  std::vector<bool> cmd_ready_;
  std::vector<int> cmd_ready_guard_;
  std::vector<uint32_t> arm_epoch_cache_;

  std::vector<uint32_t> prev_arm_epoch_;  // 上帧各轴 arm_epoch，用于检测变化沿。
  uint64_t prev_command_sync_sequence_ = 0;
  bool prev_all_axes_halted_by_fault_ = false;
  int cmd_ready_guard_frames_ = 20;
  double cmd_ready_position_threshold_rad_ = 1e-3;

  hardware_interface::JointStateInterface jnt_state_iface_;
  hardware_interface::PositionJointInterface pos_cmd_iface_;
  hardware_interface::VelocityJointInterface vel_cmd_iface_;
};

}  // namespace canopen_hw
