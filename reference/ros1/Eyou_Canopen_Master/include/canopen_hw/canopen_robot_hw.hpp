#pragma once

#include <cstddef>
#include <vector>

#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

struct CanopenMasterConfig;

// ROS 硬件层骨架:
// - 当前阶段不直接依赖 ROS 头文件，先把 read/write 数据流封装稳定
// - 后续 commit 再替换为 hardware_interface::RobotHW 继承实现
class CanopenRobotHw {
 public:
  struct AxisConversion {
    double counts_per_rev = 5308416.0;
    double rated_torque_nm = 6.0;
    double velocity_scale = 1.0;
    double torque_scale = 1.0;
  };

  // axis_count 从 SharedState 获取，保证两者一致。
  explicit CanopenRobotHw(SharedState* shared_state);

  std::size_t axis_count() const { return axis_count_; }

  // 对应 RobotHW::read():
  // 从 SharedState 拉取反馈并更新本地关节状态缓存。
  void ReadFromSharedState();

  // 对应 RobotHW::write():
  // 始终刷新目标位置/模式到 SharedState；
  // 未 operational 的轴会逐轴清零速度/力矩命令。
  void WriteToSharedState();

  // 测试/上层适配接口: 设置某轴目标位置(单位: rad)。
  void SetJointCommand(std::size_t axis_index, double pos_rad);

  // 设置某轴目标速度(单位: rad/s)。
  void SetJointVelocityCommand(std::size_t axis_index, double vel_rad_s);

  // 设置某轴目标力矩(单位: Nm)。
  void SetJointTorqueCommand(std::size_t axis_index, double torque_nm);

  // 设置某轴运动模式(kMode_CSP / kMode_CSV / kMode_CST)。
  void SetJointMode(std::size_t axis_index, int8_t mode);

  // 测试/上层适配接口: 读取关节状态缓存。
  double joint_position(std::size_t axis_index) const;
  double joint_velocity(std::size_t axis_index) const;
  double joint_effort(std::size_t axis_index) const;

  // 当前控制可运行标志(来自 SharedState::all_operational)。
  bool all_operational() const { return all_operational_; }

  // 全轴因故障被连带停机标志(来自 SharedState::all_axes_halted_by_fault)。
  bool all_axes_halted_by_fault() const { return all_axes_halted_by_fault_; }

  // 每轴当前使能会话号（来自反馈侧 arm_epoch）。
  uint32_t arm_epoch(std::size_t axis_index) const;
  // 命令重同步序列（来自 SharedState.command_sync_sequence）。
  uint64_t command_sync_sequence() const { return command_sync_sequence_; }

  // 命令元数据（valid/epoch）写入接口，由上层适配层驱动。
  void SetCommandReady(std::size_t axis_index, bool ready);
  void SetCommandEpoch(std::size_t axis_index, uint32_t epoch);

  // 配置某轴单位换算参数，供 joints.yaml 参数加载后调用。
  void ConfigureAxisConversion(std::size_t axis_index,
                               const AxisConversion& conversion);

  // 从主配置中批量应用单位换算参数。
  void ApplyConfig(const CanopenMasterConfig& config);

 private:
  bool IsValidAxis(std::size_t axis_index) const;

  // 单位换算(每轴参数化)。
  double TicksToRad(std::size_t axis_index, int32_t ticks) const;
  int32_t RadToTicks(std::size_t axis_index, double rad) const;
  double TicksPerSecToRadPerSec(std::size_t axis_index,
                                int32_t ticks_per_sec) const;
  int32_t RadPerSecToTicksPerSec(std::size_t axis_index,
                                 double rad_per_sec) const;
  double TorquePermilleToNm(std::size_t axis_index, int16_t permille) const;
  int16_t NmToTorquePermille(std::size_t axis_index, double nm) const;

  SharedState* shared_state_ = nullptr;  // 非拥有指针。
  const std::size_t axis_count_;

  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_cmd_;
  std::vector<double> joint_vel_cmd_;
  std::vector<double> joint_torque_cmd_;
  std::vector<int8_t> joint_mode_;
  std::vector<AxisConversion> axis_conv_;
  std::vector<uint32_t> axis_arm_epoch_;
  std::vector<bool> axis_cmd_ready_;
  std::vector<uint32_t> axis_cmd_epoch_;
  uint64_t command_sync_sequence_ = 0;

  bool all_operational_ = false;
  bool all_axes_halted_by_fault_ = false;

  // 每轴就绪状态缓存（per-axis is_operational），由 ReadFromSharedState 更新。
  // 用于 WriteToSharedState 中的逐轴目标位置保护，避免全局 all_operational_
  // 在多轴场景下让已就绪轴受到未就绪轴的牵连。
  std::vector<bool> axis_operational_;
};

}  // namespace canopen_hw
