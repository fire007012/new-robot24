#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/operational_coordinator.hpp"
#include "canopen_hw/urdf_joint_limits.hpp"
#include "canopen_hw/zero_soft_limit_executor.hpp"

namespace canopen_hw {

// 封装原 canopen_hw_ros_node.cpp main() 中的辅助服务逻辑：
//   - ~/set_mode   (Eyou_Canopen_Master::SetMode)
//   - ~/set_zero   (Eyou_Canopen_Master::SetZero)
//   - URDF 软限位缓存与写入
//
// 构造时自动注册 ROS 服务，析构时由 ServiceServer RAII 自动注销。
class CanopenAuxServices {
 public:
  CanopenAuxServices(ros::NodeHandle* pnh,
                     CanopenRobotHwRos* robot_hw_ros,
                     OperationalCoordinator* coordinator,
                     const CanopenMasterConfig* master_cfg,
                     CanopenMaster* master,
                     std::mutex* loop_mtx);

  bool SetModeAxis(std::size_t axis_index, int8_t mode, std::string* detail);
  bool SetZeroAxis(std::size_t axis_index,
                   double zero_offset,
                   bool use_current_position_as_zero,
                   double* current_position,
                   double* applied_zero_offset,
                   std::string* detail);
  bool ApplyLimitsAxis(std::size_t axis_index,
                       bool use_urdf_limits,
                       double min_position,
                       double max_position,
                       bool require_current_inside_limits,
                       double* current_position,
                       double* applied_min_position,
                       double* applied_max_position,
                       std::string* detail);

  // 供 ServiceGateway::SetPostInitHook 或外部调用。
  // 当 auto_write_soft_limits_from_urdf 为 false 时直接返回 true。
  bool ApplySoftLimitAll(std::string* detail);

 private:
  struct PreparedSoftLimit {
    bool disable = false;
    int32_t min_counts = 0;
    int32_t max_counts = 0;
  };

  bool EnsureUrdfLimits(std::string* detail);
  bool ApplySoftLimitAxis(std::size_t axis_index, std::string* detail);
  bool ApplySoftLimitAxisManual(std::size_t axis_index,
                                double min_position,
                                double max_position,
                                PreparedSoftLimit* prepared,
                                std::string* detail);
  bool PrepareSoftLimitAxis(std::size_t axis_index, PreparedSoftLimit* out,
                            std::string* detail);
  bool WaitForAllStartupComplete(std::chrono::milliseconds timeout,
                                 std::string* detail) const;
  bool WaitForAllSdoIdle(std::chrono::milliseconds timeout, std::string* detail) const;
  bool WaitForAxisSdoIdle(std::size_t axis_index, std::chrono::milliseconds timeout,
                          std::string* detail) const;

  // 非拥有指针。
  CanopenRobotHwRos* robot_hw_ros_;
  OperationalCoordinator* coordinator_;
  const CanopenMasterConfig* master_cfg_;
  CanopenMaster* master_;
  std::mutex* loop_mtx_;

  ZeroSoftLimitExecutor zero_executor_;
  std::vector<JointLimitSpec> urdf_limits_;
  bool urdf_limits_ready_ = false;

  ros::ServiceServer set_mode_srv_;
  ros::ServiceServer set_zero_srv_;
  ros::ServiceServer apply_limits_srv_;
};

}  // namespace canopen_hw
