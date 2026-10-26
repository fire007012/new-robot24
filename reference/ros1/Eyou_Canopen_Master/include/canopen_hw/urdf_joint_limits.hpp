#pragma once

#include <string>
#include <vector>

#include "canopen_hw/canopen_master.hpp"

namespace canopen_hw {

enum class UrdfJointLimitUnit {
  kRadians = 0,
  kMeters = 1,
};

struct JointLimitSpec {
  bool has_position_limits = false;
  double lower = 0.0;
  double upper = 0.0;
  UrdfJointLimitUnit unit = UrdfJointLimitUnit::kRadians;
};

// 从 robot_description(URDF XML) 提取 joints 配置对应的关节上下限。
// 约束:
// - 接受 revolute/prismatic/continuous 关节。
// - revolute/prismatic 且具备位置上下限时，has_position_limits=true。
// - continuous 或未提供位置上下限时，has_position_limits=false，可用于关闭软限位。
// - joints 顺序与输入 config.joints 顺序保持一致。
bool ParseUrdfJointLimits(const std::string& urdf_xml,
                          const std::vector<CanopenMasterConfig::JointConfig>& joints,
                          std::vector<JointLimitSpec>* out_limits,
                          std::string* error = nullptr);

}  // namespace canopen_hw
