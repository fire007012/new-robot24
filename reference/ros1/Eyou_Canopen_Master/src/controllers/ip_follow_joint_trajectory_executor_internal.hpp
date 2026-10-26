#pragma once

#include <string>
#include <vector>

#include <ruckig/ruckig.hpp>

#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"

namespace canopen_hw {
namespace ip_follow_joint_trajectory_executor_internal {

void SetError(std::string* error, const std::string& message);

bool ValidateConfig(const IpFollowJointTrajectoryExecutor::Config& config,
                    std::string* error);

bool ValidateState(const IpFollowJointTrajectoryExecutor::State& state,
                   std::size_t dofs, std::string* error);

void EnsureStateArrays(IpFollowJointTrajectoryExecutor::State* state,
                       std::size_t dofs);

double StateValueOrZero(const std::vector<double>& values, std::size_t index);

bool BuildGoalToConfigIndices(const std::vector<std::string>& goal_joint_names,
                              const std::vector<std::string>& config_joint_names,
                              std::vector<std::size_t>* goal_to_config_indices,
                              std::string* error);

bool ValidateTrajectoryPoint(
    const trajectory_msgs::JointTrajectoryPoint& point,
    std::size_t dofs, std::size_t point_index, std::string* error);

bool IsTerminalRuckigError(ruckig::Result result);

}  // namespace ip_follow_joint_trajectory_executor_internal
}  // namespace canopen_hw
