#include "ip_follow_joint_trajectory_executor_internal.hpp"

#include <algorithm>

namespace canopen_hw {
namespace ip_follow_joint_trajectory_executor_internal {

namespace {

bool HasDuplicateNames(const std::vector<std::string>& names,
                       std::string* duplicate_name) {
  for (std::size_t i = 0; i < names.size(); ++i) {
    for (std::size_t j = i + 1; j < names.size(); ++j) {
      if (names[i] == names[j]) {
        if (duplicate_name != nullptr) {
          *duplicate_name = names[i];
        }
        return true;
      }
    }
  }
  return false;
}

bool HasDuplicateIndices(const std::vector<std::size_t>& indices,
                         std::size_t* duplicate_index) {
  for (std::size_t i = 0; i < indices.size(); ++i) {
    for (std::size_t j = i + 1; j < indices.size(); ++j) {
      if (indices[i] == indices[j]) {
        if (duplicate_index != nullptr) {
          *duplicate_index = indices[i];
        }
        return true;
      }
    }
  }
  return false;
}

bool ValidateVectorSize(const std::vector<double>& values,
                        std::size_t expected_size,
                        const std::string& field_name,
                        std::string* error) {
  if (values.size() != expected_size) {
    SetError(error, field_name + " size mismatch: expected " +
                        std::to_string(expected_size) + ", got " +
                        std::to_string(values.size()));
    return false;
  }
  return true;
}

bool ValidateNonNegativeVector(const std::vector<double>& values,
                               const std::string& field_name,
                               bool allow_zero,
                               std::string* error) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    const bool ok = allow_zero ? (values[i] >= 0.0) : (values[i] > 0.0);
    if (!ok) {
      SetError(error, field_name + "[" + std::to_string(i) +
                          "] must be " + (allow_zero ? ">= 0" : "> 0"));
      return false;
    }
  }
  return true;
}

}  // namespace

void SetError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

bool ValidateConfig(const IpFollowJointTrajectoryExecutor::Config& config,
                    std::string* error) {
  if (config.action_ns.empty()) {
    SetError(error, "action_ns must not be empty");
    return false;
  }
  if (config.command_rate_hz <= 0.0) {
    SetError(error, "command_rate_hz must be > 0");
    return false;
  }

  const std::size_t dofs = config.joint_names.size();
  if (dofs == 0) {
    SetError(error, "joint_names must not be empty");
    return false;
  }
  if (config.joint_indices.size() != dofs) {
    SetError(error, "joint_indices size mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(config.joint_indices.size()));
    return false;
  }
  if (!ValidateVectorSize(config.max_velocities, dofs, "max_velocities",
                          error) ||
      !ValidateVectorSize(config.max_accelerations, dofs,
                          "max_accelerations", error) ||
      !ValidateVectorSize(config.max_jerks, dofs, "max_jerks", error) ||
      !ValidateVectorSize(config.goal_tolerances, dofs, "goal_tolerances",
                          error)) {
    return false;
  }

  std::string duplicate_name;
  if (HasDuplicateNames(config.joint_names, &duplicate_name)) {
    SetError(error, "joint_names contains duplicate joint: " + duplicate_name);
    return false;
  }

  std::size_t duplicate_index = 0;
  if (HasDuplicateIndices(config.joint_indices, &duplicate_index)) {
    SetError(error, "joint_indices contains duplicate axis index: " +
                        std::to_string(duplicate_index));
    return false;
  }

  if (!ValidateNonNegativeVector(config.max_velocities, "max_velocities", false,
                                 error) ||
      !ValidateNonNegativeVector(config.max_accelerations,
                                 "max_accelerations", false, error) ||
      !ValidateNonNegativeVector(config.max_jerks, "max_jerks", false, error) ||
      !ValidateNonNegativeVector(config.goal_tolerances, "goal_tolerances",
                                 true, error)) {
    return false;
  }

  return true;
}

bool ValidateState(const IpFollowJointTrajectoryExecutor::State& state,
                   std::size_t dofs, std::string* error) {
  if (state.positions.size() != dofs) {
    SetError(error, "state.positions size mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(state.positions.size()));
    return false;
  }
  if (!state.velocities.empty() && state.velocities.size() != dofs) {
    SetError(error, "state.velocities size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(state.velocities.size()));
    return false;
  }
  if (!state.accelerations.empty() && state.accelerations.size() != dofs) {
    SetError(error, "state.accelerations size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(state.accelerations.size()));
    return false;
  }
  return true;
}

void EnsureStateArrays(IpFollowJointTrajectoryExecutor::State* state,
                       std::size_t dofs) {
  if (state == nullptr) {
    return;
  }

  state->positions.resize(dofs, 0.0);
  state->velocities.resize(dofs, 0.0);
  state->accelerations.resize(dofs, 0.0);
}

double StateValueOrZero(const std::vector<double>& values, std::size_t index) {
  return index < values.size() ? values[index] : 0.0;
}

bool BuildGoalToConfigIndices(const std::vector<std::string>& goal_joint_names,
                              const std::vector<std::string>& config_joint_names,
                              std::vector<std::size_t>* goal_to_config_indices,
                              std::string* error) {
  if (goal_to_config_indices == nullptr) {
    SetError(error, "goal_to_config_indices is null");
    return false;
  }

  goal_to_config_indices->assign(goal_joint_names.size(), 0);
  for (std::size_t goal_index = 0; goal_index < goal_joint_names.size();
       ++goal_index) {
    const auto it =
        std::find(config_joint_names.begin(), config_joint_names.end(),
                  goal_joint_names[goal_index]);
    if (it == config_joint_names.end()) {
      SetError(error, "goal contains unknown joint: " +
                          goal_joint_names[goal_index]);
      return false;
    }
    (*goal_to_config_indices)[goal_index] =
        static_cast<std::size_t>(std::distance(config_joint_names.begin(), it));
  }

  return true;
}

bool ValidateTrajectoryPoint(
    const trajectory_msgs::JointTrajectoryPoint& point,
    std::size_t dofs, std::size_t point_index, std::string* error) {
  if (point.positions.size() != dofs) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] positions size mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(point.positions.size()));
    return false;
  }
  if (!point.velocities.empty() && point.velocities.size() != dofs) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] velocities size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(point.velocities.size()));
    return false;
  }
  if (!point.accelerations.empty() && point.accelerations.size() != dofs) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] accelerations size mismatch: expected 0 or " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(point.accelerations.size()));
    return false;
  }
  if (point.time_from_start.toSec() < 0.0) {
    SetError(error, "trajectory point[" + std::to_string(point_index) +
                        "] time_from_start must be >= 0");
    return false;
  }

  return true;
}

bool IsTerminalRuckigError(ruckig::Result result) {
  return result == ruckig::Result::ErrorInvalidInput ||
         result == ruckig::Result::Error ||
         result == ruckig::Result::ErrorTrajectoryDuration ||
         result == ruckig::Result::ErrorSynchronizationCalculation ||
         result == ruckig::Result::ErrorExecutionTimeCalculation;
}

}  // namespace ip_follow_joint_trajectory_executor_internal

bool IpFollowJointTrajectoryExecutor::ValidateGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal,
    const std::vector<std::string>& joint_names, std::string* error) {
  using namespace ip_follow_joint_trajectory_executor_internal;

  if (error != nullptr) {
    error->clear();
  }

  const std::size_t dofs = joint_names.size();
  if (dofs == 0) {
    SetError(error, "executor has no configured joints");
    return false;
  }

  const auto& goal_joint_names = goal.trajectory.joint_names;
  if (goal_joint_names.size() != dofs) {
    SetError(error, "goal joint count mismatch: expected " +
                        std::to_string(dofs) + ", got " +
                        std::to_string(goal_joint_names.size()));
    return false;
  }

  std::string duplicate_name;
  if (HasDuplicateNames(goal_joint_names, &duplicate_name)) {
    SetError(error, "goal contains duplicate joint: " + duplicate_name);
    return false;
  }

  std::vector<std::size_t> goal_to_config_indices;
  if (!BuildGoalToConfigIndices(goal_joint_names, joint_names,
                                &goal_to_config_indices, error)) {
    return false;
  }

  const auto& points = goal.trajectory.points;
  if (points.empty()) {
    SetError(error, "goal contains no trajectory points");
    return false;
  }

  double previous_time = -1.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!ValidateTrajectoryPoint(points[i], dofs, i, error)) {
      return false;
    }

    const double current_time = points[i].time_from_start.toSec();
    if (previous_time > current_time) {
      SetError(error,
               "trajectory points must be time-ordered and nondecreasing");
      return false;
    }
    previous_time = current_time;
  }

  return true;
}

}  // namespace canopen_hw
