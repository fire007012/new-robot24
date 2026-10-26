#include "ip_follow_joint_trajectory_executor_internal.hpp"

#include <algorithm>
#include <utility>

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

using namespace ip_follow_joint_trajectory_executor_internal;

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, hardware_interface::RobotHW* hw, std::mutex* loop_mtx)
    : IpFollowJointTrajectoryExecutor(pnh, hw, loop_mtx, Config{}) {}

IpFollowJointTrajectoryExecutor::IpFollowJointTrajectoryExecutor(
    ros::NodeHandle* pnh, hardware_interface::RobotHW* hw, std::mutex* loop_mtx,
    Config config)
    : hw_raw_(hw),
      loop_mtx_(loop_mtx),
      config_(std::move(config)),
      otg_(config_.joint_names.size(),
           1.0 / std::max(1.0, config_.command_rate_hz)),
      input_(config_.joint_names.size()),
      output_(config_.joint_names.size()) {
  config_valid_ = ValidateConfig(config_, &config_error_);
  if (!config_valid_) {
    CANOPEN_LOG_ERROR("IpFollowJointTrajectoryExecutor: invalid config: {}",
                      config_error_);
    return;
  }

  // 从 RobotHW 获取 handle 并缓存。
  if (hw_raw_ != nullptr) {
    auto* state_iface = hw_raw_->get<hardware_interface::JointStateInterface>();
    auto* pos_iface = hw_raw_->get<hardware_interface::PositionJointInterface>();
    if (state_iface == nullptr || pos_iface == nullptr) {
      config_valid_ = false;
      config_error_ = "RobotHW missing JointStateInterface or PositionJointInterface";
      CANOPEN_LOG_ERROR("IpFollowJointTrajectoryExecutor: {}", config_error_);
      return;
    }

    state_handles_.reserve(config_.joint_names.size());
    pos_cmd_handles_.reserve(config_.joint_names.size());
    for (const auto& name : config_.joint_names) {
      try {
        state_handles_.push_back(state_iface->getHandle(name));
        pos_cmd_handles_.push_back(pos_iface->getHandle(name));
      } catch (const hardware_interface::HardwareInterfaceException& e) {
        config_valid_ = false;
        config_error_ = "failed to get handle for joint '" + name + "': " + e.what();
        CANOPEN_LOG_ERROR("IpFollowJointTrajectoryExecutor: {}", config_error_);
        state_handles_.clear();
        pos_cmd_handles_.clear();
        return;
      }
    }
  }

  if (pnh == nullptr) {
    return;
  }

  server_ = std::make_unique<Server>(
      *pnh, config_.action_ns,
      [this](const GoalConstPtr& goal) { ExecuteGoal(goal); }, false);
  server_->start();
}

bool IpFollowJointTrajectoryExecutor::startGoal(
    const control_msgs::FollowJointTrajectoryGoal& goal, const State& actual,
    std::string* error) {
  if (error != nullptr) {
    error->clear();
  }

  if (!config_valid_) {
    SetError(error, config_error_);
    return false;
  }

  const std::size_t dofs = config_.joint_names.size();
  if (!ValidateState(actual, dofs, error)) {
    return false;
  }
  if (!ValidateGoal(goal, config_.joint_names, error)) {
    return false;
  }

  std::vector<std::size_t> goal_to_config_indices;
  if (!BuildGoalToConfigIndices(goal.trajectory.joint_names, config_.joint_names,
                                &goal_to_config_indices, error)) {
    return false;
  }

  std::lock_guard<std::mutex> lk(exec_mtx_);
  active_goal_ = goal;
  goal_to_config_indices_ = std::move(goal_to_config_indices);
  waypoint_index_ = 0;
  last_terminal_status_.reset();
  last_terminal_error_.clear();
  last_trajectory_time_ = 0.0;
  otg_.reset();

  for (std::size_t axis_index = 0; axis_index < dofs; ++axis_index) {
    input_.current_position[axis_index] = actual.positions[axis_index];
    input_.current_velocity[axis_index] =
        StateValueOrZero(actual.velocities, axis_index);
    input_.current_acceleration[axis_index] =
        StateValueOrZero(actual.accelerations, axis_index);
    input_.max_velocity[axis_index] = config_.max_velocities[axis_index];
    input_.max_acceleration[axis_index] =
        config_.max_accelerations[axis_index];
    input_.max_jerk[axis_index] = config_.max_jerks[axis_index];
  }

  const auto& first_point = active_goal_->trajectory.points.front();
  for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
    const std::size_t axis_index = goal_to_config_indices_[goal_index];
    input_.target_position[axis_index] = first_point.positions[goal_index];
    input_.target_velocity[axis_index] =
        first_point.velocities.empty() ? 0.0 : first_point.velocities[goal_index];
    input_.target_acceleration[axis_index] =
        first_point.accelerations.empty() ? 0.0
                                          : first_point.accelerations[goal_index];
  }

  const double first_duration = first_point.time_from_start.toSec();
  if (first_duration > 0.0) {
    input_.minimum_duration = first_duration;
  } else {
    input_.minimum_duration.reset();
  }

  if (!otg_.validate_input(input_)) {
    active_goal_.reset();
    goal_to_config_indices_.clear();
    SetError(error, "ruckig rejected goal input");
    return false;
  }

  return true;
}

void IpFollowJointTrajectoryExecutor::cancelGoal() {
  std::lock_guard<std::mutex> lk(exec_mtx_);
  active_goal_.reset();
  goal_to_config_indices_.clear();
  waypoint_index_ = 0;
  last_trajectory_time_ = 0.0;
  last_terminal_status_ = StepStatus::kIdle;
  last_terminal_error_.clear();
  otg_.reset();
  exec_cv_.notify_all();
}

IpFollowJointTrajectoryExecutor::StepStatus
IpFollowJointTrajectoryExecutor::step(const State& actual, State* command,
                                      std::string* error) {
  if (error != nullptr) {
    error->clear();
  }
  if (command == nullptr) {
    SetError(error, "command output is null");
    return StepStatus::kError;
  }
  if (!config_valid_) {
    SetError(error, config_error_);
    return StepStatus::kError;
  }

  const std::size_t dofs = config_.joint_names.size();
  if (!ValidateState(actual, dofs, error)) {
    return StepStatus::kError;
  }
  EnsureStateArrays(command, dofs);

  std::lock_guard<std::mutex> lk(exec_mtx_);
  if (!active_goal_) {
    command->positions = actual.positions;
    command->velocities.assign(dofs, 0.0);
    command->accelerations.assign(dofs, 0.0);
    for (std::size_t i = 0; i < dofs; ++i) {
      command->velocities[i] = StateValueOrZero(actual.velocities, i);
      command->accelerations[i] = StateValueOrZero(actual.accelerations, i);
    }
    return StepStatus::kIdle;
  }

  const ruckig::Result result = otg_.update(input_, output_);
  if (IsTerminalRuckigError(result)) {
    active_goal_.reset();
    goal_to_config_indices_.clear();
    waypoint_index_ = 0;
    last_trajectory_time_ = 0.0;
    last_terminal_status_ = StepStatus::kError;
    last_terminal_error_ = "ruckig step failed";
    exec_cv_.notify_all();
    command->positions = actual.positions;
    command->velocities.assign(dofs, 0.0);
    command->accelerations.assign(dofs, 0.0);
    for (std::size_t i = 0; i < dofs; ++i) {
      command->velocities[i] = StateValueOrZero(actual.velocities, i);
      command->accelerations[i] = StateValueOrZero(actual.accelerations, i);
    }
    SetError(error, last_terminal_error_);
    return StepStatus::kError;
  }

  for (std::size_t axis_index = 0; axis_index < dofs; ++axis_index) {
    command->positions[axis_index] = output_.new_position[axis_index];
    command->velocities[axis_index] = output_.new_velocity[axis_index];
    command->accelerations[axis_index] = output_.new_acceleration[axis_index];
  }
  last_trajectory_time_ = output_.time;
  output_.pass_to_input(input_);

  if (result != ruckig::Result::Finished) {
    return StepStatus::kWorking;
  }

  const auto& points = active_goal_->trajectory.points;
  const bool is_last_segment = (waypoint_index_ + 1 >= points.size());
  if (!is_last_segment) {
    ++waypoint_index_;
    const auto& previous_point = points[waypoint_index_ - 1];
    const auto& next_point = points[waypoint_index_];

    for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
      const std::size_t axis_index = goal_to_config_indices_[goal_index];
      input_.target_position[axis_index] = next_point.positions[goal_index];
      input_.target_velocity[axis_index] =
          next_point.velocities.empty() ? 0.0 : next_point.velocities[goal_index];
      input_.target_acceleration[axis_index] =
          next_point.accelerations.empty() ? 0.0
                                           : next_point.accelerations[goal_index];
    }

    const double segment_duration =
        next_point.time_from_start.toSec() - previous_point.time_from_start.toSec();
    if (segment_duration > 0.0) {
      input_.minimum_duration = segment_duration;
    } else {
      input_.minimum_duration.reset();
    }
    otg_.reset();
    return StepStatus::kWorking;
  }

  const auto& last_point = points.back();
  bool all_axes_reached = true;
  for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
    const std::size_t axis_index = goal_to_config_indices_[goal_index];
    if (std::abs(actual.positions[axis_index] - last_point.positions[goal_index]) >
        config_.goal_tolerances[axis_index]) {
      all_axes_reached = false;
      break;
    }
  }

  if (all_axes_reached) {
    active_goal_.reset();
    goal_to_config_indices_.clear();
    waypoint_index_ = 0;
    last_terminal_status_ = StepStatus::kFinished;
    last_terminal_error_.clear();
    exec_cv_.notify_all();
    return StepStatus::kFinished;
  }

  for (std::size_t goal_index = 0; goal_index < dofs; ++goal_index) {
    const std::size_t axis_index = goal_to_config_indices_[goal_index];
    command->positions[axis_index] = last_point.positions[goal_index];
    command->velocities[axis_index] = 0.0;
    command->accelerations[axis_index] = 0.0;
  }
  return StepStatus::kWorking;
}

bool IpFollowJointTrajectoryExecutor::hasActiveGoal() const {
  std::lock_guard<std::mutex> lk(exec_mtx_);
  return active_goal_.has_value();
}

}  // namespace canopen_hw
