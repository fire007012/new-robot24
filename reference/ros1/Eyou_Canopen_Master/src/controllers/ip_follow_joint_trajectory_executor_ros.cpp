#include "ip_follow_joint_trajectory_executor_internal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

using namespace ip_follow_joint_trajectory_executor_internal;

void IpFollowJointTrajectoryExecutor::update(const ros::Time& /*now*/,
                                             const ros::Duration& period) {
  if (!config_valid_ || state_handles_.empty()) {
    return;
  }

  cycle_remainder_sec_ += period.toSec();
  const double cycle = 1.0 / std::max(1.0, config_.command_rate_hz);
  if (cycle_remainder_sec_ + 1e-9 < cycle) {
    return;
  }
  cycle_remainder_sec_ = std::fmod(cycle_remainder_sec_, cycle);

  const std::size_t dofs = config_.joint_names.size();
  State actual;
  EnsureStateArrays(&actual, dofs);
  for (std::size_t i = 0; i < dofs; ++i) {
    actual.positions[i] = state_handles_[i].getPosition();
    actual.velocities[i] = state_handles_[i].getVelocity();
  }

  State command;
  std::string error;
  const StepStatus status = step(actual, &command, &error);

  if (status == StepStatus::kWorking || status == StepStatus::kFinished) {
    for (std::size_t i = 0; i < dofs; ++i) {
      pos_cmd_handles_[i].setCommand(command.positions[i]);
    }
    publishFeedback(actual, command);
  }
}

void IpFollowJointTrajectoryExecutor::publishFeedback(
    const State& actual, const State& command) const {
  if (!server_ || !server_->isActive()) {
    return;
  }

  const std::size_t dofs = config_.joint_names.size();
  control_msgs::FollowJointTrajectoryFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.joint_names = config_.joint_names;

  feedback.desired.positions = command.positions;
  feedback.desired.velocities = command.velocities;
  feedback.desired.accelerations = command.accelerations;
  feedback.desired.time_from_start = ros::Duration(last_trajectory_time_);

  feedback.actual.positions = actual.positions;
  feedback.actual.velocities.assign(dofs, 0.0);
  feedback.actual.accelerations.assign(dofs, 0.0);
  for (std::size_t i = 0; i < dofs; ++i) {
    feedback.actual.velocities[i] = StateValueOrZero(actual.velocities, i);
    feedback.actual.accelerations[i] = StateValueOrZero(actual.accelerations, i);
  }

  feedback.error.positions.resize(dofs, 0.0);
  feedback.error.velocities.resize(dofs, 0.0);
  feedback.error.accelerations.resize(dofs, 0.0);
  for (std::size_t i = 0; i < dofs; ++i) {
    feedback.error.positions[i] = command.positions[i] - actual.positions[i];
    feedback.error.velocities[i] =
        command.velocities[i] - feedback.actual.velocities[i];
    feedback.error.accelerations[i] =
        command.accelerations[i] - feedback.actual.accelerations[i];
  }
  feedback.error.time_from_start = ros::Duration(0.0);

  server_->publishFeedback(feedback);
}

void IpFollowJointTrajectoryExecutor::holdCurrentPosition() {
  if (state_handles_.empty() || loop_mtx_ == nullptr) {
    return;
  }

  // ExecuteGoal runs in actionlib callback thread; guard shared hw buffers
  // against concurrent access from the realtime control loop.
  std::lock_guard<std::mutex> lk(*loop_mtx_);
  for (std::size_t i = 0; i < state_handles_.size(); ++i) {
    const double pos = state_handles_[i].getPosition();
    pos_cmd_handles_[i].setCommand(pos);
  }
}

void IpFollowJointTrajectoryExecutor::ExecuteGoal(
    const GoalConstPtr& goal_ptr) {
  if (goal_ptr == nullptr || server_ == nullptr) {
    return;
  }

  if (!config_valid_) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = config_error_;
    server_->setAborted(result, result.error_string);
    return;
  }

  if (state_handles_.empty() || loop_mtx_ == nullptr) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "executor runtime not initialized";
    server_->setAborted(result, result.error_string);
    return;
  }

  const std::size_t dofs = config_.joint_names.size();
  State actual;
  EnsureStateArrays(&actual, dofs);
  {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    for (std::size_t i = 0; i < dofs; ++i) {
      actual.positions[i] = state_handles_[i].getPosition();
      actual.velocities[i] = state_handles_[i].getVelocity();
    }
  }

  std::string start_error;
  if (!startGoal(*goal_ptr, actual, &start_error)) {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string =
        start_error.empty() ? "failed to start goal" : start_error;
    server_->setAborted(result, result.error_string);
    return;
  }

  CANOPEN_LOG_INFO(
      "IpFollowJointTrajectoryExecutor: accepted goal for {} joints", dofs);

  std::unique_lock<std::mutex> lk(exec_mtx_);
  while (!last_terminal_status_.has_value()) {
    if (server_->isPreemptRequested()) {
      lk.unlock();
      cancelGoal();
      holdCurrentPosition();

      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      result.error_string = "goal preempted";
      server_->setPreempted(result, result.error_string);
      return;
    }

    exec_cv_.wait_for(lk, std::chrono::milliseconds(5));
  }

  control_msgs::FollowJointTrajectoryResult result;
  if (*last_terminal_status_ == StepStatus::kFinished) {
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    result.error_string = "goal completed";
    server_->setSucceeded(result, result.error_string);
    return;
  }

  holdCurrentPosition();
  result.error_code =
      control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
  result.error_string =
      last_terminal_error_.empty() ? "goal execution failed"
                                   : last_terminal_error_;
  server_->setAborted(result, result.error_string);
}

}  // namespace canopen_hw
