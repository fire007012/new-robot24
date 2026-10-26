#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"

namespace canopen_hw {
namespace {

using Executor = IpFollowJointTrajectoryExecutor;

Executor::Config MakeConfig(std::size_t dofs) {
  Executor::Config config;
  config.action_ns = "test_follow_joint_trajectory";
  config.joint_names.clear();
  config.joint_indices.clear();
  config.max_velocities.clear();
  config.max_accelerations.clear();
  config.max_jerks.clear();
  config.goal_tolerances.clear();
  config.command_rate_hz = 100.0;

  for (std::size_t i = 0; i < dofs; ++i) {
    config.joint_names.push_back("joint_" + std::to_string(i + 1));
    config.joint_indices.push_back(i);
    config.max_velocities.push_back(2.0);
    config.max_accelerations.push_back(4.0);
    config.max_jerks.push_back(20.0);
    config.goal_tolerances.push_back(1e-4);
  }

  return config;
}

trajectory_msgs::JointTrajectoryPoint MakePoint(
    const std::vector<double>& positions, double time_from_start_sec,
    const std::vector<double>& velocities = {},
    const std::vector<double>& accelerations = {}) {
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities = velocities;
  point.accelerations = accelerations;
  point.time_from_start = ros::Duration(time_from_start_sec);
  return point;
}

control_msgs::FollowJointTrajectoryGoal MakeGoal(
    const std::vector<std::string>& joint_names,
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& points) {
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = joint_names;
  goal.trajectory.points = points;
  return goal;
}

Executor::State MakeState(const std::vector<double>& positions,
                          const std::vector<double>& velocities = {},
                          const std::vector<double>& accelerations = {}) {
  Executor::State state;
  state.positions = positions;
  state.velocities = velocities;
  state.accelerations = accelerations;
  return state;
}

TEST(IpFollowJointTrajectoryExecutor, RejectsJointCountMismatch) {
  const auto goal = MakeGoal({"joint_1"},
                             {MakePoint({1.0}, 1.0)});

  std::string error;
  EXPECT_FALSE(Executor::ValidateGoal(goal, {"joint_1", "joint_2"}, &error));
  EXPECT_EQ(error, "goal joint count mismatch: expected 2, got 1");
}

TEST(IpFollowJointTrajectoryExecutor, RejectsMalformedTrajectoryPointSizes) {
  const auto goal = MakeGoal({"joint_1", "joint_2"},
                             {MakePoint({1.0}, 1.0)});

  std::string error;
  EXPECT_FALSE(Executor::ValidateGoal(goal, {"joint_1", "joint_2"}, &error));
  EXPECT_EQ(error,
            "trajectory point[0] positions size mismatch: expected 2, got 1");
}

TEST(IpFollowJointTrajectoryExecutor, RejectsNonMonotonicTimePoints) {
  const auto goal = MakeGoal({"joint_1", "joint_2"},
                             {MakePoint({0.2, 0.1}, 0.5),
                              MakePoint({0.4, 0.3}, 0.4)});

  std::string error;
  EXPECT_FALSE(Executor::ValidateGoal(goal, {"joint_1", "joint_2"}, &error));
  EXPECT_EQ(error, "trajectory points must be time-ordered and nondecreasing");
}

TEST(IpFollowJointTrajectoryExecutor, AcceptsReorderedMultiJointGoal) {
  const auto goal = MakeGoal({"joint_2", "joint_1"},
                             {MakePoint({0.2, 0.1}, 0.5),
                              MakePoint({0.6, 0.4}, 1.0)});

  std::string error;
  EXPECT_TRUE(Executor::ValidateGoal(goal, {"joint_1", "joint_2"}, &error));
  EXPECT_TRUE(error.empty());
}

TEST(IpFollowJointTrajectoryExecutor, InvalidConfigIsRejectedAtStartGoal) {
  Executor::Config config = MakeConfig(2);
  config.goal_tolerances = {1e-4};
  Executor executor(nullptr, nullptr, nullptr, config);

  const auto goal = MakeGoal({"joint_1", "joint_2"},
                             {MakePoint({0.2, 0.1}, 0.5)});
  const Executor::State actual = MakeState({0.0, 0.0});

  std::string error;
  EXPECT_FALSE(executor.startGoal(goal, actual, &error));
  EXPECT_EQ(error, "goal_tolerances size mismatch: expected 2, got 1");
}

TEST(IpFollowJointTrajectoryExecutor, StartGoalMapsGoalOrderToConfigOrder) {
  const Executor::Config config = MakeConfig(2);
  Executor executor(nullptr, nullptr, nullptr, config);

  const auto goal = MakeGoal({"joint_2", "joint_1"},
                             {MakePoint({0.0, 1.0}, 1.0)});
  const Executor::State actual = MakeState({0.0, 0.0});

  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;

  Executor::State command;
  const auto status = executor.step(actual, &command, &error);
  EXPECT_EQ(status, Executor::StepStatus::kWorking);
  EXPECT_GT(command.positions[0], 0.0);
  EXPECT_LT(command.positions[0], 1.0);
  EXPECT_NEAR(command.positions[1], 0.0, 1e-12);
}

TEST(IpFollowJointTrajectoryExecutor,
     MultiSegmentTrajectoryFinishesWhenActualTracksCommand) {
  const Executor::Config config = MakeConfig(2);
  Executor executor(nullptr, nullptr, nullptr, config);

  const auto goal = MakeGoal(
      {"joint_1", "joint_2"},
      {MakePoint({0.4, -0.2}, 0.3), MakePoint({1.0, 0.5}, 0.8)});

  Executor::State actual = MakeState({0.0, 0.0});
  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;

  Executor::State command;
  Executor::StepStatus status = Executor::StepStatus::kIdle;
  for (int step = 0; step < 400; ++step) {
    status = executor.step(actual, &command, &error);
    ASSERT_NE(status, Executor::StepStatus::kError) << error;
    if (status == Executor::StepStatus::kFinished) {
      break;
    }
    actual = command;
  }

  EXPECT_EQ(status, Executor::StepStatus::kFinished);
  EXPECT_FALSE(executor.hasActiveGoal());
  EXPECT_NEAR(command.positions[0], 1.0, 1e-3);
  EXPECT_NEAR(command.positions[1], 0.5, 1e-3);
}

TEST(IpFollowJointTrajectoryExecutor,
     HoldsFinalCommandUntilActualReachesGoalTolerance) {
  const Executor::Config config = MakeConfig(1);
  Executor executor(nullptr, nullptr, nullptr, config);

  const auto goal = MakeGoal({"joint_1"},
                             {MakePoint({1.0}, 1.0)});
  const Executor::State actual = MakeState({0.0});

  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;

  Executor::State command;
  bool saw_hold_behavior = false;
  for (int step = 0; step < 300; ++step) {
    const auto status = executor.step(actual, &command, &error);
    ASSERT_NE(status, Executor::StepStatus::kError) << error;
    if (status == Executor::StepStatus::kWorking &&
        std::abs(command.positions[0] - 1.0) <= 1e-9) {
      saw_hold_behavior = true;
      break;
    }
  }

  EXPECT_TRUE(saw_hold_behavior);
  EXPECT_TRUE(executor.hasActiveGoal());
  EXPECT_NEAR(command.positions[0], 1.0, 1e-9);
  EXPECT_NEAR(command.velocities[0], 0.0, 1e-12);
}

TEST(IpFollowJointTrajectoryExecutor, CancelClearsActiveGoal) {
  const Executor::Config config = MakeConfig(2);
  Executor executor(nullptr, nullptr, nullptr, config);

  const auto goal = MakeGoal({"joint_1", "joint_2"},
                             {MakePoint({1.0, 0.5}, 1.0)});
  const Executor::State actual = MakeState({0.0, 0.0});

  std::string error;
  ASSERT_TRUE(executor.startGoal(goal, actual, &error)) << error;
  ASSERT_TRUE(executor.hasActiveGoal());

  executor.cancelGoal();
  EXPECT_FALSE(executor.hasActiveGoal());
}

}  // namespace
}  // namespace canopen_hw
