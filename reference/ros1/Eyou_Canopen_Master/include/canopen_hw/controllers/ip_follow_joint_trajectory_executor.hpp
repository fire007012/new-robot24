#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/ruckig.hpp>

namespace canopen_hw {

class IpFollowJointTrajectoryExecutor {
 public:
  struct State {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
  };

  enum class StepStatus {
    kIdle,
    kWorking,
    kFinished,
    kError,
  };

  struct Config {
    Config() = default;

    std::string action_ns{"ip_follow_joint_trajectory"};
    std::vector<std::string> joint_names{"shoulder_yaw_joint"};
    // joint_indices 仅在内部 State 向量中使用，不再关联特定 HW 子类。
    std::vector<std::size_t> joint_indices{0};
    double command_rate_hz{100.0};
    std::vector<double> max_velocities{1.0};
    std::vector<double> max_accelerations{2.0};
    std::vector<double> max_jerks{10.0};
    std::vector<double> goal_tolerances{1e-3};
  };

  // 通用构造：接收任何 hardware_interface::RobotHW*，
  // 内部通过 JointStateInterface / PositionJointInterface 获取 handle。
  IpFollowJointTrajectoryExecutor(ros::NodeHandle* pnh,
                                  hardware_interface::RobotHW* hw,
                                  std::mutex* loop_mtx);
  IpFollowJointTrajectoryExecutor(ros::NodeHandle* pnh,
                                  hardware_interface::RobotHW* hw,
                                  std::mutex* loop_mtx, Config config);

  void update(const ros::Time& now, const ros::Duration& period);
  bool enabled() const { return config_valid_ && server_ != nullptr; }

  static bool ValidateGoal(
      const control_msgs::FollowJointTrajectoryGoal& goal,
      const std::vector<std::string>& joint_names, std::string* error);

  bool startGoal(const control_msgs::FollowJointTrajectoryGoal& goal,
                 const State& actual, std::string* error);
  void cancelGoal();
  StepStatus step(const State& actual, State* command, std::string* error);
  bool hasActiveGoal() const;

 private:
  using Action = control_msgs::FollowJointTrajectoryAction;
  using GoalConstPtr = control_msgs::FollowJointTrajectoryGoalConstPtr;
  using Server = actionlib::SimpleActionServer<Action>;

  void ExecuteGoal(const GoalConstPtr& goal);
  void publishFeedback(const State& actual, const State& command) const;
  void holdCurrentPosition();

  hardware_interface::RobotHW* hw_raw_ = nullptr;
  std::mutex* loop_mtx_ = nullptr;
  Config config_;
  std::unique_ptr<Server> server_;

  // 缓存的 handle（构造时从 hw_raw_ 获取）。
  std::vector<hardware_interface::JointStateHandle> state_handles_;
  std::vector<hardware_interface::JointHandle> pos_cmd_handles_;

  mutable std::mutex exec_mtx_;
  std::condition_variable exec_cv_;
  std::optional<control_msgs::FollowJointTrajectoryGoal> active_goal_;
  std::optional<StepStatus> last_terminal_status_;
  std::string last_terminal_error_;
  std::string config_error_;
  bool config_valid_ = false;
  double cycle_remainder_sec_ = 0.0;
  double last_trajectory_time_ = 0.0;
  std::size_t waypoint_index_ = 0;
  std::vector<std::size_t> goal_to_config_indices_;
  ruckig::Ruckig<0> otg_;
  ruckig::InputParameter<0> input_;
  ruckig::OutputParameter<0> output_;
};

}  // namespace canopen_hw
