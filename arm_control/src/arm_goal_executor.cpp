#include <atomic>
#include "arm_control/motion_validation.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <mutex>
#include <set>
#include <thread>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_hardware/srv/group_operation.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <arm_control/action/execute_arm_goal.hpp>

using namespace std::chrono_literals;
using Action = arm_control::action::ExecuteArmGoal;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using Trajectory = control_msgs::action::FollowJointTrajectory;
using Steady = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_signal = 0;
void request_stop(int) { stop_signal = 1; }

class ArmExecutor : public rclcpp::Node {
public:
  ArmExecutor() : Node("arm_goal_executor", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}
  ~ArmExecutor() override { close(); }
  void close() {
    stopping_ = true;
    if (worker_.joinable()) worker_.join();
    // MoveGroupInterface retains its Node; release it explicitly before main
    // drops the final owner to avoid a Node -> MoveGroupInterface -> Node cycle.
    server_.reset(); group_.reset(); listener_.reset();
  }
  void initialize() {
    auto self = shared_from_this();
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, self, true);
    group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(self, "arm", tf_, rclcpp::Duration::from_seconds(20));
    for (const auto & name : names_) {
      const auto & bound = group_->getRobotModel()->getVariableBounds(name);
      limits_[name] = {bound.min_position_, bound.max_position_, bound.max_velocity_};
    }
    group_->setPlanningTime(5.0);
    group_->setMaxVelocityScalingFactor(0.2);
    group_->setMaxAccelerationScalingFactor(0.2);
    group_->setEndEffectorLink("grasp_tcp");
    trajectory_ = rclcpp_action::create_client<Trajectory>(self, "/arm_position_controller/follow_joint_trajectory");
    halt_ = create_client<robot_hardware::srv::GroupOperation>("/robot_runtime/group_operation");
    controllers_ = create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
    state_sub_ = create_subscription<std_msgs::msg::String>("/robot_runtime/state", 1,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        bool arm_running = false;
        std::string token = "global";
        bool parsed = false;
        try {
          const auto state = nlohmann::json::parse(msg->data);
          if (state.is_object()) {
            std::string arm_state;
            if (state.contains("groups") && state["groups"].is_object() &&
                state["groups"].contains("arm") && state["groups"]["arm"].is_string()) {
              arm_state = state["groups"]["arm"].get<std::string>();
            } else if (state.contains("state") && state["state"].is_string()) {
              arm_state = state["state"].get<std::string>();
            }
            arm_running = (arm_state == "Running");
            if (state.contains("input_epochs") && state["input_epochs"].is_object() &&
                state["input_epochs"].contains("arm")) {
              token = state["input_epochs"]["arm"].dump();
            } else if (state.contains("operations") && state["operations"].is_object() &&
                       state["operations"].contains("arm")) {
              token = state["operations"]["arm"].dump();
            }
            parsed = true;
          }
        } catch (const std::exception &) {
          arm_running = false;
        }
        std::lock_guard<std::mutex> guard(mutex_);
        if (!parsed) {
          arm_running_ = false;
          return;
        }
        arm_running_ = arm_running;
        token_ = std::move(token);
        state_at_ = Steady::now();
      });
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        arm_control::Values positions, velocities;
        const bool valid = arm_control::arm_feedback(*msg, positions, velocities);
        std::lock_guard<std::mutex> guard(mutex_);
        if (!valid) { joints_at_ = {}; return; }
        measured_ = std::move(positions); velocities_ = std::move(velocities);
        joints_at_ = Steady::now();
      });
    poll_ = create_wall_timer(250ms, [this]() {
      const auto now = Steady::now();
      if (polling_) {
        if (now-controllers_request_at_ < 500ms) return;
        controllers_->remove_pending_request(controllers_request_id_);
        ++controllers_generation_;  // A late response must not refresh old evidence.
        polling_ = false;
      }
      if (!controllers_->service_is_ready()) return;
      polling_ = true; controllers_request_at_ = now;
      const auto generation = ++controllers_generation_;
      try {
        auto pending = controllers_->async_send_request(
          std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
          [this, generation](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            if (generation != controllers_generation_) return;
            std::lock_guard<std::mutex> guard(mutex_);
            controller_active_ = false;
            try {
              for (const auto & c : future.get()->controller)
                if (c.name == "arm_position_controller" && c.state == "active") controller_active_ = true;
              controllers_at_ = Steady::now();
            } catch (const std::exception &) {}
            polling_ = false;
          });
        controllers_request_id_ = pending.request_id;
      } catch (const std::exception &) { polling_ = false; }
    });
    server_ = rclcpp_action::create_server<Action>(self, "/arm_control/execute_goal",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        bool expected = false;
        if (stopping_ || !busy_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Handle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<Handle> handle) {
        if (worker_.joinable()) worker_.join();
        worker_ = std::thread([this, handle]() { execute(handle); });
      });
    RCLCPP_INFO(get_logger(), "ExecuteArmGoal ready; one goal at a time, explicit runtime permission required");
  }
private:
  bool ready_locked(const std::string & token = "") const {
    const auto now = Steady::now();
    return arm_running_ && controller_active_ && now-state_at_ < 800ms && now-joints_at_ < 500ms &&
      now-controllers_at_ < 2s && (token.empty() || token == token_);
  }
  bool ready(const std::string & token = "") {
    std::lock_guard<std::mutex> guard(mutex_);
    return ready_locked(token);
  }
  std::string readiness_problem(const std::string & token = "") {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto now = Steady::now();
    if (!arm_running_) return "arm group is not Running";
    if (!controller_active_) return "arm trajectory controller is not active";
    if (now-state_at_ >= 800ms) return "runtime state is stale";
    if (now-joints_at_ >= 500ms) return "six-axis measured feedback is stale or invalid";
    if (now-controllers_at_ >= 2s) return "controller state readback is stale";
    if (!token.empty() && token != token_) return "arm operation session changed";
    return "arm readiness changed";
  }
  bool interrupted(const std::shared_ptr<Handle> & handle, const std::string & token) {
    return stopping_ || !rclcpp::ok() || handle->is_canceling() || !ready(token);
  }
  void feedback(const std::shared_ptr<Handle> & handle, uint8_t phase, const std::string & message) {
    if (!rclcpp::ok() || stopping_) return;
    auto value = std::make_shared<Action::Feedback>(); value->phase = phase; value->message = message;
    handle->publish_feedback(value);
  }
  void finish(const std::shared_ptr<Handle> & handle, uint8_t code, const std::string & message) {
    if (!rclcpp::ok()) return;
    try {
    auto result = std::make_shared<Action::Result>();
    result->success = code == Action::Result::ERROR_NONE;
    result->error_code = code; result->message = message;
    if (handle->is_canceling()) { result->success = false; result->error_code = Action::Result::ERROR_PREEMPTED; handle->canceled(result); }
    else if (result->success) handle->succeed(result);
    else handle->abort(result);
    } catch (const std::exception & error) {
      if (rclcpp::ok() && !stopping_) RCLCPP_ERROR(get_logger(), "Result publication failed: %s", error.what());
    }
  }
  void emergency_halt() {
    if (!rclcpp::ok() || stopping_ || !halt_->service_is_ready()) return;
    auto request = std::make_shared<robot_hardware::srv::GroupOperation::Request>();
    request->groups = {"arm"}; request->operation = "halt";
    halt_->async_send_request(request);
  }
  void execute(const std::shared_ptr<Handle> & handle) {
    struct Release { std::atomic<bool> & busy; ~Release() { busy = false; } } release{busy_};
    rclcpp_action::ClientGoalHandle<Trajectory>::SharedPtr controller_goal;
    try {
      feedback(handle, Action::Feedback::PHASE_VALIDATING, "Validating target");
      const auto goal = handle->get_goal();
      group_->clearPoseTargets();
      bool valid = false;
      if (goal->target_type == Action::Goal::TARGET_JOINTS) {
        const std::set<std::string> supplied(goal->joint_names.begin(), goal->joint_names.end());
        if (supplied != std::set<std::string>(names_.begin(), names_.end()) || goal->joint_positions.size()!=6 || goal->joint_names.size()!=6) {
          finish(handle, Action::Result::ERROR_INVALID_GOAL, "Exactly six unique arm joints required"); return;
        }
        for (double value : goal->joint_positions) if (!std::isfinite(value)) {
          finish(handle, Action::Result::ERROR_INVALID_GOAL, "Non-finite joint target"); return;
        }
        valid = group_->setJointValueTarget(goal->joint_names, goal->joint_positions);
      } else if (goal->target_type == Action::Goal::TARGET_NAMED) {
        valid = group_->setNamedTarget(goal->named_target);
      } else if (goal->target_type == Action::Goal::TARGET_POSE) {
        const auto & p = goal->pose_target.pose;
        double norm = p.orientation.x*p.orientation.x + p.orientation.y*p.orientation.y +
                      p.orientation.z*p.orientation.z + p.orientation.w*p.orientation.w;
        if (goal->pose_target.header.frame_id.empty() || !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
            !std::isfinite(p.position.z) || !std::isfinite(norm) || std::abs(norm-1.0)>0.01) {
          finish(handle, Action::Result::ERROR_INVALID_GOAL, "Pose requires a frame, finite position and unit quaternion"); return;
        }
        try {
          valid = group_->setPoseTarget(tf_->transform(goal->pose_target, group_->getPlanningFrame(), tf2::durationFromSec(0.2)));
        } catch (const tf2::TransformException & error) {
          finish(handle, Action::Result::ERROR_TF_FAILED, error.what()); return;
        }
      }
      if (!valid) { finish(handle, Action::Result::ERROR_INVALID_GOAL, "Unknown or out-of-bounds target"); return; }
      feedback(handle, Action::Feedback::PHASE_WAITING_READY, "Checking runtime, feedback and controller");
      const auto deadline = Steady::now()+2s;
      while (!ready() && Steady::now()<deadline && !handle->is_canceling() && !stopping_ && rclcpp::ok()) std::this_thread::sleep_for(20ms);
      if (!ready() || handle->is_canceling()) { finish(handle, Action::Result::ERROR_NOT_READY, "Arm not ready: " + readiness_problem()); return; }
      std::string token;
      { std::lock_guard<std::mutex> guard(mutex_); token=token_; }
      moveit_msgs::msg::RobotState start;
      start.is_diff = true;
      {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!arm_control::stationary(velocities_) || !arm_control::within_limits(measured_, limits_)) {
          finish(handle, Action::Result::ERROR_NOT_READY, "Arm must be stationary and inside joint limits before planning"); return;
        }
        for (const auto & name : names_) {
          start.joint_state.name.push_back(name);
          start.joint_state.position.push_back(measured_.at(name));
          start.joint_state.velocity.push_back(velocities_.at(name));
        }
      }
      group_->setStartState(start);
      feedback(handle, Action::Feedback::PHASE_PLANNING, "Planning with MoveIt 2");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto planned = group_->plan(plan);
      if (interrupted(handle, token)) { finish(handle, Action::Result::ERROR_NOT_READY, "Planning canceled or runtime session changed"); return; }
      if (planned != moveit::core::MoveItErrorCode::SUCCESS || plan.trajectory_.joint_trajectory.points.empty()) {
        finish(handle, Action::Result::ERROR_PLANNING_FAILED, "MoveIt planning failed: " + std::to_string(planned.val)); return;
      }
      if (!trajectory_->action_server_is_ready()) { finish(handle, Action::Result::ERROR_NOT_READY, "JTC action unavailable"); return; }
      const auto & joints = plan.trajectory_.joint_trajectory;
      if (!arm_control::valid_trajectory(joints, limits_)) {
        finish(handle, Action::Result::ERROR_PLANNING_FAILED, "Planned trajectory violates joint limits or format"); return;
      }
      bool at_target = true;
      {
        std::lock_guard<std::mutex> guard(mutex_);
        if (Steady::now()-joints_at_ >= 300ms || !arm_control::stationary(velocities_) ||
            !arm_control::matches_start(plan.start_state_.joint_state.name,
                                       plan.start_state_.joint_state.position, measured_) ||
            !arm_control::matches_start(joints.joint_names, joints.points.front().positions, measured_)) {
          finish(handle, Action::Result::ERROR_NOT_READY, "Arm moved from planned start; request a new plan"); return;
        }
        const auto & positions = joints.points.back().positions;
        for (size_t i=0; i<joints.joint_names.size(); ++i) {
          auto value = measured_.find(joints.joint_names[i]);
          if (i>=positions.size() || value==measured_.end() || !std::isfinite(positions[i]) ||
              std::abs(value->second-positions[i])>1e-5) at_target = false;
        }
      }
      if (at_target) { finish(handle, Action::Result::ERROR_NONE, "Already at planned target"); return; }
      Trajectory::Goal execution; execution.trajectory = plan.trajectory_.joint_trajectory;
      feedback(handle, Action::Feedback::PHASE_EXECUTING, "Executing planned trajectory across both backends");
      if (interrupted(handle, token)) {
        finish(handle, Action::Result::ERROR_NOT_READY, "Arm permission changed before dispatch"); return;
      }
      std::unique_lock<std::mutex> dispatch_guard(mutex_);
      if (!ready_locked(token) || !arm_control::stationary(velocities_) || !arm_control::within_limits(measured_, limits_) ||
          !arm_control::matches_start(plan.start_state_.joint_state.name, plan.start_state_.joint_state.position, measured_) ||
          !arm_control::matches_start(joints.joint_names, joints.points.front().positions, measured_)) {
        dispatch_guard.unlock();
        finish(handle, Action::Result::ERROR_NOT_READY, "Arm moved from planned start before dispatch; request a new plan"); return;
      }
      auto accepted = trajectory_->async_send_goal(execution);
      dispatch_guard.unlock();
      const auto accept_deadline = Steady::now()+3s;
      while (accepted.wait_for(20ms)!=std::future_status::ready) {
        if (Steady::now()>accept_deadline || interrupted(handle, token)) {
          trajectory_->async_cancel_all_goals(); emergency_halt();
          finish(handle, Action::Result::ERROR_EXECUTION_FAILED, "JTC goal acceptance timeout; halt requested"); return;
        }
      }
      controller_goal = accepted.get();
      if (!controller_goal) { finish(handle, Action::Result::ERROR_EXECUTION_FAILED, "JTC rejected trajectory"); return; }
      auto outcome = trajectory_->async_get_result(controller_goal);
      const auto & duration = execution.trajectory.points.back().time_from_start;
      const auto execute_deadline = Steady::now()+std::chrono::duration<double>(duration.sec+duration.nanosec*1e-9+5.0);
      bool canceled = false;
      std::string stop_reason;
      while (outcome.wait_for(20ms)!=std::future_status::ready) {
        if (interrupted(handle, token) || Steady::now()>execute_deadline) {
          feedback(handle, Action::Feedback::PHASE_STOPPING, "Canceling controller trajectory");
          stop_reason = handle->is_canceling() ? "client cancellation" :
            Steady::now()>execute_deadline ? "trajectory execution deadline exceeded" : readiness_problem(token);
          auto cancellation = trajectory_->async_cancel_goal(controller_goal);
          if (cancellation.wait_for(2s)!=std::future_status::ready || cancellation.get()->goals_canceling.empty() ||
              outcome.wait_for(2s)!=std::future_status::ready) {
            emergency_halt(); stop_reason = "cancellation confirmation timeout; halt requested";
          }
          canceled = true; break;
        }
      }
      if (canceled) { finish(handle, Action::Result::ERROR_EXECUTION_FAILED, "Execution stopped: " + stop_reason); return; }
      const auto result = outcome.get();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || result.result->error_code!=0) {
        finish(handle, Action::Result::ERROR_EXECUTION_FAILED, "Controller execution failed"); return;
      }
      finish(handle, Action::Result::ERROR_NONE, "Target reached");
    } catch (const std::exception & error) {
      if (controller_goal && rclcpp::ok() && !stopping_) {
        try { trajectory_->async_cancel_goal(controller_goal); emergency_halt(); }
        catch (const std::exception &) { /* Context may be shutting down. */ }
      }
      if (handle->is_active()) finish(handle, Action::Result::ERROR_EXECUTION_FAILED, error.what());
    }
  }
  const std::vector<std::string> names_{"shoulder_yaw_joint", "shoulder_pitch_joint", "elbow_pitch_joint", "wrist_pitch_joint", "wrist_yaw_joint", "wrist_roll_joint"};
  std::atomic<bool> busy_{false}, stopping_{false}, polling_{false};
  std::atomic<uint64_t> controllers_generation_{0};
  int64_t controllers_request_id_{0};
  Steady::time_point controllers_request_at_{};
  std::thread worker_;
  std::mutex mutex_;
  bool arm_running_=false, controller_active_=false;
  std::string token_;
  arm_control::Values measured_, velocities_;
  std::map<std::string, arm_control::Limits> limits_;
  Steady::time_point state_at_{}, joints_at_{}, controllers_at_{};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Client<Trajectory>::SharedPtr trajectory_;
  rclcpp::Client<robot_hardware::srv::GroupOperation>::SharedPtr halt_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr poll_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop); std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<ArmExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spinning([&]() { executor.spin(); });
  int result=0;
  try {
    node->initialize();
    while (rclcpp::ok() && !stop_signal) std::this_thread::sleep_for(20ms);
    // Keep DDS alive while canceling and completing the outstanding goal.
    node->close(); executor.cancel(); spinning.join();
  }
  catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "%s", error.what()); executor.cancel(); spinning.join(); result=1;
  }
  node->close(); executor.remove_node(node); node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
