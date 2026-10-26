#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <control_msgs/JointJog.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <urdf/model.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "Eyou_Canopen_Master/SetMode.h"
#include "Eyou_ROS1_Master/JointRuntimeStateArray.h"
#include "Eyou_ROS1_Master/SetJointMode.h"
#include "flipper_control/FlipperControlState.h"
#include "flipper_control/SetControlProfile.h"
#include "flipper_control/SetLinkageMode.h"
#include "flipper_control/flipper_reference_generator.hpp"

namespace flipper_control {
namespace {

enum class ControlProfile {
  kCspPosition = 0,
  kCspJog,
  kCsvVelocity,
};

enum class HardwareMode {
  kCsp = 0,
  kCsv,
};

enum class BackendType {
  kHybrid = 0,
  kCanopen,
};

enum class CsvCommandInterface {
  kTrajectory = 0,
  kMultiArray,
};

std::string ToString(ControlProfile profile) {
  switch (profile) {
    case ControlProfile::kCspPosition:
      return "csp_position";
    case ControlProfile::kCspJog:
      return "csp_jog";
    case ControlProfile::kCsvVelocity:
      return "csv_velocity";
    default:
      return "unknown";
  }
}

bool ParseControlProfile(const std::string& value, ControlProfile* profile) {
  if (profile == nullptr) {
    return false;
  }
  if (value == "csp_position") {
    *profile = ControlProfile::kCspPosition;
    return true;
  }
  if (value == "csp_jog") {
    *profile = ControlProfile::kCspJog;
    return true;
  }
  if (value == "csv_velocity") {
    *profile = ControlProfile::kCsvVelocity;
    return true;
  }
  return false;
}

bool ParseBackendType(const std::string& value, BackendType* backend_type) {
  if (backend_type == nullptr) {
    return false;
  }
  if (value == "hybrid") {
    *backend_type = BackendType::kHybrid;
    return true;
  }
  if (value == "canopen") {
    *backend_type = BackendType::kCanopen;
    return true;
  }
  return false;
}

bool ParseCsvCommandInterface(const std::string& value,
                              CsvCommandInterface* command_interface) {
  if (command_interface == nullptr) {
    return false;
  }
  if (value == "trajectory") {
    *command_interface = CsvCommandInterface::kTrajectory;
    return true;
  }
  if (value == "multi_array") {
    *command_interface = CsvCommandInterface::kMultiArray;
    return true;
  }
  return false;
}

std::string ToString(HardwareMode mode) {
  switch (mode) {
    case HardwareMode::kCsp:
      return "CSP";
    case HardwareMode::kCsv:
      return "CSV";
    default:
      return "UNKNOWN";
  }
}

HardwareMode HardwareModeForProfile(ControlProfile profile) {
  switch (profile) {
    case ControlProfile::kCsvVelocity:
      return HardwareMode::kCsv;
    case ControlProfile::kCspPosition:
    case ControlProfile::kCspJog:
    default:
      return HardwareMode::kCsp;
  }
}

std::string RouterModeForHardwareMode(HardwareMode mode) {
  switch (mode) {
    case HardwareMode::kCsv:
      return "velocity";
    case HardwareMode::kCsp:
    default:
      return "position";
  }
}

int CanopenModeForHardwareMode(HardwareMode mode) {
  return mode == HardwareMode::kCsv ? 9 : 8;
}

bool TrajectoryCommandProfile(ControlProfile profile) {
  return profile == ControlProfile::kCspPosition;
}

bool JogCommandProfile(ControlProfile profile) {
  return profile == ControlProfile::kCspJog ||
         profile == ControlProfile::kCsvVelocity;
}

std::string EnsureAbsoluteNs(const std::string& ns) {
  if (ns.empty()) {
    return std::string();
  }
  if (ns.front() == '/') {
    return ns;
  }
  return "/" + ns;
}

std::string ControllerCommandTopic(const std::string& controller_name) {
  if (controller_name.empty()) {
    return std::string();
  }
  if (controller_name.front() == '/') {
    return controller_name + "/command";
  }
  return "/" + controller_name + "/command";
}

bool ParseBoolish(const std::string& value) {
  return value == "1" || value == "true" || value == "True" ||
         value == "TRUE" || value == "yes" || value == "on";
}

}  // namespace

class FlipperManagerNode {
 public:
  FlipperManagerNode()
      : nh_(),
        pnh_("~"),
        active_profile_(ControlProfile::kCspPosition),
        active_hardware_mode_(HardwareMode::kCsp),
        linkage_mode_(LinkageMode::kIndependent),
        switching_(false),
        ready_(false),
        have_joint_state_(false),
        have_canopen_diag_(false),
        have_runtime_state_(false),
        require_backend_feedback_(true),
        service_wait_timeout_sec_(2.0),
        controller_switch_timeout_sec_(2.0),
        short_horizon_sec_(0.1),
        fallback_min_position_(-2.356),
        fallback_max_position_(2.356),
        fallback_max_velocity_(1.57) {
    LoadParameters();
    generator_ = std::make_shared<FlipperReferenceGenerator>(joint_names_);
    generator_->SetCommandTimeout(command_timeout_sec_);
    generator_->SetDtClamp(dt_clamp_sec_);
    generator_->SetLowpassAlpha(jog_velocity_alpha_);
    generator_->SetDriftThreshold(reference_drift_threshold_);
    generator_->SetLinkageMode(linkage_mode_);
    LoadJointLimits();

    trajectory_sub_ =
        pnh_.subscribe("command", 5, &FlipperManagerNode::OnTrajectoryCommand, this);
    jog_sub_ = pnh_.subscribe("jog_cmd", 10, &FlipperManagerNode::OnJogCommand, this);
    joint_state_sub_ =
        nh_.subscribe("/joint_states", 20, &FlipperManagerNode::OnJointStates, this);
    if (backend_type_ == BackendType::kHybrid) {
      runtime_state_sub_ = nh_.subscribe(
          runtime_state_topic_, 10, &FlipperManagerNode::OnRuntimeStates, this);
    } else {
      diagnostics_sub_ = nh_.subscribe(
          canopen_diagnostics_topic_, 20, &FlipperManagerNode::OnDiagnostics, this);
    }

    state_pub_ = pnh_.advertise<flipper_control::FlipperControlState>("state", 5, true);
    active_profile_pub_ = pnh_.advertise<std_msgs::String>("active_profile", 5, true);
    csp_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
        ControllerCommandTopic(controllers_.csp), 5);
    if (csv_command_interface_ == CsvCommandInterface::kTrajectory) {
      csv_command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
          ControllerCommandTopic(controllers_.csv), 5);
    } else {
      csv_velocity_command_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>(
              ControllerCommandTopic(controllers_.csv), 5);
    }

    set_profile_srv_ = pnh_.advertiseService(
        "set_control_profile", &FlipperManagerNode::OnSetControlProfile, this);
    set_linkage_mode_srv_ = pnh_.advertiseService(
        "set_linkage_mode", &FlipperManagerNode::OnSetLinkageMode, this);

    switch_controller_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
        controller_manager_ns_ + "/switch_controller");
    load_controller_client_ = nh_.serviceClient<controller_manager_msgs::LoadController>(
        controller_manager_ns_ + "/load_controller");
    list_controllers_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>(
        controller_manager_ns_ + "/list_controllers");
    halt_client_ =
        nh_.serviceClient<std_srvs::Trigger>(ControlBackendNs() + "/halt");
    disable_client_ =
        nh_.serviceClient<std_srvs::Trigger>(ControlBackendNs() + "/disable");
    enable_client_ =
        nh_.serviceClient<std_srvs::Trigger>(ControlBackendNs() + "/enable");
    resume_client_ =
        nh_.serviceClient<std_srvs::Trigger>(ControlBackendNs() + "/resume");
    if (backend_type_ == BackendType::kHybrid) {
      set_joint_mode_client_ = nh_.serviceClient<Eyou_ROS1_Master::SetJointMode>(
          hybrid_ns_ + "/set_joint_mode");
    } else {
      canopen_set_mode_client_ =
          nh_.serviceClient<Eyou_Canopen_Master::SetMode>(canopen_ns_ + "/set_mode");
    }

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, control_loop_hz_)),
        &FlipperManagerNode::OnControlTimer, this);
    state_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, state_publish_hz_)),
        &FlipperManagerNode::OnStateTimer, this);

    detail_ = "manager initialized";
    switch_state_ = "IDLE";
    PublishActiveProfile();
  }

 private:
  struct ControllerNames {
    std::string csp = "flipper_csp_controller";
    std::string csv = "flipper_csv_forward_controller";
  };

  struct CanopenDiagnosticState {
    bool is_operational = false;
    bool is_fault = false;
    bool heartbeat_lost = false;
  };

  double DirectionCorrection(std::size_t index) const {
    if (index >= direction_corrections_.size()) {
      return 1.0;
    }
    return direction_corrections_[index];
  }

  double ToSemanticPosition(std::size_t index, double hardware_value) const {
    return hardware_value / DirectionCorrection(index);
  }

  double ToHardwarePosition(std::size_t index, double semantic_value) const {
    return semantic_value * DirectionCorrection(index);
  }

  void TransformOrderedValuesToHardware(std::vector<double>* values) const {
    if (values == nullptr || values->size() != joint_names_.size()) {
      return;
    }
    for (std::size_t i = 0; i < values->size(); ++i) {
      (*values)[i] = ToHardwarePosition(i, (*values)[i]);
    }
  }

  void TransformTrajectoryToHardware(trajectory_msgs::JointTrajectory* traj) const {
    if (traj == nullptr) {
      return;
    }
    for (auto& point : traj->points) {
      if (point.positions.size() == joint_names_.size()) {
        TransformOrderedValuesToHardware(&point.positions);
      }
      if (point.velocities.size() == joint_names_.size()) {
        TransformOrderedValuesToHardware(&point.velocities);
      }
      if (point.accelerations.size() == joint_names_.size()) {
        TransformOrderedValuesToHardware(&point.accelerations);
      }
    }
  }

  double ParseDirectionCorrectionValue(const XmlRpc::XmlRpcValue& value,
                                       const std::string& joint_name) const {
    double correction = 1.0;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      correction = static_cast<int>(value);
    } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      correction = static_cast<double>(value);
    } else {
      ROS_WARN("direction_corrections/%s must be numeric, fallback to 1.0",
               joint_name.c_str());
      return 1.0;
    }

    if (!std::isfinite(correction) || std::abs(correction) < 1e-9) {
      ROS_WARN("direction_corrections/%s must be finite and non-zero, fallback to 1.0",
               joint_name.c_str());
      return 1.0;
    }
    return correction;
  }

  void LoadParameters() {
    if (!pnh_.getParam("joint_names", joint_names_)) {
      joint_names_ = {
          "left_front_arm_joint",
          "right_front_arm_joint",
          "left_rear_arm_joint",
          "right_rear_arm_joint",
      };
    }
    direction_corrections_.assign(joint_names_.size(), 1.0);

    pnh_.param("control_loop_hz", control_loop_hz_, 100.0);
    pnh_.param("state_publish_hz", state_publish_hz_, 20.0);
    pnh_.param("command_timeout", command_timeout_sec_, 0.4);
    pnh_.param("dt_clamp", dt_clamp_sec_, 0.02);
    pnh_.param("jog_velocity_alpha", jog_velocity_alpha_, 0.35);
    pnh_.param("reference_drift_threshold", reference_drift_threshold_, 0.35);
    pnh_.param("require_backend_feedback", require_backend_feedback_, true);
    pnh_.param("service_wait_timeout", service_wait_timeout_sec_, 2.0);
    pnh_.param("controller_switch_timeout", controller_switch_timeout_sec_, 2.0);
    pnh_.param("short_horizon", short_horizon_sec_, 0.1);
    pnh_.param("fallback_min_position", fallback_min_position_, -2.356);
    pnh_.param("fallback_max_position", fallback_max_position_, 2.356);
    pnh_.param("fallback_max_velocity", fallback_max_velocity_, 1.57);

    pnh_.param("controllers/csp", controllers_.csp, controllers_.csp);
    pnh_.param("controllers/csv", controllers_.csv, controllers_.csv);
    std::string csv_command_interface = "trajectory";
    pnh_.param("csv_command_interface", csv_command_interface,
               csv_command_interface);
    if (!ParseCsvCommandInterface(csv_command_interface,
                                  &csv_command_interface_)) {
      ROS_WARN("Unknown csv_command_interface '%s', fallback to trajectory",
               csv_command_interface.c_str());
      csv_command_interface_ = CsvCommandInterface::kTrajectory;
    }

    std::string backend_type = "hybrid";
    pnh_.param("backend_type", backend_type, backend_type);
    if (!ParseBackendType(backend_type, &backend_type_)) {
      ROS_WARN("Unknown backend_type '%s', fallback to hybrid", backend_type.c_str());
      backend_type_ = BackendType::kHybrid;
    }

    pnh_.param("controller_manager_ns", controller_manager_ns_,
               std::string("/controller_manager"));
    pnh_.param("hybrid_ns", hybrid_ns_, std::string("/hybrid_motor_hw_node"));
    pnh_.param("runtime_state_topic", runtime_state_topic_,
               hybrid_ns_ + "/joint_runtime_states");
    pnh_.param("canopen_ns", canopen_ns_, std::string("/canopen_hw_node"));
    pnh_.param("canopen_diagnostics_topic", canopen_diagnostics_topic_,
               std::string("/diagnostics"));
    controller_manager_ns_ = EnsureAbsoluteNs(controller_manager_ns_);
    hybrid_ns_ = EnsureAbsoluteNs(hybrid_ns_);
    runtime_state_topic_ = EnsureAbsoluteNs(runtime_state_topic_);
    canopen_ns_ = EnsureAbsoluteNs(canopen_ns_);
    canopen_diagnostics_topic_ = EnsureAbsoluteNs(canopen_diagnostics_topic_);

    std::string initial_profile = "csp_position";
    pnh_.param("initial_profile", initial_profile, initial_profile);
    if (!ParseControlProfile(initial_profile, &active_profile_)) {
      ROS_WARN("Unknown initial_profile '%s', fallback to csp_position",
               initial_profile.c_str());
      active_profile_ = ControlProfile::kCspPosition;
    }
    active_hardware_mode_ = HardwareModeForProfile(active_profile_);

    std::string initial_linkage_mode = "independent";
    pnh_.param("initial_linkage_mode", initial_linkage_mode, initial_linkage_mode);
    if (!ParseLinkageMode(initial_linkage_mode, &linkage_mode_)) {
      ROS_WARN("Unknown initial_linkage_mode '%s', fallback to independent",
               initial_linkage_mode.c_str());
      linkage_mode_ = LinkageMode::kIndependent;
    }

    XmlRpc::XmlRpcValue direction_corrections_param;
    if (pnh_.getParam("direction_corrections", direction_corrections_param)) {
      if (direction_corrections_param.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_WARN("direction_corrections must be a dictionary keyed by joint name");
      } else {
        for (std::size_t i = 0; i < joint_names_.size(); ++i) {
          const std::string& joint_name = joint_names_[i];
          if (!direction_corrections_param.hasMember(joint_name)) {
            continue;
          }
          direction_corrections_[i] = ParseDirectionCorrectionValue(
              direction_corrections_param[joint_name], joint_name);
        }
      }
    }

    active_controller_ = ControllerForProfile(active_profile_);
  }

  void LoadJointLimits() {
    std::vector<JointLimit> limits(joint_names_.size());
    for (auto& limit : limits) {
      limit.min_position = fallback_min_position_;
      limit.max_position = fallback_max_position_;
      limit.max_velocity = fallback_max_velocity_;
    }

    std::string robot_description;
    if (!nh_.getParam("/robot_description", robot_description)) {
      generator_->SetJointLimits(limits);
      ROS_WARN("robot_description not found, using fallback flipper limits");
      return;
    }

    urdf::Model model;
    if (!model.initString(robot_description)) {
      generator_->SetJointLimits(limits);
      ROS_WARN("failed to parse robot_description, using fallback flipper limits");
      return;
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const urdf::JointConstSharedPtr joint = model.getJoint(joint_names_[i]);
      if (!joint || !joint->limits) {
        continue;
      }
      if (joint->type != urdf::Joint::CONTINUOUS) {
        const double lower = ToSemanticPosition(i, joint->limits->lower);
        const double upper = ToSemanticPosition(i, joint->limits->upper);
        limits[i].min_position = std::min(lower, upper);
        limits[i].max_position = std::max(lower, upper);
      }
      const double velocity_scale = std::abs(DirectionCorrection(i));
      const double hardware_max_velocity =
          joint->limits->velocity > 0.0 ? joint->limits->velocity : fallback_max_velocity_;
      limits[i].max_velocity =
          velocity_scale > 1e-9 ? hardware_max_velocity / velocity_scale
                                : fallback_max_velocity_;
    }

    generator_->SetJointLimits(limits);
  }

  void RefreshReadyFlag() {
    ready_ = have_joint_state_ && !switching_;
    if (!ready_) {
      return;
    }
    if (!require_backend_feedback_) {
      return;
    }

    if (backend_type_ == BackendType::kHybrid) {
      if (!have_runtime_state_) {
        ready_ = false;
        return;
      }
      for (const auto& joint_name : joint_names_) {
        const auto it = runtime_state_map_.find(joint_name);
        if (it == runtime_state_map_.end()) {
          ready_ = false;
          return;
        }
        if (!it->second.online || it->second.fault) {
          ready_ = false;
          return;
        }
      }
      return;
    }

    if (!have_canopen_diag_) {
      ready_ = false;
      return;
    }
    for (const auto& joint_name : joint_names_) {
      const auto it = canopen_diag_state_map_.find(joint_name);
      if (it == canopen_diag_state_map_.end()) {
        ready_ = false;
        return;
      }
      if (it->second.is_fault || it->second.heartbeat_lost) {
        ready_ = false;
        return;
      }
    }
  }

  std::string ControllerForProfile(ControlProfile profile) const {
    switch (profile) {
      case ControlProfile::kCsvVelocity:
        return controllers_.csv;
      case ControlProfile::kCspPosition:
      case ControlProfile::kCspJog:
      default:
        return controllers_.csp;
    }
  }

  bool MotionAllowed() const {
    if (switching_ || !ready_) {
      return false;
    }
    if (backend_type_ == BackendType::kHybrid &&
        have_runtime_state_ && lifecycle_state_ != "Running") {
      return false;
    }
    return true;
  }

  std::vector<double> CurrentSeedPositions() const {
    if (generator_->reference_initialized()) {
      return generator_->reference_positions();
    }
    if (generator_->has_measurement()) {
      return generator_->measured_positions();
    }
    return std::vector<double>(joint_names_.size(), 0.0);
  }

  void OnJointStates(const sensor_msgs::JointStateConstPtr& msg) {
    if (msg->name.size() != msg->position.size()) {
      return;
    }

    std::vector<double> positions(joint_names_.size(), 0.0);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        return;
      }
      const std::size_t index = static_cast<std::size_t>(it - msg->name.begin());
      positions[i] = ToSemanticPosition(i, msg->position[index]);
    }

    std::string error;
    if (!generator_->SetMeasuredPositions(positions, &error)) {
      ROS_WARN_THROTTLE(1.0, "Failed to update measured positions: %s",
                        error.c_str());
      return;
    }
    have_joint_state_ = true;
    RefreshReadyFlag();
  }

  void OnRuntimeStates(const Eyou_ROS1_Master::JointRuntimeStateArrayConstPtr& msg) {
    std::map<std::string, Eyou_ROS1_Master::JointRuntimeState> updated;
    std::string lifecycle_state;
    for (const auto& state : msg->states) {
      if (std::find(joint_names_.begin(), joint_names_.end(), state.joint_name) ==
          joint_names_.end()) {
        continue;
      }
      updated[state.joint_name] = state;
      if (lifecycle_state.empty()) {
        lifecycle_state = state.lifecycle_state;
      }
    }

    if (updated.size() != joint_names_.size()) {
      return;
    }

    runtime_state_map_ = std::move(updated);
    lifecycle_state_ = lifecycle_state.empty() ? "Unknown" : lifecycle_state;
    have_runtime_state_ = true;
    RefreshReadyFlag();
  }

  std::string MatchJointName(const std::string& status_name) const {
    for (const auto& joint_name : joint_names_) {
      if (status_name == joint_name) {
        return joint_name;
      }
      if (status_name.size() >= joint_name.size() &&
          status_name.compare(status_name.size() - joint_name.size(),
                              joint_name.size(), joint_name) == 0) {
        return joint_name;
      }
    }
    return std::string();
  }

  void OnDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr& msg) {
    std::map<std::string, CanopenDiagnosticState> updated;
    for (const auto& status : msg->status) {
      const std::string joint_name = MatchJointName(status.name);
      if (joint_name.empty()) {
        continue;
      }

      CanopenDiagnosticState state;
      for (const auto& kv : status.values) {
        if (kv.key == "is_operational") {
          state.is_operational = ParseBoolish(kv.value);
        } else if (kv.key == "is_fault") {
          state.is_fault = ParseBoolish(kv.value);
        } else if (kv.key == "heartbeat_lost_flag") {
          state.heartbeat_lost = ParseBoolish(kv.value);
        }
      }
      updated[joint_name] = state;
    }

    if (updated.size() != joint_names_.size()) {
      return;
    }

    canopen_diag_state_map_ = std::move(updated);
    have_canopen_diag_ = true;
    lifecycle_state_ = "Unknown";
    RefreshReadyFlag();
  }

  void OnTrajectoryCommand(const trajectory_msgs::JointTrajectoryConstPtr& msg) {
    if (!TrajectoryCommandProfile(active_profile_)) {
      ROS_WARN_THROTTLE(1.0,
                        "Ignoring /flipper_control/command while active profile is %s",
                        ToString(active_profile_).c_str());
      return;
    }
    if (!MotionAllowed()) {
      ROS_WARN_THROTTLE(1.0,
                        "Ignoring trajectory command because flipper manager is not ready/running");
      return;
    }

    trajectory_msgs::JointTrajectory normalized;
    std::string error;
    if (!NormalizePositionTrajectory(*msg, &normalized, &error)) {
      detail_ = error;
      ROS_WARN("Reject flipper trajectory command: %s", error.c_str());
      return;
    }
    TransformTrajectoryToHardware(&normalized);
    csp_command_pub_.publish(normalized);
    detail_ = "published csp_position trajectory";
  }

  void OnJogCommand(const control_msgs::JointJogConstPtr& msg) {
    if (!JogCommandProfile(active_profile_)) {
      ROS_WARN_THROTTLE(1.0,
                        "Ignoring /flipper_control/jog_cmd while active profile is %s",
                        ToString(active_profile_).c_str());
      return;
    }
    if (!MotionAllowed()) {
      ROS_WARN_THROTTLE(1.0,
                        "Ignoring jog command because flipper manager is not ready/running");
      return;
    }

    std::vector<double> velocities;
    if (!msg->velocities.empty()) {
      velocities = msg->velocities;
    } else if (!msg->displacements.empty() && msg->duration > 0.0) {
      velocities.resize(msg->displacements.size(), 0.0);
      for (std::size_t i = 0; i < msg->displacements.size(); ++i) {
        velocities[i] = msg->displacements[i] / msg->duration;
      }
    } else {
      ROS_WARN("Reject jog command: velocities empty and cannot derive from displacements");
      return;
    }

    std::string error;
    if (!generator_->UpdateVelocityCommand(msg->joint_names, velocities, ros::Time::now(),
                                           &error)) {
      detail_ = error;
      ROS_WARN("Reject jog command: %s", error.c_str());
      return;
    }
    detail_ = "accepted jog command";
  }

  void OnControlTimer(const ros::TimerEvent& event) {
    if (!generator_->has_measurement()) {
      return;
    }

    const double dt_sec = (event.current_real - event.last_real).toSec();
    generator_->Step(event.current_real, dt_sec);

    if (switching_ || !JogCommandProfile(active_profile_)) {
      return;
    }
    if (!MotionAllowed()) {
      return;
    }

    if (active_profile_ == ControlProfile::kCsvVelocity) {
      if (csv_command_interface_ == CsvCommandInterface::kMultiArray) {
        PublishCsvVelocityCommand(generator_->filtered_velocities());
      } else {
        csv_command_pub_.publish(BuildJogTrajectory(event.current_real));
      }
    } else {
      csp_command_pub_.publish(BuildJogTrajectory(event.current_real));
    }
  }

  void OnStateTimer(const ros::TimerEvent& event) {
    flipper_control::FlipperControlState msg;
    msg.header.stamp = event.current_real;
    msg.active_profile = ToString(active_profile_);
    msg.active_hardware_mode = ToString(active_hardware_mode_);
    msg.active_controller = active_controller_;
    msg.linkage_mode = ToString(linkage_mode_);
    msg.switch_state = switch_state_;
    msg.lifecycle_state = lifecycle_state_;
    msg.detail = detail_;
    msg.switching = switching_;
    msg.ready = ready_;
    msg.command_timed_out = generator_->command_timed_out();
    msg.degraded = generator_->degraded();
    msg.joint_names = joint_names_;
    msg.measured_positions = generator_->measured_positions();
    msg.reference_positions = generator_->reference_positions();
    msg.commanded_velocities = generator_->filtered_velocities();
    state_pub_.publish(msg);
    PublishActiveProfile();
  }

  bool OnSetControlProfile(flipper_control::SetControlProfile::Request& req,
                           flipper_control::SetControlProfile::Response& res) {
    ControlProfile target_profile;
    if (!ParseControlProfile(req.profile, &target_profile)) {
      res.success = false;
      res.message = "unknown profile: " + req.profile;
      PopulateProfileResponse(res);
      return true;
    }

    std::string error;
    const bool success = SwitchProfile(target_profile, &error);
    res.success = success;
    res.message = success ? "profile switched" : error;
    PopulateProfileResponse(res);
    return true;
  }

  bool OnSetLinkageMode(flipper_control::SetLinkageMode::Request& req,
                        flipper_control::SetLinkageMode::Response& res) {
    LinkageMode requested_mode;
    if (!ParseLinkageMode(req.mode, &requested_mode)) {
      res.success = false;
      res.message = "unknown linkage mode: " + req.mode;
      res.active_mode = ToString(linkage_mode_);
      return true;
    }

    linkage_mode_ = requested_mode;
    generator_->SetLinkageMode(linkage_mode_);
    generator_->ClearVelocityCommand();
    generator_->ResetReferenceToMeasured();
    detail_ = "linkage mode updated";
    res.success = true;
    res.message = "linkage mode updated";
    res.active_mode = ToString(linkage_mode_);
    return true;
  }

  void PopulateProfileResponse(flipper_control::SetControlProfile::Response& res) const {
    res.active_profile = ToString(active_profile_);
    res.active_hardware_mode = ToString(active_hardware_mode_);
    res.active_controller = active_controller_;
  }

  bool SwitchProfile(ControlProfile target_profile, std::string* error) {
    if (target_profile == active_profile_) {
      detail_ = "requested profile already active";
      return true;
    }

    if (HardwareModeForProfile(target_profile) == active_hardware_mode_) {
      return InternalSwitchProfile(target_profile, error);
    }
    return PerformColdSwitch(target_profile, error);
  }

  bool InternalSwitchProfile(ControlProfile target_profile, std::string* error) {
    if (active_hardware_mode_ != HardwareMode::kCsp) {
      if (error != nullptr) {
        *error = "internal switch is only supported inside CSP";
      }
      return false;
    }

    generator_->ClearVelocityCommand();
    generator_->ResetReferenceToMeasured();
    active_profile_ = target_profile;
    active_controller_ = controllers_.csp;
    detail_ = "profile switched inside CSP without cold switch";
    PublishHoldCommand(active_hardware_mode_, ros::Time::now());
    return true;
  }

  bool PerformColdSwitch(ControlProfile target_profile, std::string* error) {
    if (backend_type_ == BackendType::kCanopen) {
      return PerformColdSwitchCanopen(target_profile, error);
    }
    if (!have_runtime_state_) {
      if (error != nullptr) {
        *error = "joint_runtime_states not ready, cannot cold switch";
      }
      return false;
    }
    if (lifecycle_state_ == "Faulted") {
      if (error != nullptr) {
        *error = "cannot cold switch while lifecycle is Faulted";
      }
      return false;
    }

    const HardwareMode target_mode = HardwareModeForProfile(target_profile);
    const std::string target_controller = ControllerForProfile(target_profile);
    const std::string previous_lifecycle = lifecycle_state_;

    switching_ = true;
    switch_state_ = "HALTING";
    detail_ = "performing cold switch";
    generator_->ClearVelocityCommand();

    auto fail = [&](const std::string& message) {
      switching_ = false;
      switch_state_ = "ERROR";
      detail_ = message;
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };

    if (previous_lifecycle == "Running") {
      if (!CallTrigger(halt_client_, "halt", error)) {
        return fail(*error);
      }
    }

    if (previous_lifecycle == "Running" || previous_lifecycle == "Armed") {
      switch_state_ = "DISABLING";
      if (!CallTrigger(disable_client_, "disable", error)) {
        return fail(*error);
      }
    } else if (previous_lifecycle != "Standby") {
      return fail("unsupported lifecycle state for cold switch: " + previous_lifecycle);
    }

    switch_state_ = "STOPPING_CONTROLLER";
    if (!active_controller_.empty()) {
      if (!SwitchControllers({}, {active_controller_}, error)) {
        return fail(*error);
      }
    }

    switch_state_ = "SWITCHING_MODE";
    for (const auto& joint_name : joint_names_) {
      if (!CallSetJointMode(joint_name, RouterModeForHardwareMode(target_mode), error)) {
        return fail(*error);
      }
    }

    switch_state_ = "STARTING_CONTROLLER";
    if (!EnsureControllerLoaded(target_controller, error)) {
      return fail(*error);
    }
    if (!SwitchControllers({target_controller}, {}, error)) {
      return fail(*error);
    }

    switch_state_ = "PRIMING_OUTPUT";
    active_profile_ = target_profile;
    active_hardware_mode_ = target_mode;
    active_controller_ = target_controller;
    generator_->ResetReferenceToMeasured();
    PublishHoldCommand(target_mode, ros::Time::now());
    ros::Duration(0.02).sleep();
    PublishHoldCommand(target_mode, ros::Time::now());

    if (previous_lifecycle == "Armed" || previous_lifecycle == "Running") {
      switch_state_ = "ENABLING";
      if (!CallTrigger(enable_client_, "enable", error)) {
        return fail(*error);
      }
    }
    if (previous_lifecycle == "Running") {
      switch_state_ = "RESUMING";
      if (!CallTrigger(resume_client_, "resume", error)) {
        return fail(*error);
      }
    }

    switching_ = false;
    switch_state_ = "DONE";
    detail_ = "cold switch completed";
    return true;
  }

  bool PerformColdSwitchCanopen(ControlProfile target_profile, std::string* error) {
    const HardwareMode target_mode = HardwareModeForProfile(target_profile);
    const std::string target_controller = ControllerForProfile(target_profile);

    switching_ = true;
    switch_state_ = "DISABLING";
    detail_ = "performing cold switch via canopen backend";
    generator_->ClearVelocityCommand();

    auto fail = [&](const std::string& message) {
      switching_ = false;
      switch_state_ = "ERROR";
      detail_ = message;
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };

    std::string best_effort;
    CallTrigger(halt_client_, "halt", &best_effort);

    if (!CallTrigger(disable_client_, "disable", error)) {
      return fail(error != nullptr ? *error : "disable failed");
    }

    switch_state_ = "STOPPING_CONTROLLER";
    if (!active_controller_.empty()) {
      if (!SwitchControllers({}, {active_controller_}, error)) {
        return fail(*error);
      }
    }

    switch_state_ = "SWITCHING_MODE";
    for (std::size_t axis_index = 0; axis_index < joint_names_.size(); ++axis_index) {
      if (!CallSetCanopenMode(axis_index, target_mode, error)) {
        return fail(*error);
      }
    }

    switch_state_ = "STARTING_CONTROLLER";
    if (!EnsureControllerLoaded(target_controller, error)) {
      return fail(*error);
    }
    if (!SwitchControllers({target_controller}, {}, error)) {
      return fail(*error);
    }

    switch_state_ = "PRIMING_OUTPUT";
    active_profile_ = target_profile;
    active_hardware_mode_ = target_mode;
    active_controller_ = target_controller;
    generator_->ResetReferenceToMeasured();
    PublishHoldCommand(target_mode, ros::Time::now());
    ros::Duration(0.02).sleep();
    PublishHoldCommand(target_mode, ros::Time::now());

    switch_state_ = "ENABLING";
    if (!CallTrigger(enable_client_, "enable", error)) {
      return fail(*error);
    }

    switch_state_ = "RESUMING";
    if (!CallTrigger(resume_client_, "resume", error)) {
      return fail(*error);
    }

    switching_ = false;
    switch_state_ = "DONE";
    lifecycle_state_ = "Running";
    detail_ = "cold switch completed";
    return true;
  }

  bool NormalizePositionTrajectory(const trajectory_msgs::JointTrajectory& input,
                                   trajectory_msgs::JointTrajectory* output,
                                   std::string* error) const {
    if (output == nullptr) {
      if (error != nullptr) {
        *error = "output trajectory is null";
      }
      return false;
    }
    if (input.joint_names.empty() || input.points.empty()) {
      if (error != nullptr) {
        *error = "trajectory must include joint_names and at least one point";
      }
      return false;
    }

    output->header = input.header;
    output->joint_names = joint_names_;
    output->points.clear();
    output->points.reserve(input.points.size());

    std::vector<double> seed_positions = CurrentSeedPositions();
    for (const auto& point : input.points) {
      if (point.positions.size() != input.joint_names.size()) {
        if (error != nullptr) {
          *error = "each trajectory point must provide positions for all named joints";
        }
        return false;
      }

      trajectory_msgs::JointTrajectoryPoint normalized_point;
      normalized_point.time_from_start = point.time_from_start;
      if (!generator_->ExpandValues(input.joint_names, point.positions, seed_positions,
                                    &normalized_point.positions, error)) {
        return false;
      }

      if (!point.velocities.empty()) {
        if (point.velocities.size() != input.joint_names.size()) {
          if (error != nullptr) {
            *error = "trajectory velocities size mismatch";
          }
          return false;
        }
        if (!generator_->ExpandValues(
                input.joint_names, point.velocities,
                std::vector<double>(joint_names_.size(), 0.0),
                &normalized_point.velocities, error)) {
          return false;
        }
      }

      if (!point.accelerations.empty()) {
        if (point.accelerations.size() != input.joint_names.size()) {
          if (error != nullptr) {
            *error = "trajectory accelerations size mismatch";
          }
          return false;
        }
        if (!generator_->ExpandValues(
                input.joint_names, point.accelerations,
                std::vector<double>(joint_names_.size(), 0.0),
                &normalized_point.accelerations, error)) {
          return false;
        }
      }

      seed_positions = normalized_point.positions;
      output->points.push_back(std::move(normalized_point));
    }
    return true;
  }

  trajectory_msgs::JointTrajectory BuildJogTrajectory(const ros::Time& stamp) const {
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = stamp;
    traj.joint_names = joint_names_;

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(short_horizon_sec_);
    point.positions = generator_->reference_positions();
    point.velocities = generator_->filtered_velocities();
    TransformOrderedValuesToHardware(&point.positions);
    TransformOrderedValuesToHardware(&point.velocities);

    traj.points.push_back(std::move(point));
    return traj;
  }

  void PublishHoldCommand(HardwareMode mode, const ros::Time& stamp) {
    if (!generator_->has_measurement()) {
      return;
    }

    if (mode == HardwareMode::kCsv &&
        csv_command_interface_ == CsvCommandInterface::kMultiArray) {
      PublishCsvVelocityCommand(std::vector<double>(joint_names_.size(), 0.0));
      return;
    }

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = stamp;
    traj.joint_names = joint_names_;

    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(short_horizon_sec_);
    point.positions = generator_->measured_positions();
    point.velocities.assign(joint_names_.size(), 0.0);
    TransformOrderedValuesToHardware(&point.positions);
    traj.points.push_back(std::move(point));

    if (mode == HardwareMode::kCsv) {
      csv_command_pub_.publish(traj);
    } else {
      csp_command_pub_.publish(traj);
    }
  }

  void PublishCsvVelocityCommand(
      const std::vector<double>& semantic_velocities) {
    std_msgs::Float64MultiArray msg;
    msg.data = semantic_velocities;
    TransformOrderedValuesToHardware(&msg.data);
    csv_velocity_command_pub_.publish(msg);
  }

  bool WaitForService(ros::ServiceClient* client, const std::string& name,
                      std::string* error) {
    if (client == nullptr) {
      if (error != nullptr) {
        *error = "service client is null for " + name;
      }
      return false;
    }
    if (client->waitForExistence(ros::Duration(service_wait_timeout_sec_))) {
      return true;
    }
    if (error != nullptr) {
      *error = "service unavailable: " + name;
    }
    return false;
  }

  bool CallTrigger(ros::ServiceClient& client, const std::string& name,
                   std::string* error) {
    if (!WaitForService(&client, name, error)) {
      return false;
    }
    std_srvs::Trigger srv;
    if (!client.call(srv)) {
      if (error != nullptr) {
        *error = "failed to call service: " + name;
      }
      return false;
    }
    if (!srv.response.success) {
      if (error != nullptr) {
        *error = name + " failed: " + srv.response.message;
      }
      return false;
    }
    return true;
  }

  bool CallSetJointMode(const std::string& joint_name, const std::string& mode,
                        std::string* error) {
    if (!WaitForService(&set_joint_mode_client_,
                        hybrid_ns_ + "/set_joint_mode", error)) {
      return false;
    }

    Eyou_ROS1_Master::SetJointMode srv;
    srv.request.joint_name = joint_name;
    srv.request.mode = mode;
    if (!set_joint_mode_client_.call(srv)) {
      if (error != nullptr) {
        *error = "failed to call set_joint_mode for " + joint_name;
      }
      return false;
    }
    if (!srv.response.success) {
      if (error != nullptr) {
        *error = "set_joint_mode failed for " + joint_name + ": " +
                 srv.response.message;
      }
      return false;
    }
    return true;
  }

  bool CallSetCanopenMode(std::size_t axis_index, HardwareMode mode,
                          std::string* error) {
    if (!WaitForService(&canopen_set_mode_client_,
                        canopen_ns_ + "/set_mode", error)) {
      return false;
    }

    Eyou_Canopen_Master::SetMode srv;
    srv.request.axis_index = static_cast<int32_t>(axis_index);
    srv.request.mode = static_cast<int8_t>(CanopenModeForHardwareMode(mode));
    if (!canopen_set_mode_client_.call(srv)) {
      if (error != nullptr) {
        *error = "failed to call set_mode for axis " + std::to_string(axis_index);
      }
      return false;
    }
    if (!srv.response.success) {
      if (error != nullptr) {
        *error = "set_mode failed for axis " + std::to_string(axis_index) +
                 ": " + srv.response.message;
      }
      return false;
    }
    return true;
  }

  bool ListControllers(std::map<std::string, std::string>* states,
                       std::string* error) {
    if (!WaitForService(&list_controllers_client_,
                        controller_manager_ns_ + "/list_controllers", error)) {
      return false;
    }

    controller_manager_msgs::ListControllers srv;
    if (!list_controllers_client_.call(srv)) {
      if (error != nullptr) {
        *error = "failed to call list_controllers";
      }
      return false;
    }

    if (states != nullptr) {
      states->clear();
      for (const auto& controller : srv.response.controller) {
        (*states)[controller.name] = controller.state;
      }
    }
    return true;
  }

  bool EnsureControllerLoaded(const std::string& controller_name,
                              std::string* error) {
    std::map<std::string, std::string> states;
    if (!ListControllers(&states, error)) {
      return false;
    }
    if (states.find(controller_name) != states.end()) {
      return true;
    }
    if (!WaitForService(&load_controller_client_,
                        controller_manager_ns_ + "/load_controller", error)) {
      return false;
    }

    controller_manager_msgs::LoadController srv;
    srv.request.name = controller_name;
    if (!load_controller_client_.call(srv)) {
      if (error != nullptr) {
        *error = "failed to call load_controller for " + controller_name;
      }
      return false;
    }
    if (!srv.response.ok) {
      if (error != nullptr) {
        *error = "load_controller failed for " + controller_name;
      }
      return false;
    }
    return true;
  }

  bool SwitchControllers(const std::vector<std::string>& start,
                         const std::vector<std::string>& stop,
                         std::string* error) {
    if (!WaitForService(&switch_controller_client_,
                        controller_manager_ns_ + "/switch_controller", error)) {
      return false;
    }

    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = start;
    srv.request.stop_controllers = stop;
    srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    srv.request.start_asap = false;
    srv.request.timeout = controller_switch_timeout_sec_;
    if (!switch_controller_client_.call(srv)) {
      if (error != nullptr) {
        *error = "failed to call switch_controller";
      }
      return false;
    }
    if (!srv.response.ok) {
      if (error != nullptr) {
        *error = "switch_controller returned not ok";
      }
      return false;
    }
    return true;
  }

  void PublishActiveProfile() {
    std_msgs::String msg;
    msg.data = ToString(active_profile_);
    active_profile_pub_.publish(msg);
  }

  std::string ControlBackendNs() const {
    return backend_type_ == BackendType::kHybrid ? hybrid_ns_ : canopen_ns_;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::vector<std::string> joint_names_;
  std::vector<double> direction_corrections_;
  ControllerNames controllers_;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber jog_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber runtime_state_sub_;
  ros::Subscriber diagnostics_sub_;

  ros::Publisher csp_command_pub_;
  ros::Publisher csv_command_pub_;
  ros::Publisher csv_velocity_command_pub_;
  ros::Publisher state_pub_;
  ros::Publisher active_profile_pub_;

  ros::ServiceServer set_profile_srv_;
  ros::ServiceServer set_linkage_mode_srv_;

  ros::ServiceClient switch_controller_client_;
  ros::ServiceClient load_controller_client_;
  ros::ServiceClient list_controllers_client_;
  ros::ServiceClient halt_client_;
  ros::ServiceClient disable_client_;
  ros::ServiceClient enable_client_;
  ros::ServiceClient resume_client_;
  ros::ServiceClient set_joint_mode_client_;
  ros::ServiceClient canopen_set_mode_client_;

  ros::Timer control_timer_;
  ros::Timer state_timer_;

  std::shared_ptr<FlipperReferenceGenerator> generator_;

  ControlProfile active_profile_;
  HardwareMode active_hardware_mode_;
  LinkageMode linkage_mode_;
  std::string active_controller_;
  std::string lifecycle_state_ = "Unknown";
  std::string detail_;
  std::string switch_state_ = "IDLE";

  bool switching_;
  bool ready_;
  bool have_joint_state_;
  bool have_canopen_diag_;
  bool have_runtime_state_;
  bool require_backend_feedback_;
  CsvCommandInterface csv_command_interface_ = CsvCommandInterface::kTrajectory;

  BackendType backend_type_ = BackendType::kHybrid;
  std::map<std::string, Eyou_ROS1_Master::JointRuntimeState> runtime_state_map_;
  std::map<std::string, CanopenDiagnosticState> canopen_diag_state_map_;

  double control_loop_hz_ = 100.0;
  double state_publish_hz_ = 20.0;
  double command_timeout_sec_ = 0.4;
  double dt_clamp_sec_ = 0.02;
  double jog_velocity_alpha_ = 0.35;
  double reference_drift_threshold_ = 0.35;
  double service_wait_timeout_sec_;
  double controller_switch_timeout_sec_;
  double short_horizon_sec_;
  double fallback_min_position_;
  double fallback_max_position_;
  double fallback_max_velocity_;

  std::string controller_manager_ns_ = "/controller_manager";
  std::string hybrid_ns_ = "/hybrid_motor_hw_node";
  std::string runtime_state_topic_ = "/hybrid_motor_hw_node/joint_runtime_states";
  std::string canopen_ns_ = "/canopen_hw_node";
  std::string canopen_diagnostics_topic_ = "/diagnostics";
};

}  // namespace flipper_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "flipper_control");
  flipper_control::FlipperManagerNode node;
  ros::spin();
  return 0;
}
