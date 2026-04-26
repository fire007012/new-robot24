#include "flipper_control/joint_group_velocity_controller.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace flipper_control {

JointGroupVelocityController::~JointGroupVelocityController() {
  command_sub_.shutdown();
}

bool JointGroupVelocityController::init(
    hardware_interface::VelocityJointInterface* hw,
    ros::NodeHandle& controller_nh) {
  if (hw == nullptr) {
    ROS_ERROR("JointGroupVelocityController requires a valid hardware interface");
    return false;
  }

  if (!controller_nh.getParam("joints", joint_names_)) {
    ROS_ERROR_STREAM("Failed to read 'joints' in namespace "
                     << controller_nh.getNamespace());
    return false;
  }
  if (joint_names_.empty()) {
    ROS_ERROR_STREAM("Joint list is empty in namespace "
                     << controller_nh.getNamespace());
    return false;
  }

  controller_nh.param("command_timeout", command_timeout_sec_,
                      command_timeout_sec_);
  if (!std::isfinite(command_timeout_sec_) || command_timeout_sec_ < 0.0) {
    ROS_WARN_STREAM("Invalid command_timeout in namespace "
                    << controller_nh.getNamespace()
                    << ", fallback to 0.25s");
    command_timeout_sec_ = 0.25;
  }

  if (!InitJoints(hw, joint_names_, command_timeout_sec_)) {
    return false;
  }

  command_sub_ = controller_nh.subscribe<std_msgs::Float64MultiArray>(
      "command", 1, &JointGroupVelocityController::CommandCb, this);
  return true;
}

bool JointGroupVelocityController::InitJoints(
    hardware_interface::VelocityJointInterface* hw,
    const std::vector<std::string>& joint_names, double command_timeout_sec) {
  if (hw == nullptr) {
    return false;
  }

  joint_names_ = joint_names;
  command_timeout_sec_ = command_timeout_sec;
  if (joint_names_.empty()) {
    return false;
  }

  joints_.clear();
  joints_.reserve(joint_names_.size());
  for (const auto& joint_name : joint_names_) {
    try {
      joints_.push_back(hw->getHandle(joint_name));
    } catch (const hardware_interface::HardwareInterfaceException& exc) {
      ROS_ERROR_STREAM("Failed to get velocity joint handle for " << joint_name
                                                                  << ": "
                                                                  << exc.what());
      return false;
    }
  }

  CommandState initial_command;
  initial_command.values.assign(joint_names_.size(), 0.0);
  initial_command.stamp = ros::Time(0);
  initial_command.valid = false;
  command_buffer_.writeFromNonRT(initial_command);
  return true;
}

void JointGroupVelocityController::starting(const ros::Time& /*time*/) {
  SetZeroCommand();
}

void JointGroupVelocityController::update(const ros::Time& time,
                                          const ros::Duration& /*period*/) {
  const CommandState& command = *command_buffer_.readFromRT();
  const bool command_is_fresh =
      command.valid &&
      command.values.size() == joints_.size() &&
      (command_timeout_sec_ <= 0.0 ||
       (time - command.stamp).toSec() <= command_timeout_sec_);

  if (!command_is_fresh) {
    for (auto& joint : joints_) {
      joint.setCommand(0.0);
    }
    return;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    joints_[i].setCommand(command.values[i]);
  }
}

void JointGroupVelocityController::stopping(const ros::Time& /*time*/) {
  SetZeroCommand();
}

void JointGroupVelocityController::SetZeroCommand() {
  CommandState zero_command;
  zero_command.values.assign(joint_names_.size(), 0.0);
  zero_command.stamp = ros::Time::now();
  zero_command.valid = false;
  command_buffer_.writeFromNonRT(zero_command);
  for (auto& joint : joints_) {
    joint.setCommand(0.0);
  }
}

void JointGroupVelocityController::CommandCb(
    const std_msgs::Float64MultiArrayConstPtr& msg) {
  if (!msg) {
    return;
  }
  AcceptCommand(msg->data, ros::Time::now());
}

bool JointGroupVelocityController::AcceptCommand(
    const std::vector<double>& values, const ros::Time& stamp) {
  if (values.size() != joints_.size()) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "JointGroupVelocityController command size mismatch: expected "
                 << joints_.size() << ", got " << values.size());
    SetZeroCommand();
    return false;
  }

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0,
          "JointGroupVelocityController received non-finite command at index "
              << i);
      SetZeroCommand();
      return false;
    }
  }

  CommandState command;
  command.values = values;
  command.stamp = stamp;
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
  return true;
}

}  // namespace flipper_control

PLUGINLIB_EXPORT_CLASS(flipper_control::JointGroupVelocityController,
                       controller_interface::ControllerBase)
