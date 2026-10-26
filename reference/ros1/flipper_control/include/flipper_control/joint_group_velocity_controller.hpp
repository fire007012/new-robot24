#pragma once

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

namespace flipper_control {

class JointGroupVelocityController
    : public controller_interface::Controller<
          hardware_interface::VelocityJointInterface> {
 public:
  JointGroupVelocityController() = default;
  ~JointGroupVelocityController() override;

  bool init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 protected:
  struct CommandState {
    std::vector<double> values;
    ros::Time stamp;
    bool valid = false;
  };

  bool InitJoints(hardware_interface::VelocityJointInterface* hw,
                  const std::vector<std::string>& joint_names,
                  double command_timeout_sec);
  bool AcceptCommand(const std::vector<double>& values, const ros::Time& stamp);
  void CommandCb(const std_msgs::Float64MultiArrayConstPtr& msg);
  void SetZeroCommand();

 private:
  ros::Subscriber command_sub_;
  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joints_;
  realtime_tools::RealtimeBuffer<CommandState> command_buffer_;
  double command_timeout_sec_ = 0.25;
};

}  // namespace flipper_control
