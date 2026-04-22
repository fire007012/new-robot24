#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ruckig/input_parameter.hpp>
#include <ruckig/output_parameter.hpp>
#include <ruckig/ruckig.hpp>

namespace {

double SafeControlRate(double control_rate_hz) {
  return (std::isfinite(control_rate_hz) && control_rate_hz > 0.0) ? control_rate_hz
                                                                    : 50.0;
}

class BaseCmdNode {
 public:
  BaseCmdNode()
      : nh_(),
        pnh_("~"),
        control_rate_hz_(pnh_.param("control_rate_hz", 50.0)),
        timeout_sec_(pnh_.param("timeout_sec", 0.3)),
        max_linear_x_(pnh_.param("max_linear_x", 0.8)),
        max_angular_z_(pnh_.param("max_angular_z", 1.5)),
        max_linear_acc_(pnh_.param("max_linear_acc", 1.0)),
        max_linear_jerk_(pnh_.param("max_linear_jerk", 4.0)),
        max_angular_acc_(pnh_.param("max_angular_acc", 2.0)),
        max_angular_jerk_(pnh_.param("max_angular_jerk", 8.0)),
        wheel_separation_(pnh_.param("wheel_separation", 0.438)),
        cmd_linear_direction_correction_(
            pnh_.param("cmd_linear_direction_correction", 1.0)),
        cmd_angular_direction_correction_(
            pnh_.param("cmd_angular_direction_correction", 1.0)),
        left_track_direction_correction_(
            pnh_.param("left_track_direction_correction", 1.0)),
        right_track_direction_correction_(
            pnh_.param("right_track_direction_correction", 1.0)),
        input_topic_(pnh_.param<std::string>("input_topic", "/cmd_vel")),
        output_topic_(
            pnh_.param<std::string>("output_topic", "/wheel_controller/cmd_vel")),
        otg_(1.0 / SafeControlRate(control_rate_hz_)) {
    if (!ValidateParams()) {
      ros::shutdown();
      return;
    }

    input_.control_interface = ruckig::ControlInterface::Velocity;
    input_.synchronization = ruckig::Synchronization::None;
    input_.duration_discretization = ruckig::DurationDiscretization::Discrete;

    input_.current_position = {0.0, 0.0};
    input_.current_velocity = {0.0, 0.0};
    input_.current_acceleration = {0.0, 0.0};
    input_.target_position = {0.0, 0.0};
    input_.target_velocity = {0.0, 0.0};
    input_.target_acceleration = {0.0, 0.0};
    input_.max_velocity = {max_linear_x_, max_angular_z_};
    input_.max_acceleration = {max_linear_acc_, max_angular_acc_};
    input_.max_jerk = {max_linear_jerk_, max_angular_jerk_};

    pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 10);
    sub_ = nh_.subscribe(input_topic_, 10, &BaseCmdNode::CmdCb, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
                             &BaseCmdNode::ControlTimerCb, this);

    ROS_INFO(
        "base_cmd_node started: %s -> %s, rate=%.1f Hz, vmax=[%.3f, %.3f], "
        "amax=[%.3f, %.3f], jmax=[%.3f, %.3f], wheel_sep=%.3f, "
        "cmd_dir=[%.1f, %.1f], track_dir=[%.1f, %.1f]",
        input_topic_.c_str(), output_topic_.c_str(), control_rate_hz_, max_linear_x_,
        max_angular_z_, max_linear_acc_, max_angular_acc_, max_linear_jerk_,
        max_angular_jerk_, wheel_separation_, cmd_linear_direction_correction_,
        cmd_angular_direction_correction_, left_track_direction_correction_,
        right_track_direction_correction_);
  }

 private:
  bool ValidateParams() const {
    return RequirePositive("control_rate_hz", control_rate_hz_) &&
           RequirePositive("timeout_sec", timeout_sec_) &&
           RequirePositive("max_linear_x", max_linear_x_) &&
           RequirePositive("max_angular_z", max_angular_z_) &&
           RequirePositive("max_linear_acc", max_linear_acc_) &&
           RequirePositive("max_linear_jerk", max_linear_jerk_) &&
           RequirePositive("max_angular_acc", max_angular_acc_) &&
           RequirePositive("max_angular_jerk", max_angular_jerk_) &&
           RequirePositive("wheel_separation", wheel_separation_) &&
           RequireNonZeroFinite("cmd_linear_direction_correction",
                                cmd_linear_direction_correction_) &&
           RequireNonZeroFinite("cmd_angular_direction_correction",
                                cmd_angular_direction_correction_) &&
           RequireNonZeroFinite("left_track_direction_correction",
                                left_track_direction_correction_) &&
           RequireNonZeroFinite("right_track_direction_correction",
                                right_track_direction_correction_);
  }

  bool RequirePositive(const char* name, double value) const {
    if (std::isfinite(value) && value > 0.0) {
      return true;
    }
    ROS_ERROR("Parameter %s must be a positive finite number, got %.6f", name,
              value);
    return false;
  }

  bool RequireNonZeroFinite(const char* name, double value) const {
    if (std::isfinite(value) && std::abs(value) > 1e-9) {
      return true;
    }
    ROS_ERROR("Parameter %s must be a finite non-zero number, got %.6f", name,
              value);
    return false;
  }

  static double Clamp(double value, double limit) {
    return std::max(-limit, std::min(limit, value));
  }

  geometry_msgs::Twist ApplyTrackDirectionCorrections(
      const geometry_msgs::Twist& smoothed_cmd) const {
    const double half_separation = 0.5 * wheel_separation_;
    double left_track_velocity =
        smoothed_cmd.linear.x - half_separation * smoothed_cmd.angular.z;
    double right_track_velocity =
        smoothed_cmd.linear.x + half_separation * smoothed_cmd.angular.z;

    left_track_velocity *= left_track_direction_correction_;
    right_track_velocity *= right_track_direction_correction_;

    geometry_msgs::Twist corrected_cmd;
    corrected_cmd.linear.x = 0.5 * (left_track_velocity + right_track_velocity);
    corrected_cmd.angular.z =
        (right_track_velocity - left_track_velocity) / wheel_separation_;
    return corrected_cmd;
  }

  void CmdCb(const geometry_msgs::TwistConstPtr& msg) {
    target_linear_x_ = Clamp(msg->linear.x * cmd_linear_direction_correction_,
                             max_linear_x_);
    target_angular_z_ = Clamp(msg->angular.z * cmd_angular_direction_correction_,
                              max_angular_z_);
    last_cmd_time_ = ros::Time::now();
    has_cmd_ = true;
  }

  void ControlTimerCb(const ros::TimerEvent&) {
    if (has_cmd_ && (ros::Time::now() - last_cmd_time_).toSec() > timeout_sec_) {
      has_cmd_ = false;
    }

    const double desired_linear = has_cmd_ ? target_linear_x_ : 0.0;
    const double desired_angular = has_cmd_ ? target_angular_z_ : 0.0;

    input_.target_velocity[0] = desired_linear;
    input_.target_velocity[1] = desired_angular;
    input_.target_acceleration[0] = 0.0;
    input_.target_acceleration[1] = 0.0;

    const auto result = otg_.update(input_, output_);
    geometry_msgs::Twist out;

    if (result < 0) {
      ROS_WARN_THROTTLE(1.0,
                        "Ruckig update failed (%d), falling back to clamped "
                        "target velocity",
                        static_cast<int>(result));
      out.linear.x = desired_linear;
      out.angular.z = desired_angular;

      input_.current_position[0] += out.linear.x / control_rate_hz_;
      input_.current_position[1] += out.angular.z / control_rate_hz_;
      input_.current_velocity[0] = out.linear.x;
      input_.current_velocity[1] = out.angular.z;
      input_.current_acceleration[0] = 0.0;
      input_.current_acceleration[1] = 0.0;
      input_.target_velocity[0] = desired_linear;
      input_.target_velocity[1] = desired_angular;
      input_.target_acceleration[0] = 0.0;
      input_.target_acceleration[1] = 0.0;
      otg_.reset();
    } else {
      out.linear.x = output_.new_velocity[0];
      out.angular.z = output_.new_velocity[1];
      output_.pass_to_input(input_);
    }

    pub_.publish(ApplyTrackDirectionCorrections(out));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer timer_;

  double control_rate_hz_;
  double timeout_sec_;
  double max_linear_x_;
  double max_angular_z_;
  double max_linear_acc_;
  double max_linear_jerk_;
  double max_angular_acc_;
  double max_angular_jerk_;
  double wheel_separation_;
  double cmd_linear_direction_correction_;
  double cmd_angular_direction_correction_;
  double left_track_direction_correction_;
  double right_track_direction_correction_;

  std::string input_topic_;
  std::string output_topic_;

  bool has_cmd_ = false;
  ros::Time last_cmd_time_{0.0};
  double target_linear_x_ = 0.0;
  double target_angular_z_ = 0.0;

  ruckig::Ruckig<2> otg_;
  ruckig::InputParameter<2> input_;
  ruckig::OutputParameter<2> output_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_cmd_node");
  BaseCmdNode node;
  ros::spin();
  return 0;
}
