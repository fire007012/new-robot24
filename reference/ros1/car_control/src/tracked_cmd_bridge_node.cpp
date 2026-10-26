#include <algorithm>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace
{
double Clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

bool IsZero(const geometry_msgs::Twist &cmd)
{
  return std::abs(cmd.linear.x) < 1e-6 && std::abs(cmd.angular.z) < 1e-6;
}
}  // namespace

class TrackedCmdBridge
{
public:
  TrackedCmdBridge()
    : nh_(),
      pnh_("~"),
      has_cmd_(false),
      sent_zero_(true)
  {
    pnh_.param("input_topic", input_topic_, std::string("/car_urdf/cmd_vel"));
    pnh_.param("gazebo_topic", gazebo_topic_, std::string("~/car_urdf/cmd_vel_twist"));
    pnh_.param("max_linear_x", max_linear_x_, 0.8);
    pnh_.param("max_angular_z", max_angular_z_, 1.5);
    pnh_.param("linear_scale", linear_scale_, -1.0);
    pnh_.param("angular_scale", angular_scale_, -1.0);
    pnh_.param("timeout_sec", timeout_sec_, 0.5);

    gazebo::transport::init();
    gazebo::transport::run();

    gz_node_.reset(new gazebo::transport::Node());
    gz_node_->Init();
    gz_pub_ = gz_node_->Advertise<gazebo::msgs::Twist>(gazebo_topic_);

    cmd_sub_ = nh_.subscribe(input_topic_, 10, &TrackedCmdBridge::CmdCb, this);
    watchdog_ = nh_.createTimer(ros::Duration(0.05), &TrackedCmdBridge::WatchdogCb, this);

    ROS_INFO("tracked_cmd_bridge_node started: %s -> %s",
             input_topic_.c_str(), gazebo_topic_.c_str());
  }

  ~TrackedCmdBridge()
  {
    if (gz_node_)
    {
      gz_node_->Fini();
    }
    gazebo::transport::stop();
    gazebo::transport::fini();
  }

private:
  void PublishCmd(const geometry_msgs::Twist &cmd)
  {
    if (!gz_pub_)
    {
      return;
    }

    gazebo::msgs::Twist msg;
    msg.mutable_linear()->set_x(cmd.linear.x);
    msg.mutable_linear()->set_y(0.0);
    msg.mutable_linear()->set_z(0.0);
    msg.mutable_angular()->set_x(0.0);
    msg.mutable_angular()->set_y(0.0);
    msg.mutable_angular()->set_z(cmd.angular.z);
    gz_pub_->Publish(msg);
    sent_zero_ = IsZero(cmd);
  }

  void CmdCb(const geometry_msgs::TwistConstPtr &msg)
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = Clamp(msg->linear.x, -max_linear_x_, max_linear_x_) * linear_scale_;
    cmd.angular.z = Clamp(msg->angular.z, -max_angular_z_, max_angular_z_) * angular_scale_;

    PublishCmd(cmd);
    last_cmd_time_ = ros::Time::now();
    has_cmd_ = true;
  }

  void WatchdogCb(const ros::TimerEvent &)
  {
    if (!has_cmd_)
    {
      return;
    }

    if ((ros::Time::now() - last_cmd_time_).toSec() <= timeout_sec_)
    {
      return;
    }

    if (!sent_zero_)
    {
      PublishCmd(geometry_msgs::Twist());
    }
    has_cmd_ = false;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cmd_sub_;
  ros::Timer watchdog_;

  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr gz_pub_;

  std::string input_topic_;
  std::string gazebo_topic_;
  double max_linear_x_;
  double max_angular_z_;
  double linear_scale_;
  double angular_scale_;
  double timeout_sec_;
  ros::Time last_cmd_time_;
  bool has_cmd_;
  bool sent_zero_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracked_cmd_bridge_node");
  TrackedCmdBridge bridge;
  ros::spin();
  return 0;
}
