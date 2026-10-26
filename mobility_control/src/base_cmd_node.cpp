#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <ruckig/ruckig.hpp>

class BaseCmd : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  using Twist = geometry_msgs::msg::Twist;
public:
  BaseCmd() : Node("base_cmd_node") {
    const auto positive = [this](const std::string & name, double value) {
      const auto result = declare_parameter(name, value);
      if (!std::isfinite(result) || result <= 0) throw std::invalid_argument(name+" must be positive and finite");
      return result;
    };
    rate_=positive("control_rate_hz",50.0); timeout_=positive("timeout_sec",0.3);
    vmax_={positive("max_linear_x",0.8),positive("max_angular_z",1.5)};
    amax_={positive("max_linear_acc",1.0),positive("max_angular_acc",2.0)};
    jmax_={positive("max_linear_jerk",4.0),positive("max_angular_jerk",8.0)};
    otg_=std::make_unique<ruckig::Ruckig<2>>(1.0/rate_);
    input_.control_interface=ruckig::ControlInterface::Velocity;
    input_.synchronization=ruckig::Synchronization::None;
    input_.duration_discretization=ruckig::DurationDiscretization::Discrete;
    input_.max_velocity=vmax_;input_.max_acceleration=amax_;input_.max_jerk=jmax_;
    reset();
    const auto output=declare_parameter<std::string>("output_topic","/wheel_controller/cmd_vel_unstamped");
    topic_=declare_parameter<std::string>("input_topic","/cmd_vel");
    pub_=create_publisher<Twist>(output, rclcpp::QoS(1).durability_volatile());
    state_=create_subscription<std_msgs::msg::String>("/robot_runtime/state",1,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { state(msg->data); });
    last_step_=Clock::now();
    timer_=create_wall_timer(std::chrono::duration<double>(1.0/rate_),[this]() { step(); });
  }
  void stop() { pub_->publish(Twist()); }
private:
  void reset() {
    input_.current_position={0,0};input_.current_velocity={0,0};input_.current_acceleration={0,0};
    input_.target_position={0,0};input_.target_velocity={0,0};input_.target_acceleration={0,0};
    received_=Clock::time_point{};
    otg_->reset();
  }
  void state(const std::string & data) {
    try {
      const auto node=YAML::Load(data);
      const auto running=(node["groups"] && node["groups"]["base"] ? node["groups"]["base"] : node["state"]).as<std::string>()=="Running";
      const auto operation=node["operations"] && node["operations"]["base"] ? YAML::Dump(node["operations"]["base"]) : "global";
      const auto token=(running?"running:":"stopped:")+operation;
      const auto now=Clock::now();
      if (token!=token_ || now-state_at_>std::chrono::milliseconds(300)) {
        reset(); stop(); token_=token; ++session_; command_.reset();
        if (running) {
          const auto session=session_;
          command_=create_subscription<Twist>(topic_,rclcpp::QoS(1).durability_volatile(),
            [this,session](Twist::ConstSharedPtr msg) {
              if (!running_ || session!=session_ || Clock::now()-state_at_>std::chrono::milliseconds(300)) return;
              for (double value : {msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z})
                if (!std::isfinite(value)) { reset();stop();return; }
              input_.target_velocity={std::clamp(msg->linear.x,-vmax_[0],vmax_[0]),std::clamp(msg->angular.z,-vmax_[1],vmax_[1])};
              received_=Clock::now();
            });
        }
      }
      running_=running;state_at_=now;
    } catch (const std::exception &) { running_=false;reset();stop(); }
  }
  void step() {
    const auto now=Clock::now();const auto elapsed=now-last_step_;last_step_=now;
    if (!running_ || now-state_at_>std::chrono::milliseconds(300) ||
        received_==Clock::time_point{} || std::chrono::duration<double>(now-received_).count()>timeout_ ||
        elapsed>std::chrono::milliseconds(100)) { reset();stop();return; }
    const auto result=otg_->update(input_,output_);
    if (result<0) { reset();stop();return; }
    Twist out;out.linear.x=output_.new_velocity[0];out.angular.z=output_.new_velocity[1];
    output_.pass_to_input(input_);pub_->publish(out);
  }
  double rate_,timeout_;
  std::array<double,2> vmax_,amax_,jmax_;
  std::unique_ptr<ruckig::Ruckig<2>> otg_;
  ruckig::InputParameter<2> input_;
  ruckig::OutputParameter<2> output_;
  bool running_=false;
  uint64_t session_=0;
  std::string token_,topic_;
  Clock::time_point state_at_{},received_{},last_step_{};
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Twist>::SharedPtr command_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<BaseCmd>();
  rclcpp::spin(node);
  if (rclcpp::ok()) node->stop();
  node.reset();if(rclcpp::ok())rclcpp::shutdown();return 0;
}
