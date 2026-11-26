#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
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
    state_timeout_sec_=positive("state_timeout_sec",0.6);
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
  static std::string baseEpochToken(const nlohmann::json & node) {
    if (node.contains("input_epochs") && node["input_epochs"].is_object() &&
        node["input_epochs"].contains("base"))
      return node["input_epochs"]["base"].dump();
    if (node.contains("operations") && node["operations"].is_object() &&
        node["operations"].contains("base"))
      return node["operations"]["base"].dump();
    return "global";
  }
  void state(const std::string & data) {
    try {
      const auto node=nlohmann::json::parse(data);
      if (!node.is_object()) { running_=false;reset();stop();return; }
      std::string base_state;
      if (node.contains("groups") && node["groups"].is_object() &&
          node["groups"].contains("base") && node["groups"]["base"].is_string()) {
        base_state = node["groups"]["base"].get<std::string>();
      } else if (node.contains("state") && node["state"].is_string()) {
        base_state = node["state"].get<std::string>();
      }
      const auto running = (base_state == "Running");
      const auto token=(running?"running:":"stopped:")+baseEpochToken(node);
      const auto now=Clock::now();
      const auto state_timeout=std::chrono::duration<double>(state_timeout_sec_);
      if (token!=token_ || now-state_at_>state_timeout) {
        reset(); stop(); token_=token; ++session_; command_.reset();
        if (running) {
          const auto session=session_;
          command_=create_subscription<Twist>(topic_,rclcpp::QoS(1).durability_volatile(),
            [this,session](Twist::ConstSharedPtr msg) {
              const auto now_cb=Clock::now();
              if (!running_ || session!=session_ || now_cb-state_at_>std::chrono::duration<double>(state_timeout_sec_)) return;
              for (double value : {msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z})
                if (!std::isfinite(value)) { reset();stop();return; }
              input_.target_velocity={std::clamp(msg->linear.x,-vmax_[0],vmax_[0]),std::clamp(msg->angular.z,-vmax_[1],vmax_[1])};
              received_=now_cb;
            });
        }
      }
      running_=running;state_at_=now;
    } catch (const std::exception &) { running_=false;reset();stop(); }
  }
  void step() {
    const auto now=Clock::now();const auto elapsed=now-last_step_;last_step_=now;
    const auto state_timeout=std::chrono::duration<double>(state_timeout_sec_);
    if (!running_ || now-state_at_>state_timeout || elapsed>std::chrono::milliseconds(150)) {
      reset();stop();return;
    }
    if (received_==Clock::time_point{}) { stop(); return; }
    if (std::chrono::duration<double>(now-received_).count()>timeout_) {
      // Decelerate smoothly via Ruckig on cmd_vel timeout instead of snapping to 0.
      input_.target_velocity={0.0,0.0};
      input_.target_acceleration={0.0,0.0};
      if (std::abs(input_.current_velocity[0])<1e-6 && std::abs(input_.current_velocity[1])<1e-6 &&
          std::abs(input_.current_acceleration[0])<1e-6 && std::abs(input_.current_acceleration[1])<1e-6) {
        stop(); return;
      }
    }
    const auto result=otg_->update(input_,output_);
    if (result<0) { reset();stop();return; }
    Twist out;out.linear.x=output_.new_velocity[0];out.angular.z=output_.new_velocity[1];
    output_.pass_to_input(input_);pub_->publish(out);
  }
  double rate_,timeout_,state_timeout_sec_;
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
