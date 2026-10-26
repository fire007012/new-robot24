#include "robot_hardware/plugin_lifetime.hpp"
#include "diff_drive_controller/diff_drive_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
namespace robot_hardware {
// Preserve the standard differential-drive interfaces, odometry and limits while
// fencing callbacks and cached Twist/history at each controller activation.
class SessionDiffDriveController : public diff_drive_controller::DiffDriveController {
  using Message=geometry_msgs::msg::TwistStamped;
  struct Input {
    std::mutex mutex;
    uint64_t epoch=0;
    bool active=false;
    int64_t activated=0;
    std::atomic<int64_t> receipt{0};
  };
  std::shared_ptr<Input> input_=std::make_shared<Input>();
  static int64_t steady() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static bool finite(const geometry_msgs::msg::Twist & t) {
    return std::isfinite(t.linear.x)&&std::isfinite(t.linear.y)&&std::isfinite(t.linear.z)&&
      std::isfinite(t.angular.x)&&std::isfinite(t.angular.y)&&std::isfinite(t.angular.z);
  }
  void retire() {
    std::lock_guard<std::mutex> lock(input_->mutex);
    input_->active=false;++input_->epoch;input_->receipt=0;
    velocity_command_subscriber_.reset();velocity_command_unstamped_subscriber_.reset();
    received_velocity_msg_ptr_.set(std::make_shared<Message>());
  }
  void clearHistory() {
    auto zero=std::make_shared<Message>();zero->header.stamp=get_node()->now();
    received_velocity_msg_ptr_.set(zero);
    previous_commands_={};previous_commands_.push(*zero);previous_commands_.push(*zero);
    input_->receipt=0;
  }
public:
  ~SessionDiffDriveController() override {retire();}
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override {
    retainPluginCode();
  const auto result=DiffDriveController::on_configure(state);retire();return result;
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
    retire();clearHistory();const auto result=DiffDriveController::on_activate(state);
    if(result!=controller_interface::CallbackReturn::SUCCESS)return result;
    // Any queued callback from the base class's configure-time subscriptions stays
    // disabled. Our activation subscriptions use Input::active and the captured epoch.
    subscriber_is_active_=false;
    std::lock_guard<std::mutex> lock(input_->mutex);clearHistory();
    input_->active=true;input_->activated=get_node()->now().nanoseconds();const auto epoch=++input_->epoch;
    auto input=input_;
    if(use_stamped_vel_) velocity_command_subscriber_=get_node()->create_subscription<Message>("~/cmd_vel",rclcpp::SystemDefaultsQoS(),
      [this,input,epoch](std::shared_ptr<Message> message) {
        std::lock_guard<std::mutex> guard(input->mutex);
        if(!input->active||input->epoch!=epoch||!finite(message->twist))return;
        const auto now=get_node()->now();
        const auto stamp=rclcpp::Time(message->header.stamp,now.get_clock_type()).nanoseconds();
        if(stamp && (stamp<input->activated||stamp>now.nanoseconds()))return;
        if(!stamp)message->header.stamp=now;
        input->receipt=steady();received_velocity_msg_ptr_.set(message);
      });
    else velocity_command_unstamped_subscriber_=get_node()->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel_unstamped",rclcpp::SystemDefaultsQoS(),
      [this,input,epoch](std::shared_ptr<geometry_msgs::msg::Twist> message) {
        std::lock_guard<std::mutex> guard(input->mutex);
        if(!input->active||input->epoch!=epoch||!finite(*message))return;
        auto stamped=std::make_shared<Message>();stamped->twist=*message;stamped->header.stamp=get_node()->now();
        input->receipt=steady();received_velocity_msg_ptr_.set(stamped);
      });
    return result;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
    retire();return DiffDriveController::on_deactivate(state);
  }
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override {
    retire();return DiffDriveController::on_cleanup(state);
  }
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override {
    retire();return DiffDriveController::on_error(state);
  }
  controller_interface::return_type update(const rclcpp::Time & time,const rclcpp::Duration & period) override {
    const auto received=input_->receipt.load();
    if(!received || (cmd_vel_timeout_.count()>0 && steady()-received>=cmd_vel_timeout_.count()*1000000LL)) {
      auto zero=std::make_shared<Message>();zero->header.stamp=time;received_velocity_msg_ptr_.set(zero);
    }
    return DiffDriveController::update(time,period);
  }
};
}
PLUGINLIB_EXPORT_CLASS(robot_hardware::SessionDiffDriveController,controller_interface::ControllerInterface)
