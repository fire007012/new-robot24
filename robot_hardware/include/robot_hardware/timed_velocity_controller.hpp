#pragma once
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "robot_hardware/velocity_watchdog.hpp"
#include <mutex>
namespace robot_hardware {
class TimedVelocityController : public controller_interface::ControllerInterface {
public:
 controller_interface::CallbackReturn on_init() override;
 controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
 controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
 controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
 controller_interface::InterfaceConfiguration command_interface_configuration() const override;
 controller_interface::InterfaceConfiguration state_interface_configuration() const override;
 controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
 std::vector<std::string> joints_;
 std::vector<double> lower_, upper_, maximum_, reference_;
 std::string interface_;
 double timeout_ = 0.25;
 VelocityWatchdog watchdog_;
 std::mutex mutex_;
 rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;
};
}
