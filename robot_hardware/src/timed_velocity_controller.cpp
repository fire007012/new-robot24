#include "robot_hardware/plugin_lifetime.hpp"
#include "robot_hardware/timed_velocity_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
namespace robot_hardware {
namespace {
double steadySeconds() {
 return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
controller_interface::CallbackReturn TimedVelocityController::on_init() {
 try {
  retainPluginCode();
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("interface_name", "velocity");
  auto_declare<double>("command_timeout", 0.25);
  for (auto p : {"lower", "upper", "max_velocity"}) auto_declare<std::vector<double>>(p, {});
 } catch (...) { return controller_interface::CallbackReturn::ERROR; }
 return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TimedVelocityController::on_configure(const rclcpp_lifecycle::State &) {
 joints_ = get_node()->get_parameter("joints").as_string_array();
 interface_ = get_node()->get_parameter("interface_name").as_string();
 timeout_ = get_node()->get_parameter("command_timeout").as_double();
 lower_ = get_node()->get_parameter("lower").as_double_array();
 upper_ = get_node()->get_parameter("upper").as_double_array();
 maximum_ = get_node()->get_parameter("max_velocity").as_double_array();
 if (joints_.empty() || (interface_ != "velocity" && interface_ != "position") ||
     !std::isfinite(timeout_) || timeout_ <= 0 || lower_.size()!=joints_.size() ||
     upper_.size()!=joints_.size() || maximum_.size()!=joints_.size()) return controller_interface::CallbackReturn::ERROR;
 for (size_t i=0;i<joints_.size();++i)
  if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i]>=upper_[i] ||
      !std::isfinite(maximum_[i]) || maximum_[i]<=0) return controller_interface::CallbackReturn::ERROR;
 std::lock_guard<std::mutex> lock(writer_mutex_);
 epoch_.fetch_add(1, std::memory_order_acq_rel);
 command_buffer_.writeFromNonRT(CommandSample{});
 return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration TimedVelocityController::command_interface_configuration() const {
 controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
 for (auto &j:joints_) c.names.push_back(j+"/"+interface_); return c;
}
controller_interface::InterfaceConfiguration TimedVelocityController::state_interface_configuration() const {
 controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
 for (auto &j:joints_) c.names.push_back(j+"/position"); return c;
}
controller_interface::CallbackReturn TimedVelocityController::on_activate(const rclcpp_lifecycle::State &) {
 std::lock_guard<std::mutex> lock(writer_mutex_);
 reference_.clear();
 for (size_t i=0;i<joints_.size();++i) {
  const auto q = state_interfaces_[i].get_value();
  if (!std::isfinite(q)) return controller_interface::CallbackReturn::ERROR;
  if (interface_ == "position") {
   // Permit minor mechanical/encoder overshoot at boundaries to avoid bricking
   // recovery, while rejecting gross out-of-range states that would cause a jump.
   const double margin = std::min(0.02, 0.05 * (upper_[i] - lower_[i]));
   if (q < lower_[i] - margin || q > upper_[i] + margin) return controller_interface::CallbackReturn::ERROR;
   const double seeded = std::clamp(q, lower_[i], upper_[i]);
   reference_.push_back(seeded);
   command_interfaces_[i].set_value(seeded);
  } else {
   reference_.push_back(q);
   command_interfaces_[i].set_value(0.0);
  }
 }
 const auto epoch = epoch_.fetch_add(1, std::memory_order_acq_rel) + 1;
 command_buffer_.writeFromNonRT(CommandSample{epoch, 0.0, {}});
 subscriber_=get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/command", rclcpp::QoS(1),
  [this,epoch](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(writer_mutex_);
    if (epoch != epoch_.load(std::memory_order_acquire) || msg->data.size() != joints_.size() ||
        !std::all_of(msg->data.begin(), msg->data.end(), [](double v) { return std::isfinite(v); })) return;
    command_buffer_.writeFromNonRT(CommandSample{epoch, steadySeconds(), msg->data});
  });
 return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TimedVelocityController::on_deactivate(const rclcpp_lifecycle::State &) {
 std::lock_guard<std::mutex> lock(writer_mutex_);
 epoch_.fetch_add(1, std::memory_order_acq_rel);
 subscriber_.reset();
 command_buffer_.writeFromNonRT(CommandSample{});
 for (size_t i=0;i<command_interfaces_.size();++i)
   command_interfaces_[i].set_value(interface_=="velocity" ? 0.0 :
     std::clamp(state_interfaces_[i].get_value(), lower_[i], upper_[i]));
 return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type TimedVelocityController::update(const rclcpp::Time &, const rclcpp::Duration &period) {
 const auto now = steadySeconds();
 const auto epoch = epoch_.load(std::memory_order_acquire);
 const auto * sample = command_buffer_.readFromRT();
 const bool valid = sample && sample->epoch == epoch && sample->values.size() == joints_.size() &&
   std::isfinite(sample->stamp) && now >= sample->stamp && (now - sample->stamp) <= timeout_;
 const double dt = std::clamp(period.seconds(), 0.0, 0.1);
 for (size_t i=0; i<joints_.size(); ++i) {
  const double q = state_interfaces_[i].get_value();
  if (!std::isfinite(q)) return controller_interface::return_type::ERROR;
  double v = valid ? std::clamp(sample->values[i], -maximum_[i], maximum_[i]) : 0.0;
  if ((q<=lower_[i] && v<0) || (q>=upper_[i] && v>0)) v = 0.0;
  if (interface_=="position") {
    const double next = reference_[i] + v * dt;
    if (reference_[i] < lower_[i]) {
      reference_[i] = std::min(upper_[i], std::max(reference_[i], next));
    } else if (reference_[i] > upper_[i]) {
      reference_[i] = std::max(lower_[i], std::min(reference_[i], next));
    } else {
      reference_[i] = std::clamp(next, lower_[i], upper_[i]);
    }
    command_interfaces_[i].set_value(reference_[i]);
  } else {
    command_interfaces_[i].set_value(v);
  }
 }
 return controller_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(robot_hardware::TimedVelocityController, controller_interface::ControllerInterface)
