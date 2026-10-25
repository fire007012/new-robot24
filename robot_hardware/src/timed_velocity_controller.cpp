#include "robot_hardware/timed_velocity_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace robot_hardware {
controller_interface::CallbackReturn TimedVelocityController::on_init() {
 try {
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
 watchdog_ = VelocityWatchdog(joints_.size());
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
 std::lock_guard<std::mutex> lock(mutex_);
 reference_.clear();
 for (size_t i=0;i<joints_.size();++i) {
  const auto q = state_interfaces_[i].get_value();
  if (!std::isfinite(q) || q<lower_[i] || q>upper_[i]) return controller_interface::CallbackReturn::ERROR;
  reference_.push_back(q);
  command_interfaces_[i].set_value(interface_=="velocity" ? 0.0:q);
 }
 const auto epoch=watchdog_.reset();
 subscriber_=get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/command", rclcpp::QoS(1),
  [this,epoch](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    watchdog_.accept(msg->data, std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),epoch);
  });
 return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TimedVelocityController::on_deactivate(const rclcpp_lifecycle::State &) {
 std::lock_guard<std::mutex> lock(mutex_); watchdog_.reset(); subscriber_.reset();
 for (size_t i=0;i<command_interfaces_.size();++i)
   command_interfaces_[i].set_value(interface_=="velocity"?0.0:state_interfaces_[i].get_value());
 return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type TimedVelocityController::update(const rclcpp::Time &, const rclcpp::Duration &period) {
 std::lock_guard<std::mutex> lock(mutex_);
 const auto now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
 auto velocities=watchdog_.sample(now,timeout_);
 for(size_t i=0;i<joints_.size();++i) {
  const double q=state_interfaces_[i].get_value();
  if (!std::isfinite(q)) return controller_interface::return_type::ERROR;
  double v=std::clamp(velocities[i],-maximum_[i],maximum_[i]);
  if ((q<=lower_[i] && v<0)||(q>=upper_[i] && v>0)) v=0;
  if (interface_=="position") {
    reference_[i]=std::clamp(reference_[i]+v*std::clamp(period.seconds(),0.0,0.02),lower_[i],upper_[i]);
    command_interfaces_[i].set_value(reference_[i]);
  } else command_interfaces_[i].set_value(v);
 }
 return controller_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(robot_hardware::TimedVelocityController, controller_interface::ControllerInterface)
