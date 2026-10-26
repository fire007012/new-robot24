#include <gz_ros2_control/gz_system_interface.hpp>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <robot_hardware/external_plant.hpp>
#include <algorithm>
#include <cmath>
#include <map>

namespace robot_simulation {
class GazeboSystem : public gz_ros2_control::GazeboSimSystemInterface {
 pluginlib::ClassLoader<hardware_interface::SystemInterface> loader_{"hardware_interface","hardware_interface::SystemInterface"};
 std::shared_ptr<hardware_interface::SystemInterface> guard_;
 robot_hardware::ExternalPlant * plant_=nullptr;
 sim::EntityComponentManager * ecm_=nullptr;
 std::map<std::string,sim::Entity> joints_;
 std::vector<hardware_interface::ReadWriteHandle> commands_;
 std::vector<std::array<double,3>> measured_;
 bool configured_=false;
 void ensure(sim::Entity entity) {
  if(!ecm_->Component<sim::components::JointPosition>(entity))ecm_->CreateComponent(entity,sim::components::JointPosition());
  if(!ecm_->Component<sim::components::JointVelocity>(entity))ecm_->CreateComponent(entity,sim::components::JointVelocity());
  if(!ecm_->Component<sim::components::JointVelocityCmd>(entity))ecm_->CreateComponent(entity,sim::components::JointVelocityCmd({0.}));
 }
 void velocity(const std::string & name,double value) {
  auto it=joints_.find(name);if(it==joints_.end())return;
  ensure(it->second);ecm_->Component<sim::components::JointVelocityCmd>(it->second)->Data()={value};
 }
public:
 hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
  if(hardware_interface::SystemInterface::on_init(info)!=hardware_interface::CallbackReturn::SUCCESS)return hardware_interface::CallbackReturn::ERROR;
  if(guard_)return hardware_interface::CallbackReturn::SUCCESS;
  guard_=loader_.createSharedInstance("robot_hardware/MockSystem");
  auto config=info;config.hardware_parameters["external_plant"]="true";
  if(guard_->on_init(config)!=hardware_interface::CallbackReturn::SUCCESS)return hardware_interface::CallbackReturn::ERROR;
  plant_=dynamic_cast<robot_hardware::ExternalPlant *>(guard_.get());
  return plant_?hardware_interface::CallbackReturn::SUCCESS:hardware_interface::CallbackReturn::ERROR;
 }
 bool initSim(rclcpp::Node::SharedPtr &,std::map<std::string,sim::Entity> & joints,
              const hardware_interface::HardwareInfo & info,sim::EntityComponentManager & ecm,int &) override {
  ecm_=&ecm;joints_=joints;measured_.resize(info.joints.size(),{0,0,0});
  if(on_init(info)!=hardware_interface::CallbackReturn::SUCCESS)return false;
  for(auto & joint:info.joints){if(!joints_.count(joint.name))return false;ensure(joints_.at(joint.name));}
  return true;
 }
 hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override {configured_=true;return guard_->on_configure(state);}
 hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {return guard_->on_activate(state);}
 hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {return guard_->on_deactivate(state);}
 hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override {configured_=false;return guard_->on_cleanup(state);}
 hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override {configured_=false;return guard_->on_shutdown(state);}
 std::vector<hardware_interface::StateInterface> export_state_interfaces() override {return guard_->export_state_interfaces();}
 std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
  auto out=guard_->export_command_interfaces();commands_.clear();for(auto & c:out)commands_.push_back(c);return out;
 }
 hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start,const std::vector<std::string>& stop) override {return guard_->prepare_command_mode_switch(start,stop);}
 hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start,const std::vector<std::string>& stop) override {return guard_->perform_command_mode_switch(start,stop);}
 hardware_interface::return_type read(const rclcpp::Time & now,const rclcpp::Duration & period) override {
  for(size_t i=0;i<info_.joints.size();++i){
   auto entity=joints_.at(info_.joints[i].name);
   auto p=ecm_->Component<sim::components::JointPosition>(entity);auto v=ecm_->Component<sim::components::JointVelocity>(entity);
   measured_[i]={p&&!p->Data().empty()?p->Data()[0]:0.,v&&!v->Data().empty()?v->Data()[0]:0.,0.};
  }
  plant_->measuredPlant(measured_);return guard_->read(now,period);
 }
 hardware_interface::return_type write(const rclcpp::Time & now,const rclcpp::Duration & period) override {
  auto result=guard_->write(now,period);
  for(size_t i=0;i<info_.joints.size();++i){
   const auto & name=info_.joints[i].name;double output=0.;double position=NAN,vel=NAN;
   for(const auto & c:commands_)if(c.get_prefix_name()==name){if(c.get_interface_name()=="position")position=c.get_value();if(c.get_interface_name()=="velocity")vel=c.get_value();}
   if(configured_){
    if(std::isfinite(position))output=20.*(position-measured_[i][0]);
    else if(std::isfinite(vel))output=vel;
   }
   const double maximum=name.find("finger")!=std::string::npos?.02:name.find("wheel")!=std::string::npos?15.:1.57;
   output=std::clamp(output,-maximum,maximum);velocity(name,output);
   if(name=="left_front_wheel_joint")velocity("left_rear_wheel_joint",output);
   if(name=="right_front_wheel_joint")velocity("right_rear_wheel_joint",output);
   if(name=="left_gripper_finger_joint")velocity("right_gripper_finger_joint",output);
  }
  return result;
 }
};
}
PLUGINLIB_EXPORT_CLASS(robot_simulation::GazeboSystem,gz_ros2_control::GazeboSimSystemInterface)
