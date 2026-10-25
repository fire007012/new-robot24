#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_safety/domain.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"
#include <yaml-cpp/yaml.h>
#include <array>
#include <atomic>
#include <cmath>
#include <thread>
namespace robot_hardware {
class MockSystem : public hardware_interface::SystemInterface {
 std::shared_ptr<robot_safety::Domain> safety_;
 std::shared_ptr<rclcpp::Node> node_;
 rclcpp::executors::MultiThreadedExecutor executor_;
 std::thread thread_;
 std::vector<rclcpp::ServiceBase::SharedPtr> services_;
 std::vector<std::array<double,3>> states_, commands_;
 std::map<std::string,std::map<std::pair<uint16_t,uint8_t>,uint32_t>> objects_;
 std::mutex mutex_;
 std::atomic<bool> fault_{false};
 std::atomic<bool> active_{false};
 uint64_t epoch_=0;
 void clean() { executor_.cancel(); if(thread_.joinable()) thread_.join();
  if(node_) executor_.remove_node(node_); services_.clear(); node_.reset(); }
public:
 ~MockSystem() override { clean(); }
 hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
  if(SystemInterface::on_init(info)!=hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  safety_=robot_safety::domain(info.hardware_parameters.at("safety_domain"));
  states_.resize(info.joints.size(),{0,0,0}); commands_.resize(info.joints.size(),{NAN,NAN,NAN});
  return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
  if(thread_.joinable()) clean();
  node_=std::make_shared<rclcpp::Node>("mock_"+info_.name);
  services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/mock/"+info_.name+"/fault",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){
   fault_=req->data; safety_->report(info_.name,active_&&!fault_); if(fault_) safety_->fault(info_.name+" injected fault"); res->success=true;
  }));
  if(info_.name=="RobotCanopenSystem") {
   services_.push_back(node_->create_service<std_srvs::srv::Trigger>("/robot_runtime/arm",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=safety_->arm();res->message=safety_->diagnostics();}));
   services_.push_back(node_->create_service<std_srvs::srv::Trigger>("/robot_runtime/status",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=safety_->state()==robot_safety::Domain::State::Running;res->message=safety_->diagnostics();}));
   services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/robot_runtime/set_running",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){if(req->data)res->success=safety_->run();else{safety_->halt();res->success=true;}res->message=safety_->diagnostics();}));
   const auto bus=YAML::LoadFile(info_.hardware_parameters.at("bus_config"));
   for(const auto &joint:info_.joints) {
    const auto name=joint.name; const auto cfg=bus[name];
    auto &objects=objects_[name];
    for(auto id:{std::make_pair(0x1000,0),std::make_pair(0x1018,1),std::make_pair(0x1018,2),std::make_pair(0x1018,3),std::make_pair(0x1018,4)})objects[id]=0x4D4F434B;
    for(auto kind:{std::string("rpdo"),std::string("tpdo")}) for(auto entry:cfg[kind]) {
     int n=entry.first.as<int>()-1; auto p=entry.second;
     int comm=(kind=="rpdo"?0x1400:0x1800)+n,map=(kind=="rpdo"?0x1600:0x1A00)+n;
     objects[{comm,1}]=p["cob_id"].as<uint32_t>()|(p["enabled"].as<bool>()?0:0x80000000u);
     objects[{comm,2}]=p["transmission"].as<uint32_t>();
     objects[{map,0}]=p["mapping"].size();
     int sub=1; for(auto obj:p["mapping"]) {
      int i=obj["index"].as<int>(),s=obj["sub_index"].as<int>();
      int bits=(i==0x6060||i==0x6061)?8:(i==0x6040||i==0x6041||i==0x6077)?16:32;
      objects[{map,sub++}]=(i<<16)|(s<<8)|bits;
     }
    }
    objects[{0x607C,0}]=0;objects[{0x2003,0}]=0;objects[{0x607D,1}]=uint32_t(INT32_MIN);objects[{0x607D,2}]=INT32_MAX;
    objects[{0x6064,0}]=0;objects[{0x1010,1}]=1;
    services_.push_back(node_->create_service<canopen_interfaces::srv::CORead>("/"+name+"/sdo_read",[this,name](const std::shared_ptr<canopen_interfaces::srv::CORead::Request> req,std::shared_ptr<canopen_interfaces::srv::CORead::Response> res){
      std::lock_guard<std::mutex> lock(mutex_);
      if(req->index==0x100A){res->success=true;res->text="MOCK";return;}
      auto &o=objects_[name]; auto it=o.find({req->index,req->subindex});res->success=it!=o.end();if(res->success)res->data=it->second;
    }));
    services_.push_back(node_->create_service<canopen_interfaces::srv::COWrite>("/"+name+"/sdo_write",[this,name](const std::shared_ptr<canopen_interfaces::srv::COWrite::Request> req,std::shared_ptr<canopen_interfaces::srv::COWrite::Response> res){
      std::lock_guard<std::mutex> lock(mutex_);
      if(safety_->state()==robot_safety::Domain::State::Running){res->success=false;return;}
      objects_[name][{req->index,req->subindex}]=req->data;res->success=true;
    }));
    services_.push_back(node_->create_service<std_srvs::srv::Trigger>("/"+name+"/disable",[](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=true;}));
   }
  }
  executor_.add_node(node_);thread_=std::thread([this]{executor_.spin();});
  return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
  active_=true; safety_->report(info_.name,!fault_); return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
  active_=false;safety_->report(info_.name,false);for(auto &c:commands_)c={NAN,NAN,NAN};
  return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {clean();return hardware_interface::CallbackReturn::SUCCESS;}
 hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
  active_=false;safety_->report(info_.name,false);
  for(auto &s:states_)s[1]=0;for(auto &c:commands_)c={NAN,NAN,NAN};
  return hardware_interface::CallbackReturn::SUCCESS;
 }
 std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
  std::vector<hardware_interface::StateInterface> out;
  for(size_t i=0;i<info_.joints.size();++i) for(const auto &s:info_.joints[i].state_interfaces)
    out.emplace_back(info_.joints[i].name,s.name,&states_[i][s.name=="position"?0:s.name=="velocity"?1:2]);return out;
 }
 std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
  std::vector<hardware_interface::CommandInterface> out;
  for(size_t i=0;i<info_.joints.size();++i) for(const auto &s:info_.joints[i].command_interfaces)
    out.emplace_back(info_.joints[i].name,s.name,&commands_[i][s.name=="position"?0:s.name=="velocity"?1:2]);return out;
 }
 hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &,const std::vector<std::string> &) override {
  epoch_=safety_->epoch();for(auto &c:commands_)c={NAN,NAN,NAN};return hardware_interface::return_type::OK;
 }
 hardware_interface::return_type read(const rclcpp::Time &,const rclcpp::Duration &) override {
  safety_->report(info_.name,active_&&!fault_);return fault_?hardware_interface::return_type::ERROR:hardware_interface::return_type::OK;
 }
 hardware_interface::return_type write(const rclcpp::Time &,const rclcpp::Duration &period) override {
  if(info_.name=="RobotCanopenSystem") safety_->observePeriod(period.nanoseconds());
  for(size_t i=0;i<states_.size();++i) {
    if(!active_||!safety_->permits(epoch_)){states_[i][1]=0;commands_[i]={NAN,NAN,NAN};continue;}
    auto &s=states_[i];auto &c=commands_[i];
    if(std::isfinite(c[0])){s[1]=(c[0]-s[0])/period.seconds();s[0]=c[0];}
    else if(std::isfinite(c[1])){s[1]=c[1];s[0]+=c[1]*period.seconds();}
    else s[1]=0;
  }
  return hardware_interface::return_type::OK;
 }
};
}
PLUGINLIB_EXPORT_CLASS(robot_hardware::MockSystem,hardware_interface::SystemInterface)
