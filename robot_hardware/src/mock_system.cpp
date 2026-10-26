#include "robot_hardware/plugin_lifetime.hpp"
#include "hardware_interface/system_interface.hpp"
#include "robot_hardware/external_plant.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_safety/domain.hpp"
#include "robot_safety/runtime.hpp"
#include "robot_safety/axis_session.hpp"
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
class MockSystem : public hardware_interface::SystemInterface, public ExternalPlant {
 bool external_plant_=false;
 std::shared_ptr<robot_safety::Domain> safety_;
 std::shared_ptr<rclcpp::Node> node_;
 rclcpp::executors::MultiThreadedExecutor executor_;
 std::thread thread_;
 std::vector<rclcpp::ServiceBase::SharedPtr> services_;
 std::vector<std::array<double,3>> states_, commands_;
 std::map<std::string,std::map<std::pair<uint16_t,uint8_t>,uint32_t>> objects_;
 std::mutex mutex_;
 std::map<std::string,std::shared_ptr<std::atomic<bool>>> save_failures_;
 std::atomic<bool> fault_{false};
 std::atomic<bool> active_{false};
 uint64_t epoch_=0;
 bool grouped_=false;
 std::shared_ptr<robot_safety::Runtime> guard_, operations_;
 std::vector<robot_safety::AxisSession> sessions_;
 std::vector<std::shared_ptr<std::atomic<bool>>> axis_faults_;
 std::vector<bool> claimed_;
 std::vector<std::array<double,3>> physical_;
 std::vector<rclcpp::TimerBase::SharedPtr> timers_;
 bool offline(size_t i) const { return fault_ || axis_faults_[i]->load(); }
 void unprepare() {
  if(grouped_) for(size_t i=0;i<sessions_.size();++i) {
   sessions_[i].prepared->store(false); safety_->unprepareAxis(info_.joints[i].name,"component inactive");
  }
 }
 void clean() { operations_.reset(); timers_.clear(); unprepare(); executor_.cancel(); if(thread_.joinable()) thread_.join();
  if(grouped_)for(auto & state:states_)state={NAN,NAN,NAN};
  if(node_) executor_.remove_node(node_); services_.clear(); node_.reset(); }
public:
 void measuredPlant(const std::vector<std::array<double,3>> & measured) override {
  if(!external_plant_ || measured.size()!=physical_.size())throw std::runtime_error("invalid external simulation feedback");
  physical_=measured;
 }
 ~MockSystem() override { clean(); }
 hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
  retainPluginCode();
  if(SystemInterface::on_init(info)!=hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  external_plant_=info.hardware_parameters.count("external_plant") && info.hardware_parameters.at("external_plant")=="true";
  safety_=robot_safety::domain(info.hardware_parameters.at("safety_domain"));
  grouped_=info.hardware_parameters.count("group_policy");
  if(grouped_) {
   safety_=robot_safety::groupedDomain(info.hardware_parameters.at("safety_domain"),info.hardware_parameters.at("group_policy"));
   guard_=robot_safety::guard(info.hardware_parameters.at("safety_domain"),safety_);
   sessions_.resize(info.joints.size());claimed_.resize(info.joints.size(),false);
   physical_.resize(info.joints.size(),{0,0,0});
   for(size_t i=0;i<info.joints.size();++i) {
    sessions_[i].group=safety_->axisGroup(info.joints[i].name,info.name);
    axis_faults_.push_back(std::make_shared<std::atomic<bool>>(false));
   }
  }
  safety_->registerWriter(info.name,grouped_);
  states_.resize(info.joints.size(),{0,0,0}); commands_.resize(info.joints.size(),{NAN,NAN,NAN});
  return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
  if(thread_.joinable()) clean();
  node_=std::make_shared<rclcpp::Node>("mock_"+info_.name);
  services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/mock/"+info_.name+"/fault",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){
   fault_=req->data;
   if(grouped_) {
    if(fault_) for(const auto & joint:info_.joints) safety_->reportAxis(joint.name,false,"injected shared transport failure");
   } else { safety_->report(info_.name,active_&&!fault_); if(fault_) safety_->fault(info_.name+" injected fault"); }
   res->success=true;
  }));
  if(grouped_) {
   operations_=std::make_shared<robot_safety::Runtime>("mock_"+info_.name+"_operations");
   std::set<std::string> groups;
   for(size_t i=0;i<sessions_.size();++i) {
    groups.insert(sessions_[i].group);
    services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/mock/"+info_.joints[i].name+"/fault",
     [this,i](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      axis_faults_[i]->store(request->data);
      if(request->data)safety_->reportAxis(info_.joints[i].name,false,"injected axis fault"); response->success=true;
     }));
   }
   for(const auto & group:groups) {
    operations_->service("/robot_backend/"+info_.name+"/"+group+"/initialize",[] {},group);
    auto delayed=std::make_shared<std::atomic<bool>>(false);
    services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/mock/"+info_.name+"/"+group+"/prepare_delay",
      [delayed](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        delayed->store(request->data);response->success=true;
      }));
    operations_->service("/robot_backend/"+info_.name+"/"+group+"/prepare",[this,group,delayed] {
      if(!active_)throw std::runtime_error("component inactive");
      const auto state=safety_->group(group).state;
      if(state==robot_safety::Domain::State::Armed||state==robot_safety::Domain::State::Running)
        throw std::runtime_error("halt before preparation");
      const auto ticket=safety_->epoch(group);
      if(delayed->load())std::this_thread::sleep_for(std::chrono::seconds(2));
      if(!active_||safety_->epoch(group)!=ticket||safety_->globallyInhibited())throw std::runtime_error("late preparation cancelled");
      for(size_t i=0;i<sessions_.size();++i) if(sessions_[i].group==group) {
        if(offline(i))throw std::runtime_error(info_.joints[i].name+": offline");
        sessions_[i].prepared_epoch->store(ticket);sessions_[i].prepared->store(true);
      }
    },group);
   }
   operations_->start();
   if(info_.name=="CanDriverSystem") {
    const std::map<std::string,std::string> sources{{"arm_secured","commissioned_arm_pose_and_brake"},
     {"support_trusted","commissioned_support_state"},{"payload_retained","commissioned_payload_state"}};
    for(const auto & entry:sources) {
     auto trusted=std::make_shared<std::atomic<bool>>(false);
     services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/mock/conditions/"+entry.first,
      [this,trusted,entry](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
       trusted->store(request->data);
       safety_->condition(entry.first,request->data?robot_safety::Domain::Evidence::Satisfied:robot_safety::Domain::Evidence::Unknown,
        entry.second,request->data?std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()+200000000:0);
       response->success=true;
      }));
     timers_.push_back(node_->create_wall_timer(std::chrono::milliseconds(50),[this,trusted,entry] {
      if(trusted->load())safety_->condition(entry.first,robot_safety::Domain::Evidence::Satisfied,entry.second,
       std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()+200000000);
     }));
    }
   }
  }
  if(info_.name=="RobotCanopenSystem") {
   if(!grouped_) {
   services_.push_back(node_->create_service<std_srvs::srv::Trigger>("/robot_runtime/arm",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=safety_->arm();res->message=safety_->diagnostics();}));
   services_.push_back(node_->create_service<std_srvs::srv::Trigger>("/robot_runtime/status",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=safety_->state()==robot_safety::Domain::State::Running;res->message=safety_->diagnostics();}));
   services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/robot_runtime/set_running",[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){if(req->data)res->success=safety_->run();else{safety_->halt();res->success=true;}res->message=safety_->diagnostics();}));
   }
   const auto bus=YAML::LoadFile(info_.hardware_parameters.at("bus_config"));
   for(const auto &joint:info_.joints) {
    const auto name=joint.name; const auto cfg=bus[name];
    save_failures_[name]=std::make_shared<std::atomic<bool>>(false);
    if(grouped_) services_.push_back(node_->create_service<std_srvs::srv::SetBool>("/mock/"+name+"/save_failure",
      [flag=save_failures_[name]](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        flag->store(request->data);response->success=true;
      }));
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
      if(fault_){res->success=false;return;}
      if(grouped_) for(size_t i=0;i<info_.joints.size();++i) if(info_.joints[i].name==name&&offline(i)){res->success=false;return;}
      if(req->index==0x100A){res->success=true;res->text="MOCK";return;}
      auto &o=objects_[name]; auto it=o.find({req->index,req->subindex});res->success=it!=o.end();if(res->success)res->data=it->second;
    }));
    services_.push_back(node_->create_service<canopen_interfaces::srv::COWrite>("/"+name+"/sdo_write",[this,name](const std::shared_ptr<canopen_interfaces::srv::COWrite::Request> req,std::shared_ptr<canopen_interfaces::srv::COWrite::Response> res){
      std::lock_guard<std::mutex> lock(mutex_);
      if(grouped_?safety_->group(safety_->axisGroup(name,info_.name)).state==robot_safety::Domain::State::Running:safety_->state()==robot_safety::Domain::State::Running){res->success=false;return;}
      if(req->index==0x1010&&save_failures_[name]->load()){res->success=false;return;}
      objects_[name][{req->index,req->subindex}]=req->data;res->success=true;
    }));
    services_.push_back(node_->create_service<std_srvs::srv::Trigger>("/"+name+"/disable",[](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=true;}));
   }
  }
  executor_.add_node(node_);thread_=std::thread([this]{executor_.spin();});
  return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
  if(grouped_)for(auto &session:sessions_)session.permission_lost=true;
  active_=true; if(!grouped_)safety_->report(info_.name,!fault_); return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
  active_=false;if(grouped_)unprepare();else safety_->report(info_.name,false);for(auto &c:commands_)c={NAN,NAN,NAN};
  return hardware_interface::CallbackReturn::SUCCESS;
 }
 hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {clean();return hardware_interface::CallbackReturn::SUCCESS;}
 hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
  active_=false;if(grouped_)unprepare();else safety_->report(info_.name,false);
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
 hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start,const std::vector<std::string> &stop) override {
  if(grouped_) {
   for(size_t i=0;i<sessions_.size();++i) {
    auto matches=[&](const std::string & name){return name.rfind(info_.joints[i].name+"/",0)==0;};
    const auto count=std::count_if(start.begin(),start.end(),matches);
    if(count>1||(count&&(!sessions_[i].prepared->load()||safety_->group(sessions_[i].group).state!=robot_safety::Domain::State::Armed)))
     return hardware_interface::return_type::ERROR;
   }
   for(size_t i=0;i<sessions_.size();++i) {
    auto matches=[&](const std::string & name){return name.rfind(info_.joints[i].name+"/",0)==0;};
    const bool starting=std::any_of(start.begin(),start.end(),matches),stopping=std::any_of(stop.begin(),stop.end(),matches);
    if(starting||stopping) {
     claimed_[i]=starting;commands_[i]={NAN,NAN,NAN};sessions_[i].permission_lost=!starting;
     if(stopping)physical_[i][1]=0;
     if(starting)sessions_[i].epoch=safety_->epoch(sessions_[i].group);
    }
   }
   return hardware_interface::return_type::OK;
  }
  epoch_=safety_->epoch();for(auto &c:commands_)c={NAN,NAN,NAN};return hardware_interface::return_type::OK;
 }
 hardware_interface::return_type read(const rclcpp::Time &,const rclcpp::Duration &) override {
  if(grouped_) {
   for(size_t i=0;i<sessions_.size();++i) {
    states_[i]=offline(i)?std::array<double,3>{NAN,NAN,NAN}:physical_[i];
    if(sessions_[i].prepared->load())safety_->reportAxis(info_.joints[i].name,active_&&!offline(i),"mock axis feedback");
    else safety_->unprepareAxis(info_.joints[i].name,"not prepared");
   }
   return hardware_interface::return_type::OK;
  }
  safety_->report(info_.name,active_&&!fault_);return fault_?hardware_interface::return_type::ERROR:hardware_interface::return_type::OK;
 }
 hardware_interface::return_type write(const rclcpp::Time &,const rclcpp::Duration &period) override {
  if(info_.name=="RobotCanopenSystem") safety_->observePeriod(period.nanoseconds());
  for(size_t i=0;i<states_.size();++i) {
    if(grouped_) {
     if(!active_||!safety_->permits(sessions_[i].group,sessions_[i].epoch)) {
      physical_[i][1]=0;commands_[i]={NAN,NAN,NAN};
      const auto state=safety_->group(sessions_[i].group).state;
      if((state!=robot_safety::Domain::State::Armed&&!sessions_[i].permission_lost)||
         (sessions_[i].prepared->load()&&(safety_->globallyInhibited()||
          (state!=robot_safety::Domain::State::Armed&&sessions_[i].prepared_epoch->load()!=safety_->epoch(sessions_[i].group))))) {
       sessions_[i].permission_lost=true;sessions_[i].prepared->store(false);claimed_[i]=false;
      }
      continue;
     }
     sessions_[i].permission_lost=false;
     if(!claimed_[i])continue;
     if(external_plant_)continue;
     auto &s=physical_[i];auto &c=commands_[i];
     if(std::isfinite(c[0])){s[1]=(c[0]-s[0])/period.seconds();s[0]=c[0];}
     else if(std::isfinite(c[1])){s[1]=c[1];s[0]+=c[1]*period.seconds();}
     else s[1]=0;
     continue;
    }
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
