#pragma once

#include <functional>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "canopen_hw/operational_coordinator.hpp"

namespace canopen_hw {

// ServiceGateway 仅负责 ROS service 的参数校验与转发，不持有业务状态。
class ServiceGateway {
 public:
  ServiceGateway(ros::NodeHandle* pnh, OperationalCoordinator* coordinator,
                 std::mutex* loop_mtx);
  void SetPostInitHook(std::function<bool(std::string*)> hook);

 private:
  bool OnInit(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
  bool OnEnable(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
  bool OnDisable(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
  bool OnHalt(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
  bool OnResume(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
  bool OnRecover(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);
  bool OnShutdown(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);

  OperationalCoordinator* coordinator_ = nullptr;
  std::mutex* loop_mtx_ = nullptr;
  std::function<bool(std::string*)> post_init_hook_;

  ros::ServiceServer init_srv_;
  ros::ServiceServer enable_srv_;
  ros::ServiceServer disable_srv_;
  ros::ServiceServer halt_srv_;
  ros::ServiceServer resume_srv_;
  ros::ServiceServer recover_srv_;
  ros::ServiceServer shutdown_srv_;
};

}  // namespace canopen_hw
