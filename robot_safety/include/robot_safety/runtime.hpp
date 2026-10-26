#pragma once
#include "robot_safety/policy.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
namespace robot_safety {
// Owned in-process by the hardware plugins; never opens a CAN channel or creates a master.
class Runtime {
public:
  using Operation = std::function<void()>;
  explicit Runtime(const std::string & node_name);
  ~Runtime();
  Runtime(const Runtime &) = delete;
  Runtime & operator=(const Runtime &) = delete;
  void service(const std::string & path, Operation operation, const std::string & group = "");
  void retainSubscription(const rclcpp::SubscriptionBase::SharedPtr & subscription) { subscriptions_.push_back(subscription); }
  void retain(const rclcpp::ServiceBase::SharedPtr & service) { services_.push_back(service); }
  void start();
  std::shared_ptr<rclcpp::Node> node;
private:
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::thread thread_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, rclcpp::CallbackGroup::SharedPtr> callback_groups_;
};
std::shared_ptr<Runtime> guard(const std::string & name, const std::shared_ptr<Domain> & domain);
}
