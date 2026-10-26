#include "canopen_hw/service_gateway.hpp"

#include <utility>

namespace canopen_hw {

namespace {

bool EnsureGatewayReady(OperationalCoordinator* coordinator, std::mutex* loop_mtx,
                        std_srvs::Trigger::Response* res) {
  if (res == nullptr) {
    return false;
  }
  if (!coordinator || !loop_mtx) {
    res->success = false;
    res->message = "service gateway not initialized";
    return false;
  }
  return true;
}

}  // namespace

ServiceGateway::ServiceGateway(ros::NodeHandle* pnh,
                               OperationalCoordinator* coordinator,
                               std::mutex* loop_mtx)
    : coordinator_(coordinator), loop_mtx_(loop_mtx) {
  if (pnh == nullptr) {
    return;
  }

  init_srv_ = pnh->advertiseService("init", &ServiceGateway::OnInit, this);
  enable_srv_ =
      pnh->advertiseService("enable", &ServiceGateway::OnEnable, this);
  disable_srv_ =
      pnh->advertiseService("disable", &ServiceGateway::OnDisable, this);
  halt_srv_ = pnh->advertiseService("halt", &ServiceGateway::OnHalt, this);
  resume_srv_ =
      pnh->advertiseService("resume", &ServiceGateway::OnResume, this);
  recover_srv_ =
      pnh->advertiseService("recover", &ServiceGateway::OnRecover, this);
  shutdown_srv_ =
      pnh->advertiseService("shutdown", &ServiceGateway::OnShutdown, this);
}

void ServiceGateway::SetPostInitHook(std::function<bool(std::string*)> hook) {
  post_init_hook_ = std::move(hook);
}

bool ServiceGateway::OnInit(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestInit();
  if (!r.ok) {
    res.success = false;
    res.message = r.message.empty() ? "init failed" : r.message;
    return true;
  }

  const bool already = (r.message.rfind("already ", 0) == 0);
  if (!already && post_init_hook_) {
    std::string hook_detail;
    if (!post_init_hook_(&hook_detail)) {
      const auto shutdown_r = coordinator_->RequestShutdown();
      res.success = false;
      res.message = hook_detail.empty() ? "post-init hook failed" : hook_detail;
      if (!shutdown_r.ok) {
        res.message += "; rollback shutdown failed: " + shutdown_r.message;
      }
      return true;
    }
  }

  res.success = true;
  res.message = already ? "already initialized" : "initialized (armed)";
  return true;
}

bool ServiceGateway::OnEnable(std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestEnable();
  res.success = r.ok;
  if (r.ok) {
    res.message = (r.message.rfind("already ", 0) == 0) ? "already enabled"
                                                         : "enabled (armed)";
  } else {
    res.message = r.message.empty() ? "enable failed" : r.message;
  }
  return true;
}

bool ServiceGateway::OnDisable(std_srvs::Trigger::Request&,
                               std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestDisable();
  res.success = r.ok;
  if (r.ok) {
    res.message = (r.message.rfind("already ", 0) == 0)
                      ? "already disabled"
                      : "disabled (standby)";
  } else {
    res.message = r.message.empty() ? "disable failed" : r.message;
  }
  return true;
}

bool ServiceGateway::OnHalt(std_srvs::Trigger::Request&,
                            std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestHalt();
  res.success = r.ok;
  if (r.ok) {
    res.message = (r.message.rfind("already ", 0) == 0) ? "already halted"
                                                         : "halted";
  } else {
    res.message = r.message.empty() ? "halt failed" : r.message;
  }
  return true;
}

bool ServiceGateway::OnResume(std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestRelease();
  res.success = r.ok;
  if (r.ok) {
    res.message = (r.message.rfind("already ", 0) == 0) ? "already running"
                                                         : "resumed";
  } else {
    res.message = r.message.empty() ? "resume failed; call ~/enable (or ~/recover then ~/enable) first"
                                    : r.message;
  }
  return true;
}

bool ServiceGateway::OnRecover(std_srvs::Trigger::Request&,
                               std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestRecover();
  res.success = r.ok;
  if (r.ok) {
    res.message = r.message.empty() ? "recovered (standby)" : r.message;
  } else {
    res.message = r.message.empty() ? "recover failed" : r.message;
  }
  return true;
}

bool ServiceGateway::OnShutdown(std_srvs::Trigger::Request&,
                                std_srvs::Trigger::Response& res) {
  if (!EnsureGatewayReady(coordinator_, loop_mtx_, &res)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto r = coordinator_->RequestShutdown();
  res.success = r.ok;
  if (r.ok) {
    res.message = "communication stopped; call ~/init first";
  } else {
    res.message = r.message.empty() ? "shutdown failed" : r.message;
  }
  return true;
}

}  // namespace canopen_hw
