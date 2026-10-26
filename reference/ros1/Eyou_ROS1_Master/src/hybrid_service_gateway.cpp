#include "Eyou_ROS1_Master/hybrid_service_gateway.hpp"

#include <set>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace eyou_ros1_master {

namespace {

bool ResolveCanDriverInitRequest(const ros::NodeHandle& can_driver_pnh,
                                 std::string* device,
                                 bool* loopback,
                                 std::string* error) {
    if (device != nullptr) {
        device->clear();
    }
    if (loopback != nullptr) {
        *loopback = false;
    }
    if (error != nullptr) {
        error->clear();
    }

    XmlRpc::XmlRpcValue joint_list;
    if (!can_driver_pnh.getParam("joints", joint_list)) {
        if (error != nullptr) {
            *error = "can_driver joints config is missing";
        }
        return false;
    }
    if (joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        if (error != nullptr) {
            *error = "can_driver joints config is not an array";
        }
        return false;
    }

    std::set<std::string> devices;
    for (int i = 0; i < joint_list.size(); ++i) {
        if (joint_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !joint_list[i].hasMember("can_device")) {
            continue;
        }
        const std::string can_device =
            static_cast<std::string>(joint_list[i]["can_device"]);
        if (!can_device.empty()) {
            devices.insert(can_device);
        }
    }

    if (devices.empty()) {
        if (error != nullptr) {
            *error = "can_driver joints config does not define any can_device";
        }
        return false;
    }
    if (devices.size() != 1U) {
        if (error != nullptr) {
            std::string detail;
            for (const auto& candidate : devices) {
                if (!detail.empty()) {
                    detail += ", ";
                }
                detail += candidate;
            }
            *error =
                "hybrid init requires a single can_driver device; configured devices: " +
                detail;
        }
        return false;
    }

    if (device != nullptr) {
        *device = *devices.begin();
    }

    bool init_loopback = false;
    can_driver_pnh.param("init_loopback", init_loopback, false);
    if (loopback != nullptr) {
        *loopback = init_loopback;
    }
    return true;
}

}  // namespace

void HybridServiceGateway::SetPostInitHook(std::function<bool(std::string*)> hook) {
    post_init_hook_ = std::move(hook);
}

HybridServiceGateway::HybridServiceGateway(
    ros::NodeHandle& pnh,
    const ros::NodeHandle& can_driver_pnh,
    HybridOperationalCoordinator* coordinator,
    std::mutex* loop_mtx,
    bool advertise_services)
    : coordinator_(coordinator),
      loop_mtx_(loop_mtx),
      can_driver_pnh_(can_driver_pnh) {
    if (advertise_services) {
        init_srv_     = pnh.advertiseService("init",     &HybridServiceGateway::OnInit,     this);
        enable_srv_   = pnh.advertiseService("enable",   &HybridServiceGateway::OnEnable,   this);
        disable_srv_  = pnh.advertiseService("disable",  &HybridServiceGateway::OnDisable,  this);
        halt_srv_     = pnh.advertiseService("halt",     &HybridServiceGateway::OnHalt,     this);
        resume_srv_   = pnh.advertiseService("resume",   &HybridServiceGateway::OnResume,   this);
        recover_srv_  = pnh.advertiseService("recover",  &HybridServiceGateway::OnRecover,  this);
        shutdown_srv_ = pnh.advertiseService("shutdown", &HybridServiceGateway::OnShutdown, this);

        ROS_INFO("[HybridServiceGateway] 7 services registered under %s",
                 pnh.getNamespace().c_str());
    }
}

bool HybridServiceGateway::RunInitSequence(const std::string& device,
                                           bool loopback,
                                           std::string* message,
                                           bool* already_initialized) {
    if (message != nullptr) {
        message->clear();
    }
    if (already_initialized != nullptr) {
        *already_initialized = false;
    }

    if (coordinator_ == nullptr || loop_mtx_ == nullptr) {
        if (message != nullptr) {
            *message = "service gateway not initialized";
        }
        return false;
    }

    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestInit(device, loopback);
    if (!r.ok) {
        if (message != nullptr) {
            *message = r.message.empty() ? "init failed" : r.message;
        }
        return false;
    }

    const bool already = r.already;
    if (already_initialized != nullptr) {
        *already_initialized = already;
    }

    if (!already && post_init_hook_) {
        std::string hook_detail;
        if (!post_init_hook_(&hook_detail)) {
            const auto shutdown_r = coordinator_->RequestShutdown(/*force=*/false);
            if (message != nullptr) {
                *message = hook_detail.empty() ? "post-init hook failed" : hook_detail;
                if (!shutdown_r.ok) {
                    *message += "; rollback shutdown failed: " + shutdown_r.message;
                }
            }
            return false;
        }
    }

    if (message != nullptr) {
        *message = already ? "already initialized" : "initialized (armed)";
    }
    return true;
}

bool HybridServiceGateway::RunConfiguredInitSequence(std::string* message,
                                                     bool* already_initialized) {
    std::string device;
    bool loopback = false;
    std::string error;
    if (!ResolveCanDriverInitRequest(can_driver_pnh_, &device, &loopback, &error)) {
        if (message != nullptr) {
            *message = error;
        }
        if (already_initialized != nullptr) {
            *already_initialized = false;
        }
        return false;
    }
    return RunInitSequence(device, loopback, message, already_initialized);
}

bool HybridServiceGateway::OnInit(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& res) {
    bool already = false;
    res.success = RunConfiguredInitSequence(&res.message, &already);
    return true;
}

bool HybridServiceGateway::OnEnable(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestEnable();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnDisable(std_srvs::Trigger::Request&,
                                     std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestDisable();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnHalt(std_srvs::Trigger::Request&,
                                  std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestHalt();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnResume(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestRelease();  // resume = Armed → Running
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnRecover(std_srvs::Trigger::Request&,
                                     std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestRecover();
    res.success = r.ok;
    res.message = r.message;
    return true;
}

bool HybridServiceGateway::OnShutdown(std_srvs::Trigger::Request&,
                                      std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lk(*loop_mtx_);
    auto r = coordinator_->RequestShutdown(/*force=*/false);
    res.success = r.ok;
    res.message = r.message;
    return true;
}

}  // namespace eyou_ros1_master
