#pragma once

#include <string>

#include <ros/ros.h>

#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"
#include "Eyou_ROS1_Master/hybrid_service_gateway.hpp"

namespace eyou_ros1_master {

struct HybridAutoStartupOptions {
    bool auto_init{false};
    bool auto_enable{false};
    bool auto_release{false};
};

HybridAutoStartupOptions LoadHybridAutoStartupOptions(const ros::NodeHandle& pnh);

bool RunHybridAutoStartup(const HybridAutoStartupOptions& options,
                          HybridServiceGateway& service_gateway,
                          HybridOperationalCoordinator& hybrid_coord,
                          std::string* error);

bool RunHybridAutoStartupFromParams(HybridServiceGateway& service_gateway,
                                    HybridOperationalCoordinator& hybrid_coord,
                                    const ros::NodeHandle& pnh,
                                    std::string* error);

}  // namespace eyou_ros1_master
