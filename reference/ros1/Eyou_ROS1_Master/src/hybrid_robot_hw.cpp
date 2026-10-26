#include "Eyou_ROS1_Master/hybrid_robot_hw.hpp"

#include <set>
#include <string>
#include <vector>

namespace eyou_ros1_master {

HybridRobotHW::HybridRobotHW(CanDriverHW* can_hw,
                             canopen_hw::CanopenRobotHwRos* canopen_hw)
    : can_hw_(can_hw), canopen_hw_(canopen_hw) {}

bool HybridRobotHW::init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    CanDriverHW::InitOptions can_driver_options;
    can_driver_options.enable_ros_endpoints = false;
    ros::NodeHandle can_driver_pnh(pnh, "can_driver_node");
    if (!can_hw_->init(nh, can_driver_pnh, can_driver_options)) {
        ROS_ERROR("[HybridRobotHW] CanDriverHW::init() failed");
        return false;
    }

    // CanopenRobotHwRos 不需要 init()，它在构造时已完成 interface 注册。
    // 检验 joint 名全局唯一：收集两边所有 joint handle 名。
    std::set<std::string> seen;
    auto check = [&](const std::vector<std::string>& names) -> bool {
        for (const auto& n : names) {
            if (!seen.insert(n).second) {
                ROS_ERROR("[HybridRobotHW] duplicate joint name: %s", n.c_str());
                return false;
            }
        }
        return true;
    };

    {
        hardware_interface::JointStateInterface* iface =
            can_hw_->get<hardware_interface::JointStateInterface>();
        if (iface) {
            if (!check(iface->getNames())) return false;
        }
    }
    {
        hardware_interface::JointStateInterface* iface =
            canopen_hw_->get<hardware_interface::JointStateInterface>();
        if (iface) {
            if (!check(iface->getNames())) return false;
        }
    }

    // 将两边的所有 hardware_interface 合并到本 RobotHW。
    this->registerInterfaceManager(can_hw_);
    this->registerInterfaceManager(canopen_hw_);

    ROS_INFO("[HybridRobotHW] init OK — %zu unique joints", seen.size());
    return true;
}

void HybridRobotHW::read(const ros::Time& time, const ros::Duration& period) {
    can_hw_->read(time, period);
    canopen_hw_->read(time, period);
}

void HybridRobotHW::write(const ros::Time& time, const ros::Duration& period) {
    can_hw_->write(time, period);
    canopen_hw_->write(time, period);
}

}  // namespace eyou_ros1_master
