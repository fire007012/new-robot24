#pragma once

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>

#include "can_driver/CanDriverHW.h"
#include "canopen_hw/canopen_robot_hw_ros.hpp"

namespace eyou_ros1_master {

// 组合 RobotHW：将 CanDriverHW 和 CanopenRobotHwRos 聚合在一个
// controller_manager 下。两个子 HW 的所有权由 main 持有，
// 本类仅持有非拥有指针。
//
// init() 串行调用两边，合并其 hardware_interface 注册，
// 并校验所有 joint 名全局唯一（重复则返回 false）。
// read() / write() 串行调用两边。
class HybridRobotHW : public hardware_interface::RobotHW {
public:
    // can_hw 和 canopen_hw 的生命周期由调用方（main）管理，
    // 本类存活期间两个指针必须有效。
    HybridRobotHW(CanDriverHW* can_hw, canopen_hw::CanopenRobotHwRos* canopen_hw);

    // 串行调用两边 init()，合并 interface 注册，校验 joint 唯一性。
    // 返回 false 时节点应整体退出。
    bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void read(const ros::Time& time, const ros::Duration& period) override;
    void write(const ros::Time& time, const ros::Duration& period) override;

private:
    CanDriverHW* can_hw_;
    canopen_hw::CanopenRobotHwRos* canopen_hw_;
};

}  // namespace eyou_ros1_master
