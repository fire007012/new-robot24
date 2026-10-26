#pragma once

#include <string>

#include <ros/ros.h>

#include "canopen_hw/canopen_aux_services.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/operational_coordinator.hpp"

namespace canopen_hw {

// 封装 auto_init → apply_soft_limits → auto_enable → auto_release 启动序列。
// 从 rosparam 读取 auto_init / auto_enable / auto_release 标志，
// 顺序执行状态转换，任一步骤失败则返回 false。
struct CanopenStartupSequence {
  static bool Run(OperationalCoordinator& coordinator,
                  CanopenAuxServices& aux,
                  const ros::NodeHandle& pnh) {
    bool auto_init = false;
    bool auto_enable = false;
    bool auto_release = false;
    pnh.param("auto_init", auto_init, false);
    pnh.param("auto_enable", auto_enable, false);
    pnh.param("auto_release", auto_release, false);

    if ((auto_enable || auto_release) && !auto_init) {
      CANOPEN_LOG_ERROR(
          "auto_enable/auto_release require auto_init=true for strict "
          "transition path");
      return false;
    }
    if (auto_release && !auto_enable) {
      CANOPEN_LOG_ERROR("auto_release requires auto_enable=true");
      return false;
    }

    if (!auto_init) {
      return true;  // 无自动启动，由用户手动调服务。
    }

    // Init
    const auto init_result = coordinator.RequestInit();
    if (!init_result.ok) {
      CANOPEN_LOG_ERROR("auto_init enabled but init failed: {}",
                        init_result.message);
      return false;
    }

    // Soft limits
    std::string soft_limit_detail;
    if (!aux.ApplySoftLimitAll(&soft_limit_detail)) {
      const auto shutdown_result = coordinator.RequestShutdown();
      CANOPEN_LOG_ERROR("auto_init post-init soft limit apply failed: {}",
                        soft_limit_detail);
      if (!shutdown_result.ok) {
        CANOPEN_LOG_ERROR(
            "rollback shutdown failed after post-init failure: {}",
            shutdown_result.message);
      }
      return false;
    }

    // Enable
    if (auto_enable) {
      const auto enable_result = coordinator.RequestEnable();
      if (!enable_result.ok) {
        CANOPEN_LOG_ERROR("auto_enable enabled but enable failed: {}",
                          enable_result.message);
        return false;
      }
    }

    // Release
    if (auto_release) {
      const auto release_result = coordinator.RequestRelease();
      if (!release_result.ok) {
        CANOPEN_LOG_ERROR("auto_release enabled but release failed: {}",
                          release_result.message);
        return false;
      }
    }

    return true;
  }
};

}  // namespace canopen_hw
