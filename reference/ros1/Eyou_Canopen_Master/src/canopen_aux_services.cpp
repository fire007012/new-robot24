#include "canopen_hw/canopen_aux_services.hpp"

#include <sstream>

#include "Eyou_Canopen_Master/ApplyLimits.h"
#include "Eyou_Canopen_Master/SetMode.h"
#include "Eyou_Canopen_Master/SetZero.h"
#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {

namespace {

constexpr auto kSoftLimitSdoIdleTimeout = std::chrono::milliseconds(2500);
constexpr auto kSetZeroSdoIdleTimeout = std::chrono::milliseconds(2500);
constexpr auto kStartupCompleteTimeout = std::chrono::milliseconds(4000);

bool IsAllowedMode(int8_t mode) {
  return mode == kMode_IP || mode == kMode_CSP ||
         mode == kMode_CSV || mode == kMode_CST;
}

void SetDetail(std::string* detail, const std::string& message) {
  if (detail != nullptr) {
    *detail = message;
  }
}

}  // namespace

bool CanopenAuxServices::SetModeAxis(std::size_t axis_index,
                                     int8_t mode,
                                     std::string* detail) {
  if (detail != nullptr) {
    detail->clear();
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto system_mode = coordinator_->mode();
  if (system_mode != SystemOpMode::Standby) {
    if (detail != nullptr) {
      *detail = "set_mode only allowed in Standby; call ~/disable first";
    }
    return false;
  }
  if (axis_index >= robot_hw_ros_->axis_count()) {
    if (detail != nullptr) {
      *detail = "axis_index out of range";
    }
    return false;
  }
  if (!IsAllowedMode(mode)) {
    if (detail != nullptr) {
      *detail =
          "unsupported mode: " + std::to_string(mode) +
          ", allowed: 7(IP),8(CSP),9(CSV),10(CST)";
    }
    return false;
  }

  robot_hw_ros_->SetMode(axis_index, mode);
  if (detail != nullptr) {
    *detail = "mode set";
  }
  return true;
}

bool CanopenAuxServices::SetZeroAxis(std::size_t axis_index,
                                     double zero_offset,
                                     bool use_current_position_as_zero,
                                     double* current_position,
                                     double* applied_zero_offset,
                                     std::string* detail) {
  if (detail != nullptr) {
    detail->clear();
  }
  if (current_position != nullptr) {
    *current_position = 0.0;
  }
  if (applied_zero_offset != nullptr) {
    *applied_zero_offset = 0.0;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto mode = coordinator_->mode();
  if (mode != SystemOpMode::Standby) {
    SetDetail(detail, "set_zero only allowed in Standby; call ~/disable first");
    return false;
  }
  if (axis_index >= master_cfg_->joints.size()) {
    SetDetail(detail, "axis_index out of range");
    return false;
  }

  const double current = robot_hw_ros_->joint_position(axis_index);
  if (current_position != nullptr) {
    *current_position = current;
  }

  std::string local_detail;
  if (!WaitForAxisSdoIdle(axis_index, kSetZeroSdoIdleTimeout, &local_detail)) {
    SetDetail(detail,
              local_detail.empty() ? "axis SDO idle wait failed" : local_detail);
    return false;
  }

  if (use_current_position_as_zero) {
    if (!zero_executor_.SetCurrentPositionAsZero(axis_index, &local_detail)) {
      SetDetail(detail, local_detail.empty() ? "set_zero failed" : local_detail);
      return false;
    }
    if (applied_zero_offset != nullptr) {
      *applied_zero_offset = -current;
    }
  } else {
    if (!EnsureUrdfLimits(&local_detail)) {
      SetDetail(detail, local_detail);
      return false;
    }
    if (axis_index >= urdf_limits_.size()) {
      SetDetail(detail, "axis_index out of URDF limit range");
      return false;
    }

    const auto& limit = urdf_limits_[axis_index];
    bool ok = false;
    if (limit.unit == UrdfJointLimitUnit::kRadians) {
      ok = zero_executor_.SetHomeOffsetRadians(axis_index, zero_offset, &local_detail);
    } else if (limit.unit == UrdfJointLimitUnit::kMeters) {
      ok = zero_executor_.SetHomeOffsetMeters(axis_index, zero_offset, &local_detail);
    } else {
      local_detail = "unsupported URDF limit unit";
    }
    if (!ok) {
      SetDetail(detail, local_detail.empty() ? "set_zero failed" : local_detail);
      return false;
    }
    if (applied_zero_offset != nullptr) {
      *applied_zero_offset = zero_offset;
    }
  }

  SetDetail(detail, local_detail.empty() ? "zero set" : local_detail);
  return true;
}

bool CanopenAuxServices::ApplyLimitsAxis(std::size_t axis_index,
                                         bool use_urdf_limits,
                                         double min_position,
                                         double max_position,
                                         bool require_current_inside_limits,
                                         double* current_position,
                                         double* applied_min_position,
                                         double* applied_max_position,
                                         std::string* detail) {
  if (detail != nullptr) {
    detail->clear();
  }
  if (current_position != nullptr) {
    *current_position = 0.0;
  }
  if (applied_min_position != nullptr) {
    *applied_min_position = 0.0;
  }
  if (applied_max_position != nullptr) {
    *applied_max_position = 0.0;
  }

  std::lock_guard<std::mutex> lk(*loop_mtx_);
  const auto mode = coordinator_->mode();
  if (mode != SystemOpMode::Standby) {
    SetDetail(detail, "apply_limits only allowed in Standby; call ~/disable first");
    return false;
  }
  if (axis_index >= master_cfg_->joints.size()) {
    SetDetail(detail, "axis_index out of range");
    return false;
  }
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (!WaitForAxisSdoIdle(axis_index, kSoftLimitSdoIdleTimeout, detail)) {
    return false;
  }

  PreparedSoftLimit prepared;
  double resolved_min = min_position;
  double resolved_max = max_position;
  if (use_urdf_limits) {
    if (!PrepareSoftLimitAxis(axis_index, &prepared, detail)) {
      return false;
    }
    const auto& limit = urdf_limits_[axis_index];
    if (limit.has_position_limits) {
      resolved_min = limit.lower;
      resolved_max = limit.upper;
    }
  } else {
    if (!ApplySoftLimitAxisManual(axis_index, min_position, max_position,
                                  &prepared, detail)) {
      return false;
    }
  }

  const double current = robot_hw_ros_->joint_position(axis_index);
  if (current_position != nullptr) {
    *current_position = current;
  }
  if (applied_min_position != nullptr) {
    *applied_min_position = resolved_min;
  }
  if (applied_max_position != nullptr) {
    *applied_max_position = resolved_max;
  }
  if (!prepared.disable && require_current_inside_limits &&
      (current < resolved_min || current > resolved_max)) {
    SetDetail(detail, "current position is outside requested limit range");
    return false;
  }

  if (prepared.disable) {
    if (!zero_executor_.DisableSoftLimit(axis_index, detail)) {
      return false;
    }
  } else {
    if (!zero_executor_.ApplySoftLimitCounts(axis_index,
                                             prepared.min_counts,
                                             prepared.max_counts,
                                             detail)) {
      return false;
    }
  }

  if (detail != nullptr && detail->empty()) {
    *detail = prepared.disable ? "soft limits disabled" : "soft limits applied";
  }
  return true;
}

CanopenAuxServices::CanopenAuxServices(
    ros::NodeHandle* pnh,
    CanopenRobotHwRos* robot_hw_ros,
    OperationalCoordinator* coordinator,
    const CanopenMasterConfig* master_cfg,
    CanopenMaster* master,
    std::mutex* loop_mtx)
    : robot_hw_ros_(robot_hw_ros),
      coordinator_(coordinator),
      master_cfg_(master_cfg),
      master_(master),
      loop_mtx_(loop_mtx),
      zero_executor_(master, master_cfg) {
  // ── ~/set_mode ──
  set_mode_srv_ = pnh->advertiseService<Eyou_Canopen_Master::SetMode::Request,
                                        Eyou_Canopen_Master::SetMode::Response>(
      "set_mode", [this](Eyou_Canopen_Master::SetMode::Request& req,
                         Eyou_Canopen_Master::SetMode::Response& res) {
        std::string detail;
        res.success = SetModeAxis(req.axis_index, req.mode, &detail);
        res.message = detail;
        return true;
      });

  // ── ~/set_zero ──
  set_zero_srv_ = pnh->advertiseService<Eyou_Canopen_Master::SetZero::Request,
                                        Eyou_Canopen_Master::SetZero::Response>(
      "set_zero", [this](Eyou_Canopen_Master::SetZero::Request& req,
                         Eyou_Canopen_Master::SetZero::Response& res) {
        std::string detail;
        res.success = SetZeroAxis(req.axis_index,
                                  req.zero_offset_rad,
                                  req.use_current_position_as_zero,
                                  &res.current_position,
                                  &res.applied_zero_offset,
                                  &detail);
        res.message = detail;
        return true;
      });

  // ── ~/apply_limits ──
  apply_limits_srv_ =
      pnh->advertiseService<Eyou_Canopen_Master::ApplyLimits::Request,
                            Eyou_Canopen_Master::ApplyLimits::Response>(
          "apply_limits",
          [this](Eyou_Canopen_Master::ApplyLimits::Request& req,
                 Eyou_Canopen_Master::ApplyLimits::Response& res) {
            std::string detail;
            res.success = ApplyLimitsAxis(req.axis_index,
                                          req.use_urdf_limits,
                                          req.min_position,
                                          req.max_position,
                                          req.require_current_inside_limits,
                                          &res.current_position,
                                          &res.applied_min_position,
                                          &res.applied_max_position,
                                          &detail);
            res.message = detail;
            return true;
          });
}

// ── URDF 软限位 ──────────────────────────────────────

bool CanopenAuxServices::EnsureUrdfLimits(std::string* detail) {
  if (urdf_limits_ready_) {
    return true;
  }
  std::string urdf_xml;
  ros::NodeHandle nh;
  if (!nh.getParam("robot_description", urdf_xml) || urdf_xml.empty()) {
    if (detail) {
      *detail = "robot_description is missing; cannot derive URDF soft limits";
    }
    return false;
  }
  std::string parse_error;
  if (!ParseUrdfJointLimits(urdf_xml, master_cfg_->joints, &urdf_limits_,
                            &parse_error)) {
    if (detail) {
      *detail = parse_error.empty() ? "failed to parse URDF joint limits"
                                    : parse_error;
    }
    return false;
  }
  urdf_limits_ready_ = true;
  return true;
}

bool CanopenAuxServices::ApplySoftLimitAxis(std::size_t axis_index,
                                            std::string* detail) {
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (axis_index >= urdf_limits_.size()) {
    if (detail) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " out of URDF limit range";
      *detail = oss.str();
    }
    return false;
  }
  const auto& limit = urdf_limits_[axis_index];
  if (!limit.has_position_limits) {
    return zero_executor_.DisableSoftLimit(axis_index, detail);
  }
  if (limit.unit == UrdfJointLimitUnit::kRadians) {
    return zero_executor_.ApplySoftLimitRadians(
        axis_index, limit.lower, limit.upper, detail);
  }
  if (limit.unit == UrdfJointLimitUnit::kMeters) {
    return zero_executor_.ApplySoftLimitMeters(
        axis_index, limit.lower, limit.upper, detail);
  }
  if (detail) {
    *detail = "unsupported URDF limit unit";
  }
  return false;
}

bool CanopenAuxServices::ApplySoftLimitAxisManual(std::size_t axis_index,
                                                  double min_position,
                                                  double max_position,
                                                  PreparedSoftLimit* out,
                                                  std::string* detail) {
  if (!out) {
    if (detail) {
      *detail = "prepared soft limit output is null";
    }
    return false;
  }
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (axis_index >= urdf_limits_.size() ||
      axis_index >= master_cfg_->joints.size()) {
    if (detail) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " out of URDF limit range";
      *detail = oss.str();
    }
    return false;
  }

  const auto& limit = urdf_limits_[axis_index];
  if (limit.unit == UrdfJointLimitUnit::kRadians) {
    return zero_executor_.PrepareSoftLimitRadians(
        axis_index, min_position, max_position,
        &out->min_counts, &out->max_counts, detail);
  }
  if (limit.unit == UrdfJointLimitUnit::kMeters) {
    return zero_executor_.PrepareSoftLimitMeters(
        axis_index, min_position, max_position,
        &out->min_counts, &out->max_counts, detail);
  }
  if (detail) {
    *detail = "unsupported URDF limit unit";
  }
  return false;
}

bool CanopenAuxServices::PrepareSoftLimitAxis(std::size_t axis_index,
                                               PreparedSoftLimit* out,
                                               std::string* detail) {
  if (!out) {
    if (detail) {
      *detail = "prepared soft limit output is null";
    }
    return false;
  }
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (axis_index >= urdf_limits_.size() ||
      axis_index >= master_cfg_->joints.size()) {
    if (detail) {
      std::ostringstream oss;
      oss << "axis " << axis_index << " out of URDF limit range";
      *detail = oss.str();
    }
    return false;
  }

  const auto& limit = urdf_limits_[axis_index];
  out->disable = false;
  if (!limit.has_position_limits) {
    out->disable = true;
    out->min_counts = 0;
    out->max_counts = 0;
    if (detail) {
      *detail = "URDF has no position limits; soft limits will be disabled";
    }
    return true;
  }
  if (limit.unit == UrdfJointLimitUnit::kRadians) {
    return zero_executor_.PrepareSoftLimitRadians(
        axis_index, limit.lower, limit.upper,
        &out->min_counts, &out->max_counts, detail);
  }
  if (limit.unit == UrdfJointLimitUnit::kMeters) {
    return zero_executor_.PrepareSoftLimitMeters(
        axis_index, limit.lower, limit.upper,
        &out->min_counts, &out->max_counts, detail);
  }
  if (detail) {
    *detail = "unsupported URDF limit unit";
  }
  return false;
}

bool CanopenAuxServices::WaitForAllSdoIdle(std::chrono::milliseconds timeout,
                                           std::string* detail) const {
  if (!master_) {
    if (detail) {
      *detail = "master is null; cannot wait for SDO idle";
    }
    return false;
  }

  std::vector<std::size_t> pending_axes;
  if (master_->WaitForAllSdoIdle(timeout, &pending_axes)) {
    return true;
  }

  if (detail) {
    std::ostringstream oss;
    oss << "timed out waiting for all-axis SDO idle";
    if (!pending_axes.empty()) {
      oss << "; pending axes: ";
      for (std::size_t i = 0; i < pending_axes.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << pending_axes[i];
        if (master_cfg_ && pending_axes[i] < master_cfg_->joints.size()) {
          oss << "(node="
              << static_cast<int>(master_cfg_->joints[pending_axes[i]].node_id)
              << ")";
        }
      }
    }
    *detail = oss.str();
  }
  return false;
}

bool CanopenAuxServices::WaitForAllStartupComplete(
    std::chrono::milliseconds timeout, std::string* detail) const {
  if (!master_) {
    if (detail) {
      *detail = "master is null; cannot wait for startup complete";
    }
    return false;
  }

  std::vector<std::size_t> pending_axes;
  if (master_->WaitForAllStartupComplete(timeout, &pending_axes)) {
    return true;
  }

  if (detail) {
    std::ostringstream oss;
    oss << "timed out waiting for all-axis startup completion";
    if (!pending_axes.empty()) {
      oss << "; pending axes: ";
      for (std::size_t i = 0; i < pending_axes.size(); ++i) {
        if (i > 0) {
          oss << ", ";
        }
        oss << pending_axes[i];
        if (master_cfg_ && pending_axes[i] < master_cfg_->joints.size()) {
          oss << "(node="
              << static_cast<int>(master_cfg_->joints[pending_axes[i]].node_id)
              << ")";
        }
      }
    }
    *detail = oss.str();
  }
  return false;
}

bool CanopenAuxServices::WaitForAxisSdoIdle(std::size_t axis_index,
                                            std::chrono::milliseconds timeout,
                                            std::string* detail) const {
  if (!master_) {
    if (detail) {
      *detail = "master is null; cannot wait for SDO idle";
    }
    return false;
  }
  if (axis_index >= master_cfg_->joints.size()) {
    if (detail) {
      *detail = "axis_index out of range for SDO idle wait";
    }
    return false;
  }
  if (master_->WaitForSdoIdle(axis_index, timeout)) {
    return true;
  }

  if (detail) {
    std::ostringstream oss;
    oss << "timed out waiting for axis " << axis_index << " SDO idle"
        << " (node=" << static_cast<int>(master_cfg_->joints[axis_index].node_id)
        << ")";
    *detail = oss.str();
  }
  return false;
}

bool CanopenAuxServices::ApplySoftLimitAll(std::string* detail) {
  if (!master_cfg_->auto_write_soft_limits_from_urdf) {
    return true;
  }
  if (!EnsureUrdfLimits(detail)) {
    return false;
  }
  if (!WaitForAllStartupComplete(kStartupCompleteTimeout, detail)) {
    return false;
  }
  if (!WaitForAllSdoIdle(kSoftLimitSdoIdleTimeout, detail)) {
    return false;
  }
  for (std::size_t i = 0; i < master_cfg_->joints.size(); ++i) {
    std::string axis_detail;
    if (!ApplySoftLimitAxis(i, &axis_detail)) {
      if (detail) {
        std::ostringstream oss;
        oss << "apply soft limit failed at axis " << i;
        if (!axis_detail.empty()) {
          oss << ": " << axis_detail;
        }
        *detail = oss.str();
      }
      return false;
    }
  }
  if (detail) {
    *detail = "soft limits synchronized from URDF";
  }
  return true;
}

}  // namespace canopen_hw
