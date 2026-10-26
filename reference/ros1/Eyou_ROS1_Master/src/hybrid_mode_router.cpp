#include "Eyou_ROS1_Master/hybrid_mode_router.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <sstream>
#include <stdexcept>

namespace eyou_ros1_master {

namespace {

std::string NormalizeModeName(const std::string& mode_name) {
    std::string normalized = mode_name;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return normalized;
}

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool ParseRequiredInteger(const XmlRpc::XmlRpcValue& root,
                          const char* field_name,
                          const std::string& mode_name,
                          int* out,
                          std::string* error) {
    if (!root.hasMember(field_name)) {
        SetError(error, "mode '" + mode_name + "' missing backend mapping '" +
                            std::string(field_name) + "'");
        return false;
    }
    const auto& value = root[field_name];
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        SetError(error, "mode '" + mode_name + "' backend mapping '" +
                            std::string(field_name) + "' must be int");
        return false;
    }
    *out = static_cast<int>(value);
    return true;
}

}  // namespace

bool LoadHybridModeMappings(const ros::NodeHandle& pnh,
                            std::map<std::string, HybridModeMapping>* mappings,
                            std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (mappings == nullptr) {
        SetError(error, "mode mapping output is null");
        return false;
    }

    XmlRpc::XmlRpcValue root;
    if (!pnh.getParam("joint_mode_mappings/modes", root)) {
        SetError(error, "joint_mode_mappings/modes is missing");
        return false;
    }
    if (root.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        SetError(error, "joint_mode_mappings/modes is not a struct");
        return false;
    }

    mappings->clear();
    for (auto it = root.begin(); it != root.end(); ++it) {
        const std::string mode_name = NormalizeModeName(it->first);
        const auto& backend_map = it->second;
        if (backend_map.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            SetError(error, "mode '" + mode_name + "' mapping is not a struct");
            return false;
        }

        HybridModeMapping mapping;
        if (backend_map.hasMember("canopen")) {
            if (!ParseRequiredInteger(backend_map, "canopen", mode_name,
                                      &mapping.canopen_mode, error)) {
                return false;
            }
            mapping.has_canopen = true;
        }
        if (backend_map.hasMember("can_driver")) {
            if (!ParseRequiredInteger(backend_map, "can_driver", mode_name,
                                      &mapping.can_driver_mode, error)) {
                return false;
            }
            mapping.has_can_driver = true;
        }
        if (!mapping.has_canopen && !mapping.has_can_driver) {
            SetError(error, "mode '" + mode_name + "' has no backend mapping");
            return false;
        }

        (*mappings)[mode_name] = mapping;
    }

    if (mappings->empty()) {
        SetError(error, "joint_mode_mappings/modes is empty");
        return false;
    }
    return true;
}

bool BuildHybridModeJointTargets(
    const canopen_hw::CanopenMasterConfig& canopen_config,
    const XmlRpc::XmlRpcValue& can_driver_joint_list,
    std::map<std::string, HybridModeJointTarget>* joint_targets,
    std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (joint_targets == nullptr) {
        SetError(error, "joint target output is null");
        return false;
    }
    if (can_driver_joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        SetError(error, "can_driver joints config is not an array");
        return false;
    }

    joint_targets->clear();
    for (std::size_t index = 0; index < canopen_config.joints.size(); ++index) {
        const auto& joint = canopen_config.joints[index];
        if (!joint_targets
                 ->emplace(joint.name,
                           HybridModeJointTarget{
                               HybridModeJointTarget::Backend::kCanopen, index, 0})
                 .second) {
            SetError(error, "duplicate joint name across mode targets: " + joint.name);
            return false;
        }
    }

    for (int index = 0; index < can_driver_joint_list.size(); ++index) {
        const auto& joint = can_driver_joint_list[index];
        if (joint.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            SetError(error, "can_driver joints[" + std::to_string(index) +
                                "] is not a struct");
            return false;
        }
        if (!joint.hasMember("name") || !joint.hasMember("motor_id")) {
            SetError(error, "can_driver joints[" + std::to_string(index) +
                                "] missing name/motor_id");
            return false;
        }

        const std::string joint_name = static_cast<std::string>(joint["name"]);
        const auto& motor_id_value = joint["motor_id"];
        uint16_t motor_id = 0;
        if (motor_id_value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            motor_id = static_cast<uint16_t>(static_cast<int>(motor_id_value));
        } else if (motor_id_value.getType() == XmlRpc::XmlRpcValue::TypeString) {
            motor_id = static_cast<uint16_t>(
                std::stoul(static_cast<std::string>(motor_id_value), nullptr, 0));
        } else {
            SetError(error, "Joint '" + joint_name + "': invalid motor_id type.");
            return false;
        }

        if (!joint_targets
                 ->emplace(joint_name,
                           HybridModeJointTarget{
                               HybridModeJointTarget::Backend::kCanDriver, 0, motor_id})
                 .second) {
            SetError(error, "duplicate joint name across mode targets: " + joint_name);
            return false;
        }
    }

    return true;
}

HybridModeRouter::HybridModeRouter(ros::NodeHandle& pnh,
                                   const ros::NodeHandle& can_driver_pnh,
                                   const canopen_hw::CanopenMasterConfig& canopen_config,
                                   canopen_hw::CanopenAuxServices* canopen_aux,
                                   MotorMaintenanceService* can_driver_maintenance,
                                   bool advertise_service)
    : canopen_aux_(canopen_aux),
      can_driver_maintenance_(can_driver_maintenance) {
    std::string error;
    if (!LoadHybridModeMappings(pnh, &mappings_, &error)) {
        ROS_FATAL("[hybrid] failed to load joint mode mappings: %s", error.c_str());
        throw std::runtime_error(error);
    }

    XmlRpc::XmlRpcValue can_driver_joint_list;
    if (!can_driver_pnh.getParam("joints", can_driver_joint_list)) {
        throw std::runtime_error("can_driver joints config is missing");
    }
    if (!BuildHybridModeJointTargets(canopen_config, can_driver_joint_list,
                                     &joint_targets_, &error)) {
        ROS_FATAL("[hybrid] failed to build joint mode targets: %s", error.c_str());
        throw std::runtime_error(error);
    }

    if (advertise_service) {
        set_joint_mode_srv_ = pnh.advertiseService(
            "set_joint_mode", &HybridModeRouter::OnSetJointMode, this);
        set_joint_zero_srv_ = pnh.advertiseService(
            "set_joint_zero", &HybridModeRouter::OnSetJointZero, this);
        apply_joint_limits_srv_ = pnh.advertiseService(
            "apply_joint_limits", &HybridModeRouter::OnApplyJointLimits, this);
    }
}

bool HybridModeRouter::HandleSetJointMode(const std::string& joint_name,
                                          const std::string& mode_name,
                                          std::string* backend,
                                          int* mapped_mode,
                                          std::string* message) const {
    if (message != nullptr) {
        message->clear();
    }

    const auto joint_it = joint_targets_.find(joint_name);
    if (joint_it == joint_targets_.end()) {
        SetError(message, "unknown joint: " + joint_name);
        return false;
    }

    const auto normalized_mode = NormalizeModeName(mode_name);
    const auto mapping_it = mappings_.find(normalized_mode);
    if (mapping_it == mappings_.end()) {
        std::ostringstream oss;
        oss << "unknown mode: " << mode_name << "; available:";
        for (const auto& entry : mappings_) {
            oss << " " << entry.first;
        }
        SetError(message, oss.str());
        return false;
    }

    const auto& target = joint_it->second;
    const auto& mapping = mapping_it->second;
    if (target.backend == HybridModeJointTarget::Backend::kCanopen) {
        if (!mapping.has_canopen) {
            SetError(message, "mode '" + normalized_mode +
                                  "' is not mapped for canopen backend");
            return false;
        }
        std::string detail;
        if (!canopen_aux_ ||
            !canopen_aux_->SetModeAxis(target.axis_index,
                                       static_cast<int8_t>(mapping.canopen_mode),
                                       &detail)) {
            SetError(message, detail.empty() ? "canopen set mode failed" : detail);
            return false;
        }
        if (backend != nullptr) {
            *backend = "canopen";
        }
        if (mapped_mode != nullptr) {
            *mapped_mode = mapping.canopen_mode;
        }
        SetError(message, detail.empty() ? "mode set" : detail);
        return true;
    }

    if (!mapping.has_can_driver) {
        SetError(message, "mode '" + normalized_mode +
                              "' is not mapped for can_driver backend");
        return false;
    }
    std::string detail;
    if (!can_driver_maintenance_ ||
        !can_driver_maintenance_->SetModeByMotorId(target.motor_id,
                                                   mapping.can_driver_mode,
                                                   &detail)) {
        SetError(message, detail.empty() ? "can_driver set mode failed" : detail);
        return false;
    }
    if (backend != nullptr) {
        *backend = "can_driver";
    }
    if (mapped_mode != nullptr) {
        *mapped_mode = mapping.can_driver_mode;
    }
    SetError(message, detail.empty() ? "mode set" : detail);
    return true;
}

bool HybridModeRouter::OnSetJointMode(Eyou_ROS1_Master::SetJointMode::Request& req,
                                      Eyou_ROS1_Master::SetJointMode::Response& res) {
    res.success = HandleSetJointMode(req.joint_name, req.mode, &res.backend,
                                     &res.mapped_mode, &res.message);
    return true;
}

bool HybridModeRouter::HandleSetJointZero(const std::string& joint_name,
                                          double zero_offset_rad,
                                          bool use_current_position_as_zero,
                                          std::string* backend,
                                          double* current_position_rad,
                                          double* applied_zero_offset_rad,
                                          std::string* message) const {
    if (message != nullptr) {
        message->clear();
    }
    if (current_position_rad != nullptr) {
        *current_position_rad = 0.0;
    }
    if (applied_zero_offset_rad != nullptr) {
        *applied_zero_offset_rad = 0.0;
    }

    const auto joint_it = joint_targets_.find(joint_name);
    if (joint_it == joint_targets_.end()) {
        SetError(message, "unknown joint: " + joint_name);
        return false;
    }

    const auto& target = joint_it->second;
    if (target.backend == HybridModeJointTarget::Backend::kCanopen) {
        std::string detail;
        if (!canopen_aux_ ||
            !canopen_aux_->SetZeroAxis(target.axis_index,
                                       zero_offset_rad,
                                       use_current_position_as_zero,
                                       current_position_rad,
                                       applied_zero_offset_rad,
                                       &detail)) {
            SetError(message, detail.empty() ? "canopen set zero failed" : detail);
            return false;
        }
        if (backend != nullptr) {
            *backend = "canopen";
        }
        SetError(message, detail.empty() ? "zero set" : detail);
        return true;
    }

    std::string detail;
    if (!can_driver_maintenance_ ||
        !can_driver_maintenance_->SetZeroByMotorId(target.motor_id,
                                                   zero_offset_rad,
                                                   use_current_position_as_zero,
                                                   false,
                                                   nullptr,
                                                   current_position_rad,
                                                   applied_zero_offset_rad,
                                                   &detail)) {
        SetError(message, detail.empty() ? "can_driver set zero failed" : detail);
        return false;
    }
    if (backend != nullptr) {
        *backend = "can_driver";
    }
    SetError(message, detail.empty() ? "zero set" : detail);
    return true;
}

bool HybridModeRouter::HandleApplyJointLimits(const std::string& joint_name,
                                              double min_position_rad,
                                              double max_position_rad,
                                              bool use_urdf_limits,
                                              bool require_current_inside_limits,
                                              std::string* backend,
                                              double* current_position_rad,
                                              double* applied_min_rad,
                                              double* applied_max_rad,
                                              std::string* message) const {
    if (message != nullptr) {
        message->clear();
    }
    if (current_position_rad != nullptr) {
        *current_position_rad = 0.0;
    }
    if (applied_min_rad != nullptr) {
        *applied_min_rad = 0.0;
    }
    if (applied_max_rad != nullptr) {
        *applied_max_rad = 0.0;
    }

    const auto joint_it = joint_targets_.find(joint_name);
    if (joint_it == joint_targets_.end()) {
        SetError(message, "unknown joint: " + joint_name);
        return false;
    }

    const auto& target = joint_it->second;
    if (target.backend == HybridModeJointTarget::Backend::kCanopen) {
        std::string detail;
        if (!canopen_aux_ ||
            !canopen_aux_->ApplyLimitsAxis(target.axis_index,
                                           use_urdf_limits,
                                           min_position_rad,
                                           max_position_rad,
                                           require_current_inside_limits,
                                           current_position_rad,
                                           applied_min_rad,
                                           applied_max_rad,
                                           &detail)) {
            SetError(message, detail.empty() ? "canopen apply limits failed" : detail);
            return false;
        }
        if (backend != nullptr) {
            *backend = "canopen";
        }
        SetError(message, detail.empty() ? "limits applied" : detail);
        return true;
    }

    std::string detail;
    if (!can_driver_maintenance_ ||
        !can_driver_maintenance_->ApplyLimitsByMotorId(target.motor_id,
                                                       min_position_rad,
                                                       max_position_rad,
                                                       use_urdf_limits,
                                                       true,
                                                       require_current_inside_limits,
                                                       current_position_rad,
                                                       nullptr,
                                                       applied_min_rad,
                                                       applied_max_rad,
                                                       &detail)) {
        SetError(message, detail.empty() ? "can_driver apply limits failed" : detail);
        return false;
    }
    if (backend != nullptr) {
        *backend = "can_driver";
    }
    SetError(message, detail.empty() ? "limits applied" : detail);
    return true;
}

bool HybridModeRouter::OnSetJointZero(Eyou_ROS1_Master::SetJointZero::Request& req,
                                      Eyou_ROS1_Master::SetJointZero::Response& res) {
    res.success = HandleSetJointZero(req.joint_name,
                                     req.zero_offset_rad,
                                     req.use_current_position_as_zero,
                                     &res.backend,
                                     &res.current_position_rad,
                                     &res.applied_zero_offset_rad,
                                     &res.message);
    return true;
}

bool HybridModeRouter::OnApplyJointLimits(
    Eyou_ROS1_Master::ApplyJointLimits::Request& req,
    Eyou_ROS1_Master::ApplyJointLimits::Response& res) {
    res.success = HandleApplyJointLimits(req.joint_name,
                                         req.min_position_rad,
                                         req.max_position_rad,
                                         req.use_urdf_limits,
                                         req.require_current_inside_limits,
                                         &res.backend,
                                         &res.current_position_rad,
                                         &res.applied_min_rad,
                                         &res.applied_max_rad,
                                         &res.message);
    return true;
}

}  // namespace eyou_ros1_master
