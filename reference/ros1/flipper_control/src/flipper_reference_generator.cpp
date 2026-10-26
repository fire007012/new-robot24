#include "flipper_control/flipper_reference_generator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace flipper_control {

namespace {

constexpr double kConflictTolerance = 1e-9;

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

std::string Normalize(const std::string& value) {
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return normalized;
}

}  // namespace

std::string ToString(LinkageMode mode) {
  switch (mode) {
    case LinkageMode::kIndependent:
      return "independent";
    case LinkageMode::kLeftRightMirror:
      return "left_right_mirror";
    case LinkageMode::kFrontRearSync:
      return "front_rear_sync";
    case LinkageMode::kSidePair:
      return "side_pair";
    case LinkageMode::kDiagonalPair:
      return "diagonal_pair";
    default:
      return "unknown";
  }
}

bool ParseLinkageMode(const std::string& value, LinkageMode* mode) {
  if (mode == nullptr) {
    return false;
  }

  const std::string normalized = Normalize(value);
  if (normalized == "independent") {
    *mode = LinkageMode::kIndependent;
    return true;
  }
  if (normalized == "left_right_mirror") {
    *mode = LinkageMode::kLeftRightMirror;
    return true;
  }
  if (normalized == "front_rear_sync") {
    *mode = LinkageMode::kFrontRearSync;
    return true;
  }
  if (normalized == "side_pair") {
    *mode = LinkageMode::kSidePair;
    return true;
  }
  if (normalized == "diagonal_pair") {
    *mode = LinkageMode::kDiagonalPair;
    return true;
  }
  return false;
}

FlipperReferenceGenerator::FlipperReferenceGenerator(
    const std::vector<std::string>& joint_names)
    : joint_names_(joint_names),
      measured_positions_(joint_names.size(), 0.0),
      reference_positions_(joint_names.size(), 0.0),
      target_velocities_(joint_names.size(), 0.0),
      filtered_velocities_(joint_names.size(), 0.0),
      joint_limits_(joint_names.size()) {
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    joint_index_[joint_names_[i]] = i;
  }
}

void FlipperReferenceGenerator::SetJointLimits(
    const std::vector<JointLimit>& limits) {
  if (limits.size() == joint_limits_.size()) {
    joint_limits_ = limits;
  }
}

void FlipperReferenceGenerator::SetCommandTimeout(double timeout_sec) {
  command_timeout_sec_ = std::max(0.0, timeout_sec);
}

void FlipperReferenceGenerator::SetDtClamp(double dt_clamp_sec) {
  dt_clamp_sec_ = std::max(1e-4, dt_clamp_sec);
}

void FlipperReferenceGenerator::SetLowpassAlpha(double alpha) {
  lowpass_alpha_ = ClampValue(alpha, 0.0, 1.0);
}

void FlipperReferenceGenerator::SetDriftThreshold(double threshold_rad) {
  drift_threshold_rad_ = std::max(0.0, threshold_rad);
}

void FlipperReferenceGenerator::SetLinkageMode(LinkageMode mode) {
  linkage_mode_ = mode;
}

bool FlipperReferenceGenerator::SetMeasuredPositions(
    const std::vector<double>& positions, std::string* error) {
  if (positions.size() != measured_positions_.size()) {
    if (error != nullptr) {
      std::ostringstream oss;
      oss << "expected " << measured_positions_.size()
          << " measured positions, got " << positions.size();
      *error = oss.str();
    }
    return false;
  }

  measured_positions_ = positions;
  has_measurement_ = true;
  if (!reference_initialized_) {
    reference_positions_ = measured_positions_;
    reference_initialized_ = true;
  }
  return true;
}

bool FlipperReferenceGenerator::ExpandValues(
    const std::vector<std::string>& source_names,
    const std::vector<double>& source_values,
    const std::vector<double>& seed_values,
    std::vector<double>* out,
    std::string* error) const {
  if (out == nullptr) {
    if (error != nullptr) {
      *error = "output vector is null";
    }
    return false;
  }
  if (source_names.size() != source_values.size()) {
    if (error != nullptr) {
      *error = "source_names and source_values size mismatch";
    }
    return false;
  }
  if (seed_values.size() != joint_names_.size()) {
    if (error != nullptr) {
      *error = "seed_values size mismatch";
    }
    return false;
  }

  *out = seed_values;
  std::vector<bool> assigned(joint_names_.size(), false);
  for (std::size_t i = 0; i < source_names.size(); ++i) {
    const auto joint_it = joint_index_.find(source_names[i]);
    if (joint_it == joint_index_.end()) {
      if (error != nullptr) {
        *error = "unknown joint in command: " + source_names[i];
      }
      return false;
    }

    const std::vector<std::size_t> affected_indices =
        ResolveAffectedIndices(joint_it->second);
    for (const std::size_t index : affected_indices) {
      if (assigned[index] &&
          std::abs((*out)[index] - source_values[i]) > kConflictTolerance) {
        if (error != nullptr) {
          *error = "conflicting linkage command for joint: " + joint_names_[index];
        }
        return false;
      }
      (*out)[index] = source_values[i];
      assigned[index] = true;
    }
  }
  return true;
}

bool FlipperReferenceGenerator::UpdateVelocityCommand(
    const std::vector<std::string>& source_names,
    const std::vector<double>& source_values,
    const ros::Time& stamp,
    std::string* error) {
  std::vector<double> expanded(target_velocities_.size(), 0.0);
  if (!ExpandValues(source_names, source_values,
                    std::vector<double>(target_velocities_.size(), 0.0),
                    &expanded, error)) {
    return false;
  }
  ApplyVelocityLimits(&expanded);
  target_velocities_ = std::move(expanded);
  last_velocity_command_stamp_ = stamp;
  has_velocity_command_ = true;
  command_timed_out_ = false;
  return true;
}

void FlipperReferenceGenerator::ResetReferenceToMeasured() {
  if (has_measurement_) {
    reference_positions_ = measured_positions_;
    reference_initialized_ = true;
  }
  std::fill(filtered_velocities_.begin(), filtered_velocities_.end(), 0.0);
}

void FlipperReferenceGenerator::ClearVelocityCommand() {
  std::fill(target_velocities_.begin(), target_velocities_.end(), 0.0);
  std::fill(filtered_velocities_.begin(), filtered_velocities_.end(), 0.0);
  has_velocity_command_ = false;
  command_timed_out_ = true;
}

bool FlipperReferenceGenerator::Step(const ros::Time& now, double dt_sec) {
  if (!reference_initialized_) {
    return false;
  }

  const double clamped_dt = ClampValue(dt_sec, 0.0, dt_clamp_sec_);
  std::vector<double> limited_target = target_velocities_;
  if (!has_velocity_command_) {
    std::fill(limited_target.begin(), limited_target.end(), 0.0);
    std::fill(filtered_velocities_.begin(), filtered_velocities_.end(), 0.0);
    command_timed_out_ = true;
  } else if (command_timeout_sec_ > 0.0 &&
             (now - last_velocity_command_stamp_).toSec() > command_timeout_sec_) {
    std::fill(limited_target.begin(), limited_target.end(), 0.0);
    std::fill(filtered_velocities_.begin(), filtered_velocities_.end(), 0.0);
    command_timed_out_ = true;
  } else {
    command_timed_out_ = false;
  }

  ApplyVelocityLimits(&limited_target);
  for (std::size_t i = 0; i < filtered_velocities_.size(); ++i) {
    filtered_velocities_[i] +=
        (limited_target[i] - filtered_velocities_[i]) * lowpass_alpha_;
    reference_positions_[i] += filtered_velocities_[i] * clamped_dt;
  }
  ApplyPositionLimits(&reference_positions_);

  degraded_ = false;
  if (has_measurement_ && std::isfinite(drift_threshold_rad_)) {
    for (std::size_t i = 0; i < reference_positions_.size(); ++i) {
      if (std::abs(reference_positions_[i] - measured_positions_[i]) >
          drift_threshold_rad_) {
        degraded_ = true;
        break;
      }
    }
  }
  return true;
}

std::vector<std::size_t> FlipperReferenceGenerator::ResolveAffectedIndices(
    std::size_t source_index) const {
  const std::size_t lf = RequireIndex("left_front_arm_joint");
  const std::size_t rf = RequireIndex("right_front_arm_joint");
  const std::size_t lr = RequireIndex("left_rear_arm_joint");
  const std::size_t rr = RequireIndex("right_rear_arm_joint");

  switch (linkage_mode_) {
    case LinkageMode::kIndependent:
      return {source_index};
    case LinkageMode::kLeftRightMirror:
      if (source_index == lf || source_index == rf) {
        return {lf, rf};
      }
      return {lr, rr};
    case LinkageMode::kFrontRearSync:
      return {lf, rf, lr, rr};
    case LinkageMode::kSidePair:
      if (source_index == lf || source_index == lr) {
        return {lf, lr};
      }
      return {rf, rr};
    case LinkageMode::kDiagonalPair:
      if (source_index == lf || source_index == rr) {
        return {lf, rr};
      }
      return {rf, lr};
    default:
      return {source_index};
  }
}

void FlipperReferenceGenerator::ApplyVelocityLimits(
    std::vector<double>* values) const {
  if (values == nullptr || values->size() != joint_limits_.size()) {
    return;
  }
  for (std::size_t i = 0; i < values->size(); ++i) {
    const double limit = std::abs(joint_limits_[i].max_velocity);
    if (std::isfinite(limit)) {
      (*values)[i] = ClampValue((*values)[i], -limit, limit);
    }
  }
}

void FlipperReferenceGenerator::ApplyPositionLimits(
    std::vector<double>* values) const {
  if (values == nullptr || values->size() != joint_limits_.size()) {
    return;
  }
  for (std::size_t i = 0; i < values->size(); ++i) {
    if (std::isfinite(joint_limits_[i].min_position) &&
        std::isfinite(joint_limits_[i].max_position)) {
      (*values)[i] = ClampValue((*values)[i],
                                joint_limits_[i].min_position,
                                joint_limits_[i].max_position);
    }
  }
}

bool FlipperReferenceGenerator::HasName(const std::string& name) const {
  return joint_index_.find(name) != joint_index_.end();
}

std::size_t FlipperReferenceGenerator::RequireIndex(const std::string& name) const {
  const auto it = joint_index_.find(name);
  if (it == joint_index_.end()) {
    throw std::runtime_error("missing required joint name: " + name);
  }
  return it->second;
}

}  // namespace flipper_control
