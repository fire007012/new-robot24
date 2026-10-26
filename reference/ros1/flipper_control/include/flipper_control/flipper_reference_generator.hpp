#pragma once

#include <limits>
#include <map>
#include <string>
#include <vector>

#include <ros/time.h>

namespace flipper_control {

enum class LinkageMode {
  kIndependent = 0,
  kLeftRightMirror,
  kFrontRearSync,
  kSidePair,
  kDiagonalPair,
};

std::string ToString(LinkageMode mode);
bool ParseLinkageMode(const std::string& value, LinkageMode* mode);

struct JointLimit {
  double min_position = -std::numeric_limits<double>::infinity();
  double max_position = std::numeric_limits<double>::infinity();
  double max_velocity = std::numeric_limits<double>::infinity();
};

class FlipperReferenceGenerator {
 public:
  explicit FlipperReferenceGenerator(const std::vector<std::string>& joint_names);

  const std::vector<std::string>& joint_names() const { return joint_names_; }

  void SetJointLimits(const std::vector<JointLimit>& limits);
  void SetCommandTimeout(double timeout_sec);
  void SetDtClamp(double dt_clamp_sec);
  void SetLowpassAlpha(double alpha);
  void SetDriftThreshold(double threshold_rad);
  void SetLinkageMode(LinkageMode mode);

  LinkageMode linkage_mode() const { return linkage_mode_; }

  bool SetMeasuredPositions(const std::vector<double>& positions, std::string* error);
  bool ExpandValues(const std::vector<std::string>& source_names,
                    const std::vector<double>& source_values,
                    const std::vector<double>& seed_values,
                    std::vector<double>* out,
                    std::string* error) const;
  bool UpdateVelocityCommand(const std::vector<std::string>& source_names,
                             const std::vector<double>& source_values,
                             const ros::Time& stamp,
                             std::string* error);

  void ResetReferenceToMeasured();
  void ClearVelocityCommand();

  bool Step(const ros::Time& now, double dt_sec);

  bool has_measurement() const { return has_measurement_; }
  bool reference_initialized() const { return reference_initialized_; }
  bool command_timed_out() const { return command_timed_out_; }
  bool degraded() const { return degraded_; }

  const std::vector<double>& measured_positions() const { return measured_positions_; }
  const std::vector<double>& reference_positions() const { return reference_positions_; }
  const std::vector<double>& filtered_velocities() const { return filtered_velocities_; }

 private:
  std::vector<std::size_t> ResolveAffectedIndices(std::size_t source_index) const;
  void ApplyVelocityLimits(std::vector<double>* values) const;
  void ApplyPositionLimits(std::vector<double>* values) const;
  bool HasName(const std::string& name) const;
  std::size_t RequireIndex(const std::string& name) const;

  std::vector<std::string> joint_names_;
  std::map<std::string, std::size_t> joint_index_;
  std::vector<JointLimit> joint_limits_;

  std::vector<double> measured_positions_;
  std::vector<double> reference_positions_;
  std::vector<double> target_velocities_;
  std::vector<double> filtered_velocities_;

  LinkageMode linkage_mode_ = LinkageMode::kIndependent;
  ros::Time last_velocity_command_stamp_;
  bool has_velocity_command_ = false;

  bool has_measurement_ = false;
  bool reference_initialized_ = false;
  bool command_timed_out_ = true;
  bool degraded_ = false;

  double command_timeout_sec_ = 0.4;
  double dt_clamp_sec_ = 0.02;
  double lowpass_alpha_ = 0.35;
  double drift_threshold_rad_ = 0.35;
};

}  // namespace flipper_control
