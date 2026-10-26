#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "canopen_hw/cia402_defs.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/shared_state.hpp"

TEST(JointsConfig, LoadValidYaml) {
  canopen_hw::SharedState shared(6);
  canopen_hw::CanopenRobotHw hw(&shared);

  const std::string path = "/tmp/joints_test.yaml";
  {
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "      verify_pdo_mapping: true\n"
           "      default_mode: 7\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 10\n"
           "    max_velocity_for_clamp: 1000\n"
           "    velocity_scale: 2\n"
           "    torque_scale: 0.5\n"
           "  - name: joint_2\n"
           "    canopen:\n"
           "      node_id: 2\n"
           "      verify_pdo_mapping: false\n"
           "      default_mode: 9\n"
           "    counts_per_rev: 2000\n"
           "    rated_torque_nm: 20\n"
           "    max_velocity_for_clamp: 2000\n"
           "    velocity_scale: 1.5\n"
           "    torque_scale: 2.0\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(path, &error, &master_cfg);
  EXPECT_TRUE(ok);
  hw.ApplyConfig(master_cfg);
  EXPECT_EQ(master_cfg.joints.size(), 2u);
  EXPECT_EQ(master_cfg.joints[0].node_id, 1u);
  EXPECT_TRUE(master_cfg.joints[0].verify_pdo_mapping);
  EXPECT_EQ(master_cfg.joints[0].default_mode, canopen_hw::kMode_IP);
  EXPECT_DOUBLE_EQ(master_cfg.joints[0].max_velocity_for_clamp, 1000.0);
  EXPECT_EQ(master_cfg.joints[1].node_id, 2u);
  EXPECT_FALSE(master_cfg.joints[1].verify_pdo_mapping);
  EXPECT_EQ(master_cfg.joints[1].default_mode, canopen_hw::kMode_CSV);
  EXPECT_DOUBLE_EQ(master_cfg.joints[1].max_velocity_for_clamp, 2000.0);

  canopen_hw::AxisFeedback fb0;
  fb0.actual_position = 500;  // 0.5 rev -> pi
  fb0.actual_torque = 1000;   // 100% -> 10Nm * 0.5 -> 5Nm
  shared.UpdateFeedback(0, fb0);

  canopen_hw::AxisFeedback fb1;
  fb1.actual_position = 1000;  // 0.5 rev -> pi
  fb1.actual_torque = 1000;    // 100% -> 20Nm * 2 -> 40Nm
  shared.UpdateFeedback(1, fb1);

  hw.ReadFromSharedState();

  EXPECT_GT(hw.joint_position(0), 3.13);
  EXPECT_LT(hw.joint_position(0), 3.15);
  EXPECT_GT(hw.joint_position(1), 3.13);
  EXPECT_LT(hw.joint_position(1), 3.15);
  EXPECT_GT(hw.joint_effort(0), 4.99);
  EXPECT_LT(hw.joint_effort(0), 5.01);
  EXPECT_GT(hw.joint_effort(1), 39.99);
  EXPECT_LT(hw.joint_effort(1), 40.01);
}

TEST(JointsConfig, InvalidNodeIdRejected) {
  const std::string invalid_path = "/tmp/joints_test_invalid_node_id.yaml";
  {
    std::ofstream ofs(invalid_path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 300\n"
           "    counts_per_rev: 1000\n";
  }

  std::string invalid_error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool invalid_ok =
      canopen_hw::LoadJointsYaml(invalid_path, &invalid_error, &master_cfg);
  EXPECT_FALSE(invalid_ok);
  EXPECT_NE(invalid_error.find("invalid node_id"), std::string::npos);
}

TEST(JointsConfig, InvalidFieldTypeRejected) {
  const std::string invalid_path = "/tmp/joints_test_invalid_type.yaml";
  {
    std::ofstream ofs(invalid_path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: bad\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 6\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(invalid_path, &error, &master_cfg);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("invalid field type at joints[0]"), std::string::npos);
}

TEST(JointsConfig, InvalidMaxVelocityForClampRejected) {
  const std::string invalid_path = "/tmp/joints_test_invalid_max_velocity.yaml";
  {
    std::ofstream ofs(invalid_path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 0\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(invalid_path, &error, &master_cfg);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("invalid max_velocity_for_clamp"), std::string::npos);
}

TEST(JointsConfig, InvalidDefaultModeRejected) {
  const std::string invalid_path = "/tmp/joints_test_invalid_default_mode.yaml";
  {
    std::ofstream ofs(invalid_path);
    ofs << "joints:\n"
           "  - name: joint_bad\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "      default_mode: 11\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 1000\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(invalid_path, &error, &master_cfg);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("invalid default_mode"), std::string::npos);
}

TEST(JointsConfig, InterpolationPeriodDefaultsFromLoopHzAndAllowsOverride) {
  const std::string path = "/tmp/joints_test_ip_period.yaml";
  {
    std::ofstream ofs(path);
    ofs << "canopen:\n"
           "  loop_hz: 250\n"
           "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 1000\n"
           "  - name: joint_2\n"
           "    canopen:\n"
           "      node_id: 2\n"
           "    ip_interpolation_period_ms: 8\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 1000\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(path, &error, &master_cfg);
  ASSERT_TRUE(ok) << error;
  ASSERT_EQ(master_cfg.joints.size(), 2u);

  // loop_hz=250 -> 1000/250 = 4ms inferred default.
  EXPECT_EQ(master_cfg.joints[0].ip_interpolation_period_ms, 4u);
  EXPECT_EQ(master_cfg.joints[1].ip_interpolation_period_ms, 8u);
}

TEST(JointsConfig, AutoWriteSoftLimitsFromUrdfDefaultsFalse) {
  const std::string path = "/tmp/joints_test_auto_soft_limit_default.yaml";
  {
    std::ofstream ofs(path);
    ofs << "canopen:\n"
           "  interface: canable0\n"
           "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 1000\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(path, &error, &master_cfg);
  ASSERT_TRUE(ok) << error;
  EXPECT_FALSE(master_cfg.auto_write_soft_limits_from_urdf);
}

TEST(JointsConfig, AutoWriteSoftLimitsFromUrdfCanBeEnabled) {
  const std::string path = "/tmp/joints_test_auto_soft_limit_enabled.yaml";
  {
    std::ofstream ofs(path);
    ofs << "canopen:\n"
           "  auto_write_soft_limits_from_urdf: true\n"
           "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 1000\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(path, &error, &master_cfg);
  ASSERT_TRUE(ok) << error;
  EXPECT_TRUE(master_cfg.auto_write_soft_limits_from_urdf);
}

TEST(JointsConfig, CountsPerMeterCanBeConfigured) {
  const std::string path = "/tmp/joints_test_counts_per_meter_valid.yaml";
  {
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    counts_per_meter: 20000\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 1000\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(path, &error, &master_cfg);
  ASSERT_TRUE(ok) << error;
  ASSERT_EQ(master_cfg.joints.size(), 1u);
  EXPECT_DOUBLE_EQ(master_cfg.joints[0].counts_per_meter, 20000.0);
}

TEST(JointsConfig, InvalidCountsPerMeterRejected) {
  const std::string path = "/tmp/joints_test_counts_per_meter_invalid.yaml";
  {
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    counts_per_meter: 0\n"
           "    rated_torque_nm: 6\n"
           "    max_velocity_for_clamp: 1000\n";
  }

  std::string error;
  canopen_hw::CanopenMasterConfig master_cfg;
  const bool ok = canopen_hw::LoadJointsYaml(path, &error, &master_cfg);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("invalid counts_per_meter"), std::string::npos);
}
