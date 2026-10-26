#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <sys/wait.h>
#include <vector>

#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/shared_state.hpp"

namespace {

int RunAndGetExitCode(const std::string& command) {
  const int status = std::system(command.c_str());
  if (status == -1) {
    return -1;
  }
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
}

std::string Quote(const std::string& path) {
  return "'" + path + "'";
}

std::filesystem::path ResolveNodeBinary() {
  const std::filesystem::path build_dir(PROJECT_BINARY_DIR);
  const std::vector<std::filesystem::path> candidates = {
      build_dir / "canopen_hw_node",
      build_dir / "Eyou_Canopen_Master" / "canopen_hw_node",
      build_dir.parent_path() / "devel" / "lib" / "Eyou_Canopen_Master" /
          "canopen_hw_node",
      build_dir.parent_path().parent_path() / "devel" / "lib" /
          "Eyou_Canopen_Master" / "canopen_hw_node"};

  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }
  return candidates.front();
}

}  // namespace

TEST(StartupIntegration, ConfigPropagationFromYamlToRobotHw) {
  const std::string path = "/tmp/joints_startup_integration.yaml";
  {
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 10\n"
           "    velocity_scale: 2\n"
           "    torque_scale: 0.5\n"
           "  - name: joint_2\n"
           "    canopen:\n"
           "      node_id: 2\n"
           "    counts_per_rev: 2000\n"
           "    rated_torque_nm: 20\n"
           "    velocity_scale: 1.5\n"
           "    torque_scale: 2.0\n";
  }

  canopen_hw::CanopenMasterConfig cfg;
  std::string error;
  ASSERT_TRUE(canopen_hw::LoadJointsYaml(path, &error, &cfg)) << error;
  ASSERT_EQ(cfg.axis_count, 2u);

  canopen_hw::SharedState shared(cfg.axis_count);
  canopen_hw::CanopenRobotHw hw(&shared);
  hw.ApplyConfig(cfg);

  canopen_hw::AxisFeedback fb0;
  fb0.actual_position = 500;  // 0.5 rev -> pi
  fb0.actual_torque = 1000;   // 10Nm * 0.5 = 5Nm
  shared.UpdateFeedback(0, fb0);

  canopen_hw::AxisFeedback fb1;
  fb1.actual_position = 1000;  // 0.5 rev -> pi
  fb1.actual_torque = 1000;    // 20Nm * 2.0 = 40Nm
  shared.UpdateFeedback(1, fb1);

  hw.ReadFromSharedState();

  EXPECT_NEAR(hw.joint_position(0), 3.1415926, 0.01);
  EXPECT_NEAR(hw.joint_position(1), 3.1415926, 0.01);
  EXPECT_NEAR(hw.joint_effort(0), 5.0, 0.01);
  EXPECT_NEAR(hw.joint_effort(1), 40.0, 0.01);
}

TEST(StartupIntegration, MissingJointsYamlAbortsStartup) {
  const std::filesystem::path node_bin = ResolveNodeBinary();
  ASSERT_TRUE(std::filesystem::exists(node_bin)) << node_bin;

  const std::filesystem::path dcf_path =
      std::filesystem::path(PROJECT_SOURCE_DIR) / "config/master.dcf";
  const std::filesystem::path missing_joints =
      std::filesystem::path("/tmp/definitely_missing_joints.yaml");
  std::filesystem::remove(missing_joints);

  const std::string cmd =
      Quote(node_bin.string()) + " --dcf " + Quote(dcf_path.string()) +
      " --joints " + Quote(missing_joints.string()) +
      " >/tmp/startup_missing_joints.log 2>&1";
  EXPECT_EQ(RunAndGetExitCode(cmd), 1);
}

TEST(StartupIntegration, MissingDcfAbortsStartup) {
  const std::filesystem::path node_bin = ResolveNodeBinary();
  ASSERT_TRUE(std::filesystem::exists(node_bin)) << node_bin;

  const std::filesystem::path joints_path =
      std::filesystem::path(PROJECT_SOURCE_DIR) / "config/joints.yaml";
  const std::filesystem::path missing_dcf =
      std::filesystem::path("/tmp/definitely_missing_master.dcf");
  std::filesystem::remove(missing_dcf);

  const std::string cmd =
      Quote(node_bin.string()) + " --dcf " + Quote(missing_dcf.string()) +
      " --joints " + Quote(joints_path.string()) +
      " >/tmp/startup_missing_dcf.log 2>&1";
  EXPECT_EQ(RunAndGetExitCode(cmd), 1);
}

TEST(StartupIntegration, PositionCommandsFlowBeforeAllAxesReady) {
  const std::string path = "/tmp/joints_startup_operational_regression.yaml";
  {
    std::ofstream ofs(path);
    ofs << "joints:\n"
           "  - name: joint_1\n"
           "    canopen:\n"
           "      node_id: 1\n"
           "    counts_per_rev: 1000\n"
           "    rated_torque_nm: 10\n"
           "    velocity_scale: 1.0\n"
           "    torque_scale: 1.0\n"
           "  - name: joint_2\n"
           "    canopen:\n"
           "      node_id: 2\n"
           "    counts_per_rev: 2000\n"
           "    rated_torque_nm: 20\n"
           "    velocity_scale: 1.0\n"
           "    torque_scale: 1.0\n";
  }

  canopen_hw::CanopenMasterConfig cfg;
  std::string error;
  ASSERT_TRUE(canopen_hw::LoadJointsYaml(path, &error, &cfg)) << error;
  ASSERT_EQ(cfg.axis_count, 2u);

  canopen_hw::SharedState shared(cfg.axis_count);
  canopen_hw::CanopenRobotHw hw(&shared);
  hw.ApplyConfig(cfg);

  canopen_hw::AxisCommand sentinel0;
  sentinel0.target_position = 111;
  shared.UpdateCommand(0, sentinel0);
  canopen_hw::AxisCommand sentinel1;
  sentinel1.target_position = 222;
  shared.UpdateCommand(1, sentinel1);

  canopen_hw::AxisFeedback fb0;
  fb0.is_operational = true;
  fb0.is_fault = false;
  canopen_hw::AxisFeedback fb1;
  fb1.is_operational = false;
  fb1.is_fault = false;

  shared.UpdateFeedback(0, fb0);
  shared.UpdateFeedback(1, fb1);
  shared.RecomputeAllOperational();

  hw.ReadFromSharedState();
  EXPECT_FALSE(hw.all_operational());

  hw.SetJointCommand(0, 1.0);
  hw.SetJointCommand(1, 2.0);
  hw.WriteToSharedState();

  auto blocked = shared.Snapshot();
  EXPECT_NE(blocked.commands[0].target_position, 111);
  EXPECT_NE(blocked.commands[1].target_position, 222);
  EXPECT_FALSE(blocked.all_operational);

  fb1.is_operational = true;
  shared.UpdateFeedback(1, fb1);
  shared.RecomputeAllOperational();

  hw.ReadFromSharedState();
  EXPECT_TRUE(hw.all_operational());

  hw.SetJointCommand(0, 1.0);
  hw.SetJointCommand(1, 2.0);
  hw.WriteToSharedState();

  auto unblocked = shared.Snapshot();
  EXPECT_TRUE(unblocked.all_operational);
  EXPECT_NE(unblocked.commands[0].target_position, 111);
  EXPECT_NE(unblocked.commands[1].target_position, 222);
}
