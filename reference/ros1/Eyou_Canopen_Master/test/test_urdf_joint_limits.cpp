#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "canopen_hw/urdf_joint_limits.hpp"

namespace canopen_hw {
namespace {

CanopenMasterConfig::JointConfig MakeJoint(const std::string& name, uint8_t node_id) {
  CanopenMasterConfig::JointConfig j;
  j.name = name;
  j.node_id = node_id;
  return j;
}

TEST(UrdfJointLimits, ParseSuccessForRevoluteAndPrismatic) {
  const std::string urdf = R"(
<robot name="r">
  <link name="l0"/>
  <link name="l1"/>
  <link name="l2"/>
  <joint name="joint_1" type="revolute">
    <parent link="l0"/>
    <child link="l1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.5" effort="10" velocity="1"/>
  </joint>
  <joint name="joint_2" type="prismatic">
    <parent link="l1"/>
    <child link="l2"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.2" upper="0.3" effort="10" velocity="1"/>
  </joint>
</robot>
)";

  std::vector<CanopenMasterConfig::JointConfig> joints = {
      MakeJoint("joint_1", 5), MakeJoint("joint_2", 6)};
  std::vector<JointLimitSpec> limits;
  std::string error;

  ASSERT_TRUE(ParseUrdfJointLimits(urdf, joints, &limits, &error)) << error;
  ASSERT_EQ(limits.size(), 2u);
  EXPECT_TRUE(limits[0].has_position_limits);
  EXPECT_DOUBLE_EQ(limits[0].lower, -1.0);
  EXPECT_DOUBLE_EQ(limits[0].upper, 1.5);
  EXPECT_EQ(limits[0].unit, UrdfJointLimitUnit::kRadians);
  EXPECT_TRUE(limits[1].has_position_limits);
  EXPECT_DOUBLE_EQ(limits[1].lower, -0.2);
  EXPECT_DOUBLE_EQ(limits[1].upper, 0.3);
  EXPECT_EQ(limits[1].unit, UrdfJointLimitUnit::kMeters);
}

TEST(UrdfJointLimits, MissingJointReturnsError) {
  const std::string urdf = R"(
<robot name="r">
  <link name="l0"/>
  <link name="l1"/>
  <joint name="joint_1" type="revolute">
    <parent link="l0"/>
    <child link="l1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
</robot>
)";

  std::vector<CanopenMasterConfig::JointConfig> joints = {
      MakeJoint("joint_missing", 5)};
  std::vector<JointLimitSpec> limits;
  std::string error;

  EXPECT_FALSE(ParseUrdfJointLimits(urdf, joints, &limits, &error));
  EXPECT_NE(error.find("not found"), std::string::npos);
}

TEST(UrdfJointLimits, ContinuousJointDisablesPositionLimits) {
  const std::string urdf = R"(
<robot name="r">
  <link name="l0"/>
  <link name="l1"/>
  <joint name="joint_1" type="continuous">
    <parent link="l0"/>
    <child link="l1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="10" velocity="1"/>
  </joint>
</robot>
)";

  std::vector<CanopenMasterConfig::JointConfig> joints = {
      MakeJoint("joint_1", 5)};
  std::vector<JointLimitSpec> limits;
  std::string error;

  ASSERT_TRUE(ParseUrdfJointLimits(urdf, joints, &limits, &error)) << error;
  ASSERT_EQ(limits.size(), 1u);
  EXPECT_FALSE(limits[0].has_position_limits);
  EXPECT_EQ(limits[0].unit, UrdfJointLimitUnit::kRadians);
}

TEST(UrdfJointLimits, InvalidUrdfRejected) {
  const std::string urdf = R"(
<robot name="r">
  <link name="l0"/>
  <link name="l1">
</robot>
)";

  std::vector<CanopenMasterConfig::JointConfig> joints = {
      MakeJoint("joint_1", 5)};
  std::vector<JointLimitSpec> limits;
  std::string error;

  EXPECT_FALSE(ParseUrdfJointLimits(urdf, joints, &limits, &error));
  EXPECT_NE(error.find("failed to parse robot_description"), std::string::npos);
}

}  // namespace
}  // namespace canopen_hw
