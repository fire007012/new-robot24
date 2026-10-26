#include <gtest/gtest.h>
#include "robot_safety/policy.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>
namespace {
std::filesystem::path policyPath() { return std::filesystem::path(__FILE__).parent_path().parent_path()/"config/group_policy_v1.yaml"; }
void invalidPolicy(const std::string & before,const std::string & after) {
  std::ifstream input(policyPath());std::stringstream contents;contents<<input.rdbuf();auto text=contents.str();
  auto position=text.find(before);ASSERT_NE(position,std::string::npos);text.replace(position,before.size(),after);
  auto temporary=std::filesystem::temp_directory_path()/("invalid-group-policy-"+std::to_string(getpid())+".yaml");
  {std::ofstream out(temporary);out<<text;}
  EXPECT_THROW(robot_safety::Policy::load(temporary.string()),std::exception);std::filesystem::remove(temporary);
}
}
TEST(GroupPolicy, LoadsTheFrozenOwnershipAndControllerContract) {
  auto p=robot_safety::Policy::load(policyPath().string());EXPECT_EQ(p.axes.size(),13u);EXPECT_EQ(p.controllers.size(),4u);
  EXPECT_EQ(p.axes.at("shoulder_yaw_joint").group,"arm");EXPECT_EQ(p.associations.size(),3u);
}
TEST(GroupPolicy, RejectsDuplicateKeysAndControllerConflicts) {
  invalidPolicy("version: group-policy-v1","version: group-policy-v1\nversion: other");
  invalidPolicy("controllers: [gripper_controller]","controllers: [wheel_controller]");
}
TEST(GroupPolicy, RejectsFakePhysicalEvidenceAndRelaxedMechanicalRules) {
  invalidPolicy("initial: unknown","initial: satisfied");
  invalidPolicy("source_group: arm, category: '*'","source_group: arm, category: emcy");
}
TEST(GroupPolicy, RejectsOwnershipChangesAndUnsupportedEmcyMappings) {
  invalidPolicy("shoulder_yaw_joint: {group: arm","shoulder_yaw_joint: {group: base");
  invalidPolicy("known_emcy: []","known_emcy: [1234]");
  invalidPolicy("version: group-policy-v1","version: group-policy-v2");
}
