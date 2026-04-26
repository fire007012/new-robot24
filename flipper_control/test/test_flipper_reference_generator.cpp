#include <gtest/gtest.h>

#include <ros/time.h>

#include "flipper_control/flipper_reference_generator.hpp"

namespace flipper_control {
namespace {

FlipperReferenceGenerator MakeGenerator() {
  FlipperReferenceGenerator generator({
      "left_front_arm_joint",
      "right_front_arm_joint",
      "left_rear_arm_joint",
      "right_rear_arm_joint",
  });

  std::vector<JointLimit> limits(4);
  for (auto& limit : limits) {
    limit.min_position = -10.0;
    limit.max_position = 10.0;
    limit.max_velocity = 2.0;
  }
  generator.SetJointLimits(limits);
  generator.SetLowpassAlpha(1.0);
  generator.SetCommandTimeout(0.5);
  generator.SetDtClamp(0.02);
  std::string error;
  EXPECT_TRUE(generator.SetMeasuredPositions({0.0, 0.0, 0.0, 0.0}, &error)) << error;
  return generator;
}

TEST(FlipperReferenceGeneratorTest, LeftRightMirrorExpandsVelocityCommand) {
  auto generator = MakeGenerator();
  generator.SetLinkageMode(LinkageMode::kLeftRightMirror);

  std::string error;
  ASSERT_TRUE(generator.UpdateVelocityCommand({"left_front_arm_joint"}, {1.0},
                                              ros::Time(0.0), &error))
      << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.1), 0.01));

  const auto& ref = generator.reference_positions();
  EXPECT_NEAR(ref[0], 0.01, 1e-9);
  EXPECT_NEAR(ref[1], 0.01, 1e-9);
  EXPECT_NEAR(ref[2], 0.0, 1e-9);
  EXPECT_NEAR(ref[3], 0.0, 1e-9);
}

TEST(FlipperReferenceGeneratorTest, DtClampLimitsIntegrationJump) {
  auto generator = MakeGenerator();

  std::string error;
  ASSERT_TRUE(generator.UpdateVelocityCommand({"left_front_arm_joint"}, {1.0},
                                              ros::Time(0.0), &error))
      << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.1), 0.1));

  const auto& ref = generator.reference_positions();
  EXPECT_NEAR(ref[0], 0.02, 1e-9);
}

TEST(FlipperReferenceGeneratorTest, FrontRearSyncExpandsToAllFourJoints) {
  auto generator = MakeGenerator();
  generator.SetLinkageMode(LinkageMode::kFrontRearSync);

  std::string error;
  ASSERT_TRUE(generator.UpdateVelocityCommand({"left_front_arm_joint"}, {1.0},
                                              ros::Time(0.0), &error))
      << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.1), 0.01));

  const auto& ref = generator.reference_positions();
  EXPECT_NEAR(ref[0], 0.01, 1e-9);
  EXPECT_NEAR(ref[1], 0.01, 1e-9);
  EXPECT_NEAR(ref[2], 0.01, 1e-9);
  EXPECT_NEAR(ref[3], 0.01, 1e-9);
}

TEST(FlipperReferenceGeneratorTest, SidePairExpandsAcrossSameSide) {
  auto generator = MakeGenerator();
  generator.SetLinkageMode(LinkageMode::kSidePair);

  std::string error;
  ASSERT_TRUE(generator.UpdateVelocityCommand({"left_front_arm_joint"}, {1.0},
                                              ros::Time(0.0), &error))
      << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.1), 0.01));

  const auto& ref = generator.reference_positions();
  EXPECT_NEAR(ref[0], 0.01, 1e-9);
  EXPECT_NEAR(ref[1], 0.0, 1e-9);
  EXPECT_NEAR(ref[2], 0.01, 1e-9);
  EXPECT_NEAR(ref[3], 0.0, 1e-9);
}

TEST(FlipperReferenceGeneratorTest, TimeoutZerosVelocityWithoutDriftingFurther) {
  auto generator = MakeGenerator();

  std::string error;
  ASSERT_TRUE(generator.UpdateVelocityCommand({"left_front_arm_joint"}, {1.0},
                                              ros::Time(0.0), &error))
      << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.1), 0.01));
  const double after_first_step = generator.reference_positions()[0];

  ASSERT_TRUE(generator.Step(ros::Time(1.0), 0.01));
  EXPECT_TRUE(generator.command_timed_out());
  EXPECT_NEAR(generator.reference_positions()[0], after_first_step, 1e-9);
  EXPECT_NEAR(generator.filtered_velocities()[0], 0.0, 1e-9);
}

TEST(FlipperReferenceGeneratorTest, NoCommandKeepsZeroVelocityOutput) {
  auto generator = MakeGenerator();

  std::string error;
  ASSERT_TRUE(generator.SetMeasuredPositions({0.5, -0.4, 0.3, -0.2}, &error)) << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.1), 0.01));

  EXPECT_TRUE(generator.command_timed_out());
  EXPECT_NEAR(generator.filtered_velocities()[0], 0.0, 1e-9);
}

TEST(FlipperReferenceGeneratorTest, ClearVelocityCommandStopsFurtherOutput) {
  auto generator = MakeGenerator();

  std::string error;
  ASSERT_TRUE(generator.UpdateVelocityCommand({"left_front_arm_joint"}, {1.0},
                                              ros::Time(0.0), &error))
      << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.1), 0.01));

  generator.ClearVelocityCommand();
  ASSERT_TRUE(generator.SetMeasuredPositions({0.6, -0.2, 0.1, -0.1}, &error)) << error;
  ASSERT_TRUE(generator.Step(ros::Time(0.2), 0.01));

  EXPECT_TRUE(generator.command_timed_out());
  EXPECT_NEAR(generator.filtered_velocities()[0], 0.0, 1e-9);
}

TEST(FlipperReferenceGeneratorTest, ConflictingLinkedInputsAreRejected) {
  auto generator = MakeGenerator();
  generator.SetLinkageMode(LinkageMode::kLeftRightMirror);

  std::string error;
  EXPECT_FALSE(generator.UpdateVelocityCommand(
      {"left_front_arm_joint", "right_front_arm_joint"}, {0.5, -0.5},
      ros::Time(0.0), &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace flipper_control

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
