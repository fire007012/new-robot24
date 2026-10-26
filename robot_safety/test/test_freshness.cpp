#include <gtest/gtest.h>
#include "robot_safety/freshness.hpp"
using F = robot_safety::Freshness;
TEST(Freshness, ConfigurationRequiresDistinctWarningAndStopDeadlines) {
  EXPECT_THROW((F({0, 100, 10})), std::invalid_argument);
  EXPECT_THROW((F({100, 100, 10})), std::invalid_argument);
  EXPECT_THROW((F({10, 100, -1})), std::invalid_argument);
}
TEST(Freshness, ExactStopBoundaryClosesGateWithoutAdditionalDelay) {
  F f({50, 100, 0}); EXPECT_EQ(f.observe(1000, 1000, true), F::State::Healthy);
  EXPECT_EQ(f.observe(1049, 1000, true), F::State::Healthy);
  EXPECT_EQ(f.observe(1050, 1000, true), F::State::Warn); EXPECT_TRUE(f.trustworthyForMotion());
  EXPECT_EQ(f.observe(1099, 1000, true), F::State::Warn);
  EXPECT_EQ(f.observe(1100, 1000, true), F::State::Stop); EXPECT_FALSE(f.trustworthyForMotion());
}
TEST(Freshness, StableWindowIsRequiredAtStartupAndRecovery) {
  F f({50, 100, 30}); EXPECT_EQ(f.observe(1000, 1000, true), F::State::Stabilizing);
  EXPECT_FALSE(f.trustworthyForMotion()); EXPECT_EQ(f.observe(1030, 1030, true), F::State::Healthy);
  EXPECT_EQ(f.observe(1040, 1040, true, true), F::State::Stop);
  EXPECT_EQ(f.observe(1050, 1050, true), F::State::Stabilizing);
  EXPECT_EQ(f.observe(1079, 1079, true), F::State::Stabilizing);
  EXPECT_EQ(f.observe(1080, 1080, true), F::State::Healthy);
}
TEST(Freshness, ShortDelayDoesNotHideExpiryBetweenCycles) {
  F f({50, 100, 30}); f.observe(1000, 1000, true); f.observe(1030, 1030, true);
  EXPECT_EQ(f.observe(1080, 1030, true), F::State::Warn);
  EXPECT_EQ(f.observe(1081, 1081, true), F::State::Warn);
  EXPECT_EQ(f.observe(1111, 1111, true), F::State::Healthy);
}
TEST(Freshness, ASecondFaultRestartsRecoveryStableWindow) {
  F f({50, 100, 30}); f.observe(1000, 1000, true); f.observe(1030, 1030, true);
  f.observe(1040, 1040, false); f.observe(1050, 1050, true);
  EXPECT_EQ(f.observe(1060, 1060, true, true), F::State::Stop);
  f.observe(1070, 1070, true); EXPECT_EQ(f.observe(1090, 1090, true), F::State::Stabilizing);
  EXPECT_EQ(f.observe(1100, 1100, true), F::State::Healthy);
}
TEST(Freshness, FutureOrInvalidSamplesCannotCountAsTrustworthy) {
  F f({50, 100, 0}); EXPECT_EQ(f.observe(1000, 1001, true), F::State::Stop);
  EXPECT_EQ(f.observe(1000, -1, true), F::State::Stop);
  EXPECT_EQ(f.observe(1000, 1000, false), F::State::Stop);
  EXPECT_THROW(f.observe(999, 999, true), std::invalid_argument);
}
TEST(Freshness, DifferentFeedbackChannelsUseIndependentDeadlines) {
  F pdo({50, 100, 0}), private_can({250, 500, 0});
  pdo.observe(1000, 1000, true); private_can.observe(1000, 1000, true);
  EXPECT_EQ(pdo.observe(1100, 1000, true), F::State::Stop);
  EXPECT_EQ(private_can.observe(1100, 1000, true), F::State::Healthy);
}
TEST(Freshness, WarningAgeCannotBypassStartupStableWindow) {
  F f({50, 100, 30}); EXPECT_EQ(f.observe(1050, 1000, true), F::State::Stop);
  EXPECT_FALSE(f.trustworthyForMotion());
  EXPECT_EQ(f.observe(1060, 1060, true), F::State::Stabilizing);
  EXPECT_EQ(f.observe(1090, 1090, true), F::State::Healthy);
}
TEST(Freshness, NewFrameCannotHideDeadlineCrossedBetweenHardwareCycles) {
  F f({50, 100, 30}); f.observe(1000, 1000, true); f.observe(1030, 1030, true);
  EXPECT_EQ(f.observe(1200, 1200, true), F::State::Stop); EXPECT_FALSE(f.trustworthyForMotion());
  EXPECT_EQ(f.observe(1201, 1201, true), F::State::Stabilizing);
  EXPECT_EQ(f.observe(1231, 1231, true), F::State::Healthy);
}
