#include <gtest/gtest.h>
#include "robot_safety/domain.hpp"
#include <stdexcept>
#include <thread>
#include <atomic>
using D = robot_safety::Domain;
class GroupDomain : public ::testing::Test {
protected:
  int64_t now = 1000;
  D d{[this] { return now; }};
  std::map<std::string, D::Axis> axes{
    {"shoulder", {"arm", "canopen"}}, {"elbow", {"arm", "private"}},
    {"flipper1", {"flippers", "canopen"}}, {"flipper2", {"flippers", "canopen"}},
    {"left", {"base", "private"}}, {"right", {"base", "private"}},
    {"finger", {"gripper", "private"}}};
  std::vector<D::Association> rules{
    {"arm", "*", "base", "secured", "brake"},
    {"flippers", "*", "base", "supported", "support"},
    {"gripper", "*", "arm", "retained", "payload"}};
  void SetUp() override { d.configure(axes, rules, "test-v1"); }
  void readyAll() { for (const auto & a : axes) d.reportAxis(a.first, true); }
  uint64_t start(const std::string & group) {
    EXPECT_TRUE(d.arm(group)); auto epoch = d.epoch(group); EXPECT_TRUE(d.run(group)); return epoch;
  }
  D::Event failure(const std::string & axis, const std::string & id = "failure") {
    D::Event e; e.id = id; e.axis = axis; e.category = "feedback"; e.code = "0x1234";
    e.reason = "drive unavailable"; e.recovery = "fresh feedback and operator acknowledgement"; return e;
  }
};
TEST_F(GroupDomain, BaseCanStartWithoutPreparingOtherAxes) {
  d.reportAxis("left", true); d.reportAxis("right", true);
  EXPECT_FALSE(d.arm("base"));
  d.condition("secured", D::Evidence::Satisfied, "brake", now + 100);
  d.condition("supported", D::Evidence::Satisfied, "support", now + 100);
  d.condition("retained", D::Evidence::Satisfied, "payload", now + 100);
  auto epoch = start("base"); EXPECT_TRUE(d.permits("base", epoch));
  EXPECT_FALSE(d.arm("arm")); EXPECT_FALSE(d.arm("flippers")); EXPECT_FALSE(d.arm("gripper"));
  EXPECT_NE(d.group("arm").state, D::State::Running);
}
TEST_F(GroupDomain, OneTrackFailureStopsBothTracksAndPreservesOtherSessions) {
  readyAll(); auto base = start("base"), arm = start("arm"), grip = start("gripper");
  d.reportAxis("left", false, "offline");
  EXPECT_FALSE(d.permits("base", base)); EXPECT_TRUE(d.permits("arm", arm));
  EXPECT_TRUE(d.permits("gripper", grip)); EXPECT_EQ(d.epoch("arm"), arm);
}
TEST_F(GroupDomain, ShoulderFailureStopsCrossBackendArmAndUnknownDependency) {
  readyAll(); auto base = start("base"), arm = start("arm"), grip = start("gripper"), flippers = start("flippers");
  d.event(failure("shoulder"));
  EXPECT_FALSE(d.permits("arm", arm)); EXPECT_FALSE(d.permits("base", base));
  EXPECT_TRUE(d.permits("gripper", grip)); EXPECT_TRUE(d.permits("flippers", flippers));
  EXPECT_EQ(d.group("base").blockers, std::vector<std::string>{"failure"});
}
TEST_F(GroupDomain, PrivateArmAxisHasSameGroupScopeAsShoulder) {
  readyAll(); auto arm = start("arm"), base = start("base"); d.event(failure("elbow"));
  EXPECT_FALSE(d.permits("arm", arm)); EXPECT_FALSE(d.permits("base", base));
}
TEST_F(GroupDomain, TrustedAssociationAllowsHealthyGroupAndExpiryClosesGate) {
  readyAll(); auto base = start("base");
  d.condition("secured", D::Evidence::Satisfied, "brake", now + 100);
  d.event(failure("shoulder")); EXPECT_TRUE(d.permits("base", base));
  now += 100; EXPECT_FALSE(d.permits("base", base));
  // Renewing evidence after expiry cannot resurrect the old base session.
  d.condition("secured", D::Evidence::Satisfied, "brake", now + 100);
  EXPECT_FALSE(d.permits("base", base));
  ASSERT_TRUE(d.arm("base")); EXPECT_NE(d.epoch("base"), base);
}
TEST_F(GroupDomain, NoSourceCanClaimAnotherSourcesEvidence) {
  EXPECT_THROW(d.condition("secured", D::Evidence::Satisfied, "guess", now + 100), std::invalid_argument);
  EXPECT_THROW(d.condition("secured", D::Evidence::Satisfied, "brake", now), std::invalid_argument);
  readyAll(); auto base = start("base"); d.event(failure("shoulder")); EXPECT_FALSE(d.permits("base", base));
}
TEST_F(GroupDomain, UnknownAndFalseConditionsBothBlock) {
  readyAll(); auto base = start("base");
  d.condition("supported", D::Evidence::Unsatisfied, "support", now + 100);
  d.event(failure("flipper1")); EXPECT_FALSE(d.permits("base", base));
  EXPECT_EQ(d.group("flippers").state, D::State::Fault);
}
TEST_F(GroupDomain, GripperFailureIsNotPrivateBusFailureAndDependenciesAreTransitive) {
  readyAll(); auto flippers = start("flippers"), base = start("base"), arm = start("arm");
  d.event(failure("finger")); EXPECT_TRUE(d.permits("flippers", flippers));
  EXPECT_FALSE(d.permits("base", base)); EXPECT_FALSE(d.permits("arm", arm));
  EXPECT_EQ(d.events().front().backend, "private");
}
TEST_F(GroupDomain, RetainedPayloadPreventsGripperFailureFromStoppingArm) {
  readyAll(); auto arm = start("arm"), base = start("base");
  d.condition("retained", D::Evidence::Satisfied, "payload", now + 100);
  d.event(failure("finger")); EXPECT_TRUE(d.permits("arm", arm)); EXPECT_TRUE(d.permits("base", base));
}
TEST_F(GroupDomain, BusFailureCoversItsAxesAndMechanicalDependencies) {
  readyAll(); auto arm = start("arm"), base = start("base"), flippers = start("flippers"), grip = start("gripper");
  d.reportBackend("canopen", false, "bus offline");
  EXPECT_FALSE(d.permits("arm", arm)); EXPECT_FALSE(d.permits("base", base));
  EXPECT_FALSE(d.permits("flippers", flippers)); EXPECT_TRUE(d.permits("gripper", grip));
  EXPECT_FALSE(d.group("arm").ready); EXPECT_TRUE(d.group("gripper").ready);
}
TEST_F(GroupDomain, RejectAndWarnDoNotRevokeSessions) {
  readyAll(); auto base = start("base");
  auto e = failure("left", "invalid-request"); e.severity = D::Severity::Reject; d.event(e);
  e.id = "jitter"; e.severity = D::Severity::Warn; d.event(e);
  EXPECT_TRUE(d.permits("base", base)); EXPECT_EQ(d.events().size(), 2u);
}
TEST_F(GroupDomain, RepeatedFaultReportsAreDeduplicated) {
  readyAll(); auto base = start("base"), grip = start("gripper");
  for (int i = 0; i < 100; ++i) d.reportAxis("left", false, "offline");
  EXPECT_EQ(d.epoch("base"), base + 1); EXPECT_EQ(d.events().size(), 1u);
  EXPECT_TRUE(d.permits("gripper", grip)); EXPECT_EQ(d.epoch("gripper"), grip);
  EXPECT_THROW(d.acknowledge("readiness:left"), std::logic_error);
}
TEST_F(GroupDomain, RecoveryOnlyArmsSelectedGroupAndNeverReplaysOldEpoch) {
  readyAll(); auto old = start("base"), grip = start("gripper");
  d.reportAxis("left", false, "offline"); d.reportAxis("left", true);
  EXPECT_FALSE(d.arm("base")); EXPECT_FALSE(d.run("base"));
  d.acknowledge("readiness:left"); EXPECT_FALSE(d.run("base"));
  ASSERT_TRUE(d.arm("base")); EXPECT_FALSE(d.permits("base", d.epoch("base")));
  ASSERT_TRUE(d.run("base")); EXPECT_FALSE(d.permits("base", old));
  EXPECT_TRUE(d.permits("gripper", grip)); EXPECT_EQ(d.epoch("gripper"), grip);
}
TEST_F(GroupDomain, RecurrenceDuringRecoveryClosesGateAgain) {
  readyAll(); start("base"); d.event(failure("left")); d.acknowledge("failure");
  ASSERT_TRUE(d.arm("base")); auto prepared = d.epoch("base"); d.event(failure("left", "again"));
  EXPECT_FALSE(d.run("base")); EXPECT_FALSE(d.permits("base", prepared));
  EXPECT_NE(d.epoch("base"), prepared);
}
TEST_F(GroupDomain, GlobalEmergencyCannotBeBypassedByLocalArmOrLegacyGate) {
  readyAll(); start("base"); start("arm");
  auto e = failure("left", "emergency"); e.severity = D::Severity::GlobalStop; d.event(e);
  EXPECT_TRUE(d.globallyInhibited()); EXPECT_FALSE(d.arm("gripper"));
  EXPECT_FALSE(d.resetGlobalStop()); d.acknowledge("emergency");
  EXPECT_FALSE(d.arm("base")); ASSERT_TRUE(d.resetGlobalStop());
  EXPECT_EQ(d.group("base").state, D::State::Fault); ASSERT_TRUE(d.arm("base"));
  EXPECT_FALSE(d.permits("base", d.epoch("base")));
  d.globalStop("physical emergency"); EXPECT_FALSE(d.arm()); EXPECT_FALSE(d.arm("base"));
}
TEST_F(GroupDomain, GlobalHaltRevokesAllGroupsButLocalHaltOnlyOne) {
  readyAll(); auto base = start("base"), arm = start("arm"); d.halt("base");
  EXPECT_FALSE(d.permits("base", base)); EXPECT_TRUE(d.permits("arm", arm));
  d.halt(); EXPECT_FALSE(d.permits("arm", arm));
}
TEST_F(GroupDomain, LegacyFaultRemainsGlobalWhileNewInterfaceIsStaged) {
  readyAll(); auto base = start("base"); d.fault("unclassified old callsite");
  EXPECT_FALSE(d.permits("base", base)); EXPECT_FALSE(d.arm("gripper"));
}
TEST_F(GroupDomain, ConfigurationAndEventSourceErrorsAreRejected) {
  auto bad = axes; bad["left"].group = "typo";
  EXPECT_THROW(d.configure(bad, rules, "bad"), std::invalid_argument);
  auto duplicates = rules; duplicates.push_back(rules.front());
  EXPECT_THROW(d.configure(axes, duplicates, "bad"), std::invalid_argument);
  EXPECT_THROW(d.reportAxis("unknown", true), std::out_of_range);
  auto e = failure("left"); e.group = "arm"; EXPECT_THROW(d.event(e), std::invalid_argument);
  e = failure("left"); d.event(e); e.code = "different"; EXPECT_THROW(d.event(e), std::invalid_argument);
  EXPECT_THROW(d.configure(axes, rules, "bad"), std::logic_error);
}
TEST_F(GroupDomain, ConfigurationCannotInvalidateRunningSessions) {
  readyAll(); auto base = start("base");
  EXPECT_THROW(d.configure(axes, rules, "bad"), std::logic_error);
  EXPECT_TRUE(d.permits("base", base));
}
TEST_F(GroupDomain, UnknownEmcyPreservesOriginalCodeAndExplicitRecoveryCondition) {
  readyAll(); auto arm = start("arm"), grip = start("gripper");
  auto e = failure("shoulder"); e.category = "unknown_emcy"; d.event(e);
  EXPECT_FALSE(d.permits("arm", arm)); EXPECT_TRUE(d.permits("gripper", grip));
  EXPECT_EQ(d.events().front().code, "0x1234");
  EXPECT_EQ(d.events().front().recovery, e.recovery);
}
TEST_F(GroupDomain, DiagnosticsEscapeControlCharactersAndIncludeFaultScope) {
  readyAll(); auto e = failure("left"); e.reason = "a\n\"b\\c\t"; d.event(e);
  auto text = d.diagnostics(); EXPECT_EQ(text.find('\n'), std::string::npos);
  EXPECT_NE(text.find("a\\u000a\\\"b\\\\c\\u0009"), std::string::npos);
  EXPECT_NE(text.find("\"blockers\":[\"failure\"]"), std::string::npos);
  EXPECT_NE(text.find("\"configuration_version\":\"test-v1\""), std::string::npos);
}
TEST_F(GroupDomain, MultipleFaultsCannotBeClearedByAcknowledgingOne) {
  readyAll(); start("base"); d.event(failure("left", "first")); d.event(failure("right", "second"));
  d.acknowledge("first"); EXPECT_FALSE(d.arm("base"));
  d.acknowledge("second"); EXPECT_TRUE(d.arm("base"));
}
TEST_F(GroupDomain, LegacyBackendReadinessLossCannotLeaveGroupSessionsOpen) {
  readyAll(); auto base = start("base"); d.report("RobotCanopenSystem", false);
  EXPECT_FALSE(d.permits("base", base)); EXPECT_TRUE(d.globallyInhibited());
}
TEST_F(GroupDomain, DiagnosticsExpressPartialRunningAndUnknownEvidence) {
  readyAll(); start("base"); auto text = d.diagnostics();
  EXPECT_NE(text.find("\"aggregate_state\":\"PartialRunning\""), std::string::npos);
  EXPECT_NE(text.find("\"required_source\":\"brake\",\"satisfied\":false"), std::string::npos);
}
TEST_F(GroupDomain, ConfiguredReadinessIsNeverAcceptedAsDeviceFeedback) {
  for (auto & a : axes) a.second.ready = true;
  d.configure(axes, rules, "test-v2"); EXPECT_FALSE(d.arm("base"));
}
TEST_F(GroupDomain, ConcurrentDeviceReportsNeverInvalidateIndependentSession) {
  readyAll(); auto base = start("base"), grip = start("gripper");
  std::atomic<bool> healthy{true}; std::vector<std::thread> reporters;
  for (int i = 0; i < 4; ++i) reporters.emplace_back([&] {
    for (int n = 0; n < 100; ++n) {
      d.reportAxis("left", false, "offline");
      if (!d.permits("gripper", grip)) healthy = false;
    }
  });
  for (auto & thread : reporters) thread.join();
  EXPECT_TRUE(healthy); EXPECT_EQ(d.epoch("gripper"), grip); EXPECT_EQ(d.epoch("base"), base + 1);
}
TEST_F(GroupDomain, RestoredBusCannotInventReadyAxisFeedback) {
  readyAll(); start("arm"); d.reportBackend("canopen", false, "offline");
  d.reportBackend("canopen", true, "transport restored");
  EXPECT_FALSE(d.group("arm").ready);
  EXPECT_THROW(d.acknowledge("backend:canopen"), std::logic_error);
  for (const auto & a : axes) if (a.second.backend == "canopen") d.reportAxis(a.first, true);
  EXPECT_NO_THROW(d.acknowledge("backend:canopen"));
  EXPECT_TRUE(d.arm("arm")); EXPECT_FALSE(d.permits("arm", d.epoch("arm")));
}
TEST_F(GroupDomain, EvidenceExpiryRevokesEpochOnPermissionCheck) {
  readyAll(); auto base = start("base");
  d.condition("secured", D::Evidence::Satisfied, "brake", now + 10);
  d.event(failure("shoulder")); EXPECT_TRUE(d.permits("base", base));
  now += 10; EXPECT_FALSE(d.permits("base", base)); EXPECT_NE(d.epoch("base"), base);
}
TEST_F(GroupDomain, InvalidEventContractCannotEnterTheEventRegistry) {
  auto e = failure("left"); e.severity = static_cast<D::Severity>(999);
  EXPECT_THROW(d.event(e), std::invalid_argument); EXPECT_TRUE(d.events().empty());
  e = failure("left"); e.timestamp_ns = -1; EXPECT_THROW(d.event(e), std::invalid_argument);
  e = failure("left", "readiness:left"); EXPECT_THROW(d.event(e), std::invalid_argument);
}
TEST_F(GroupDomain, MixedLegacyAndGroupedWritersCannotOpenSeparateGates) {
  EXPECT_THROW(d.registerWriter("legacy",false),std::invalid_argument);
  EXPECT_NO_THROW(d.registerWriter("private",true));EXPECT_FALSE(d.arm());
  D legacy;legacy.registerWriter("private",false);
  EXPECT_THROW(legacy.configure(axes,rules,"bad"),std::logic_error);
}
TEST_F(GroupDomain, CancellationFencesPreparationEvenWhenAlreadyStopped) {
  auto old=d.epoch("flippers");d.invalidate("flippers");EXPECT_NE(old,d.epoch("flippers"));
}
TEST_F(GroupDomain, FeedbackWarningsCountEpisodesWithoutInvalidatingSessions) {
 readyAll();auto base=start("base");
 d.warning("left",true,"delay");d.warning("left",true,"delay");
 EXPECT_TRUE(d.permits("base",base));EXPECT_EQ(d.events().size(),1u);
 EXPECT_NE(d.diagnostics().find("\"warning_count\":1"),std::string::npos);
 d.warning("left",false,"");EXPECT_TRUE(d.events().empty());
 d.warning("left",true,"delay");EXPECT_NE(d.diagnostics().find("\"warning_count\":2"),std::string::npos);
}
TEST_F(GroupDomain, RuntimeOwnerLeaseExpiresGloballyAndRenewalCannotReplayOldSessions) {
 readyAll();d.requireOwnerLease();EXPECT_FALSE(d.arm("base"));d.renewOwner(100);auto old=start("base");
 now+=100;EXPECT_FALSE(d.permits("base",old));EXPECT_TRUE(d.globallyInhibited());
 d.renewOwner(100);EXPECT_FALSE(d.arm("base"));ASSERT_TRUE(d.resetGlobalStop());
 ASSERT_TRUE(d.arm("base"));ASSERT_TRUE(d.run("base"));EXPECT_FALSE(d.permits("base",old));
 EXPECT_THROW(d.renewOwner(100,now-100),std::invalid_argument);
 EXPECT_THROW(d.renewOwner(100,now+1),std::invalid_argument);
}
