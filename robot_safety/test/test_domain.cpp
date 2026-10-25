#include <gtest/gtest.h>
#include "robot_safety/domain.hpp"
TEST(Domain, BothBackendsRequiredAndFaultNeverAutoResumes) {
 robot_safety::Domain d;
 EXPECT_FALSE(d.arm()); d.report("RobotCanopenSystem", true); EXPECT_FALSE(d.arm());
 d.report("CanDriverSystem", true); ASSERT_TRUE(d.arm()); auto old = d.epoch();
 EXPECT_FALSE(d.permits(old)); ASSERT_TRUE(d.run()); EXPECT_TRUE(d.permits(old));
 d.report("CanDriverSystem", false); EXPECT_FALSE(d.permits(old));
 d.report("CanDriverSystem", true); EXPECT_FALSE(d.run()); EXPECT_FALSE(d.permits(old));
 ASSERT_TRUE(d.arm()); ASSERT_TRUE(d.run()); EXPECT_FALSE(d.permits(old));
 d.halt(); EXPECT_FALSE(d.permits(d.epoch()));
}
TEST(Domain, FaultIsSharedAcrossPluginOwners) {
 auto a = robot_safety::domain("test"); auto b = robot_safety::domain("test");
 a->report("RobotCanopenSystem", true); a->report("CanDriverSystem", true);
 ASSERT_TRUE(a->arm()); ASSERT_TRUE(a->run()); auto epoch = a->epoch();
 b->fault("CANopen heartbeat"); EXPECT_FALSE(a->permits(epoch));
 EXPECT_EQ(a->reason(), "CANopen heartbeat"); EXPECT_FALSE(a->run());
}
TEST(Domain, RecordsRunningPeriodWithoutCountingArmedAsMotion) {
 robot_safety::Domain d;d.observePeriod(5000000);EXPECT_NE(d.diagnostics().find("\"samples\":0"),std::string::npos);
 d.report("RobotCanopenSystem",true);d.report("CanDriverSystem",true);ASSERT_TRUE(d.arm());ASSERT_TRUE(d.run());
 d.observePeriod(5000000);d.observePeriod(10000000);
 EXPECT_NE(d.diagnostics().find("\"period_over_7_5ms\":1"),std::string::npos);
}
