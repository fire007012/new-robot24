#include <gtest/gtest.h>
#include "robot_hardware/velocity_watchdog.hpp"
TEST(Watchdog, TimeoutInvalidInputAndOldActivationAreZero) {
 robot_hardware::VelocityWatchdog w(2); auto e=w.reset();
 EXPECT_EQ(w.sample(1,.25), std::vector<double>({0,0}));
 ASSERT_TRUE(w.accept({1,-1},1,e)); EXPECT_EQ(w.sample(1.1,.25),std::vector<double>({1,-1}));
 EXPECT_EQ(w.sample(1.3,.25),std::vector<double>({0,0}));
 EXPECT_FALSE(w.accept({NAN,1},2,e)); EXPECT_EQ(w.sample(2,.25),std::vector<double>({0,0}));
 auto next=w.reset(); EXPECT_FALSE(w.accept({1,1},3,e)); ASSERT_TRUE(w.accept({.5,.5},3,next));
 EXPECT_EQ(w.sample(2.9,.25),std::vector<double>({0,0}));
 EXPECT_FALSE(w.accept({1},3,next)); EXPECT_EQ(w.sample(3,.25),std::vector<double>({0,0}));
}
