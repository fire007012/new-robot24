#include <gtest/gtest.h>

#include <string>

#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"
#include "canopen_hw/shared_state.hpp"

namespace {

can_driver::OperationalCoordinator::DriverOps MakeCanDriverOps(bool init_ok = true,
                                                               bool enable_ok = true,
                                                               bool disable_ok = true,
                                                               bool halt_ok = true,
                                                               bool recover_ok = true,
                                                               bool shutdown_ok = true) {
    can_driver::OperationalCoordinator::DriverOps ops;
    ops.init_device = [init_ok](const std::string&, bool) {
        return can_driver::OperationalCoordinator::Result{
            init_ok, init_ok ? "initialized" : "init failed"};
    };
    ops.enable_all = [enable_ok]() {
        return can_driver::OperationalCoordinator::Result{
            enable_ok, enable_ok ? "enabled" : "enable failed"};
    };
    ops.disable_all = [disable_ok]() {
        return can_driver::OperationalCoordinator::Result{
            disable_ok, disable_ok ? "disabled" : "disable failed"};
    };
    ops.halt_all = [halt_ok]() {
        return can_driver::OperationalCoordinator::Result{
            halt_ok, halt_ok ? "halted" : "halt failed"};
    };
    ops.recover_all = [recover_ok]() {
        return can_driver::OperationalCoordinator::Result{
            recover_ok, recover_ok ? "recovered" : "recover failed"};
    };
    ops.shutdown_all = [shutdown_ok](bool) {
        return can_driver::OperationalCoordinator::Result{
            shutdown_ok, shutdown_ok ? "shutdown" : "shutdown failed"};
    };
    ops.enable_healthy = [](std::string*) { return true; };
    ops.motion_healthy = [](std::string*) { return true; };
    ops.any_fault_active = []() { return false; };
    ops.hold_commands = []() {};
    ops.arm_fresh_command_latch = []() {};
    return ops;
}

struct FakeCanopenMaster {
    bool start_ok = true;
    bool running = false;
    bool reset_ok = true;
    bool graceful_ok = true;
};

canopen_hw::OperationalCoordinator::MasterOps MakeCanopenOps(FakeCanopenMaster* master) {
    canopen_hw::OperationalCoordinator::MasterOps ops;
    ops.start = [master]() {
        if (!master->start_ok) {
            return false;
        }
        master->running = true;
        return true;
    };
    ops.running = [master]() { return master->running; };
    ops.reset_all_faults = [master](std::string* detail) {
        if (!master->reset_ok) {
            if (detail) {
                *detail = "reset failed";
            }
            return false;
        }
        return true;
    };
    ops.graceful_shutdown = [master](std::string* detail) {
        if (!master->graceful_ok) {
            if (detail) {
                *detail = "graceful shutdown failed";
            }
            return false;
        }
        return true;
    };
    ops.stop = [master]() { master->running = false; };
    return ops;
}

TEST(HybridOperationalCoordinatorTest, InitFailureRollsBackCanDriverToConfigured) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    fake_canopen.start_ok = false;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestInit("fake0", false);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[canopen]"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST(HybridOperationalCoordinatorTest, RepeatedInitSetsExplicitAlreadyFlag) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto first = hybrid.RequestInit("fake0", false);
    ASSERT_TRUE(first.ok);
    EXPECT_FALSE(first.already);
    EXPECT_EQ(first.message, "both backends initialized");

    const auto second = hybrid.RequestInit("fake0", false);
    ASSERT_TRUE(second.ok);
    EXPECT_TRUE(second.already);
    EXPECT_EQ(second.message, "already initialized");
}

TEST(HybridOperationalCoordinatorTest, ReleaseFailureRollsBackCanDriverToArmed) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Armed);

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);

    canopen_hw::AxisFeedback fb;
    fb.is_fault = true;
    shared.UpdateFeedback(0, fb);
    fake_canopen.running = false;

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestRelease();
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[canopen]"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Armed);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);
}

TEST(HybridOperationalCoordinatorTest, EnableRollbackFailureFallsBackToConfiguredShutdown) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    ASSERT_TRUE(can_coord.RequestDisable().ok);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Standby);
    can_coord.SetDriverOps(MakeCanDriverOps(true, true, false));

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    ASSERT_TRUE(canopen_coord.RequestDisable().ok);
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Standby);
    shared.SetGlobalFault(true);

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestEnable();
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[canopen]"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST(HybridOperationalCoordinatorTest, DisableRollbackFailureFallsBackToConfiguredShutdown) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    ASSERT_TRUE(can_coord.RequestRelease().ok);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Running);
    can_coord.SetDriverOps(MakeCanDriverOps(true, false, true));

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    ASSERT_TRUE(canopen_coord.RequestRelease().ok);
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Running);
    fake_canopen.running = false;

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestDisable();
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[canopen]"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST(HybridOperationalCoordinatorTest, ReleaseRollbackFailureFallsBackToConfiguredShutdown) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Armed);
    can_coord.SetDriverOps(MakeCanDriverOps(true, true, true, false));

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);

    canopen_hw::AxisFeedback fb;
    fb.is_fault = true;
    shared.UpdateFeedback(0, fb);
    fake_canopen.running = false;

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestRelease();
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[canopen]"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST(HybridOperationalCoordinatorTest, GroupedCanopenFaultDoesNotBlockHybridHalt) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    ASSERT_TRUE(can_coord.RequestRelease().ok);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Running);
    can_coord.SetDriverOps(MakeCanDriverOps(true, true, true, true, true, true));

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    ASSERT_TRUE(canopen_coord.RequestRelease().ok);
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Running);

    canopen_hw::AxisFeedback fb;
    fb.is_fault = true;
    shared.UpdateFeedback(0, fb);
    canopen_coord.UpdateFromFeedback();
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Running);
    ASSERT_TRUE(shared.GetAxisHaltedByFault(0));

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestHalt();
    EXPECT_TRUE(result.ok) << result.message;
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Armed);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);
}

TEST(HybridOperationalCoordinatorTest, RecoverFailureFallsBackToConfiguredShutdown) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    can_coord.UpdateFromFeedback(true);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Faulted);

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    fake_canopen.reset_ok = false;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    canopen_hw::AxisFeedback fb;
    fb.is_fault = true;
    shared.UpdateFeedback(0, fb);
    canopen_coord.UpdateFromFeedback();
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);
    ASSERT_TRUE(shared.GetAxisHaltedByFault(0));

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestRecover();
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[canopen]"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST(HybridOperationalCoordinatorTest, ShutdownStillAttemptsCanopenWhenCanDriverFails) {
    auto can_ops = MakeCanDriverOps();
    can_ops.shutdown_all = [](bool) {
        return can_driver::OperationalCoordinator::Result{false, "can shutdown failed"};
    };
    can_driver::OperationalCoordinator can_coord(std::move(can_ops));
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Armed);

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestShutdown(false);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[can_driver] can shutdown failed"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST(HybridOperationalCoordinatorTest, ShutdownStillAttemptsCanDriverWhenCanopenFails) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();
    ASSERT_TRUE(can_coord.RequestInit("fake0", false).ok);
    ASSERT_EQ(can_coord.mode(), can_driver::SystemOpMode::Armed);

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();
    ASSERT_TRUE(canopen_coord.RequestInit().ok);
    ASSERT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);
    fake_canopen.graceful_ok = false;

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);

    const auto result = hybrid.RequestShutdown(false);
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.message.find("[canopen] graceful shutdown failed"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

}  // namespace
