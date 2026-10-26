#include <gtest/gtest.h>

#include <ros/ros.h>

#include <string>

#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"
#include "Eyou_ROS1_Master/hybrid_service_gateway.hpp"
#include "canopen_hw/shared_state.hpp"

namespace {

can_driver::OperationalCoordinator::DriverOps MakeCanDriverOps() {
    can_driver::OperationalCoordinator::DriverOps ops;
    ops.init_device = [](const std::string&, bool) {
        return can_driver::OperationalCoordinator::Result{true, "initialized"};
    };
    ops.enable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "enabled"};
    };
    ops.disable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "disabled"};
    };
    ops.halt_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "halted"};
    };
    ops.recover_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "recovered"};
    };
    ops.shutdown_all = [](bool) {
        return can_driver::OperationalCoordinator::Result{true, "shutdown"};
    };
    ops.enable_healthy = [](std::string*) { return true; };
    ops.motion_healthy = [](std::string*) { return true; };
    ops.any_fault_active = []() { return false; };
    ops.hold_commands = []() {};
    ops.arm_fresh_command_latch = []() {};
    return ops;
}

struct FakeCanopenMaster {
    bool running = false;
};

canopen_hw::OperationalCoordinator::MasterOps MakeCanopenOps(FakeCanopenMaster* master) {
    canopen_hw::OperationalCoordinator::MasterOps ops;
    ops.start = [master]() {
        master->running = true;
        return true;
    };
    ops.running = [master]() { return master->running; };
    ops.reset_all_faults = [](std::string*) { return true; };
    ops.graceful_shutdown = [](std::string*) { return true; };
    ops.stop = [master]() { master->running = false; };
    return ops;
}

class HybridServiceGatewayTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_service_gateway",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler |
                          ros::init_options::NoRosout);
        }
    }
};

TEST_F(HybridServiceGatewayTest, RunInitSequenceExecutesHookOnceAndPreservesAlreadyInitialized) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);
    std::mutex loop_mtx;
    ros::NodeHandle pnh("~");
    ros::NodeHandle can_driver_pnh(pnh, "can_driver_node");
    eyou_ros1_master::HybridServiceGateway gateway(
        pnh, can_driver_pnh, &hybrid, &loop_mtx, false);

    int hook_calls = 0;
    gateway.SetPostInitHook([&hook_calls](std::string*) {
        ++hook_calls;
        return true;
    });

    std::string message;
    bool already = false;
    EXPECT_TRUE(gateway.RunInitSequence("fake0", false, &message, &already));
    EXPECT_FALSE(already);
    EXPECT_EQ(message, "initialized (armed)");
    EXPECT_EQ(hook_calls, 1);

    EXPECT_TRUE(gateway.RunInitSequence("fake0", false, &message, &already));
    EXPECT_TRUE(already);
    EXPECT_EQ(message, "already initialized");
    EXPECT_EQ(hook_calls, 1);
}

TEST_F(HybridServiceGatewayTest, HookFailureTriggersShutdownRollback) {
    can_driver::OperationalCoordinator can_coord(MakeCanDriverOps());
    can_coord.SetConfigured();

    canopen_hw::SharedState shared(1);
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1);
    canopen_coord.SetConfigured();

    eyou_ros1_master::HybridOperationalCoordinator hybrid(&can_coord, &canopen_coord);
    std::mutex loop_mtx;
    ros::NodeHandle pnh("~");
    ros::NodeHandle can_driver_pnh(pnh, "can_driver_node");
    eyou_ros1_master::HybridServiceGateway gateway(
        pnh, can_driver_pnh, &hybrid, &loop_mtx, false);

    gateway.SetPostInitHook([](std::string* detail) {
        if (detail) {
            *detail = "soft limit apply failed";
        }
        return false;
    });

    std::string message;
    bool already = false;
    EXPECT_FALSE(gateway.RunInitSequence("fake0", false, &message, &already));
    EXPECT_FALSE(already);
    EXPECT_NE(message.find("soft limit apply failed"), std::string::npos);
    EXPECT_EQ(can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

}  // namespace
