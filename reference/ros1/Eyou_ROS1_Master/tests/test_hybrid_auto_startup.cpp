#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include "Eyou_ROS1_Master/hybrid_auto_startup.hpp"
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

std::string NextNamespace() {
    static std::atomic<int> counter{0};
    return "/hybrid_auto_startup_test_" + std::to_string(counter.fetch_add(1));
}

struct HybridAutoStartupHarness {
    can_driver::OperationalCoordinator can_coord;
    canopen_hw::SharedState shared;
    FakeCanopenMaster fake_canopen;
    canopen_hw::OperationalCoordinator canopen_coord;
    eyou_ros1_master::HybridOperationalCoordinator hybrid_coord;
    std::mutex loop_mtx;
    ros::NodeHandle pnh;
    ros::NodeHandle can_driver_pnh;
    eyou_ros1_master::HybridServiceGateway gateway;
    int hook_calls = 0;

    explicit HybridAutoStartupHarness(const std::string& ns)
        : can_coord(MakeCanDriverOps()),
          shared(1),
          canopen_coord(MakeCanopenOps(&fake_canopen), &shared, 1),
          hybrid_coord(&can_coord, &canopen_coord),
          pnh(ns),
          can_driver_pnh(pnh, "can_driver_node"),
          gateway(pnh, can_driver_pnh, &hybrid_coord, &loop_mtx, false) {
        can_coord.SetConfigured();
        canopen_coord.SetConfigured();
        XmlRpc::XmlRpcValue joints;
        joints.setSize(1);
        joints[0]["name"] = "elbow_pitch_joint";
        joints[0]["motor_id"] = static_cast<int>(0x13);
        joints[0]["protocol"] = "PP";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "csp";
        joints[0]["position_scale"] = 65536;
        joints[0]["velocity_scale"] = 65536;
        can_driver_pnh.setParam("joints", joints);
        can_driver_pnh.setParam("init_loopback", false);
        gateway.SetPostInitHook([this](std::string*) {
            ++hook_calls;
            return true;
        });
    }
};

class HybridAutoStartupTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_auto_startup",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler |
                          ros::init_options::NoRosout);
        }
    }
};

TEST_F(HybridAutoStartupTest, AutoInitFalseDoesNotAdvanceLifecycle) {
    HybridAutoStartupHarness harness(NextNamespace());
    harness.pnh.setParam("auto_init", false);
    harness.pnh.setParam("auto_enable", false);
    harness.pnh.setParam("auto_release", false);

    std::string error;
    EXPECT_TRUE(eyou_ros1_master::RunHybridAutoStartupFromParams(harness.gateway,
                                                                 harness.hybrid_coord,
                                                                 harness.pnh,
                                                                 &error));
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(harness.hook_calls, 0);
    EXPECT_EQ(harness.can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(harness.canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST_F(HybridAutoStartupTest, AutoInitTrueRunsUnifiedInitPathAndHook) {
    HybridAutoStartupHarness harness(NextNamespace());
    harness.pnh.setParam("auto_init", true);
    harness.pnh.setParam("auto_enable", false);
    harness.pnh.setParam("auto_release", false);

    std::string error;
    EXPECT_TRUE(eyou_ros1_master::RunHybridAutoStartupFromParams(harness.gateway,
                                                                 harness.hybrid_coord,
                                                                 harness.pnh,
                                                                 &error));
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(harness.hook_calls, 1);
    EXPECT_EQ(harness.can_coord.mode(), can_driver::SystemOpMode::Armed);
    EXPECT_EQ(harness.canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);
}

TEST_F(HybridAutoStartupTest, AutoEnableTrueLeavesBothBackendsArmed) {
    HybridAutoStartupHarness harness(NextNamespace());
    harness.pnh.setParam("auto_init", true);
    harness.pnh.setParam("auto_enable", true);
    harness.pnh.setParam("auto_release", false);

    std::string error;
    EXPECT_TRUE(eyou_ros1_master::RunHybridAutoStartupFromParams(harness.gateway,
                                                                 harness.hybrid_coord,
                                                                 harness.pnh,
                                                                 &error));
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(harness.hook_calls, 1);
    EXPECT_EQ(harness.can_coord.mode(), can_driver::SystemOpMode::Armed);
    EXPECT_EQ(harness.canopen_coord.mode(), canopen_hw::SystemOpMode::Armed);
}

TEST_F(HybridAutoStartupTest, AutoReleaseTrueDrivesBothBackendsRunning) {
    HybridAutoStartupHarness harness(NextNamespace());
    harness.pnh.setParam("auto_init", true);
    harness.pnh.setParam("auto_enable", true);
    harness.pnh.setParam("auto_release", true);

    std::string error;
    EXPECT_TRUE(eyou_ros1_master::RunHybridAutoStartupFromParams(harness.gateway,
                                                                 harness.hybrid_coord,
                                                                 harness.pnh,
                                                                 &error));
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(harness.hook_calls, 1);
    EXPECT_EQ(harness.can_coord.mode(), can_driver::SystemOpMode::Running);
    EXPECT_EQ(harness.canopen_coord.mode(), canopen_hw::SystemOpMode::Running);
}

TEST_F(HybridAutoStartupTest, InvalidAutoEnableWithoutAutoInitFailsFast) {
    HybridAutoStartupHarness harness(NextNamespace());
    harness.pnh.setParam("auto_init", false);
    harness.pnh.setParam("auto_enable", true);
    harness.pnh.setParam("auto_release", false);

    std::string error;
    EXPECT_FALSE(eyou_ros1_master::RunHybridAutoStartupFromParams(harness.gateway,
                                                                  harness.hybrid_coord,
                                                                  harness.pnh,
                                                                  &error));
    EXPECT_NE(error.find("auto_enable/auto_release require auto_init=true"),
              std::string::npos);
    EXPECT_EQ(harness.hook_calls, 0);
    EXPECT_EQ(harness.can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(harness.canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

TEST_F(HybridAutoStartupTest, InvalidAutoReleaseWithoutAutoEnableFailsFast) {
    HybridAutoStartupHarness harness(NextNamespace());
    harness.pnh.setParam("auto_init", true);
    harness.pnh.setParam("auto_enable", false);
    harness.pnh.setParam("auto_release", true);

    std::string error;
    EXPECT_FALSE(eyou_ros1_master::RunHybridAutoStartupFromParams(harness.gateway,
                                                                  harness.hybrid_coord,
                                                                  harness.pnh,
                                                                  &error));
    EXPECT_NE(error.find("auto_release requires auto_enable=true"),
              std::string::npos);
    EXPECT_EQ(harness.hook_calls, 0);
    EXPECT_EQ(harness.can_coord.mode(), can_driver::SystemOpMode::Configured);
    EXPECT_EQ(harness.canopen_coord.mode(), canopen_hw::SystemOpMode::Configured);
}

}  // namespace
