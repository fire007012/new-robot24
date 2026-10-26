#include <gtest/gtest.h>

#include <ros/master.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "Eyou_ROS1_Master/hybrid_mode_router.hpp"

namespace {

class HybridModeRouterConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!ros::master::check()) {
            GTEST_SKIP() << "test_hybrid_mode_router requires a running ROS master";
        }
    }

    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_mode_router",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler |
                          ros::init_options::NoRosout);
        }
    }
};

canopen_hw::CanopenMasterConfig MakeCanopenConfig() {
    canopen_hw::CanopenMasterConfig config;
    canopen_hw::CanopenMasterConfig::JointConfig joint;
    joint.name = "shoulder_yaw_joint";
    config.joints.push_back(joint);
    return config;
}

XmlRpc::XmlRpcValue MakeCanDriverJointList() {
    XmlRpc::XmlRpcValue joints;
    joints.setSize(1);
    joints[0]["name"] = std::string("elbow_pitch_joint");
    joints[0]["motor_id"] = 0x13;
    return joints;
}

TEST_F(HybridModeRouterConfigTest, LoadsModeMappingsFromParams) {
    ros::NodeHandle pnh("~");
    pnh.setParam("joint_mode_mappings/modes/position/canopen", 8);
    pnh.setParam("joint_mode_mappings/modes/position/can_driver", 0);
    pnh.setParam("joint_mode_mappings/modes/velocity/canopen", 9);
    pnh.setParam("joint_mode_mappings/modes/velocity/can_driver", 1);

    std::map<std::string, eyou_ros1_master::HybridModeMapping> mappings;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::LoadHybridModeMappings(pnh, &mappings, &error));
    EXPECT_TRUE(error.empty());
    ASSERT_EQ(mappings.size(), 2u);
    EXPECT_EQ(mappings["position"].canopen_mode, 8);
    EXPECT_EQ(mappings["position"].can_driver_mode, 0);
    EXPECT_EQ(mappings["velocity"].canopen_mode, 9);
    EXPECT_EQ(mappings["velocity"].can_driver_mode, 1);
}

TEST(HybridModeRouterTargets, BuildsTargetsForBothBackends) {
    auto canopen_config = MakeCanopenConfig();
    auto can_driver_joints = MakeCanDriverJointList();

    std::map<std::string, eyou_ros1_master::HybridModeJointTarget> targets;
    std::string error;
    ASSERT_TRUE(eyou_ros1_master::BuildHybridModeJointTargets(
        canopen_config, can_driver_joints, &targets, &error));
    EXPECT_TRUE(error.empty());
    ASSERT_EQ(targets.size(), 2u);
    EXPECT_EQ(targets["shoulder_yaw_joint"].backend,
              eyou_ros1_master::HybridModeJointTarget::Backend::kCanopen);
    EXPECT_EQ(targets["shoulder_yaw_joint"].axis_index, 0u);
    EXPECT_EQ(targets["elbow_pitch_joint"].backend,
              eyou_ros1_master::HybridModeJointTarget::Backend::kCanDriver);
    EXPECT_EQ(targets["elbow_pitch_joint"].motor_id, 0x13u);
}

TEST(HybridModeRouterTargets, RejectsDuplicateJointNamesAcrossBackends) {
    auto canopen_config = MakeCanopenConfig();
    XmlRpc::XmlRpcValue joints;
    joints.setSize(1);
    joints[0]["name"] = std::string("shoulder_yaw_joint");
    joints[0]["motor_id"] = 0x13;

    std::map<std::string, eyou_ros1_master::HybridModeJointTarget> targets;
    std::string error;
    EXPECT_FALSE(eyou_ros1_master::BuildHybridModeJointTargets(
        canopen_config, joints, &targets, &error));
    EXPECT_NE(error.find("duplicate joint name"), std::string::npos);
}

}  // namespace
