#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include <controller_manager_msgs/ListControllers.h>
#include <ros/master.h>
#include <ros/ros.h>

namespace {

class HybridLaunchSmokeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "test_hybrid_launch_smoke",
                      ros::init_options::AnonymousName |
                          ros::init_options::NoSigintHandler);
        }
        ros::Time::init();
        ASSERT_TRUE(ros::master::check())
            << "test_hybrid_launch_smoke requires a running ROS master; run it via rostest.";
    }

    static bool HasPublishedTopic(const std::string& topic_name,
                                  const std::string& topic_type) {
        ros::master::V_TopicInfo topics;
        if (!ros::master::getTopics(topics)) {
            return false;
        }
        return std::any_of(topics.begin(), topics.end(),
                           [&](const ros::master::TopicInfo& info) {
                               return info.name == topic_name &&
                                      info.datatype == topic_type;
                           });
    }
};

TEST_F(HybridLaunchSmokeTest,
       DefaultLaunchAdvertisesServicesAndStartsJtcControllers) {
    ros::NodeHandle nh;

    const ros::Duration service_timeout(15.0);
    for (const std::string& service_name : {
             "/hybrid_motor_hw_node/init",
             "/hybrid_motor_hw_node/enable",
             "/hybrid_motor_hw_node/disable",
             "/hybrid_motor_hw_node/halt",
             "/hybrid_motor_hw_node/resume",
             "/hybrid_motor_hw_node/recover",
             "/hybrid_motor_hw_node/shutdown",
             "/hybrid_motor_hw_node/set_joint_mode",
             "/hybrid_motor_hw_node/set_joint_zero",
             "/hybrid_motor_hw_node/apply_joint_limits",
             "/hybrid_motor_hw_node/can_driver_node/set_zero",
             "/hybrid_motor_hw_node/can_driver_node/apply_limits",
             "/hybrid_motor_hw_node/can_driver_node/set_zero_limit",
             "/hybrid_motor_hw_node/canopen_node/set_mode",
             "/hybrid_motor_hw_node/canopen_node/set_zero",
             "/hybrid_motor_hw_node/canopen_node/apply_limits",
             "/controller_manager/list_controllers"}) {
        EXPECT_TRUE(ros::service::waitForService(service_name, service_timeout))
            << "service did not appear: " << service_name;
    }

    ros::ServiceClient list_client =
        nh.serviceClient<controller_manager_msgs::ListControllers>(
            "/controller_manager/list_controllers", true);
    ASSERT_TRUE(list_client.waitForExistence(service_timeout));

    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(10.0);
    bool found_joint_state_controller = false;
    bool found_arm_controller = false;
    bool found_wheel_controller = false;
    while (ros::WallTime::now() < deadline) {
        controller_manager_msgs::ListControllers list_srv;
        ASSERT_TRUE(list_client.call(list_srv));

        const auto joint_state_it = std::find_if(
            list_srv.response.controller.begin(), list_srv.response.controller.end(),
            [](const controller_manager_msgs::ControllerState& state) {
                return state.name == "joint_state_controller" &&
                       state.state == "running";
            });
        found_joint_state_controller =
            joint_state_it != list_srv.response.controller.end();

        const auto arm_controller_it = std::find_if(
            list_srv.response.controller.begin(), list_srv.response.controller.end(),
            [](const controller_manager_msgs::ControllerState& state) {
                return state.name == "arm_position_controller" &&
                       state.state == "running";
            });
        found_arm_controller =
            arm_controller_it != list_srv.response.controller.end();

        const auto wheel_controller_it = std::find_if(
            list_srv.response.controller.begin(), list_srv.response.controller.end(),
            [](const controller_manager_msgs::ControllerState& state) {
                return state.name == "wheel_controller" &&
                       state.state == "running";
            });
        found_wheel_controller =
            wheel_controller_it != list_srv.response.controller.end();

        if (found_joint_state_controller && found_arm_controller &&
            found_wheel_controller) {
            break;
        }
        ros::WallDuration(0.1).sleep();
    }

    EXPECT_TRUE(found_joint_state_controller)
        << "joint_state_controller did not appear in running state within timeout";
    EXPECT_TRUE(found_arm_controller)
        << "arm_position_controller did not appear in running state within timeout";
    EXPECT_TRUE(found_wheel_controller)
        << "wheel_controller did not appear in running state within timeout";

    const ros::WallTime topic_deadline =
        ros::WallTime::now() + ros::WallDuration(10.0);
    bool found_joint_runtime_topic = false;
    while (ros::WallTime::now() < topic_deadline) {
        found_joint_runtime_topic = HasPublishedTopic(
            "/hybrid_motor_hw_node/joint_runtime_states",
            "Eyou_ROS1_Master/JointRuntimeStateArray");
        if (found_joint_runtime_topic) {
            break;
        }
        ros::WallDuration(0.1).sleep();
    }

    EXPECT_TRUE(found_joint_runtime_topic)
        << "joint runtime state topic was not advertised";
}

}  // namespace
