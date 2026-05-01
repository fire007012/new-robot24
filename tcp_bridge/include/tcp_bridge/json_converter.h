#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "tcp_bridge/MotorCommand.h"
#include "tcp_bridge/ExecutorState.h"
#include "tcp_bridge/CameraInfo.h"
#include "tcp_bridge/JointData.h"
#include "tcp_bridge/JointCommand.h"
#include "tcp_bridge/CartesianCommand.h"
#include "tcp_bridge/ControlCommand.h"
#include "tcp_bridge/SystemStatus.h"
#include "tcp_bridge/SystemCommand.h"

using json = nlohmann::json;

class JsonConverter {
public:
    // ROS → JSON (下位机发给上位机)
    static std::string motorStateToJson(
        const sensor_msgs::JointState& joint_state,
        const tcp_bridge::ExecutorState& executor_state);

    static std::string jointDataToJson(
        const tcp_bridge::JointData& joint_data);

    static std::string imuDataToJson(
        const sensor_msgs::Imu& imu_data);

    static std::string co2DataToJson(
        const std_msgs::Float32& co2);

    static std::string cameraInfoToJson(
        const tcp_bridge::CameraInfo& camera_info);

    static std::string systemStatusToJson(
        const tcp_bridge::SystemStatus& status);

    // JSON → ROS (上位机发给下位机)
    static geometry_msgs::Twist jsonToCmdVel(const std::string& json_str);
    static tcp_bridge::JointCommand jsonToJointCommand(const std::string& json_str);
    static tcp_bridge::CartesianCommand jsonToCartesianCommand(const std::string& json_str);
    static tcp_bridge::MotorCommand jsonToMotorCommand(const std::string& json_str);
    static tcp_bridge::ControlCommand jsonToControlCommand(const std::string& json_str);
    static tcp_bridge::SystemCommand jsonToSystemCommand(const std::string& json_str);

    // Utility
    static bool validateJson(const std::string& json_str);
    static std::string getMessageType(const std::string& json_str);

private:
    static int64_t getCurrentTimestamp();
};
