#include "tcp_bridge/json_converter.h"
#include <ros/ros.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

// ── 下位机 → 上位机 ──────────────────────────────────────────

std::string JsonConverter::motorStateToJson(
    const sensor_msgs::JointState& joint_state,
    const tcp_bridge::ExecutorState& executor_state)
{
    json joints = json::array();
    for (size_t i = 0; i < 6; ++i) {
        float pos = (i < joint_state.position.size()) ? joint_state.position[i] : 0.0f;
        float cur = (i < joint_state.effort.size())   ? joint_state.effort[i]   : 0.0f;
        joints.push_back({{"position", pos}, {"current", cur}});
    }

    return json{
        {"type", "motor_state"},
        {"joints", joints},
        {"executor_position", static_cast<float>(executor_state.position) / 1000.0f},
        {"executor_torque",   static_cast<float>(executor_state.torque) / 1000.0f},
        {"executor_flags",    executor_state.flags},
        {"reserved",          0}
    }.dump();
}

std::string JsonConverter::jointDataToJson(const tcp_bridge::JointData& d)
{
    return json{
        {"type",     "joint_data"},
        {"joint_id", d.joint_id},
        {"position", d.position},
        {"current",  d.current},
        {"torque",   d.torque}
    }.dump();
}

std::string JsonConverter::imuDataToJson(const sensor_msgs::Imu& imu)
{
    // 四元数转欧拉角 (roll/pitch/yaw)
    double x = imu.orientation.x, y = imu.orientation.y;
    double z = imu.orientation.z, w = imu.orientation.w;
    double roll  = std::atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
    double pitch = std::asin(std::max(-1.0, std::min(1.0, 2*(w*y - z*x))));
    double yaw   = std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));

    return json{
        {"type",    "imu_data"},
        {"roll",    roll},
        {"pitch",   pitch},
        {"yaw",     yaw},
        {"accel_x", imu.linear_acceleration.x},
        {"accel_y", imu.linear_acceleration.y},
        {"accel_z", imu.linear_acceleration.z}
    }.dump();
}

std::string JsonConverter::co2DataToJson(const std_msgs::Float32& co2)
{
    return json{{"type", "co2_data"}, {"ppm", co2.data}}.dump();
}

std::string JsonConverter::cameraInfoToJson(const tcp_bridge::CameraInfo& c)
{
    return json{
        {"type",         "camera_info"},
        {"camera_id",    c.camera_id},
        {"online",       c.online},
        {"codec",        c.codec},
        {"width",        c.width},
        {"height",       c.height},
        {"fps",          c.fps},
        {"bitrate_kbps", c.bitrate_kbps},
        {"rtsp_url",     c.rtsp_url}
    }.dump();
}

std::string JsonConverter::systemStatusToJson(const tcp_bridge::SystemStatus& s)
{
    return json{
        {"type",          "system_status"},
        {"cpu_usage",     s.cpu_usage},
        {"memory_usage",  s.memory_usage},
        {"uptime",        s.uptime},
        {"ros_status",    s.ros_status}
    }.dump();
}

// ── 上位机 → 下位机 ──────────────────────────────────────────

geometry_msgs::Twist JsonConverter::jsonToCmdVel(const std::string& json_str)
{
    geometry_msgs::Twist msg;
    try {
        json j = json::parse(json_str);
        msg.linear.x  = j.value("linear_x",  0.0);
        msg.linear.y  = j.value("linear_y",  0.0);
        msg.angular.z = j.value("angular_z", 0.0);
    } catch (const std::exception& e) {
        ROS_ERROR("jsonToCmdVel error: %s", e.what());
    }
    return msg;
}

tcp_bridge::JointCommand JsonConverter::jsonToJointCommand(const std::string& json_str)
{
    tcp_bridge::JointCommand msg;
    try {
        json j = json::parse(json_str);
        msg.header.stamp = ros::Time::now();
        msg.joint_id  = j.value("joint_id",  0);
        msg.position  = j.value("position",  0.0f);
        msg.velocity  = j.value("velocity",  0.0f);
    } catch (const std::exception& e) {
        ROS_ERROR("jsonToJointCommand error: %s", e.what());
    }
    return msg;
}

tcp_bridge::CartesianCommand JsonConverter::jsonToCartesianCommand(const std::string& json_str)
{
    tcp_bridge::CartesianCommand msg;
    try {
        json j = json::parse(json_str);
        msg.header.stamp = ros::Time::now();
        msg.x     = j.value("x",     0.0f);
        msg.y     = j.value("y",     0.0f);
        msg.z     = j.value("z",     0.0f);
        msg.roll  = j.value("roll",  0.0f);
        msg.pitch = j.value("pitch", 0.0f);
        msg.yaw   = j.value("yaw",   0.0f);
    } catch (const std::exception& e) {
        ROS_ERROR("jsonToCartesianCommand error: %s", e.what());
    }
    return msg;
}

tcp_bridge::MotorCommand JsonConverter::jsonToMotorCommand(const std::string& json_str)
{
    tcp_bridge::MotorCommand cmd;
    try {
        json j = json::parse(json_str);
        cmd.header.stamp = ros::Time::now();

        auto& joints = j["joints"];
        for (size_t i = 0; i < joints.size() && i < 6; ++i) {
            cmd.joint_ids.push_back(i);
            cmd.target_positions.push_back(
                static_cast<int16_t>(joints[i].value("position", 0.0f) * 1000));
            cmd.target_torques.push_back(
                static_cast<int16_t>(joints[i].value("current", 0.0f) * 1000));
        }

        cmd.executor_target_position = static_cast<int16_t>(
            j.value("executor_position", 0.0f) * 1000);
        cmd.executor_target_torque = static_cast<int16_t>(
            j.value("executor_torque", 0.0f) * 1000);
        cmd.command_flags = j.value("executor_flags", 0);
    } catch (const std::exception& e) {
        ROS_ERROR("jsonToMotorCommand error: %s", e.what());
    }
    return cmd;
}

tcp_bridge::ControlCommand JsonConverter::jsonToControlCommand(const std::string& json_str)
{
    tcp_bridge::ControlCommand msg;
    try {
        json j = json::parse(json_str);
        msg.header.stamp = ros::Time::now();

        auto& imu = j["imu"];
        msg.imu_yaw   = imu.value("yaw",   0.0f);
        msg.imu_roll  = imu.value("roll",  0.0f);
        msg.imu_pitch = imu.value("pitch", 0.0f);

        auto& sw = j["swing_arm_current"];
        for (int i = 0; i < 4; ++i)
            msg.swing_arm_current[i] = (i < (int)sw.size()) ? sw[i].get<float>() : 0.0f;

        auto& arm = j["arm_end_position"];
        msg.arm_x     = arm.value("x",     0.0f);
        msg.arm_y     = arm.value("y",     0.0f);
        msg.arm_z     = arm.value("z",     0.0f);
        msg.arm_roll  = arm.value("roll",  0.0f);
        msg.arm_pitch = arm.value("pitch", 0.0f);
        msg.arm_yaw   = arm.value("yaw",   0.0f);

        msg.command_flags = j.value("command_flags", 0);
    } catch (const std::exception& e) {
        ROS_ERROR("jsonToControlCommand error: %s", e.what());
    }
    return msg;
}

tcp_bridge::SystemCommand JsonConverter::jsonToSystemCommand(const std::string& json_str)
{
    tcp_bridge::SystemCommand msg;
    try {
        json j = json::parse(json_str);
        msg.header.stamp = ros::Time::now();
        msg.command = j.value("command", "");
        msg.params  = j.contains("params") ? j["params"].dump() : "{}";
    } catch (const std::exception& e) {
        ROS_ERROR("jsonToSystemCommand error: %s", e.what());
    }
    return msg;
}

// ── Utility ──────────────────────────────────────────────────

bool JsonConverter::validateJson(const std::string& json_str)
{
    return json::accept(json_str);
}

std::string JsonConverter::getMessageType(const std::string& json_str)
{
    try {
        return json::parse(json_str).value("type", "unknown");
    } catch (...) {
        return "invalid";
    }
}

int64_t JsonConverter::getCurrentTimestamp()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
