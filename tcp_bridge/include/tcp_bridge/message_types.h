#pragma once
#include <string>

namespace MessageTypes {
    // 下位机 → 上位机
    constexpr const char* MOTOR_STATE     = "motor_state";
    constexpr const char* JOINT_DATA      = "joint_data";
    constexpr const char* IMU_DATA        = "imu_data";
    constexpr const char* CO2_DATA        = "co2_data";
    constexpr const char* CAMERA_INFO     = "camera_info";
    constexpr const char* SYSTEM_STATUS   = "system_status";

    // 上位机 → 下位机
    constexpr const char* CMD_VEL         = "cmd_vel";
    constexpr const char* EMERGENCY_STOP  = "emergency_stop";
    constexpr const char* JOINT_CONTROL   = "joint_control";
    constexpr const char* CARTESIAN_CTRL  = "cartesian_control";
    constexpr const char* MOTOR_COMMAND   = "motor_command";
    constexpr const char* CONTROL_COMMAND = "control_command";
    constexpr const char* SYSTEM_COMMAND  = "system_command";
    constexpr const char* HEARTBEAT       = "heartbeat";
} // namespace MessageTypes
