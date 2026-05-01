#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
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
#include <functional>
#include <string>
#include <mutex>
#include <map>

class RosHandler {
public:
    using JsonCallback = std::function<void(const std::string&)>;

    explicit RosHandler(ros::NodeHandle& nh);
    ~RosHandler();

    void setJsonCallback(JsonCallback callback);
    void handleJsonCommand(const std::string& json_message);
    void onClientConnected();

private:
    // 订阅回调（下位机 → 上位机）
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void executorStateCallback(const tcp_bridge::ExecutorState::ConstPtr& msg);
    void jointDataCallback(const tcp_bridge::JointData::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void co2Callback(const std_msgs::Float32::ConstPtr& msg);
    void cameraInfoCallback(const tcp_bridge::CameraInfo::ConstPtr& msg);
    void systemStatusCallback(const tcp_bridge::SystemStatus::ConstPtr& msg);

    // 发布（上位机 → 下位机）
    void publishCmdVel(const std::string& json);
    void publishEmergencyStop();
    void publishJointCommand(const std::string& json);
    void publishCartesianCommand(const std::string& json);
    void publishMotorCommand(const std::string& json);
    void publishControlCommand(const std::string& json);
    void publishSystemCommand(const std::string& json);
    void heartbeatCheckCallback(const ros::TimerEvent& event);



    ros::NodeHandle& nh_;

    // Subscribers
    ros::Subscriber joint_state_sub_;
    ros::Subscriber executor_state_sub_;
    ros::Subscriber joint_data_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber co2_sub_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber system_status_sub_;

    // Publishers
    ros::Publisher cmd_vel_pub_;
    ros::Publisher emergency_stop_pub_;
    ros::Publisher joint_command_pub_;
    ros::Publisher cartesian_command_pub_;
    ros::Publisher motor_command_pub_;
    ros::Publisher control_command_pub_;
    ros::Publisher system_command_pub_;

    JsonCallback json_callback_;

    sensor_msgs::JointState last_joint_state_;
    tcp_bridge::ExecutorState last_executor_state_;
    std::map<uint8_t, tcp_bridge::CameraInfo> camera_info_cache_;
    ros::Timer heartbeat_check_timer_;
    ros::Time last_heartbeat_time_;
    double heartbeat_timeout_sec_;
    bool heartbeat_seen_;
    bool heartbeat_timed_out_;
    std::mutex state_mutex_;
};
