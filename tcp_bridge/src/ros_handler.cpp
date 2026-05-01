#include "tcp_bridge/ros_handler.h"
#include "tcp_bridge/json_converter.h"
#include "tcp_bridge/message_types.h"
#include <ros/ros.h>

RosHandler::RosHandler(ros::NodeHandle& nh) : nh_(nh) {
    heartbeat_timeout_sec_ = nh_.param("heartbeat_timeout_sec", 3.0);
    heartbeat_seen_ = false;
    heartbeat_timed_out_ = false;
    heartbeat_check_timer_ = nh_.createTimer(
        ros::Duration(1.0), &RosHandler::heartbeatCheckCallback, this);

    // 订阅（下位机 → 上位机）
    joint_state_sub_   = nh_.subscribe("/joint_states",   10, &RosHandler::jointStateCallback,    this);
    executor_state_sub_= nh_.subscribe("/executor_state", 10, &RosHandler::executorStateCallback, this);
    joint_data_sub_    = nh_.subscribe("/joint_data",     10, &RosHandler::jointDataCallback,     this);
    imu_sub_           = nh_.subscribe("/imu/data",       10, &RosHandler::imuCallback,           this);
    co2_sub_           = nh_.subscribe("/co2_sensor",     10, &RosHandler::co2Callback,           this);
    camera_info_sub_   = nh_.subscribe("/camera_info",    10, &RosHandler::cameraInfoCallback,    this);
    system_status_sub_ = nh_.subscribe("/system_status",  10, &RosHandler::systemStatusCallback,  this);

    // 发布（上位机 → 下位机）
    cmd_vel_pub_           = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    emergency_stop_pub_    = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1);
    joint_command_pub_     = nh_.advertise<tcp_bridge::JointCommand>("/joint_command", 10);
    cartesian_command_pub_ = nh_.advertise<tcp_bridge::CartesianCommand>("/cartesian_command", 10);
    motor_command_pub_     = nh_.advertise<tcp_bridge::MotorCommand>("/motor_command", 10);
    control_command_pub_   = nh_.advertise<tcp_bridge::ControlCommand>("/control_command", 10);
    system_command_pub_    = nh_.advertise<tcp_bridge::SystemCommand>("/system_command", 10);

    ROS_INFO("RosHandler initialized");
}

RosHandler::~RosHandler() {}

void RosHandler::setJsonCallback(JsonCallback callback) {
    json_callback_ = std::move(callback);
}

void RosHandler::onClientConnected() {
    std::vector<tcp_bridge::CameraInfo> cached_cameras;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto& entry : camera_info_cache_) {
            cached_cameras.push_back(entry.second);
        }
    }

    for (const auto& camera_info : cached_cameras) {
        if (json_callback_) {
            json_callback_(JsonConverter::cameraInfoToJson(camera_info));
        }
    }

    ROS_INFO("Client connected, resent %zu cached camera_info messages", cached_cameras.size());
}

// ── 订阅回调 ─────────────────────────────────────────────────

void RosHandler::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_joint_state_ = *msg;
    if (json_callback_)
        json_callback_(JsonConverter::motorStateToJson(last_joint_state_, last_executor_state_));
}

void RosHandler::executorStateCallback(const tcp_bridge::ExecutorState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_executor_state_ = *msg;
    if (json_callback_)
        json_callback_(JsonConverter::motorStateToJson(last_joint_state_, last_executor_state_));
}

void RosHandler::jointDataCallback(const tcp_bridge::JointData::ConstPtr& msg) {
    if (json_callback_)
        json_callback_(JsonConverter::jointDataToJson(*msg));
}

void RosHandler::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (json_callback_)
        json_callback_(JsonConverter::imuDataToJson(*msg));
}

void RosHandler::co2Callback(const std_msgs::Float32::ConstPtr& msg) {
    if (json_callback_)
        json_callback_(JsonConverter::co2DataToJson(*msg));
}

void RosHandler::cameraInfoCallback(const tcp_bridge::CameraInfo::ConstPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        camera_info_cache_[msg->camera_id] = *msg;
    }

    if (json_callback_)
        json_callback_(JsonConverter::cameraInfoToJson(*msg));
}

void RosHandler::systemStatusCallback(const tcp_bridge::SystemStatus::ConstPtr& msg) {
    if (json_callback_)
        json_callback_(JsonConverter::systemStatusToJson(*msg));
}

// ── 接收上位机指令 ────────────────────────────────────────────

void RosHandler::handleJsonCommand(const std::string& json_message) {
    if (!JsonConverter::validateJson(json_message)) {
        ROS_WARN("Invalid JSON: %s", json_message.c_str());
        return;
    }

    std::string msg_type = JsonConverter::getMessageType(json_message);
    ROS_INFO("[CMD] type=%s  raw=%s", msg_type.c_str(), json_message.c_str());

    if      (msg_type == MessageTypes::CMD_VEL)         publishCmdVel(json_message);
    else if (msg_type == MessageTypes::EMERGENCY_STOP)  publishEmergencyStop();
    else if (msg_type == MessageTypes::JOINT_CONTROL)   publishJointCommand(json_message);
    else if (msg_type == MessageTypes::CARTESIAN_CTRL)  publishCartesianCommand(json_message);
    else if (msg_type == MessageTypes::MOTOR_COMMAND)   publishMotorCommand(json_message);
    else if (msg_type == MessageTypes::CONTROL_COMMAND) publishControlCommand(json_message);
    else if (msg_type == MessageTypes::SYSTEM_COMMAND)  publishSystemCommand(json_message);
    else if (msg_type == MessageTypes::HEARTBEAT) {
        bool recovered = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            recovered = heartbeat_timed_out_;
            last_heartbeat_time_ = ros::Time::now();
            heartbeat_seen_ = true;
            heartbeat_timed_out_ = false;
        }

        if (recovered) {
            ROS_INFO("Heartbeat restored");
        } else {
            ROS_DEBUG("Heartbeat received");
        }
    }
    else    ROS_WARN("Unknown message type: %s", msg_type.c_str());
}

// ── 发布函数 ──────────────────────────────────────────────────

void RosHandler::publishCmdVel(const std::string& json) {
    cmd_vel_pub_.publish(JsonConverter::jsonToCmdVel(json));
}

void RosHandler::publishEmergencyStop() {
    ROS_WARN("EMERGENCY STOP received!");
    // 立即发布零速度
    geometry_msgs::Twist zero;
    cmd_vel_pub_.publish(zero);
    // 发布急停信号
    std_msgs::Bool stop;
    stop.data = true;
    emergency_stop_pub_.publish(stop);
}

void RosHandler::publishJointCommand(const std::string& json) {
    joint_command_pub_.publish(JsonConverter::jsonToJointCommand(json));
}

void RosHandler::publishCartesianCommand(const std::string& json) {
    cartesian_command_pub_.publish(JsonConverter::jsonToCartesianCommand(json));
}

void RosHandler::publishMotorCommand(const std::string& json) {
    motor_command_pub_.publish(JsonConverter::jsonToMotorCommand(json));
}

void RosHandler::publishControlCommand(const std::string& json) {
    control_command_pub_.publish(JsonConverter::jsonToControlCommand(json));
}

void RosHandler::publishSystemCommand(const std::string& json) {
    system_command_pub_.publish(JsonConverter::jsonToSystemCommand(json));
}

void RosHandler::heartbeatCheckCallback(const ros::TimerEvent& /*event*/) {
    bool should_stop = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!heartbeat_seen_ || heartbeat_timed_out_) {
            return;
        }

        if ((ros::Time::now() - last_heartbeat_time_).toSec() > heartbeat_timeout_sec_) {
            heartbeat_timed_out_ = true;
            should_stop = true;
        }
    }

    if (should_stop) {
        ROS_WARN("Heartbeat timeout exceeded %.1f seconds, triggering safety stop",
                 heartbeat_timeout_sec_);
        publishEmergencyStop();
    }
}
