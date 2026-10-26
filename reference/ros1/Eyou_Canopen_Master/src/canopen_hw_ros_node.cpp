#include <chrono>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <controller_manager/controller_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "canopen_hw/canopen_aux_services.hpp"
#include "canopen_hw/canopen_robot_hw_ros.hpp"
#include "canopen_hw/canopen_startup_sequence.hpp"
#include "canopen_hw/controllers/ip_follow_joint_trajectory_executor.hpp"
#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/lifecycle_manager.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/service_gateway.hpp"

namespace {

std::string MakeAbsolutePath(const std::string& path) {
  if (path.empty()) return path;
  std::filesystem::path p(path);
  return p.is_absolute() ? p.string() : std::filesystem::absolute(p).string();
}

bool FileExists(const std::string& path) {
  return !path.empty() && std::filesystem::exists(path);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "canopen_hw_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 从 ROS 参数读取配置文件路径。
  std::string dcf_path, joints_path;
  pnh.param<std::string>("dcf_path", dcf_path, "config/master.dcf");
  pnh.param<std::string>("joints_path", joints_path, "config/joints.yaml");
  dcf_path = MakeAbsolutePath(dcf_path);
  joints_path = MakeAbsolutePath(joints_path);

  if (!FileExists(dcf_path)) {
    CANOPEN_LOG_ERROR("master_dcf_path not found: {}", dcf_path);
    return 1;
  }
  if (!FileExists(joints_path)) {
    CANOPEN_LOG_ERROR("joints.yaml not found: {}", joints_path);
    return 1;
  }

  // 解析配置（需要 joint names 来构造 ROS adapter）。
  canopen_hw::CanopenMasterConfig master_cfg;
  master_cfg.master_dcf_path = dcf_path;

  std::string error;
  if (!canopen_hw::LoadJointsYaml(joints_path, &error, &master_cfg)) {
    CANOPEN_LOG_ERROR("Load joints.yaml failed: {}", error);
    return 1;
  }

  std::vector<std::string> joint_names;
  joint_names.reserve(master_cfg.joints.size());
  for (const auto& jcfg : master_cfg.joints) {
    joint_names.push_back(jcfg.name);
  }

  // 先进入 Configured，不自动初始化电机。
  canopen_hw::LifecycleManager lifecycle;
  if (!lifecycle.Configure(master_cfg)) {
    return 1;
  }

  // ROS 适配层。
  canopen_hw::CanopenRobotHwRos robot_hw_ros(lifecycle.robot_hw(), joint_names);
  for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
    robot_hw_ros.SetMode(i, master_cfg.joints[i].default_mode);
  }
  std::mutex loop_mtx;
  canopen_hw::OperationalCoordinator coordinator(
      lifecycle.master(), lifecycle.shared_state(), master_cfg.joints.size());
  coordinator.SetConfigured();
  canopen_hw::ServiceGateway service_gateway(&pnh, &coordinator, &loop_mtx);

  // 辅助服务（set_mode、set_zero、软限位）。
  canopen_hw::CanopenAuxServices aux_services(
      &pnh, &robot_hw_ros, &coordinator, &master_cfg,
      lifecycle.master(), &loop_mtx);
  service_gateway.SetPostInitHook(
      [&](std::string* detail) { return aux_services.ApplySoftLimitAll(detail); });

  // IP 轨迹执行器（可选）。
  bool use_ip_executor = false;
  double ip_executor_rate_hz = master_cfg.loop_hz;
  std::string ip_executor_action_ns =
      "arm_position_controller/follow_joint_trajectory";
  pnh.param("use_ip_executor", use_ip_executor, false);
  pnh.param("ip_executor_rate_hz", ip_executor_rate_hz, ip_executor_rate_hz);
  pnh.param("ip_executor_action_ns", ip_executor_action_ns,
            ip_executor_action_ns);

  std::unique_ptr<canopen_hw::IpFollowJointTrajectoryExecutor> ip_executor;
  if (use_ip_executor) {
    canopen_hw::IpFollowJointTrajectoryExecutor::Config exec_cfg;
    exec_cfg.joint_names.clear();
    exec_cfg.joint_indices.clear();
    exec_cfg.max_velocities.clear();
    exec_cfg.max_accelerations.clear();
    exec_cfg.max_jerks.clear();
    exec_cfg.goal_tolerances.clear();
    exec_cfg.action_ns = ip_executor_action_ns;
    exec_cfg.command_rate_hz = ip_executor_rate_hz;
    exec_cfg.joint_names.reserve(master_cfg.joints.size());
    exec_cfg.joint_indices.reserve(master_cfg.joints.size());
    exec_cfg.max_velocities.reserve(master_cfg.joints.size());
    exec_cfg.max_accelerations.reserve(master_cfg.joints.size());
    exec_cfg.max_jerks.reserve(master_cfg.joints.size());
    exec_cfg.goal_tolerances.reserve(master_cfg.joints.size());

    for (std::size_t i = 0; i < master_cfg.joints.size(); ++i) {
      const auto& jcfg = master_cfg.joints[i];
      exec_cfg.joint_names.push_back(jcfg.name);
      exec_cfg.joint_indices.push_back(i);
      exec_cfg.max_velocities.push_back(jcfg.ip_max_velocity);
      exec_cfg.max_accelerations.push_back(jcfg.ip_max_acceleration);
      exec_cfg.max_jerks.push_back(jcfg.ip_max_jerk);
      exec_cfg.goal_tolerances.push_back(jcfg.ip_goal_tolerance);
    }

    ip_executor = std::make_unique<canopen_hw::IpFollowJointTrajectoryExecutor>(
        &pnh, &robot_hw_ros, &loop_mtx, std::move(exec_cfg));
  }

  // 启动序列（auto_init / auto_enable / auto_release）。
  {
    std::lock_guard<std::mutex> lk(loop_mtx);
    if (!canopen_hw::CanopenStartupSequence::Run(coordinator, aux_services, pnh)) {
      return 1;
    }
  }

  controller_manager::ControllerManager cm(&robot_hw_ros, nh);

  // Diagnostics。
  diagnostic_updater::Updater diag_updater;
  diag_updater.setHardwareID("canopen_hw");

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    diag_updater.add(joint_names[i], [&lifecycle, i](diagnostic_updater::DiagnosticStatusWrapper& stat) {
      const auto* counters = lifecycle.master() ? lifecycle.master()->GetHealthCounters(i) : nullptr;
      canopen_hw::AxisFeedback fb;
      bool got_fb = lifecycle.master() && lifecycle.master()->GetAxisFeedback(i, &fb);

      if (!counters || !got_fb) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "no data");
        return;
      }

      stat.add("heartbeat_lost", counters->heartbeat_lost.load());
      stat.add("emcy_count", counters->emcy_count.load());
      stat.add("fault_reset_attempts", counters->fault_reset_attempts.load());
      stat.add("is_operational", fb.is_operational);
      stat.add("is_fault", fb.is_fault);
      stat.add("heartbeat_lost_flag", fb.heartbeat_lost);

      if (fb.is_fault) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "fault");
      } else if (fb.heartbeat_lost) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "heartbeat lost");
      } else if (!fb.is_operational) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "not operational");
      } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "operational");
      }
    });
  }

  double loop_hz = master_cfg.loop_hz;
  pnh.param("loop_hz", loop_hz, loop_hz);  // launch 参数可覆盖 yaml 值。
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(loop_hz);
  ros::Time last_time = ros::Time::now();

  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - last_time;
    last_time = now;

    {
      std::lock_guard<std::mutex> lk(loop_mtx);
      coordinator.UpdateFromFeedback();
      coordinator.ComputeIntents();
      robot_hw_ros.read(now, period);
      cm.update(now, period);
      if (ip_executor) {
        ip_executor->update(now, period);
      }
      robot_hw_ros.write(now, period);
      diag_updater.update();
    }
    rate.sleep();
  }

  spinner.stop();
  {
    std::lock_guard<std::mutex> lk(loop_mtx);
    lifecycle.Shutdown();
  }
  return 0;
}
