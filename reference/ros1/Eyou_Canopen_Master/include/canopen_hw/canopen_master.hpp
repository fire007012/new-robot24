#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/ctx.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>

#include "canopen_hw/axis_driver.hpp"
#include "canopen_hw/cia402_defs.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

// 主站基础配置，包含总线参数和每轴参数。
// 由 LoadJointsYaml() 一次性填充，运行期只读。
struct CanopenMasterConfig {
  std::string can_interface = "canable0";
  std::string master_dcf_path;
  uint8_t master_node_id = 127;
  std::size_t axis_count = 6;  // 由 joints.yaml 覆盖，上限 SharedState::kMaxAxisCount。
  double loop_hz = 200.0;      // 控制循环频率，由 joints.yaml canopen.loop_hz 覆盖。
  // 为 true 时，在每次 init 成功后按 URDF 限位自动写入 0x2003/0x607D 软件限位。
  bool auto_write_soft_limits_from_urdf = false;

  // 每轴配置。C08 之前分散在 JointCanopenConfig / CanopenRuntimeConfig，
  // 现在合并为单一 JointConfig，消除 main.cpp 中的手动对拷。
  struct JointConfig {
    std::string name;  // URDF / ros_control 关节名，来自 joints.yaml。
    uint8_t node_id = 0;
    bool verify_pdo_mapping = false;
    int32_t position_lock_threshold = 15000;
    int max_fault_resets = 3;
    int fault_reset_hold_cycles = 5;
    int8_t default_mode = kMode_CSP;
    // 0x60C2:01 interpolation time period value (ms).
    uint8_t ip_interpolation_period_ms = 5;
    double max_velocity_for_clamp = 500000.0;
    double counts_per_rev = 5308416.0;
    // 线性关节换算系数（counts/m）。仅在 URDF 关节类型为 prismatic 且写软限位时使用。
    // <=0 表示未配置。
    double counts_per_meter = 0.0;
    double rated_torque_nm = 6.0;
    double velocity_scale = 1.0;
    double torque_scale = 1.0;
    // IP executor 运动约束，由 joints.yaml 配置。
    double ip_max_velocity = 1.0;
    double ip_max_acceleration = 2.0;
    double ip_max_jerk = 10.0;
    double ip_goal_tolerance = 1e-3;
  };
  std::vector<JointConfig> joints;
};

class CanopenMaster {
 public:
  explicit CanopenMaster(const CanopenMasterConfig& config,
                         SharedState* shared_state);

  // 启动主站骨架:
  // - 标记运行状态
  // - 为后续 Lely 初始化预留入口
  // 返回 true 表示流程进入“已启动”状态。
  bool Start();

  // 停止主站骨架，释放已创建的轴驱动。
  void Stop();

  bool running() const { return running_.load(); }
  std::size_t axis_count() const { return axis_drivers_.size(); }
  const CanopenMasterConfig& config() const { return config_; }

  // 优雅关机: 逐步将所有轴退出 OperationEnabled，最后 NMT STOP。
  // 必须在 Lely 事件循环仍在运行时调用。
  bool GracefulShutdown(std::string* detail = nullptr);

  // 单轴手动控制接口。
  bool EnableAxis(std::size_t axis_index);
  bool EnableAll();
  bool DisableAxis(std::size_t axis_index);
  bool ResetAxisFault(std::size_t axis_index);

  // 全轴轻量级 halt/resume（保持 Operation Enabled）。
  bool HaltAll();
  bool ResumeAll();

  // 兼容入口：仅对 fault 轴执行复位（内部转发到 ResetAllFaults）。
  bool RecoverFaultedAxes(std::string* detail = nullptr);
  // 故障复位执行器：发送 0x0080 复位脉冲并等待 fault 轴退出故障态。
  bool ResetAllFaults(std::string* detail = nullptr);

  // 全轴紧急停止: 对所有轴发送 DisableVoltage。
  void EmergencyStop();

  // 每轴状态查询。越界返回 false。
  bool GetAxisFeedback(std::size_t axis_index, AxisFeedback* out) const;

  // 每轴健康计数快照（非拷贝，返回指针供读取）。越界返回 nullptr。
  const HealthCounters* GetHealthCounters(std::size_t axis_index) const;

  // 按 node_id 查找 AxisDriver（供 SdoAccessor 使用）。未找到返回 nullptr。
  AxisDriver* FindDriverByNodeId(uint8_t node_id);
  bool WaitForSdoIdle(
      std::size_t axis_index,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) const;
  bool WaitForAllSdoIdle(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
      std::vector<std::size_t>* pending_axes = nullptr) const;
  bool WaitForStartupComplete(
      std::size_t axis_index,
      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) const;
  bool WaitForAllStartupComplete(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
      std::vector<std::size_t>* pending_axes = nullptr) const;

 private:
  // 供后续真实 master 初始化后调用:
  // 基于 node-id 1..N 创建 AxisDriver。
  // 约束: 该函数只能在初始化阶段调用，禁止在运行循环中调用。
  void CreateAxisDrivers(lely::canopen::BasicMaster& can_master);
  bool WaitForAllState(CiA402State target_state,
                       std::chrono::steady_clock::time_point deadline,
                       std::vector<std::size_t>* pending_axes = nullptr);

  CanopenMasterConfig config_;
  SharedState* shared_state_ = nullptr;  // 非拥有指针。
  std::atomic<bool> running_{false};

  std::unique_ptr<lely::io::IoGuard> io_guard_;
  std::unique_ptr<lely::io::Context> io_ctx_;
  std::unique_ptr<lely::io::Poll> io_poll_;
  std::unique_ptr<lely::ev::Loop> ev_loop_;
  std::unique_ptr<lely::io::Timer> io_timer_;
  std::unique_ptr<lely::io::CanController> can_ctrl_;
  std::unique_ptr<lely::io::CanChannel> can_chan_;
  std::unique_ptr<lely::canopen::AsyncMaster> master_;
  std::thread ev_thread_;
  std::string runtime_master_dcf_path_;

  // 每轴一个 driver，索引与轴号一一对应(0 -> node_id 1)。
  // 约束: 容量在初始化阶段 reserve，运行阶段只读/不扩容。
  std::vector<std::unique_ptr<AxisDriver>> axis_drivers_;
};

}  // namespace canopen_hw
