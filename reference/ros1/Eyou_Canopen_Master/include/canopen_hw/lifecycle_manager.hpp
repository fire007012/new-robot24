#pragma once

#include <memory>
#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/canopen_robot_hw.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

enum class LifecycleState {
  Unconfigured,
  Configured,
  Active,
  ShuttingDown,
};

// 生命周期管理器:
// 封装 SharedState / CanopenMaster / CanopenRobotHw 的构造与状态跃迁，
// 对外提供 Configure / InitMotors / Halt / Recover / Shutdown 等操作。
class LifecycleManager {
 public:
  LifecycleManager();

  // 仅加载配置并构造对象，不启动主站。
  // Unconfigured -> Configured。
  bool Configure(const CanopenMasterConfig& config);

  // 在已配置状态下执行首次电机初始化（启动主站）。
  // Configured -> Active。
  bool InitMotors();

  // 兼容接口：加载配置并立即启动主站。
  // 保持原有语义：失败后回到 Unconfigured。
  bool Init(const std::string& dcf_path, const std::string& joints_path);
  bool Init(const CanopenMasterConfig& config);

  // 轻量级停运动（全轴置 halt bit），保持通信与 Active 状态。
  bool Halt();

  // 清除全轴 halt bit，恢复指令透传。
  bool Resume();

  // 关闭现场通信但不退出进程，后续必须重新 InitMotors。
  // Active/Configured -> Configured。
  // 返回 false 表示 402 降级存在超时或当前状态不允许。
  bool StopCommunication(std::string* detail = nullptr);

  // 仅处理 fault 轴的复位与重使能，不做通信层重启。
  // Active -> Active。
  bool Recover(std::string* detail = nullptr);

  // 优雅关闭主站并释放对象。
  // 任意状态 -> Unconfigured。
  bool Shutdown();

  LifecycleState state() const { return state_; }
  bool ever_initialized() const { return ever_initialized_; }
  // 已退化为资源 owner 后，不再维护 require_init 影子语义。
  bool require_init() const { return false; }
  bool halted() const { return false; }

  // 访问内部组件（供上层集成使用）。
  CanopenMaster* master() { return master_.get(); }
  CanopenRobotHw* robot_hw() { return robot_hw_.get(); }
  SharedState* shared_state() { return shared_state_.get(); }

 private:
  LifecycleState state_ = LifecycleState::Unconfigured;
  bool ever_initialized_ = false;
  CanopenMasterConfig config_;
  std::unique_ptr<SharedState> shared_state_;
  std::unique_ptr<CanopenMaster> master_;
  std::unique_ptr<CanopenRobotHw> robot_hw_;
};

}  // namespace canopen_hw
