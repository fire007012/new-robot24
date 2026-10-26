#include "canopen_hw/lifecycle_manager.hpp"

#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/logging.hpp"

namespace canopen_hw {

LifecycleManager::LifecycleManager() = default;

bool LifecycleManager::Configure(const CanopenMasterConfig& config) {
  if (state_ != LifecycleState::Unconfigured) {
    CANOPEN_LOG_WARN("Configure called in state {}, expected Unconfigured",
                     static_cast<int>(state_));
    return false;
  }

  config_ = config;

  if (config_.axis_count == 0 || config_.axis_count > SharedState::kMaxAxisCount) {
    CANOPEN_LOG_ERROR("Configure: invalid axis_count {}", config_.axis_count);
    return false;
  }

  shared_state_ = std::make_unique<SharedState>(config_.axis_count);
  robot_hw_ = std::make_unique<CanopenRobotHw>(shared_state_.get());
  robot_hw_->ApplyConfig(config_);

  master_ = std::make_unique<CanopenMaster>(config_, shared_state_.get());

  state_ = LifecycleState::Configured;
  ever_initialized_ = false;
  CANOPEN_LOG_INFO("LifecycleManager: Configured");
  return true;
}

bool LifecycleManager::InitMotors() {
  if (state_ != LifecycleState::Configured) {
    CANOPEN_LOG_WARN("InitMotors called in state {}, expected Configured",
                     static_cast<int>(state_));
    return false;
  }

  if (!master_) {
    CANOPEN_LOG_ERROR("InitMotors: master is null");
    return false;
  }

  if (!master_->Start()) {
    CANOPEN_LOG_ERROR("InitMotors: master start failed");
    return false;
  }

  if (!master_->EnableAll()) {
    CANOPEN_LOG_ERROR("InitMotors: EnableAll failed");
    master_->Stop();
    return false;
  }

  state_ = LifecycleState::Active;
  ever_initialized_ = true;
  CANOPEN_LOG_INFO("LifecycleManager: Active (initialized)");
  return true;
}

bool LifecycleManager::Init(const std::string& dcf_path,
                            const std::string& joints_path) {
  if (state_ != LifecycleState::Unconfigured) {
    CANOPEN_LOG_WARN("Init called in state {}, expected Unconfigured",
                     static_cast<int>(state_));
    return false;
  }

  CanopenMasterConfig config;
  config.master_dcf_path = dcf_path;

  std::string error;
  if (!LoadJointsYaml(joints_path, &error, &config)) {
    CANOPEN_LOG_ERROR("Init: load joints.yaml failed: {}", error);
    return false;
  }

  return Init(config);
}

bool LifecycleManager::Init(const CanopenMasterConfig& config) {
  if (state_ != LifecycleState::Unconfigured) {
    CANOPEN_LOG_WARN("Init called in state {}, expected Unconfigured",
                     static_cast<int>(state_));
    return false;
  }

  if (!Configure(config)) {
    return false;
  }

  if (!InitMotors()) {
    // 兼容旧语义：Init 失败时回到 Unconfigured。
    Shutdown();
    return false;
  }

  return true;
}

bool LifecycleManager::Halt() {
  if (state_ != LifecycleState::Active) {
    CANOPEN_LOG_WARN("Halt called in state {}, expected Active",
                     static_cast<int>(state_));
    return false;
  }

  if (!master_ || !master_->running()) {
    CANOPEN_LOG_ERROR("Halt: master not running");
    return false;
  }

  if (!master_->HaltAll()) {
    CANOPEN_LOG_ERROR("Halt: HaltAll failed");
    return false;
  }

  CANOPEN_LOG_INFO("LifecycleManager: Active (halted)");
  return true;
}

bool LifecycleManager::Resume() {
  if (state_ != LifecycleState::Active) {
    CANOPEN_LOG_WARN("Resume called in state {}, expected Active",
                     static_cast<int>(state_));
    return false;
  }

  if (shared_state_ && shared_state_->GetGlobalFault()) {
    CANOPEN_LOG_WARN("Resume rejected: global fault latch active, call recover first");
    return false;
  }

  if (shared_state_ && shared_state_->GetAllAxesHaltedByFault()) {
    CANOPEN_LOG_WARN("Resume rejected: all axes halted by fault, call recover first");
    return false;
  }

  if (!master_ || !master_->running()) {
    CANOPEN_LOG_ERROR("Resume: master not running");
    return false;
  }

  if (shared_state_) {
    const SharedSnapshot snap = shared_state_->Snapshot();
    for (const auto& fb : snap.feedback) {
      if (fb.is_fault) {
        CANOPEN_LOG_WARN("Resume rejected: fault exists, call recover first");
        return false;
      }
    }
  }

  if (!master_->ResumeAll()) {
    CANOPEN_LOG_ERROR("Resume: ResumeAll failed");
    return false;
  }

  CANOPEN_LOG_INFO("LifecycleManager: Active (resumed)");
  return true;
}

bool LifecycleManager::StopCommunication(std::string* detail) {
  if (detail) {
    detail->clear();
  }

  if (state_ != LifecycleState::Active &&
      state_ != LifecycleState::Configured) {
    CANOPEN_LOG_WARN(
        "StopCommunication called in state {}, expected Active/Configured",
        static_cast<int>(state_));
    if (detail) {
      *detail = "invalid lifecycle state";
    }
    return false;
  }

  if (!master_) {
    CANOPEN_LOG_ERROR("StopCommunication: master is null");
    if (detail) {
      *detail = "master is null";
    }
    return false;
  }

  bool graceful_ok = true;
  auto append_detail = [&](const std::string& msg) {
    if (!detail || msg.empty()) {
      return;
    }
    if (!detail->empty()) {
      *detail += "; ";
    }
    *detail += msg;
  };

  if (master_->running()) {
    std::string graceful_detail;
    if (!master_->GracefulShutdown(&graceful_detail)) {
      graceful_ok = false;
      append_detail(graceful_detail);
    }
  }

  // 即使 402 失能超时，也继续执行通信停机，避免 service 卡死。
  master_->Stop();
  if (shared_state_) {
    shared_state_->SetGlobalFault(false);
    shared_state_->SetAllAxesHaltedByFault(false);
  }
  state_ = LifecycleState::Configured;
  if (!graceful_ok) {
    CANOPEN_LOG_WARN("LifecycleManager: communication stopped with detail: {}",
                     detail ? *detail : std::string("graceful shutdown timeout"));
  } else {
    CANOPEN_LOG_INFO("LifecycleManager: Configured (communication stopped)");
  }
  return graceful_ok;
}

bool LifecycleManager::Recover(std::string* detail) {
  if (detail) {
    detail->clear();
  }

  if (state_ != LifecycleState::Active) {
    CANOPEN_LOG_WARN("Recover called in state {}, expected Active",
                     static_cast<int>(state_));
    if (detail) {
      *detail = "recover requires Active state";
    }
    return false;
  }

  if (!master_ || !master_->running()) {
    CANOPEN_LOG_ERROR("Recover: master not running");
    if (detail) {
      *detail = "master not running";
    }
    return false;
  }

  std::string recover_detail;
  const bool ok = master_->ResetAllFaults(&recover_detail);
  if (!ok) {
    const std::string msg = recover_detail.empty()
                                ? std::string("fault reset failed, try ~/init to reinitialize")
                                : recover_detail;
    CANOPEN_LOG_WARN("Recover failed: {}", msg);
    if (detail) {
      *detail = msg;
    }
    if (shared_state_) {
      shared_state_->SetGlobalFault(true);
      shared_state_->SetAllAxesHaltedByFault(true);
    }
    return false;
  }

  if (detail && !recover_detail.empty()) {
    *detail = recover_detail;
  }

  if (shared_state_) {
    const SharedSnapshot snap = shared_state_->Snapshot();
    bool all_cleared = true;
    for (const auto& fb : snap.feedback) {
      if (fb.is_fault) {
        all_cleared = false;
        break;
      }
    }
    if (!all_cleared) {
      if (detail && detail->empty()) {
        *detail = "recover incomplete: fault still present";
      }
      shared_state_->SetGlobalFault(true);
      shared_state_->SetAllAxesHaltedByFault(true);
      CANOPEN_LOG_WARN("Recover incomplete: fault latch kept active");
      return false;
    }
    shared_state_->SetGlobalFault(false);
    shared_state_->SetAllAxesHaltedByFault(false);
  }

  if (detail) {
    if (!detail->empty()) {
      *detail += "; ";
    }
    *detail += "fault cleared; call ~/resume to re-enable";
  }

  CANOPEN_LOG_INFO("LifecycleManager: Active (fault recovered)");
  return true;
}

bool LifecycleManager::Shutdown() {
  if (state_ == LifecycleState::Unconfigured) {
    return true;
  }

  state_ = LifecycleState::ShuttingDown;

  if (master_) {
    master_->Stop();
  }

  master_.reset();
  robot_hw_.reset();
  shared_state_.reset();
  config_ = CanopenMasterConfig();
  ever_initialized_ = false;

  state_ = LifecycleState::Unconfigured;
  CANOPEN_LOG_INFO("LifecycleManager: Unconfigured (shutdown)");
  return true;
}

}  // namespace canopen_hw
