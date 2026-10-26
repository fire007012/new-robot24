#include "canopen_hw/operational_coordinator.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>
#include <utility>

#include "canopen_hw/logging.hpp"

namespace canopen_hw {

const char* SystemOpModeName(SystemOpMode mode) {
  switch (mode) {
    case SystemOpMode::Inactive:
      return "Inactive";
    case SystemOpMode::Configured:
      return "Configured";
    case SystemOpMode::Standby:
      return "Standby";
    case SystemOpMode::Armed:
      return "Armed";
    case SystemOpMode::Running:
      return "Running";
    case SystemOpMode::Faulted:
      return "Faulted";
    case SystemOpMode::Recovering:
      return "Recovering";
    case SystemOpMode::ShuttingDown:
      return "ShuttingDown";
  }
  return "Unknown";
}

OperationalCoordinator::OperationalCoordinator(CanopenMaster* master,
                                               SharedState* shared_state,
                                               std::size_t axis_count)
    : master_(master), shared_state_(shared_state), axis_count_(axis_count) {
  master_ops_.start = [this]() {
    if (master_ == nullptr) {
      return false;
    }
    if (master_->running()) {
      return true;
    }
    return master_->Start();
  };
  master_ops_.running = [this]() {
    return master_ != nullptr && master_->running();
  };
  master_ops_.reset_all_faults = [this](std::string* detail) {
    if (master_ == nullptr) {
      if (detail) {
        *detail = "master is null";
      }
      return false;
    }
    return master_->ResetAllFaults(detail);
  };
  master_ops_.graceful_shutdown = [this](std::string* detail) {
    if (master_ == nullptr) {
      if (detail) {
        *detail = "master is null";
      }
      return false;
    }
    return master_->GracefulShutdown(detail);
  };
  master_ops_.stop = [this]() {
    if (master_ != nullptr) {
      master_->Stop();
    }
  };
}

OperationalCoordinator::OperationalCoordinator(MasterOps master_ops,
                                               SharedState* shared_state,
                                               std::size_t axis_count)
    : master_(nullptr),
      master_ops_(std::move(master_ops)),
      shared_state_(shared_state),
      axis_count_(axis_count) {}

void OperationalCoordinator::SetConfigured() {
  mode_.store(SystemOpMode::Configured, std::memory_order_release);
}

bool OperationalCoordinator::MasterStart(std::string* detail) {
  if (!master_ops_.start || !master_ops_.start()) {
    if (detail && detail->empty()) {
      *detail = "master start failed";
    }
    return false;
  }
  return true;
}

bool OperationalCoordinator::MasterRunning(std::string* detail) const {
  if (!master_ops_.running || !master_ops_.running()) {
    if (detail && detail->empty()) {
      *detail = "master not running";
    }
    return false;
  }
  return true;
}

bool OperationalCoordinator::MasterResetAllFaults(std::string* detail) {
  if (!master_ops_.reset_all_faults) {
    if (detail && detail->empty()) {
      *detail = "fault reset path not available";
    }
    return false;
  }
  return master_ops_.reset_all_faults(detail);
}

bool OperationalCoordinator::MasterGracefulShutdown(std::string* detail) {
  if (!master_ops_.graceful_shutdown) {
    if (detail && detail->empty()) {
      *detail = "graceful shutdown path not available";
    }
    return false;
  }
  return master_ops_.graceful_shutdown(detail);
}

void OperationalCoordinator::MasterStop() {
  if (master_ops_.stop) {
    master_ops_.stop();
  }
}

bool OperationalCoordinator::CheckHealthyForMotion(std::string* detail) const {
  if (!shared_state_) {
    return true;
  }

  const SharedSnapshot snap = shared_state_->Snapshot();
  if (snap.global_fault) {
    if (detail) {
      *detail = "global fault latch active";
    }
    return false;
  }

  const std::size_t n = std::min(axis_count_, snap.feedback.size());
  for (std::size_t i = 0; i < n; ++i) {
    if (snap.feedback[i].is_fault) {
      if (detail) {
        std::ostringstream oss;
        oss << "axis " << i << " fault active";
        *detail = oss.str();
      }
      return false;
    }
    if (snap.feedback[i].heartbeat_lost) {
      if (detail) {
        std::ostringstream oss;
        oss << "axis " << i << " heartbeat lost";
        *detail = oss.str();
      }
      return false;
    }
  }
  return true;
}

OperationalCoordinator::Result OperationalCoordinator::DoTransition(
    std::initializer_list<SystemOpMode> allowed_from, SystemOpMode to,
    std::function<bool(std::string*)> action) {
  std::lock_guard<std::mutex> lk(transition_mtx_);

  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  bool allowed = false;
  for (const auto from : allowed_from) {
    if (from == current) {
      allowed = true;
      break;
    }
  }

  if (!allowed) {
    if (current == to) {
      return {true, std::string("already ") + SystemOpModeName(to)};
    }
    std::ostringstream oss;
    oss << "cannot transition from " << SystemOpModeName(current) << " to "
        << SystemOpModeName(to);
    return {false, oss.str()};
  }

  std::string detail;
  if (action && !action(&detail)) {
    if (detail.empty()) {
      detail = "action failed";
    }
    return {false, detail};
  }

  mode_.store(to, std::memory_order_release);
  CANOPEN_LOG_INFO("OperationalCoordinator: {} -> {}", SystemOpModeName(current),
                   SystemOpModeName(to));
  if (detail.empty()) {
    detail = std::string("-> ") + SystemOpModeName(to);
  }
  return {true, detail};
}

OperationalCoordinator::Result OperationalCoordinator::RequestInit() {
  return DoTransition(
      {SystemOpMode::Configured}, SystemOpMode::Armed,
      [this](std::string* detail) {
        if (!MasterStart(detail)) {
          return false;
        }
        if (!MasterRunning(detail)) {
          // 主站启动不完整，回落到 Standby 以便人工排查后再 enable。
          mode_.store(SystemOpMode::Standby, std::memory_order_release);
          return false;
        }

        if (shared_state_) {
          const SharedSnapshot snap = shared_state_->Snapshot();
          const std::size_t n = std::min(axis_count_, snap.feedback.size());
          bool any_fault = false;
          bool heartbeat_lost = false;
          for (std::size_t i = 0; i < n; ++i) {
            if (snap.feedback[i].is_fault) {
              any_fault = true;
              break;
            }
            if (snap.feedback[i].heartbeat_lost) {
              heartbeat_lost = true;
            }
          }
          if (any_fault || snap.global_fault) {
            shared_state_->SetGlobalFault(true);
            shared_state_->SetAllAxesHaltedByFault(true);
            mode_.store(SystemOpMode::Faulted, std::memory_order_release);
            if (detail) {
              *detail = "safety check failed: fault present after init";
            }
            return false;
          }
          if (heartbeat_lost) {
            // 心跳问题按可恢复启动异常处理，停留 Standby，不自动上电。
            shared_state_->SetGlobalFault(false);
            shared_state_->SetAllAxesHaltedByFault(false);
            mode_.store(SystemOpMode::Standby, std::memory_order_release);
            if (detail) {
              *detail = "safety check failed: heartbeat lost";
            }
            return false;
          }
          shared_state_->SetGlobalFault(false);
          shared_state_->SetAllAxesHaltedByFault(false);
          shared_state_->AdvanceCommandSyncSequence();
        }
        return true;
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestEnable() {
  return DoTransition(
      {SystemOpMode::Standby}, SystemOpMode::Armed,
      [this](std::string* detail) {
        if (!MasterRunning(detail)) {
          return false;
        }
        return CheckHealthyForMotion(detail);
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestDisable() {
  return DoTransition(
      {SystemOpMode::Standby, SystemOpMode::Armed, SystemOpMode::Running},
      SystemOpMode::Standby,
      [this](std::string* detail) { return MasterRunning(detail); });
}

OperationalCoordinator::Result OperationalCoordinator::RequestRelease() {
  return DoTransition(
      {SystemOpMode::Armed}, SystemOpMode::Running,
      [this](std::string* detail) {
        if (!MasterRunning(detail)) {
          return false;
        }
        return CheckHealthyForMotion(detail);
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestHalt() {
  return DoTransition({SystemOpMode::Running}, SystemOpMode::Armed, nullptr);
}

OperationalCoordinator::Result OperationalCoordinator::RequestRecover() {
  return DoTransition(
      {SystemOpMode::Faulted}, SystemOpMode::Standby,
      [this](std::string* detail) {
        if (!MasterRunning(detail)) {
          return false;
        }
        mode_.store(SystemOpMode::Recovering, std::memory_order_release);

        std::string recover_detail;
        if (!MasterResetAllFaults(&recover_detail)) {
          mode_.store(SystemOpMode::Faulted, std::memory_order_release);
          if (detail) {
            *detail = recover_detail.empty() ? "recover failed" : recover_detail;
          }
          return false;
        }

        if (shared_state_) {
          const auto deadline =
              std::chrono::steady_clock::now() + std::chrono::seconds(2);

          while (true) {
            const SharedSnapshot snap = shared_state_->Snapshot();
            const std::size_t n = std::min(axis_count_, snap.feedback.size());
            bool any_unhealthy = false;
            std::size_t first_bad_axis = 0;
            bool first_bad_is_heartbeat = false;
            for (std::size_t i = 0; i < n; ++i) {
              if (snap.feedback[i].is_fault || snap.feedback[i].heartbeat_lost) {
                any_unhealthy = true;
                first_bad_axis = i;
                first_bad_is_heartbeat = snap.feedback[i].heartbeat_lost;
                break;
              }
            }
            if (!any_unhealthy) {
              break;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
              mode_.store(SystemOpMode::Faulted, std::memory_order_release);
              if (detail) {
                std::ostringstream oss;
                oss << "recover timeout: axis " << first_bad_axis << ' '
                    << (first_bad_is_heartbeat ? "heartbeat_lost"
                                               : "fault_active");
                *detail = oss.str();
              }
              return false;
            }
            shared_state_->WaitForStateChange(std::min(
                deadline, std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(20)));
          }
        }

        if (shared_state_) {
          shared_state_->SetGlobalFault(false);
          shared_state_->SetAllAxesHaltedByFault(false);
          shared_state_->AdvanceCommandSyncSequence();
        }

        if (detail && !recover_detail.empty()) {
          *detail = recover_detail;
        }
        return true;
      });
}

OperationalCoordinator::Result OperationalCoordinator::RequestShutdown() {
  return DoTransition(
      {SystemOpMode::Inactive, SystemOpMode::Configured, SystemOpMode::Standby,
       SystemOpMode::Armed, SystemOpMode::Running, SystemOpMode::Faulted,
       SystemOpMode::Recovering, SystemOpMode::ShuttingDown},
      SystemOpMode::Configured,
      [this](std::string* detail) {
        mode_.store(SystemOpMode::ShuttingDown, std::memory_order_release);

        std::string graceful_detail;
        const bool running = master_ops_.running ? master_ops_.running() : false;
        const bool graceful_ok =
            running ? MasterGracefulShutdown(&graceful_detail) : true;
        MasterStop();

        if (!graceful_ok) {
          if (detail) {
            *detail = graceful_detail.empty() ? "graceful shutdown timeout"
                                              : graceful_detail;
          }
          // 保持与现有 stop-communication 语义一致：即使超时，通信也已停止。
          mode_.store(SystemOpMode::Configured, std::memory_order_release);
          return false;
        }

        if (shared_state_) {
          shared_state_->SetGlobalFault(false);
          shared_state_->SetAllAxesHaltedByFault(false);
          shared_state_->AdvanceCommandSyncSequence();
        }

        if (detail && !graceful_detail.empty()) {
          *detail = graceful_detail;
        }
        return true;
      });
}

void OperationalCoordinator::ComputeIntents() {
  if (!shared_state_) {
    return;
  }

  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  AxisIntent intent = AxisIntent::Disable;

  switch (current) {
    case SystemOpMode::Running:
      intent = AxisIntent::Run;
      break;
    case SystemOpMode::Armed:
    case SystemOpMode::Faulted:
      intent = AxisIntent::Halt;
      break;
    case SystemOpMode::Inactive:
    case SystemOpMode::Configured:
    case SystemOpMode::Standby:
    case SystemOpMode::ShuttingDown:
      intent = AxisIntent::Disable;
      break;
    case SystemOpMode::Recovering:
      intent = AxisIntent::Halt;
      break;
  }

  for (std::size_t i = 0; i < axis_count_; ++i) {
    shared_state_->SetAxisIntent(i, intent);
  }
  shared_state_->AdvanceIntentSequence();
}

void OperationalCoordinator::UpdateFromFeedback() {
  const SystemOpMode current = mode_.load(std::memory_order_acquire);
  if (current != SystemOpMode::Armed && current != SystemOpMode::Running) {
    return;
  }
  if (!shared_state_) {
    return;
  }

  const SharedSnapshot snap = shared_state_->Snapshot();
  const std::size_t n = std::min(axis_count_, snap.feedback.size());
  bool any_fault = false;
  for (std::size_t i = 0; i < n; ++i) {
    if (snap.feedback[i].is_fault || snap.feedback[i].heartbeat_lost) {
      any_fault = true;
      break;
    }
  }

  if (!any_fault) {
    return;
  }

  SystemOpMode expected = current;
  if (mode_.compare_exchange_strong(expected, SystemOpMode::Faulted,
                                    std::memory_order_acq_rel)) {
    shared_state_->SetGlobalFault(true);
    shared_state_->SetAllAxesHaltedByFault(true);
    CANOPEN_LOG_WARN("OperationalCoordinator: {} -> Faulted (auto)",
                     SystemOpModeName(current));
  }
}

}  // namespace canopen_hw
