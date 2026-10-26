#include "Eyou_ROS1_Master/hybrid_operational_coordinator.hpp"

#include <algorithm>
#include <ros/ros.h>

namespace eyou_ros1_master {

namespace {

// 两个命名空间的 SystemOpMode 枚举值完全对齐（Inactive=0 … ShuttingDown=7），
// 可安全互转。
can_driver::SystemOpMode FromCanopen(canopen_hw::SystemOpMode m) {
    return static_cast<can_driver::SystemOpMode>(static_cast<unsigned char>(m));
}

// 悲观合并：fault / recovering / shuttingDown 优先，
// 否则取正常序中较低的那个。
can_driver::SystemOpMode PessimisticMerge(can_driver::SystemOpMode a,
                                           can_driver::SystemOpMode b) {
    using M = can_driver::SystemOpMode;
    // fault 状态优先级：Faulted > Recovering > ShuttingDown
    for (auto s : {M::Faulted, M::Recovering, M::ShuttingDown}) {
        if (a == s || b == s) return s;
    }
    // 正常序 Inactive < Configured < Standby < Armed < Running → 取较小
    return static_cast<M>(std::min(static_cast<unsigned char>(a),
                                   static_cast<unsigned char>(b)));
}

bool RestoreCanDriverTo(can_driver::OperationalCoordinator* coord,
                        const can_driver::SystemOpMode target_mode,
                        std::string* detail) {
    if (coord == nullptr) {
        if (detail) {
            *detail = "can_driver coordinator is null";
        }
        return false;
    }

    const auto current = coord->mode();
    if (current == target_mode) {
        return true;
    }

    can_driver::OperationalCoordinator::Result result;
    switch (target_mode) {
    case can_driver::SystemOpMode::Configured:
        result = coord->RequestShutdown(false);
        break;
    case can_driver::SystemOpMode::Standby:
        if (current == can_driver::SystemOpMode::Faulted) {
            result = coord->RequestRecover();
        } else {
            result = coord->RequestDisable();
        }
        break;
    case can_driver::SystemOpMode::Armed:
        if (current == can_driver::SystemOpMode::Running) {
            result = coord->RequestHalt();
        } else {
            result = coord->RequestEnable();
        }
        break;
    case can_driver::SystemOpMode::Running:
        if (current == can_driver::SystemOpMode::Standby) {
            result = coord->RequestEnable();
            if (!result.ok) {
                if (detail) {
                    *detail = result.message;
                }
                return false;
            }
        }
        result = coord->RequestRelease();
        break;
    default:
        if (detail) {
            *detail = "unsupported can_driver rollback target";
        }
        return false;
    }

    if (!result.ok) {
        if (detail) {
            *detail = result.message;
        }
        return false;
    }
    return true;
}

bool RestoreCanopenTo(canopen_hw::OperationalCoordinator* coord,
                      const canopen_hw::SystemOpMode target_mode,
                      std::string* detail) {
    if (coord == nullptr) {
        if (detail) {
            *detail = "canopen coordinator is null";
        }
        return false;
    }

    const auto current = coord->mode();
    if (current == target_mode) {
        return true;
    }

    canopen_hw::OperationalCoordinator::Result result;
    switch (target_mode) {
    case canopen_hw::SystemOpMode::Configured:
        result = coord->RequestShutdown();
        break;
    case canopen_hw::SystemOpMode::Standby:
        if (current == canopen_hw::SystemOpMode::Faulted) {
            result = coord->RequestRecover();
        } else {
            result = coord->RequestDisable();
        }
        break;
    case canopen_hw::SystemOpMode::Armed:
        if (current == canopen_hw::SystemOpMode::Running) {
            result = coord->RequestHalt();
        } else {
            result = coord->RequestEnable();
        }
        break;
    case canopen_hw::SystemOpMode::Running:
        if (current == canopen_hw::SystemOpMode::Standby) {
            result = coord->RequestEnable();
            if (!result.ok) {
                if (detail) {
                    *detail = result.message;
                }
                return false;
            }
        }
        result = coord->RequestRelease();
        break;
    default:
        if (detail) {
            *detail = "unsupported canopen rollback target";
        }
        return false;
    }

    if (!result.ok) {
        if (detail) {
            *detail = result.message;
        }
        return false;
    }
    return true;
}

std::string JoinMessages(const std::string& a, const std::string& b) {
    if (a.empty()) {
        return b;
    }
    if (b.empty()) {
        return a;
    }
    return a + "; " + b;
}

std::string FormatRollbackFailure(const std::string& primary_message,
                                  const std::string& rollback_message,
                                  const std::string& failsafe_message) {
    std::string out = primary_message;
    if (!rollback_message.empty()) {
        out = JoinMessages(out, "rollback failed: " + rollback_message);
    }
    if (!failsafe_message.empty()) {
        out = JoinMessages(out, "failsafe shutdown: " + failsafe_message);
    }
    return out;
}

std::string RunFailSafeShutdown(can_driver::OperationalCoordinator* can_coord,
                                canopen_hw::OperationalCoordinator* canopen_coord) {
    std::string failsafe_error;

    const auto can_shutdown = can_coord->RequestShutdown(false);
    if (!can_shutdown.ok) {
        failsafe_error = JoinMessages(failsafe_error,
                                      "[can_driver] " + can_shutdown.message);
    }

    const auto canopen_shutdown = canopen_coord->RequestShutdown();
    if (!canopen_shutdown.ok) {
        failsafe_error = JoinMessages(failsafe_error,
                                      "[canopen] " + canopen_shutdown.message);
    }

    if (failsafe_error.empty()) {
        return "executed";
    }
    return failsafe_error;
}

}  // namespace

HybridOperationalCoordinator::HybridOperationalCoordinator(
    can_driver::OperationalCoordinator* can_coord,
    canopen_hw::OperationalCoordinator* canopen_coord)
    : can_coord_(can_coord), canopen_coord_(canopen_coord) {}

can_driver::SystemOpMode HybridOperationalCoordinator::mode() const {
    return PessimisticMerge(can_coord_->mode(),
                            FromCanopen(canopen_coord_->mode()));
}

// ----------- 生命周期请求 -----------

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestInit(const std::string& device,
                                           bool loopback) {
    const auto can_prev = can_coord_->mode();
    const auto canopen_prev = canopen_coord_->mode();

    if (can_prev == can_driver::SystemOpMode::Armed &&
        canopen_prev == canopen_hw::SystemOpMode::Armed) {
        return {true, "already initialized", true};
    }

    auto r1 = can_coord_->RequestInit(device, loopback);
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestInit();
    if (!r2.ok) {
        std::string rollback_error;
        const bool canopen_restored =
            RestoreCanopenTo(canopen_coord_, canopen_prev, &rollback_error);
        const bool can_restored =
            RestoreCanDriverTo(can_coord_, can_prev, &rollback_error);

        if (canopen_restored && can_restored) {
            return {false, "[canopen] " + r2.message + "; rolled back both backends"};
        }

        const std::string failsafe_error = RunFailSafeShutdown(can_coord_, canopen_coord_);
        return {false, FormatRollbackFailure("[canopen] " + r2.message,
                                             rollback_error,
                                             failsafe_error)};
    }

    return {true, "both backends initialized"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestEnable() {
    const auto can_prev = can_coord_->mode();
    const auto canopen_prev = canopen_coord_->mode();

    auto r1 = can_coord_->RequestEnable();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestEnable();
    if (!r2.ok) {
        std::string rollback_error;
        const bool canopen_restored =
            RestoreCanopenTo(canopen_coord_, canopen_prev, &rollback_error);
        const bool can_restored =
            RestoreCanDriverTo(can_coord_, can_prev, &rollback_error);
        if (canopen_restored && can_restored) {
            return {false, "[canopen] " + r2.message + "; rolled back both backends"};
        }
        const std::string failsafe_error = RunFailSafeShutdown(can_coord_, canopen_coord_);
        return {false, FormatRollbackFailure("[canopen] " + r2.message,
                                             rollback_error,
                                             failsafe_error)};
    }

    return {true, "both backends enabled"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestDisable() {
    const auto can_prev = can_coord_->mode();
    const auto canopen_prev = canopen_coord_->mode();

    auto r1 = can_coord_->RequestDisable();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestDisable();
    if (!r2.ok) {
        std::string rollback_error;
        const bool canopen_restored =
            RestoreCanopenTo(canopen_coord_, canopen_prev, &rollback_error);
        const bool can_restored =
            RestoreCanDriverTo(can_coord_, can_prev, &rollback_error);
        if (canopen_restored && can_restored) {
            return {false, "[canopen] " + r2.message + "; rolled back both backends"};
        }
        const std::string failsafe_error = RunFailSafeShutdown(can_coord_, canopen_coord_);
        return {false, FormatRollbackFailure("[canopen] " + r2.message,
                                             rollback_error,
                                             failsafe_error)};
    }

    return {true, "both backends disabled"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestRelease() {
    const auto can_prev = can_coord_->mode();
    const auto canopen_prev = canopen_coord_->mode();

    auto r1 = can_coord_->RequestRelease();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestRelease();
    if (!r2.ok) {
        std::string rollback_error;
        const bool canopen_restored =
            RestoreCanopenTo(canopen_coord_, canopen_prev, &rollback_error);
        const bool can_restored =
            RestoreCanDriverTo(can_coord_, can_prev, &rollback_error);
        if (canopen_restored && can_restored) {
            return {false, "[canopen] " + r2.message + "; rolled back both backends"};
        }
        const std::string failsafe_error = RunFailSafeShutdown(can_coord_, canopen_coord_);
        return {false, FormatRollbackFailure("[canopen] " + r2.message,
                                             rollback_error,
                                             failsafe_error)};
    }

    return {true, "both backends released"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestHalt() {
    const auto can_prev = can_coord_->mode();
    const auto canopen_prev = canopen_coord_->mode();

    auto r1 = can_coord_->RequestHalt();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestHalt();
    if (!r2.ok) {
        std::string rollback_error;
        const bool canopen_restored =
            RestoreCanopenTo(canopen_coord_, canopen_prev, &rollback_error);
        const bool can_restored =
            RestoreCanDriverTo(can_coord_, can_prev, &rollback_error);
        if (canopen_restored && can_restored) {
            return {false, "[canopen] " + r2.message + "; rolled back both backends"};
        }
        const std::string failsafe_error = RunFailSafeShutdown(can_coord_, canopen_coord_);
        return {false, FormatRollbackFailure("[canopen] " + r2.message,
                                             rollback_error,
                                             failsafe_error)};
    }

    return {true, "both backends halted"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestRecover() {
    const auto can_prev = can_coord_->mode();
    const auto canopen_prev = canopen_coord_->mode();

    auto r1 = can_coord_->RequestRecover();
    if (!r1.ok) return {false, "[can_driver] " + r1.message};

    auto r2 = canopen_coord_->RequestRecover();
    if (!r2.ok) {
        std::string rollback_error;
        const bool canopen_restored =
            RestoreCanopenTo(canopen_coord_, canopen_prev, &rollback_error);
        const bool can_restored =
            RestoreCanDriverTo(can_coord_, can_prev, &rollback_error);
        if (canopen_restored && can_restored) {
            return {false, "[canopen] " + r2.message + "; rolled back both backends"};
        }

        const std::string failsafe_error = RunFailSafeShutdown(can_coord_, canopen_coord_);
        return {false, FormatRollbackFailure("[canopen] " + r2.message,
                                             rollback_error,
                                             failsafe_error)};
    }

    return {true, "both backends recovered"};
}

HybridOperationalCoordinator::Result
HybridOperationalCoordinator::RequestShutdown(bool force) {
    std::string shutdown_error;

    const auto can_result = can_coord_->RequestShutdown(force);
    if (!can_result.ok) {
        shutdown_error = JoinMessages(
            shutdown_error, "[can_driver] " +
                                (can_result.message.empty() ? "shutdown failed"
                                                            : can_result.message));
    }

    const auto canopen_result = canopen_coord_->RequestShutdown();
    if (!canopen_result.ok) {
        shutdown_error = JoinMessages(
            shutdown_error, "[canopen] " +
                                (canopen_result.message.empty() ? "shutdown failed"
                                                                : canopen_result.message));
    }

    if (!shutdown_error.empty()) {
        return {false, "best-effort shutdown completed with errors: " + shutdown_error};
    }
    return {true, "both backends shut down"};
}

}  // namespace eyou_ros1_master
