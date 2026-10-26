#include "Eyou_ROS1_Master/hybrid_auto_startup.hpp"

#include "can_driver/operational_coordinator.hpp"

namespace eyou_ros1_master {

namespace {

bool SetError(const std::string& message, std::string* error) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

bool EnsureHybridArmed(HybridOperationalCoordinator& hybrid_coord,
                       std::string* error) {
    switch (hybrid_coord.mode()) {
    case can_driver::SystemOpMode::Standby: {
        const auto enable_result = hybrid_coord.RequestEnable();
        if (!enable_result.ok) {
            return SetError(enable_result.message, error);
        }
        return true;
    }
    case can_driver::SystemOpMode::Armed:
    case can_driver::SystemOpMode::Running:
        return true;
    default:
        return SetError("auto startup expected hybrid lifecycle in Standby/Armed/Running after init",
                        error);
    }
}

bool EnsureHybridRunning(HybridOperationalCoordinator& hybrid_coord,
                         std::string* error) {
    if (!EnsureHybridArmed(hybrid_coord, error)) {
        return false;
    }

    switch (hybrid_coord.mode()) {
    case can_driver::SystemOpMode::Running:
        return true;
    case can_driver::SystemOpMode::Armed: {
        const auto release_result = hybrid_coord.RequestRelease();
        if (!release_result.ok) {
            return SetError(release_result.message, error);
        }
        return true;
    }
    default:
        return SetError("auto startup could not drive hybrid lifecycle to running",
                        error);
    }
}

}  // namespace

HybridAutoStartupOptions LoadHybridAutoStartupOptions(const ros::NodeHandle& pnh) {
    HybridAutoStartupOptions options;
    pnh.param("auto_init", options.auto_init, false);
    pnh.param("auto_enable", options.auto_enable, false);
    pnh.param("auto_release", options.auto_release, false);
    return options;
}

bool RunHybridAutoStartup(const HybridAutoStartupOptions& options,
                          HybridServiceGateway& service_gateway,
                          HybridOperationalCoordinator& hybrid_coord,
                          std::string* error) {
    if (error != nullptr) {
        error->clear();
    }

    if ((options.auto_enable || options.auto_release) && !options.auto_init) {
        return SetError("auto_enable/auto_release require auto_init=true", error);
    }
    if (options.auto_release && !options.auto_enable) {
        return SetError("auto_release requires auto_enable=true", error);
    }
    if (!options.auto_init) {
        return true;
    }

    bool already_initialized = false;
    std::string detail;
    if (!service_gateway.RunConfiguredInitSequence(&detail, &already_initialized)) {
        return SetError(detail, error);
    }

    if (options.auto_enable && !EnsureHybridArmed(hybrid_coord, error)) {
        return false;
    }
    if (options.auto_release && !EnsureHybridRunning(hybrid_coord, error)) {
        return false;
    }

    return true;
}

bool RunHybridAutoStartupFromParams(HybridServiceGateway& service_gateway,
                                    HybridOperationalCoordinator& hybrid_coord,
                                    const ros::NodeHandle& pnh,
                                    std::string* error) {
    return RunHybridAutoStartup(LoadHybridAutoStartupOptions(pnh),
                                service_gateway,
                                hybrid_coord,
                                error);
}

}  // namespace eyou_ros1_master
