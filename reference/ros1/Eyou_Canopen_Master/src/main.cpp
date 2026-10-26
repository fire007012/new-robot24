#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <cstdint>
#include <string>
#include <thread>

#include "canopen_hw/joints_config.hpp"
#include "canopen_hw/lifecycle_manager.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/operational_coordinator.hpp"
#include "canopen_hw/realtime_loop.hpp"

namespace {

std::atomic<bool> g_run{true};

void HandleSignal(int) {
  g_run.store(false);
}

struct StartupOptions {
  std::string dcf_path = "config/master.dcf";
  std::string joints_path = "config/joints.yaml";
  bool auto_init = true;
  bool auto_enable = true;
  bool auto_release = true;
};

bool ParseBool01(const std::string& text, bool* out) {
  if (!out) {
    return false;
  }
  if (text == "1") {
    *out = true;
    return true;
  }
  if (text == "0") {
    *out = false;
    return true;
  }
  return false;
}

StartupOptions ParseArgs(int argc, char** argv) {
  StartupOptions opts;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--dcf" && i + 1 < argc) {
      opts.dcf_path = argv[++i];
    } else if (arg == "--joints" && i + 1 < argc) {
      opts.joints_path = argv[++i];
    } else if (arg == "--auto-init" && i + 1 < argc) {
      ParseBool01(argv[++i], &opts.auto_init);
    } else if (arg == "--auto-enable" && i + 1 < argc) {
      ParseBool01(argv[++i], &opts.auto_enable);
    } else if (arg == "--auto-release" && i + 1 < argc) {
      ParseBool01(argv[++i], &opts.auto_release);
    }
  }
  return opts;
}

std::string MakeAbsolutePath(const std::string& path) {
  if (path.empty()) {
    return path;
  }
  std::filesystem::path p(path);
  if (p.is_absolute()) {
    return p.string();
  }
  return std::filesystem::absolute(p).string();
}

bool FileExists(const std::string& path) {
  return !path.empty() && std::filesystem::exists(path);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  const StartupOptions opts = ParseArgs(argc, argv);
  const std::string dcf_path = MakeAbsolutePath(opts.dcf_path);
  const std::string joints_path = MakeAbsolutePath(opts.joints_path);

  if (!FileExists(dcf_path)) {
    CANOPEN_LOG_ERROR("master_dcf_path not found: {}", dcf_path);
    return 1;
  }
  if (!FileExists(joints_path)) {
    CANOPEN_LOG_ERROR("joints.yaml not found: {}", joints_path);
    return 1;
  }

  if ((opts.auto_enable || opts.auto_release) && !opts.auto_init) {
    CANOPEN_LOG_ERROR(
        "--auto-enable/--auto-release require --auto-init 1 for strict transition path");
    return 1;
  }
  if (opts.auto_release && !opts.auto_enable) {
    CANOPEN_LOG_ERROR("--auto-release requires --auto-enable 1");
    return 1;
  }

  canopen_hw::CanopenMasterConfig config;
  config.master_dcf_path = dcf_path;
  std::string error;
  if (!canopen_hw::LoadJointsYaml(joints_path, &error, &config)) {
    CANOPEN_LOG_ERROR("Load joints.yaml failed: {}", error);
    return 1;
  }

  canopen_hw::LifecycleManager lifecycle;
  if (!lifecycle.Configure(config)) {
    return 1;
  }

  canopen_hw::OperationalCoordinator coordinator(
      lifecycle.master(), lifecycle.shared_state(), config.axis_count);
  coordinator.SetConfigured();

  if (opts.auto_init) {
    const auto r = coordinator.RequestInit();
    if (!r.ok) {
      CANOPEN_LOG_ERROR("auto-init failed: {}", r.message);
      return 1;
    }
  }
  if (opts.auto_enable) {
    const auto r = coordinator.RequestEnable();
    if (!r.ok) {
      CANOPEN_LOG_ERROR("auto-enable failed: {}", r.message);
      return 1;
    }
  }
  if (opts.auto_release) {
    const auto r = coordinator.RequestRelease();
    if (!r.ok) {
      CANOPEN_LOG_ERROR("auto-release failed: {}", r.message);
      return 1;
    }
  }

  // 应用主循环：按配置频率执行 read/write。
  // 使用 RealtimeLoop 的 clock_nanosleep 绝对时间等待，消除累积漂移。
  canopen_hw::RealtimeLoop::Config loop_config;
  const double loop_hz = (config.loop_hz > 0.0) ? config.loop_hz : 100.0;
  const int64_t period_ns =
      static_cast<int64_t>(1e9 / loop_hz);
  loop_config.period = std::chrono::nanoseconds(
      (period_ns > 0) ? period_ns : static_cast<int64_t>(10'000'000));
  canopen_hw::RealtimeLoop loop(loop_config);

  loop.Run([&]() -> bool {
    if (!g_run.load()) return false;
    coordinator.UpdateFromFeedback();
    coordinator.ComputeIntents();
    lifecycle.robot_hw()->ReadFromSharedState();
    lifecycle.robot_hw()->WriteToSharedState();
    return true;
  });

  (void)coordinator.RequestShutdown();
  lifecycle.Shutdown();
  return 0;
}
