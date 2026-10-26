#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <memory>

namespace canopen_hw {

// 全局 logger 单例。
// 使用方式: canopen_hw::Logger()->info("...");
// 或使用宏: CANOPEN_LOG_INFO("axis={} node={}: {}", axis, node, msg);
inline std::shared_ptr<spdlog::logger>& Logger() {
  static auto logger = [] {
    auto l = spdlog::stdout_color_mt("canopen_hw");
    l->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [canopen_hw] %v");
    return l;
  }();
  return logger;
}

}  // namespace canopen_hw

// 便捷宏：自动带模块前缀。
#define CANOPEN_LOG_INFO(...)  canopen_hw::Logger()->info(__VA_ARGS__)
#define CANOPEN_LOG_WARN(...)  canopen_hw::Logger()->warn(__VA_ARGS__)
#define CANOPEN_LOG_ERROR(...) canopen_hw::Logger()->error(__VA_ARGS__)
