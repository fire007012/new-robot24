#pragma once
#include <dlfcn.h>
#include <stdexcept>
namespace robot_hardware {
// Humble executors can retain weak callback/control blocks after pluginlib drops
// its loader. Their deleters live in this DSO. Keep code mapped until process
// exit; node/controller objects still follow normal cleanup and destruction.
static inline void retainPluginCode() {
  static const bool retained=[] {
    Dl_info info{};
    if (!dladdr(reinterpret_cast<void *>(&retainPluginCode),&info) ||
        !dlopen(info.dli_fname,RTLD_NOW|RTLD_NOLOAD|RTLD_NODELETE)) {
      throw std::runtime_error("Cannot retain ROS2 plugin code for executor teardown");
    }
    return true;
  }();
  (void)retained;
}
}
