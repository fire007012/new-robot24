#pragma once

#include <cstdint>

#include "canopen_hw/cia402_defs.hpp"

namespace canopen_hw {
namespace detail {

// IP 模式下优先写 0x60C1:01，但仍同步镜像 0x607A，避免部分驱动在
// halt/release 边沿瞬时参考 legacy 目标位置通道。
template <typename WriteIpFn, typename WriteLegacyFn, typename WarnFn>
bool WritePositionChannels(int8_t target_mode, int32_t pos, WriteIpFn&& write_ip,
                           WriteLegacyFn&& write_legacy,
                           WarnFn&& warn_ip_fallback) {
  if (target_mode == kMode_IP) {
    if (!write_ip(pos)) {
      warn_ip_fallback();
    }
  }
  return write_legacy(pos);
}

}  // namespace detail
}  // namespace canopen_hw
