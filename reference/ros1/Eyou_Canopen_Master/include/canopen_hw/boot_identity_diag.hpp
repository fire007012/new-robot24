#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace canopen_hw {

struct BootIdentityTuple {
  bool has_device_type = false;
  uint32_t device_type = 0;

  bool has_vendor_id = false;
  uint32_t vendor_id = 0;

  bool has_product_code = false;
  uint32_t product_code = 0;

  bool has_revision = false;
  uint32_t revision = 0;

  bool HasAny() const {
    return has_device_type || has_vendor_id || has_product_code ||
           has_revision;
  }
};

// 从 master.dcf 中读取给定 node_id 的 1F84/1F85/1F86/1F87 期望身份值。
// 返回 true 表示至少读取到一个身份字段。
bool LoadExpectedBootIdentityFromDcf(const std::string& dcf_path,
                                     uint8_t node_id,
                                     BootIdentityTuple* out,
                                     std::string* error);

// 对比期望/实测身份并返回失配字段标识。
// 仅当 expected 与 actual 对应字段都存在时才参与比对。
std::vector<std::string> DiffBootIdentity(const BootIdentityTuple& expected,
                                          const BootIdentityTuple& actual);

}  // namespace canopen_hw
