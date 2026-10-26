#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace canopen_hw {

class CanopenMaster;

// SDO 操作结果。
struct SdoResult {
  bool ok = false;
  std::vector<uint8_t> data;  // 读取时有效。
  std::string error;

  // 便捷类型转换（小端序）。
  uint8_t as_u8() const;
  uint16_t as_u16() const;
  uint32_t as_u32() const;
  int8_t as_i8() const;
  int16_t as_i16() const;
  int32_t as_i32() const;
};

// SDO 异步回调签名。
using SdoCallback = std::function<void(const SdoResult&)>;

// SDO 读写接口。
// 底层异步（在 Lely 事件线程执行），上层提供同步阻塞便捷方法。
class SdoAccessor {
 public:
  explicit SdoAccessor(CanopenMaster* master);

  // 异步读取。expected_size 支持 1..4 字节；回调在 Lely 事件线程中执行。
  void AsyncRead(uint8_t node_id, uint16_t index, uint8_t subindex,
                 SdoCallback callback, std::size_t expected_size = 4);

  // 异步写入。
  void AsyncWrite(uint8_t node_id, uint16_t index, uint8_t subindex,
                  const std::vector<uint8_t>& data, SdoCallback callback);

  // 同步阻塞读取（内部用 promise/future 包装异步调用）。
  // expected_size 支持 1..4 字节。
  SdoResult Read(uint8_t node_id, uint16_t index, uint8_t subindex,
                 std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
                 std::size_t expected_size = 4);

  // 同步阻塞写入。
  SdoResult Write(uint8_t node_id, uint16_t index, uint8_t subindex,
                  const std::vector<uint8_t>& data,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

  // 便捷同步写入（单值）。
  SdoResult WriteU8(uint8_t node_id, uint16_t index, uint8_t subindex,
                    uint8_t value,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  SdoResult WriteU16(uint8_t node_id, uint16_t index, uint8_t subindex,
                     uint16_t value,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  SdoResult WriteU32(uint8_t node_id, uint16_t index, uint8_t subindex,
                     uint32_t value,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

 private:
  CanopenMaster* master_ = nullptr;
};

}  // namespace canopen_hw
