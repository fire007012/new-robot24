#include "canopen_hw/sdo_accessor.hpp"

#include <future>
#include <memory>

#include "canopen_hw/canopen_master.hpp"

namespace canopen_hw {

// SdoResult 便捷转换。
uint8_t SdoResult::as_u8() const {
  return data.size() >= 1 ? data[0] : 0;
}

uint16_t SdoResult::as_u16() const {
  if (data.size() < 2) return as_u8();
  const uint32_t val = static_cast<uint32_t>(data[0]) |
                       (static_cast<uint32_t>(data[1]) << 8);
  return static_cast<uint16_t>(val);
}

uint32_t SdoResult::as_u32() const {
  if (data.size() < 4) return as_u16();
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int8_t SdoResult::as_i8() const {
  return static_cast<int8_t>(as_u8());
}

int16_t SdoResult::as_i16() const {
  return static_cast<int16_t>(as_u16());
}

int32_t SdoResult::as_i32() const {
  return static_cast<int32_t>(as_u32());
}

SdoAccessor::SdoAccessor(CanopenMaster* master) : master_(master) {}

void SdoAccessor::AsyncRead(uint8_t node_id, uint16_t index, uint8_t subindex,
                            SdoCallback callback, std::size_t expected_size) {
  if (!master_) {
    if (callback) {
      callback(SdoResult{false, {}, "master is null"});
    }
    return;
  }
  AxisDriver* driver = master_->FindDriverByNodeId(node_id);
  if (!driver) {
    if (callback) {
      callback(SdoResult{false, {}, "node_id not found"});
    }
    return;
  }
  driver->AsyncSdoRead(index, subindex,
      [callback](bool ok, const std::vector<uint8_t>& data,
                 const std::string& error) {
        if (callback) {
          callback(SdoResult{ok, data, error});
        }
      }, expected_size);
}

void SdoAccessor::AsyncWrite(uint8_t node_id, uint16_t index, uint8_t subindex,
                             const std::vector<uint8_t>& data,
                             SdoCallback callback) {
  if (!master_) {
    if (callback) {
      callback(SdoResult{false, {}, "master is null"});
    }
    return;
  }
  AxisDriver* driver = master_->FindDriverByNodeId(node_id);
  if (!driver) {
    if (callback) {
      callback(SdoResult{false, {}, "node_id not found"});
    }
    return;
  }
  driver->AsyncSdoWrite(index, subindex, data,
      [callback](bool ok, const std::string& error) {
        if (callback) {
          callback(SdoResult{ok, {}, error});
        }
      });
}

SdoResult SdoAccessor::Read(uint8_t node_id, uint16_t index, uint8_t subindex,
                            std::chrono::milliseconds timeout,
                            std::size_t expected_size) {
  auto promise = std::make_shared<std::promise<SdoResult>>();
  auto future = promise->get_future();

  AsyncRead(node_id, index, subindex,
            [promise](const SdoResult& result) {
              promise->set_value(result);
            }, expected_size);

  if (future.wait_for(timeout) == std::future_status::timeout) {
    return SdoResult{false, {}, "timeout"};
  }
  return future.get();
}

SdoResult SdoAccessor::Write(uint8_t node_id, uint16_t index, uint8_t subindex,
                             const std::vector<uint8_t>& data,
                             std::chrono::milliseconds timeout) {
  auto promise = std::make_shared<std::promise<SdoResult>>();
  auto future = promise->get_future();

  AsyncWrite(node_id, index, subindex, data,
             [promise](const SdoResult& result) {
               promise->set_value(result);
             });

  if (future.wait_for(timeout) == std::future_status::timeout) {
    return SdoResult{false, {}, "timeout"};
  }
  return future.get();
}

SdoResult SdoAccessor::WriteU8(uint8_t node_id, uint16_t index,
                               uint8_t subindex, uint8_t value,
                               std::chrono::milliseconds timeout) {
  return Write(node_id, index, subindex, {value}, timeout);
}

SdoResult SdoAccessor::WriteU16(uint8_t node_id, uint16_t index,
                                uint8_t subindex, uint16_t value,
                                std::chrono::milliseconds timeout) {
  std::vector<uint8_t> data = {
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF)};
  return Write(node_id, index, subindex, data, timeout);
}

SdoResult SdoAccessor::WriteU32(uint8_t node_id, uint16_t index,
                                uint8_t subindex, uint32_t value,
                                std::chrono::milliseconds timeout) {
  std::vector<uint8_t> data = {
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>((value >> 16) & 0xFF),
      static_cast<uint8_t>((value >> 24) & 0xFF)};
  return Write(node_id, index, subindex, data, timeout);
}

}  // namespace canopen_hw
