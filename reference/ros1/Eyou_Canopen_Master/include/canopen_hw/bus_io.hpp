#pragma once

#include <cstdint>

namespace canopen_hw {

// 总线写入抽象层，隔离 Lely PDO 依赖以便单元测试。
class BusIO {
 public:
  virtual ~BusIO() = default;
  virtual bool WriteControlword(uint16_t cw) = 0;
  virtual bool WriteTargetPosition(int32_t pos) = 0;
  virtual bool WriteTargetVelocity(int32_t vel) = 0;
  virtual bool WriteTargetTorque(int16_t torque) = 0;
  virtual bool WriteModeOfOperation(int8_t mode) = 0;
};

}  // namespace canopen_hw
