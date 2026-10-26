#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <lely/coapp/driver.hpp>

#include "canopen_hw/axis_logic.hpp"
#include "canopen_hw/bus_io.hpp"
#include "canopen_hw/sdo_single_flight.hpp"
#include "canopen_hw/shared_state.hpp"

namespace canopen_hw {

struct PdoMapping;
class PdoMappingReader;

// 单轴 Lely 适配器:
// - 继承 Lely BasicDriver，挂接节点事件回调
// - 实现 BusIO 接口，将 PDO 写入委托给 tpdo_mapped
// - 核心逻辑委托给 AxisLogic（状态机、反馈缓存、健康计数）
class AxisDriver final : public lely::canopen::BasicDriver, public BusIO {
 public:
  AxisDriver(lely::canopen::BasicMaster& can_master, uint8_t node_id,
             std::size_t axis_index, SharedState* shared_state,
             bool verify_pdo_mapping, const std::string& dcf_path,
             uint8_t ip_interpolation_period_ms, int8_t default_mode);

  // 显式反馈注入（无硬件场景下的逻辑验证入口）。
  void InjectFeedback(int32_t actual_position, int32_t actual_velocity,
                      int16_t actual_torque, uint16_t statusword,
                      int8_t mode_display);

  // BusIO 实现。
  bool WriteControlword(uint16_t cw) override;
  bool WriteTargetPosition(int32_t pos) override;
  bool WriteTargetVelocity(int32_t vel) override;
  bool WriteTargetTorque(int16_t torque) override;
  bool WriteModeOfOperation(int8_t mode) override;

  // 兼容别名（关机流程等外部调用）。
  bool SendControlword(uint16_t cw) { return WriteControlword(cw); }
  bool SendTargetPosition(int32_t pos) { return WriteTargetPosition(pos); }
  bool SendTargetVelocity(int32_t vel) { return WriteTargetVelocity(vel); }
  bool SendTargetTorque(int16_t torque) { return WriteTargetTorque(torque); }
  bool SendModeOfOperation(int8_t mode) { return WriteModeOfOperation(mode); }
  bool SendNmtStopAll();

  CiA402State feedback_state() const;
  void ConfigureStateMachine(int32_t position_lock_threshold,
                             int max_fault_resets,
                             int fault_reset_hold_cycles);
  const HealthCounters& health() const { return logic_.health(); }
  HealthCounters& mutable_health() { return logic_.mutable_health(); }

  // 手动控制接口（由 CanopenMaster 从上层线程调用）。
  void RequestEnable();
  void RequestDisable();
  void RequestHalt();
  void RequestResume();
  void ResetFault();

  // SDO 异步读写（由 SdoAccessor 通过 CanopenMaster 调用）。
  using SdoReadCallback =
      std::function<void(bool ok, const std::vector<uint8_t>& data,
                         const std::string& error)>;
  using SdoWriteCallback =
      std::function<void(bool ok, const std::string& error)>;

  void AsyncSdoRead(uint16_t index, uint8_t subindex, SdoReadCallback cb,
                    std::size_t expected_size = 4);
  void AsyncSdoWrite(uint16_t index, uint8_t subindex,
                     const std::vector<uint8_t>& data, SdoWriteCallback cb);
  bool WaitForSdoIdle(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) const;
  bool sdo_idle() const { return sdo_queue_.IsIdle(); }
  bool WaitForStartupComplete(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) const;
  bool startup_complete() const {
    return startup_complete_.load(std::memory_order_acquire);
  }

 private:
  void SetStartupComplete(bool complete);
  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;
  void OnHeartbeat(bool occurred) noexcept override;
  void OnBoot(lely::canopen::NmtState st, char es,
              const std::string& what) noexcept override;

  std::size_t axis_index_ = 0;
  SharedState* shared_state_ = nullptr;
  AxisLogic logic_;

  // PDO 验证相关。
  bool verify_pdo_mapping_ = false;
  std::atomic<bool> pdo_verified_{true};
  std::atomic<bool> pdo_verification_done_{false};
  std::string dcf_path_;
  uint8_t ip_interpolation_period_ms_ = 1;
  std::shared_ptr<PdoMapping> expected_pdo_;
  bool expected_pdo_loaded_ = false;
  std::shared_ptr<PdoMappingReader> pdo_reader_;
  SdoSingleFlightQueue sdo_queue_;
  mutable std::mutex startup_mtx_;
  mutable std::condition_variable startup_cv_;
  std::atomic<bool> startup_complete_{false};
  std::atomic<int> boot_retry_count_{0};
  std::atomic<bool> ip_target_fallback_warned_{false};
  std::atomic<bool> csv_target_unmapped_warned_{false};
  int max_boot_retries_ = 3;
};

}  // namespace canopen_hw
