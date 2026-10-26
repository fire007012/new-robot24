#include "canopen_hw/axis_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <system_error>

#include <lely/coapp/node.hpp>

#include "canopen_hw/boot_identity_diag.hpp"
#include "canopen_hw/logging.hpp"
#include "canopen_hw/pdo_mapping.hpp"
#include "canopen_hw/position_channel_routing.hpp"

namespace canopen_hw {

namespace {

struct BootIdentityDiagResult {
  bool has_device_type = false;
  uint32_t device_type = 0;
  std::string device_type_error;

  bool has_vendor_id = false;
  uint32_t vendor_id = 0;
  std::string vendor_id_error;

  bool has_product_code = false;
  uint32_t product_code = 0;
  std::string product_code_error;

  bool has_revision = false;
  uint32_t revision = 0;
  std::string revision_error;

  BootIdentityTuple expected;
  bool has_expected = false;
};

uint32_t DecodeLeU32(const std::vector<uint8_t>& data) {
  uint32_t value = 0;
  const std::size_t n = std::min<std::size_t>(data.size(), 4);
  for (std::size_t i = 0; i < n; ++i) {
    value |= static_cast<uint32_t>(data[i]) << (8 * i);
  }
  return value;
}

std::string Hex32(uint32_t value) {
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(8)
      << std::setfill('0') << value;
  return oss.str();
}

std::string FormatDiagValue(bool ok, uint32_t value, const std::string& error) {
  if (ok) {
    return Hex32(value);
  }
  if (!error.empty()) {
    return std::string("n/a(") + error + ")";
  }
  return "n/a";
}

std::vector<uint8_t> PackLe(uint32_t value, std::size_t size) {
  std::vector<uint8_t> data(size, 0);
  for (std::size_t i = 0; i < size; ++i) {
    data[i] = static_cast<uint8_t>((value >> (i * 8)) & 0xFFu);
  }
  return data;
}

}  // namespace

AxisDriver::AxisDriver(lely::canopen::BasicMaster& can_master, uint8_t node_id,
                       std::size_t axis_index, SharedState* shared_state,
                       bool verify_pdo_mapping, const std::string& dcf_path,
                       uint8_t ip_interpolation_period_ms, int8_t default_mode)
    : lely::canopen::BasicDriver(can_master, node_id),
      axis_index_(axis_index),
      shared_state_(shared_state),
      logic_(axis_index, this, shared_state),
      verify_pdo_mapping_(verify_pdo_mapping),
      dcf_path_(dcf_path),
      ip_interpolation_period_ms_(ip_interpolation_period_ms) {
  logic_.SetTargetMode(default_mode);

  pdo_verified_.store(!verify_pdo_mapping_);
  pdo_verification_done_.store(!verify_pdo_mapping_);
  if (verify_pdo_mapping_) {
    expected_pdo_ = std::make_shared<PdoMapping>();
    if (dcf_path_.empty()) {
      CANOPEN_LOG_WARN("axis={} node={}: DCF path empty, skip PDO verify",
                       axis_index_, static_cast<int>(id()));
      pdo_verified_.store(false);
      pdo_verification_done_.store(true);
    } else {
      std::string err;
      if (LoadExpectedPdoMappingFromDcf(dcf_path_, expected_pdo_.get(), &err)) {
        expected_pdo_loaded_ = true;
        pdo_verified_.store(false);
        pdo_verification_done_.store(false);
      } else {
        CANOPEN_LOG_ERROR("axis={} node={}: DCF load failed: {}",
                          axis_index_, static_cast<int>(id()), err);
        pdo_verified_.store(false);
        pdo_verification_done_.store(true);
      }
    }
  }
}

void AxisDriver::InjectFeedback(int32_t actual_position, int32_t actual_velocity,
                                int16_t actual_torque, uint16_t statusword,
                                int8_t mode_display) {
  logic_.ProcessRpdo(statusword, actual_position, actual_velocity,
                     actual_torque, mode_display);
}

// --- BusIO implementation ---

bool AxisDriver::WriteControlword(uint16_t cw) {
  std::error_code ec;
  tpdo_mapped[0x6040][0].Write(cw, ec);
  if (ec) return false;
  tpdo_mapped[0x6040][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::WriteTargetPosition(int32_t pos) {
  const int8_t target_mode = logic_.target_mode();
  return detail::WritePositionChannels(
      target_mode, pos,
      [this](int32_t target_pos) {
        std::error_code ec;
        tpdo_mapped[0x60C1][1].Write(target_pos, ec);
        if (ec) {
          return false;
        }
        tpdo_mapped[0x60C1][1].WriteEvent(ec);
        return !ec;
      },
      [this](int32_t target_pos) {
        std::error_code ec;
        tpdo_mapped[0x607A][0].Write(target_pos, ec);
        if (ec) {
          return false;
        }
        tpdo_mapped[0x607A][0].WriteEvent(ec);
        return !ec;
      },
      [this]() {
        const bool warned = ip_target_fallback_warned_.exchange(true);
        if (!warned) {
          CANOPEN_LOG_WARN(
              "axis={} node={}: write 0x60C1:01 failed, mirror only 0x607A",
              axis_index_, static_cast<int>(id()));
        }
      });
}

bool AxisDriver::WriteTargetVelocity(int32_t vel) {
  std::error_code ec;
  tpdo_mapped[0x60FF][0].Write(vel, ec);
  if (ec) {
    const bool warned = csv_target_unmapped_warned_.exchange(true);
    if (!warned) {
      CANOPEN_LOG_WARN(
          "axis={} node={}: write 0x60FF failed; CSV target velocity is not mapped in current DCF/PDO config",
          axis_index_, static_cast<int>(id()));
    }
    return false;
  }
  tpdo_mapped[0x60FF][0].WriteEvent(ec);
  if (!ec) {
    csv_target_unmapped_warned_.store(false);
  }
  return !ec;
}

bool AxisDriver::WriteTargetTorque(int16_t torque) {
  std::error_code ec;
  tpdo_mapped[0x6071][0].Write(torque, ec);
  if (ec) return false;
  tpdo_mapped[0x6071][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::WriteModeOfOperation(int8_t mode) {
  std::error_code ec;
  tpdo_mapped[0x6060][0].Write(mode, ec);
  if (ec) return false;
  tpdo_mapped[0x6060][0].WriteEvent(ec);
  return !ec;
}

bool AxisDriver::SendNmtStopAll() {
  master.Command(lely::canopen::NmtCommand::STOP);
  return true;
}

// --- Delegated to AxisLogic ---

CiA402State AxisDriver::feedback_state() const {
  return logic_.feedback_state();
}

void AxisDriver::ConfigureStateMachine(int32_t position_lock_threshold,
                                       int max_fault_resets,
                                       int fault_reset_hold_cycles) {
  logic_.Configure(position_lock_threshold, max_fault_resets,
                   fault_reset_hold_cycles);
}

void AxisDriver::RequestEnable() { logic_.RequestEnable(); }
void AxisDriver::RequestDisable() { logic_.RequestDisable(); }
void AxisDriver::RequestHalt() { logic_.RequestHalt(); }
void AxisDriver::RequestResume() { logic_.RequestResume(); }
void AxisDriver::ResetFault() { logic_.ResetFault(); }

// --- SDO (stays in AxisDriver, Lely-specific) ---

void AxisDriver::AsyncSdoRead(uint16_t index, uint8_t subindex,
                              SdoReadCallback cb,
                              std::size_t expected_size) {
  if (expected_size == 0 || expected_size > 4) {
    if (cb) {
      cb(false, {}, "unsupported SDO read size (expected 1..4 bytes)");
    }
    return;
  }

  sdo_queue_.Enqueue(
      [this, index, subindex, expected_size,
       cb = std::move(cb)](SdoSingleFlightQueue::Completion done) mutable {
        try {
          if (expected_size == 1) {
            SubmitRead<uint8_t>(
                index, subindex,
                [cb, done](
                    uint8_t, uint16_t, uint8_t, std::error_code ec,
                    uint8_t value) mutable {
                  if (ec) {
                    if (cb) cb(false, {}, ec.message());
                    done();
                    return;
                  }
                  if (cb) cb(true, {value}, std::string());
                  done();
                });
            return;
          }

          if (expected_size == 2) {
            SubmitRead<uint16_t>(
                index, subindex,
                [cb, done](
                    uint8_t, uint16_t, uint8_t, std::error_code ec,
                    uint16_t value) mutable {
                  if (ec) {
                    if (cb) cb(false, {}, ec.message());
                    done();
                    return;
                  }
                  if (cb) cb(true, PackLe(value, 2), std::string());
                  done();
                });
            return;
          }

          // 3/4 字节统一按 u32 读取，再按请求长度裁剪。
          SubmitRead<uint32_t>(
              index, subindex,
              [cb, done, expected_size](
                  uint8_t, uint16_t, uint8_t, std::error_code ec,
                  uint32_t value) mutable {
                if (ec) {
                  if (cb) cb(false, {}, ec.message());
                  done();
                  return;
                }
                if (cb) cb(true, PackLe(value, expected_size), std::string());
                done();
              });
        } catch (const std::exception& e) {
          if (cb) {
            cb(false, {}, e.what());
          }
          done();
        } catch (...) {
          if (cb) {
            cb(false, {}, "unknown exception during SubmitRead");
          }
          done();
        }
      });
}

void AxisDriver::AsyncSdoWrite(uint16_t index, uint8_t subindex,
                                const std::vector<uint8_t>& data,
                                SdoWriteCallback cb) {
  if (data.empty()) {
    if (cb) cb(false, "empty SDO write payload");
    return;
  }
  if (data.size() > 4) {
    if (cb) cb(false, "unsupported SDO write size (>4 bytes)");
    return;
  }

  uint32_t value = 0;
  if (data.size() >= 1) value |= static_cast<uint32_t>(data[0]);
  if (data.size() >= 2) value |= static_cast<uint32_t>(data[1]) << 8;
  if (data.size() >= 3) value |= static_cast<uint32_t>(data[2]) << 16;
  if (data.size() >= 4) value |= static_cast<uint32_t>(data[3]) << 24;

  sdo_queue_.Enqueue(
      [this, index, subindex, value, size = data.size(),
       cb = std::move(cb)](SdoSingleFlightQueue::Completion done) mutable {
        try {
          if (size == 1) {
            SubmitWrite(
                index, subindex, static_cast<uint8_t>(value),
                [cb, done](
                    uint8_t, uint16_t, uint8_t, std::error_code ec) mutable {
                  if (cb) cb(!ec, ec ? ec.message() : std::string());
                  done();
                });
            return;
          }

          if (size == 2) {
            SubmitWrite(
                index, subindex, static_cast<uint16_t>(value),
                [cb, done](
                    uint8_t, uint16_t, uint8_t, std::error_code ec) mutable {
                  if (cb) cb(!ec, ec ? ec.message() : std::string());
                  done();
                });
            return;
          }

          SubmitWrite(
              index, subindex, value,
              [cb, done](
                  uint8_t, uint16_t, uint8_t, std::error_code ec) mutable {
                if (cb) cb(!ec, ec ? ec.message() : std::string());
                done();
              });
        } catch (const std::exception& e) {
          if (cb) {
            cb(false, e.what());
          }
          done();
        } catch (...) {
          if (cb) {
            cb(false, "unknown exception during SubmitWrite");
          }
          done();
        }
      });
}

bool AxisDriver::WaitForSdoIdle(std::chrono::milliseconds timeout) const {
  return sdo_queue_.WaitForIdle(timeout);
}

bool AxisDriver::WaitForStartupComplete(std::chrono::milliseconds timeout) const {
  if (startup_complete_.load(std::memory_order_acquire)) {
    return true;
  }
  std::unique_lock<std::mutex> lk(startup_mtx_);
  return startup_cv_.wait_for(lk, timeout, [this]() {
    return startup_complete_.load(std::memory_order_acquire);
  });
}

void AxisDriver::SetStartupComplete(bool complete) {
  startup_complete_.store(complete, std::memory_order_release);
  if (complete) {
    std::lock_guard<std::mutex> lk(startup_mtx_);
    startup_cv_.notify_all();
  }
}

// --- Lely callbacks ---

void AxisDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept {
  (void)idx;
  (void)subidx;

  // 从 SharedState 读取上层命令包并注入 AxisLogic。
  if (shared_state_) {
    AxisCommand cmd;
    if (shared_state_->GetCommand(axis_index_, &cmd)) {
      logic_.SetExternalCommand(cmd);
      logic_.SetGlobalFault(shared_state_->GetGlobalFault());
    }
  }

  // 读取 RPDO 反馈字段。
  std::error_code ec;
  const auto statusword = rpdo_mapped[0x6041][0].Read<uint16_t>(ec);
  if (ec) return;
  const auto actual_position = rpdo_mapped[0x6064][0].Read<int32_t>(ec);
  if (ec) return;
  const auto mode_display = rpdo_mapped[0x6061][0].Read<int8_t>(ec);
  if (ec) return;
  const auto actual_velocity = rpdo_mapped[0x606C][0].Read<int32_t>(ec);
  if (ec) return;
  const auto actual_torque = rpdo_mapped[0x6077][0].Read<int16_t>(ec);
  if (ec) return;

  logic_.ProcessRpdo(statusword, actual_position, actual_velocity,
                     actual_torque, mode_display);
}

void AxisDriver::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept {
  (void)msef;
  logic_.ProcessEmcy(eec, er);
}

void AxisDriver::OnHeartbeat(bool occurred) noexcept {
  logic_.ProcessHeartbeat(occurred);
}

void AxisDriver::OnBoot(lely::canopen::NmtState st, char es,
                        const std::string& what) noexcept {
  (void)st;
  SetStartupComplete(false);

  if (es != 0) {
    CANOPEN_LOG_ERROR(
        "axis={} node={}: OnConfig failed (es={}) what='{}'",
        axis_index_, static_cast<int>(id()), static_cast<int>(es), what);

    int retry_count = boot_retry_count_.load();
    const int observed_retry = retry_count;

    const bool should_dump_identity =
        (observed_retry == 0 || observed_retry >= max_boot_retries_);
    if (should_dump_identity) {
      BootIdentityTuple expected_identity;
      std::string expected_err;
      const bool expected_ok =
          LoadExpectedBootIdentityFromDcf(dcf_path_, id(), &expected_identity,
                                          &expected_err);

      if (expected_ok) {
        CANOPEN_LOG_ERROR(
            "axis={} node={}: expected identity from master.dcf: "
            "1000:00={} 1018:01={} 1018:02={} 1018:03={}",
            axis_index_, static_cast<int>(id()),
            expected_identity.has_device_type ? Hex32(expected_identity.device_type)
                                              : std::string("n/a"),
            expected_identity.has_vendor_id ? Hex32(expected_identity.vendor_id)
                                            : std::string("n/a"),
            expected_identity.has_product_code
                ? Hex32(expected_identity.product_code)
                : std::string("n/a"),
            expected_identity.has_revision ? Hex32(expected_identity.revision)
                                           : std::string("n/a"));
      } else {
        CANOPEN_LOG_WARN(
            "axis={} node={}: expected identity unavailable (dcf={}): {}",
            axis_index_, static_cast<int>(id()), dcf_path_, expected_err);
      }

      auto diag = std::make_shared<BootIdentityDiagResult>();
      diag->expected = expected_identity;
      diag->has_expected = expected_ok;

      auto finalize_diag = [this, diag]() {
        CANOPEN_LOG_ERROR(
            "axis={} node={}: actual identity snapshot: "
            "1000:00={} 1018:01={} 1018:02={} 1018:03={}",
            axis_index_, static_cast<int>(id()),
            FormatDiagValue(diag->has_device_type, diag->device_type,
                            diag->device_type_error),
            FormatDiagValue(diag->has_vendor_id, diag->vendor_id,
                            diag->vendor_id_error),
            FormatDiagValue(diag->has_product_code, diag->product_code,
                            diag->product_code_error),
            FormatDiagValue(diag->has_revision, diag->revision,
                            diag->revision_error));

        if (!diag->has_expected) {
          return;
        }

        BootIdentityTuple actual_identity;
        actual_identity.has_device_type = diag->has_device_type;
        actual_identity.device_type = diag->device_type;
        actual_identity.has_vendor_id = diag->has_vendor_id;
        actual_identity.vendor_id = diag->vendor_id;
        actual_identity.has_product_code = diag->has_product_code;
        actual_identity.product_code = diag->product_code;
        actual_identity.has_revision = diag->has_revision;
        actual_identity.revision = diag->revision;

        const auto mismatch_fields =
            DiffBootIdentity(diag->expected, actual_identity);
        if (mismatch_fields.empty()) {
          return;
        }

        std::ostringstream oss;
        for (std::size_t i = 0; i < mismatch_fields.size(); ++i) {
          if (i > 0) {
            oss << ", ";
          }
          oss << mismatch_fields[i];
        }

        CANOPEN_LOG_ERROR(
            "axis={} node={}: boot identity mismatch fields: {}",
            axis_index_, static_cast<int>(id()), oss.str());
      };

      AsyncSdoRead(
          0x1000, 0,
          [this, diag, finalize_diag](bool ok, const std::vector<uint8_t>& data,
                                      const std::string& error) {
            if (ok) {
              diag->has_device_type = true;
              diag->device_type = DecodeLeU32(data);
            } else {
              diag->device_type_error = error;
            }

            AsyncSdoRead(
                0x1018, 1,
                [this, diag, finalize_diag](bool ok_1018_1,
                                            const std::vector<uint8_t>& data_1018_1,
                                            const std::string& error_1018_1) {
                  if (ok_1018_1) {
                    diag->has_vendor_id = true;
                    diag->vendor_id = DecodeLeU32(data_1018_1);
                  } else {
                    diag->vendor_id_error = error_1018_1;
                  }

                  AsyncSdoRead(
                      0x1018, 2,
                      [this, diag,
                       finalize_diag](bool ok_1018_2,
                                      const std::vector<uint8_t>& data_1018_2,
                                      const std::string& error_1018_2) {
                        if (ok_1018_2) {
                          diag->has_product_code = true;
                          diag->product_code = DecodeLeU32(data_1018_2);
                        } else {
                          diag->product_code_error = error_1018_2;
                        }

                        AsyncSdoRead(
                            0x1018, 3,
                            [diag, finalize_diag](
                                bool ok_1018_3,
                                const std::vector<uint8_t>& data_1018_3,
                                const std::string& error_1018_3) {
                              if (ok_1018_3) {
                                diag->has_revision = true;
                                diag->revision = DecodeLeU32(data_1018_3);
                              } else {
                                diag->revision_error = error_1018_3;
                              }
                              finalize_diag();
                            },
                            4);
                      },
                      4);
                },
                4);
          },
          4);
    }

    while (retry_count < max_boot_retries_ &&
           !boot_retry_count_.compare_exchange_weak(retry_count,
                                                    retry_count + 1)) {
    }
    if (retry_count < max_boot_retries_) {
      const int attempt = retry_count + 1;
      logic_.mutable_health().boot_retries.fetch_add(1,
                                                     std::memory_order_relaxed);
      CANOPEN_LOG_WARN(
          "axis={} node={}: OnConfig failed (es={}), retry {}/{} with RESET_NODE",
          axis_index_, static_cast<int>(id()), static_cast<int>(es), attempt,
          max_boot_retries_);
      master.Command(lely::canopen::NmtCommand::RESET_NODE, id());
      return;
    }
    CANOPEN_LOG_ERROR(
        "axis={} node={}: OnConfig failed (es={}), retries exhausted; "
        "mark PDO verify failed",
        axis_index_, static_cast<int>(id()), static_cast<int>(es));
    pdo_verified_.store(false);
    pdo_verification_done_.store(true);
    logic_.mutable_health().pdo_verify_fail.fetch_add(1,
                                                      std::memory_order_relaxed);
    return;
  }

  boot_retry_count_.store(0);
  logic_.ProcessHeartbeat(false);  // 节点重新 boot 成功，清除心跳丢失标记。

  // 节点刚进入 Operational 时，tpdo_mapped 缓冲区全零。
  // Lely 的 SYNC-triggered RPDO 在下一个 SYNC 到来时立即读取快照发送，
  // 而 OnRpdoWrite 的 WriteEvent 需要再等一个 SYNC 才能覆盖。
  // 这导致节点会在 boot 完成后的第一两个 SYNC 内收到 mode=0x00，
  // 进而清除 CSP 模式，造成 mode_display 异常归零。
  // 修复：在 OnBoot 成功路径中立即预写正确的初始值。
  {
    std::error_code ec;
    const int8_t target_mode = logic_.target_mode();
    // 写入当前目标模式，防止节点收到 mode=0 后清除模式设置。
    tpdo_mapped[0x6060][0].Write(target_mode, ec);
    if (!ec) {
      tpdo_mapped[0x6060][0].WriteEvent(ec);
    }
    if (ec) {
      CANOPEN_LOG_WARN("axis={} node={}: OnBoot pre-init mode write failed",
                       axis_index_, static_cast<int>(id()));
    }
    // 写入安全的初始 controlword（Shutdown=0x0006），
    // 驱动器在 SwitchOnDisabled/ReadyToSwitchOn 均可安全接受。
    ec.clear();
    tpdo_mapped[0x6040][0].Write(kCtrl_Shutdown, ec);
    if (!ec) {
      tpdo_mapped[0x6040][0].WriteEvent(ec);
    }
    if (ec) {
      CANOPEN_LOG_WARN("axis={} node={}: OnBoot pre-init controlword write failed",
                       axis_index_, static_cast<int>(id()));
    }
    CANOPEN_LOG_INFO(
        "axis={} node={}: OnBoot tpdo pre-initialized (mode={}, cw=Shutdown)",
        axis_index_, static_cast<int>(id()), static_cast<int>(target_mode));
  }

  AsyncSdoWrite(0x60C2, 1, {ip_interpolation_period_ms_},
                [this](bool ok, const std::string& error) {
                  if (!ok) {
                    CANOPEN_LOG_WARN(
                        "axis={} node={}: write 0x60C2:01 failed: {}",
                        axis_index_, static_cast<int>(id()), error);
                    if (!verify_pdo_mapping_) {
                      SetStartupComplete(true);
                    }
                    return;
                  }
                  CANOPEN_LOG_INFO(
                      "axis={} node={}: 0x60C2:01 set to {} ms",
                      axis_index_, static_cast<int>(id()),
                      static_cast<int>(ip_interpolation_period_ms_));
                  if (!verify_pdo_mapping_) {
                    SetStartupComplete(true);
                  }
                });

  if (!verify_pdo_mapping_) {
    pdo_verified_.store(true);
    pdo_verification_done_.store(true);
    return;
  }

  if (pdo_verification_done_.load()) {
    return;
  }

  if (!expected_pdo_loaded_ || !expected_pdo_) {
    CANOPEN_LOG_WARN("axis={} node={}: DCF not loaded, skip PDO verify",
                     axis_index_, static_cast<int>(id()));
    pdo_verified_.store(false);
    pdo_verification_done_.store(true);
    SetStartupComplete(true);
    return;
  }

  pdo_reader_ = std::make_shared<PdoMappingReader>();
  pdo_reader_->Start(
      [this](uint16_t index, uint8_t subindex, bool is_u8,
             PdoMappingReader::ReadValueCallback cb) {
        const std::size_t expected_size = is_u8 ? 1u : 4u;
        AsyncSdoRead(
            index, subindex,
            [cb = std::move(cb)](bool ok, const std::vector<uint8_t>& data,
                                 const std::string& error) mutable {
              if (!ok) {
                if (cb) {
                  cb(false, 0, error);
                }
                return;
              }
              if (data.empty() || data.size() > 4) {
                if (cb) {
                  cb(false, 0, "unexpected SDO read size");
                }
                return;
              }
              if (cb) {
                cb(true, DecodeLeU32(data), std::string());
              }
            },
            expected_size);
      },
      [this](bool ok, const std::string& error, const PdoMapping& actual) {
        if (!ok) {
          CANOPEN_LOG_ERROR("axis={} node={}: PDO read failed: {}",
                            axis_index_, static_cast<int>(id()), error);
          pdo_verified_.store(false);
          pdo_verification_done_.store(true);
          if (error.find("timeout") != std::string::npos) {
            logic_.mutable_health().pdo_verify_timeout.fetch_add(
                1, std::memory_order_relaxed);
          } else {
            logic_.mutable_health().pdo_verify_fail.fetch_add(
                1, std::memory_order_relaxed);
          }
          SetStartupComplete(true);
          return;
        }

        std::vector<std::string> diffs;
        if (!DiffPdoMapping(*expected_pdo_, actual, &diffs)) {
          CANOPEN_LOG_WARN("axis={} node={}: PDO mapping mismatch",
                           axis_index_, static_cast<int>(id()));
          for (const auto& diff : diffs) {
            CANOPEN_LOG_WARN("  {}", diff);
          }
          pdo_verified_.store(false);
          logic_.mutable_health().pdo_verify_fail.fetch_add(
              1, std::memory_order_relaxed);
        } else {
          CANOPEN_LOG_INFO("axis={} node={}: PDO mapping verified",
                           axis_index_, static_cast<int>(id()));
          pdo_verified_.store(true);
          logic_.mutable_health().pdo_verify_ok.fetch_add(
              1, std::memory_order_relaxed);
        }

        pdo_verification_done_.store(true);
        SetStartupComplete(true);
      },
      std::chrono::milliseconds(2000));
}

}  // namespace canopen_hw
