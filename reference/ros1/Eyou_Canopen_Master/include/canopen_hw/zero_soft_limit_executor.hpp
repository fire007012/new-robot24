#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/sdo_accessor.hpp"

namespace canopen_hw {

class ZeroSoftLimitExecutor {
 public:
  static constexpr uint16_t kObj_HomeOffset = 0x607C;
  static constexpr uint16_t kObj_PositionActual = 0x6064;
  static constexpr uint16_t kObj_StoreParameters = 0x1010;
  static constexpr uint16_t kObj_SoftLimitState = 0x2003;
  static constexpr uint16_t kObj_SoftwarePositionLimit = 0x607D;

  static constexpr uint32_t kStoreSaveSignature = 0x65766173;  // "save"
  static constexpr uint32_t kSoftLimitEnableMagic = 0x4C494D54;  // "LIMT"

  struct Ops {
    std::function<SdoResult(uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                            std::size_t)>
        read;
    std::function<SdoResult(uint8_t, uint16_t, uint8_t, uint32_t,
                            std::chrono::milliseconds)>
        write_u32;
  };

  ZeroSoftLimitExecutor(CanopenMaster* master, const CanopenMasterConfig* config);
  ZeroSoftLimitExecutor(const CanopenMasterConfig* config, Ops ops);

  bool SetCurrentPositionAsZero(std::size_t axis_index, std::string* detail = nullptr);
  bool SetHomeOffsetRadians(std::size_t axis_index, double offset_rad,
                            std::string* detail = nullptr);
  bool SetHomeOffsetMeters(std::size_t axis_index, double offset_meters,
                           std::string* detail = nullptr);
  bool ReadHomeOffset(std::size_t axis_index, int32_t* out,
                      std::string* detail = nullptr);
  bool RestoreHomeOffset(std::size_t axis_index, int32_t home_offset,
                         std::string* detail = nullptr);
  bool PrepareSoftLimitRadians(std::size_t axis_index, double min_rad, double max_rad,
                               int32_t* min_counts, int32_t* max_counts,
                               std::string* detail = nullptr);
  bool PrepareSoftLimitMeters(std::size_t axis_index, double min_meters,
                              double max_meters, int32_t* min_counts,
                              int32_t* max_counts, std::string* detail = nullptr);

  bool ApplySoftLimitCounts(std::size_t axis_index, int32_t min_counts,
                            int32_t max_counts, std::string* detail = nullptr);
  bool DisableSoftLimit(std::size_t axis_index, std::string* detail = nullptr);
  bool ApplySoftLimitRadians(std::size_t axis_index, double min_rad, double max_rad,
                             std::string* detail = nullptr);
  bool ApplySoftLimitMeters(std::size_t axis_index, double min_meters,
                            double max_meters, std::string* detail = nullptr);

  static bool RadToCounts(double rad, double counts_per_rev, int32_t* out,
                          std::string* error = nullptr);
  static bool MetersToCounts(double meters, double counts_per_meter, int32_t* out,
                             std::string* error = nullptr);

 private:
  bool ValidateAxis(std::size_t axis_index, std::string* detail) const;
  bool ValidatePreparedSoftLimitOutput(int32_t* min_counts, int32_t* max_counts,
                                       std::string* detail) const;
  bool ValidatePreparedSoftLimitRange(std::size_t axis_index, int32_t min_counts,
                                      int32_t max_counts, std::string* detail) const;
  bool WriteHomeOffset(std::size_t axis_index, int32_t home_offset, const char* step,
                       std::string* detail);
  bool StoreParameters(std::size_t axis_index, const char* step, std::string* detail);
  bool WriteU32(std::size_t axis_index, uint16_t index, uint8_t subindex,
                uint32_t value, const char* step, std::string* detail);
  bool ReadI32(std::size_t axis_index, uint16_t index, uint8_t subindex, int32_t* out,
               const char* step, std::string* detail);

  static void SetError(std::string* detail, const std::string& message);

  const CanopenMasterConfig* config_ = nullptr;  // non-owning
  CanopenMaster* master_ = nullptr;              // non-owning
  SdoAccessor sdo_accessor_{nullptr};
  Ops ops_;
};

}  // namespace canopen_hw
