#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

#include "canopen_hw/zero_soft_limit_executor.hpp"

namespace canopen_hw {
namespace {

std::vector<uint8_t> PackLeI32(int32_t value) {
  const uint32_t u = static_cast<uint32_t>(value);
  return {static_cast<uint8_t>(u & 0xFF), static_cast<uint8_t>((u >> 8) & 0xFF),
          static_cast<uint8_t>((u >> 16) & 0xFF), static_cast<uint8_t>((u >> 24) & 0xFF)};
}

TEST(ZeroSoftLimitExecutor, RadToCountsRoundsWithSign) {
  int32_t counts = 0;
  std::string error;

  ASSERT_TRUE(ZeroSoftLimitExecutor::RadToCounts(M_PI, 5308416.0, &counts, &error));
  EXPECT_EQ(counts, 2654208);

  ASSERT_TRUE(ZeroSoftLimitExecutor::RadToCounts(-M_PI / 2.0, 5308416.0, &counts, &error));
  EXPECT_EQ(counts, -1327104);
}

TEST(ZeroSoftLimitExecutor, SetCurrentPositionAsZeroWritesExpectedSequence) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 5;
  joint.counts_per_rev = 5308416.0;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  std::vector<std::string> op_log;
  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [&](uint8_t node_id, uint16_t index, uint8_t subindex,
                 std::chrono::milliseconds, std::size_t expected_size) -> SdoResult {
    op_log.push_back("R " + std::to_string(node_id) + " " + std::to_string(index) +
                     " " + std::to_string(subindex) + " " +
                     std::to_string(expected_size));
    if (index == ZeroSoftLimitExecutor::kObj_PositionActual && subindex == 0 &&
        expected_size == 4) {
      return SdoResult{true, PackLeI32(12345), {}};
    }
    return SdoResult{false, {}, "unexpected read"};
  };
  ops.write_u32 = [&](uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value,
                      std::chrono::milliseconds) -> SdoResult {
    op_log.push_back("W " + std::to_string(node_id) + " " + std::to_string(index) +
                     " " + std::to_string(subindex) + " " +
                     std::to_string(value));
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  std::string detail;
  ASSERT_TRUE(executor.SetCurrentPositionAsZero(0, &detail)) << detail;

  const uint32_t expected_offset = static_cast<uint32_t>(-12345);
  std::vector<std::string> expected = {
      "W 5 " + std::to_string(ZeroSoftLimitExecutor::kObj_HomeOffset) + " 0 0",
      "R 5 24676 0 4",
      "W 5 " + std::to_string(ZeroSoftLimitExecutor::kObj_HomeOffset) + " 0 " +
          std::to_string(expected_offset),
      "W 5 4112 1 1702257011"};
  EXPECT_EQ(op_log, expected);
}

TEST(ZeroSoftLimitExecutor, ReadHomeOffsetReadsSignedValue) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 5;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [&](uint8_t node_id, uint16_t index, uint8_t subindex,
                 std::chrono::milliseconds, std::size_t expected_size) -> SdoResult {
    EXPECT_EQ(node_id, 5);
    EXPECT_EQ(index, ZeroSoftLimitExecutor::kObj_HomeOffset);
    EXPECT_EQ(subindex, 0);
    EXPECT_EQ(expected_size, 4u);
    return SdoResult{true, PackLeI32(-321), {}};
  };
  ops.write_u32 = [](uint8_t, uint16_t, uint8_t, uint32_t,
                     std::chrono::milliseconds) -> SdoResult {
    return SdoResult{false, {}, "unused"};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  int32_t home_offset = 0;
  std::string detail;
  ASSERT_TRUE(executor.ReadHomeOffset(0, &home_offset, &detail)) << detail;
  EXPECT_EQ(home_offset, -321);
}

TEST(ZeroSoftLimitExecutor, RestoreHomeOffsetWritesOffsetAndSave) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 5;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  std::vector<std::string> op_log;
  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [&](uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value,
                      std::chrono::milliseconds) -> SdoResult {
    op_log.push_back("W " + std::to_string(node_id) + " " + std::to_string(index) +
                     " " + std::to_string(subindex) + " " +
                     std::to_string(value));
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  std::string detail;
  ASSERT_TRUE(executor.RestoreHomeOffset(0, -123, &detail)) << detail;

  const std::vector<std::string> expected = {
      "W 5 " + std::to_string(ZeroSoftLimitExecutor::kObj_HomeOffset) + " 0 " +
          std::to_string(static_cast<uint32_t>(-123)),
      "W 5 " + std::to_string(ZeroSoftLimitExecutor::kObj_StoreParameters) +
          " 1 " + std::to_string(ZeroSoftLimitExecutor::kStoreSaveSignature)};
  EXPECT_EQ(op_log, expected);
}

TEST(ZeroSoftLimitExecutor, PrepareSoftLimitRadiansUsesCountsPerRev) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 7;
  joint.counts_per_rev = 360.0;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [](uint8_t, uint16_t, uint8_t, uint32_t,
                     std::chrono::milliseconds) -> SdoResult {
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  int32_t min_counts = 0;
  int32_t max_counts = 0;
  std::string detail;
  ASSERT_TRUE(executor.PrepareSoftLimitRadians(0, 0.0, M_PI, &min_counts,
                                               &max_counts, &detail))
      << detail;
  EXPECT_EQ(min_counts, 0);
  EXPECT_EQ(max_counts, 180);
}

TEST(ZeroSoftLimitExecutor, PrepareSoftLimitMetersUsesCountsPerMeter) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 9;
  joint.counts_per_meter = 10000.0;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [](uint8_t, uint16_t, uint8_t, uint32_t,
                     std::chrono::milliseconds) -> SdoResult {
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  int32_t min_counts = 0;
  int32_t max_counts = 0;
  std::string detail;
  ASSERT_TRUE(executor.PrepareSoftLimitMeters(0, -0.2, 0.3, &min_counts,
                                              &max_counts, &detail))
      << detail;
  EXPECT_EQ(min_counts, -2000);
  EXPECT_EQ(max_counts, 3000);
}

TEST(ZeroSoftLimitExecutor, ApplySoftLimitCountsWritesExpectedSequence) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 6;
  joint.counts_per_rev = 5308416.0;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  std::vector<std::tuple<uint8_t, uint16_t, uint8_t, uint32_t>> writes;
  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [&](uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value,
                      std::chrono::milliseconds) -> SdoResult {
    writes.emplace_back(node_id, index, subindex, value);
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  std::string detail;
  ASSERT_TRUE(executor.ApplySoftLimitCounts(0, -100, 200, &detail)) << detail;

  ASSERT_EQ(writes.size(), 3u);
  const auto expected0 = std::make_tuple(
      static_cast<uint8_t>(6), ZeroSoftLimitExecutor::kObj_SoftLimitState,
      static_cast<uint8_t>(0), ZeroSoftLimitExecutor::kSoftLimitEnableMagic);
  const auto expected1 = std::make_tuple(
      static_cast<uint8_t>(6), ZeroSoftLimitExecutor::kObj_SoftwarePositionLimit,
      static_cast<uint8_t>(2), static_cast<uint32_t>(200));
  const auto expected2 = std::make_tuple(
      static_cast<uint8_t>(6), ZeroSoftLimitExecutor::kObj_SoftwarePositionLimit,
      static_cast<uint8_t>(1), static_cast<uint32_t>(-100));
  EXPECT_EQ(writes[0], expected0);
  EXPECT_EQ(writes[1], expected1);
  EXPECT_EQ(writes[2], expected2);
}

TEST(ZeroSoftLimitExecutor, DisableSoftLimitWritesOnlySoftLimitState) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 6;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  std::vector<std::tuple<uint8_t, uint16_t, uint8_t, uint32_t>> writes;
  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [&](uint8_t node_id, uint16_t index, uint8_t subindex, uint32_t value,
                      std::chrono::milliseconds) -> SdoResult {
    writes.emplace_back(node_id, index, subindex, value);
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  std::string detail;
  ASSERT_TRUE(executor.DisableSoftLimit(0, &detail)) << detail;

  ASSERT_EQ(writes.size(), 1u);
  const auto expected = std::make_tuple(
      static_cast<uint8_t>(6), ZeroSoftLimitExecutor::kObj_SoftLimitState,
      static_cast<uint8_t>(0), 0u);
  EXPECT_EQ(writes[0], expected);
}

TEST(ZeroSoftLimitExecutor, ApplySoftLimitRadiansUsesCountsPerRev) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 7;
  joint.counts_per_rev = 360.0;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  std::vector<std::tuple<uint16_t, uint8_t, uint32_t>> writes;
  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [&](uint8_t, uint16_t index, uint8_t subindex, uint32_t value,
                      std::chrono::milliseconds) -> SdoResult {
    writes.emplace_back(index, subindex, value);
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  std::string detail;
  ASSERT_TRUE(executor.ApplySoftLimitRadians(0, 0.0, M_PI, &detail)) << detail;

  ASSERT_EQ(writes.size(), 3u);
  const auto expected1 = std::make_tuple(
      ZeroSoftLimitExecutor::kObj_SoftwarePositionLimit, static_cast<uint8_t>(2),
      static_cast<uint32_t>(180));
  const auto expected2 = std::make_tuple(
      ZeroSoftLimitExecutor::kObj_SoftwarePositionLimit, static_cast<uint8_t>(1),
      static_cast<uint32_t>(0));
  EXPECT_EQ(writes[1], expected1);
  EXPECT_EQ(writes[2], expected2);
}

TEST(ZeroSoftLimitExecutor, ApplySoftLimitMetersUsesCountsPerMeter) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 9;
  joint.counts_per_meter = 10000.0;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  std::vector<std::tuple<uint16_t, uint8_t, uint32_t>> writes;
  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [&](uint8_t, uint16_t index, uint8_t subindex, uint32_t value,
                      std::chrono::milliseconds) -> SdoResult {
    writes.emplace_back(index, subindex, value);
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  std::string detail;
  ASSERT_TRUE(executor.ApplySoftLimitMeters(0, -0.2, 0.3, &detail)) << detail;

  ASSERT_EQ(writes.size(), 3u);
  const auto expected1 = std::make_tuple(
      ZeroSoftLimitExecutor::kObj_SoftwarePositionLimit, static_cast<uint8_t>(2),
      static_cast<uint32_t>(3000));
  const auto expected2 = std::make_tuple(
      ZeroSoftLimitExecutor::kObj_SoftwarePositionLimit, static_cast<uint8_t>(1),
      static_cast<uint32_t>(-2000));
  EXPECT_EQ(writes[1], expected1);
  EXPECT_EQ(writes[2], expected2);
}

TEST(ZeroSoftLimitExecutor, ApplySoftLimitMetersRejectsMissingScale) {
  CanopenMasterConfig cfg;
  CanopenMasterConfig::JointConfig joint;
  joint.node_id = 9;
  joint.counts_per_meter = 0.0;
  cfg.joints = {joint};
  cfg.axis_count = 1;

  ZeroSoftLimitExecutor::Ops ops;
  ops.read = [](uint8_t, uint16_t, uint8_t, std::chrono::milliseconds,
                std::size_t) -> SdoResult { return SdoResult{false, {}, "unused"}; };
  ops.write_u32 = [](uint8_t, uint16_t, uint8_t, uint32_t,
                     std::chrono::milliseconds) -> SdoResult {
    return SdoResult{true, {}, {}};
  };

  ZeroSoftLimitExecutor executor(&cfg, ops);
  std::string detail;
  EXPECT_FALSE(executor.ApplySoftLimitMeters(0, -0.2, 0.3, &detail));
  EXPECT_NE(detail.find("counts_per_meter"), std::string::npos);
}

}  // namespace
}  // namespace canopen_hw
