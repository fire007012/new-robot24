#include <gtest/gtest.h>

#include <cstdint>

#include "canopen_hw/position_channel_routing.hpp"

namespace canopen_hw {
namespace detail {
namespace {

TEST(PositionChannelRouting, IpModeWritesBothChannelsWhenIpChannelSucceeds) {
  int ip_calls = 0;
  int legacy_calls = 0;
  int warns = 0;

  const bool ok = WritePositionChannels(
      kMode_IP, 1234,
      [&](int32_t pos) {
        ++ip_calls;
        EXPECT_EQ(pos, 1234);
        return true;
      },
      [&](int32_t pos) {
        ++legacy_calls;
        EXPECT_EQ(pos, 1234);
        return true;
      },
      [&]() { ++warns; });

  EXPECT_TRUE(ok);
  EXPECT_EQ(ip_calls, 1);
  EXPECT_EQ(legacy_calls, 1);
  EXPECT_EQ(warns, 0);
}

TEST(PositionChannelRouting, IpModeWarnsAndFallsBackWhenIpChannelFails) {
  int ip_calls = 0;
  int legacy_calls = 0;
  int warns = 0;

  const bool ok = WritePositionChannels(
      kMode_IP, 5678,
      [&](int32_t pos) {
        ++ip_calls;
        EXPECT_EQ(pos, 5678);
        return false;
      },
      [&](int32_t pos) {
        ++legacy_calls;
        EXPECT_EQ(pos, 5678);
        return true;
      },
      [&]() { ++warns; });

  EXPECT_TRUE(ok);
  EXPECT_EQ(ip_calls, 1);
  EXPECT_EQ(legacy_calls, 1);
  EXPECT_EQ(warns, 1);
}

TEST(PositionChannelRouting, CspModeWritesLegacyOnly) {
  int ip_calls = 0;
  int legacy_calls = 0;
  int warns = 0;

  const bool ok = WritePositionChannels(
      kMode_CSP, 99,
      [&](int32_t) {
        ++ip_calls;
        return true;
      },
      [&](int32_t pos) {
        ++legacy_calls;
        EXPECT_EQ(pos, 99);
        return true;
      },
      [&]() { ++warns; });

  EXPECT_TRUE(ok);
  EXPECT_EQ(ip_calls, 0);
  EXPECT_EQ(legacy_calls, 1);
  EXPECT_EQ(warns, 0);
}

}  // namespace
}  // namespace detail
}  // namespace canopen_hw
