#include <gtest/gtest.h>

#include "canopen_hw/sdo_accessor.hpp"

using canopen_hw::SdoAccessor;
using canopen_hw::SdoResult;

// SdoResult 类型转换测试。

TEST(SdoResult, AsU8) {
  SdoResult r;
  r.ok = true;
  r.data = {0x42};
  EXPECT_EQ(r.as_u8(), 0x42);
}

TEST(SdoResult, AsU16LittleEndian) {
  SdoResult r;
  r.ok = true;
  r.data = {0x34, 0x12};
  EXPECT_EQ(r.as_u16(), 0x1234);
}

TEST(SdoResult, AsU32LittleEndian) {
  SdoResult r;
  r.ok = true;
  r.data = {0x78, 0x56, 0x34, 0x12};
  EXPECT_EQ(r.as_u32(), 0x12345678u);
}

TEST(SdoResult, AsI8Negative) {
  SdoResult r;
  r.ok = true;
  r.data = {0xFF};
  EXPECT_EQ(r.as_i8(), -1);
}

TEST(SdoResult, AsI16Negative) {
  SdoResult r;
  r.ok = true;
  r.data = {0x00, 0x80};  // -32768
  EXPECT_EQ(r.as_i16(), -32768);
}

TEST(SdoResult, AsI32Negative) {
  SdoResult r;
  r.ok = true;
  r.data = {0xFF, 0xFF, 0xFF, 0xFF};
  EXPECT_EQ(r.as_i32(), -1);
}

TEST(SdoResult, EmptyDataReturnsZero) {
  SdoResult r;
  r.ok = true;
  EXPECT_EQ(r.as_u8(), 0);
  EXPECT_EQ(r.as_u16(), 0u);
  EXPECT_EQ(r.as_u32(), 0u);
}

TEST(SdoResult, ShortDataFallback) {
  SdoResult r;
  r.ok = true;
  r.data = {0x42};
  // as_u16 with only 1 byte falls back to as_u8.
  EXPECT_EQ(r.as_u16(), 0x42);
  // as_u32 with only 1 byte falls back to as_u16 → as_u8.
  EXPECT_EQ(r.as_u32(), 0x42u);
}

// SdoAccessor 错误路径测试。

TEST(SdoAccessor, NullMasterAsyncReadCallsBack) {
  SdoAccessor accessor(nullptr);
  bool called = false;
  accessor.AsyncRead(1, 0x6041, 0, [&](const SdoResult& result) {
    called = true;
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.error, "master is null");
  });
  EXPECT_TRUE(called);
}

TEST(SdoAccessor, NullMasterAsyncWriteCallsBack) {
  SdoAccessor accessor(nullptr);
  bool called = false;
  accessor.AsyncWrite(1, 0x6040, 0, {0x06, 0x00}, [&](const SdoResult& result) {
    called = true;
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.error, "master is null");
  });
  EXPECT_TRUE(called);
}

TEST(SdoAccessor, NullMasterSyncReadReturnsError) {
  SdoAccessor accessor(nullptr);
  SdoResult result = accessor.Read(1, 0x6041, 0, std::chrono::milliseconds(100));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, "master is null");
}

TEST(SdoAccessor, NullMasterSyncWriteReturnsError) {
  SdoAccessor accessor(nullptr);
  SdoResult result = accessor.WriteU8(1, 0x6040, 0, 0x06,
                                      std::chrono::milliseconds(100));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.error, "master is null");
}
