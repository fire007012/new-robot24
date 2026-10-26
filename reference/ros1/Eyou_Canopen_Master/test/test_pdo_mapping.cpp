#include <gtest/gtest.h>

#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <tuple>
#include <vector>

#include "canopen_hw/pdo_mapping.hpp"

namespace canopen_hw {
namespace {

using Key = std::tuple<uint16_t, uint8_t>;

TEST(PdoMappingReader, ReadsMappingThroughInjectedAsyncReadFn) {
  auto reader = std::make_shared<PdoMappingReader>();

  std::map<Key, uint32_t> values = {
      {{0x1400, 1}, 0x201}, {{0x1600, 0}, 1},     {{0x1800, 1}, 0x181},
      {{0x1A00, 0}, 1},     {{0x1401, 1}, 0x202}, {{0x1601, 0}, 0},
      {{0x1801, 1}, 0x182}, {{0x1A01, 0}, 0},     {{0x1402, 1}, 0x203},
      {{0x1602, 0}, 0},     {{0x1802, 1}, 0x183}, {{0x1A02, 0}, 0},
      {{0x1403, 1}, 0x204}, {{0x1603, 0}, 0},     {{0x1803, 1}, 0x184},
      {{0x1A03, 0}, 0},     {{0x1600, 1}, 0x60400010},
      {{0x1A00, 1}, 0x60640020},
  };

  std::mutex mtx;
  std::condition_variable cv;
  bool finished = false;
  bool ok = false;
  std::string error;
  PdoMapping mapping;
  std::vector<Key> calls;

  reader->Start(
      [&](uint16_t idx, uint8_t sub, bool is_u8,
          PdoMappingReader::ReadValueCallback cb) {
        calls.emplace_back(idx, sub);
        const auto it = values.find(Key{idx, sub});
        if (it == values.end()) {
          cb(false, 0, "missing test fixture value");
          return;
        }
        const uint32_t value = it->second;
        if (is_u8 && value > 0xFFu) {
          cb(false, 0, "fixture value exceeds u8");
          return;
        }
        cb(true, value, std::string());
      },
      [&](bool read_ok, const std::string& read_error,
          const PdoMapping& actual) {
        std::lock_guard<std::mutex> lk(mtx);
        finished = true;
        ok = read_ok;
        error = read_error;
        mapping = actual;
        cv.notify_all();
      },
      std::chrono::milliseconds(200));

  {
    std::unique_lock<std::mutex> lk(mtx);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(200),
                            [&]() { return finished; }));
  }

  EXPECT_TRUE(ok) << error;
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(mapping.rpdo[0].cob_id, 0x201u);
  ASSERT_EQ(mapping.rpdo[0].entries.size(), 1u);
  EXPECT_EQ(mapping.rpdo[0].entries[0], 0x60400010u);
  EXPECT_EQ(mapping.tpdo[0].cob_id, 0x181u);
  ASSERT_EQ(mapping.tpdo[0].entries.size(), 1u);
  EXPECT_EQ(mapping.tpdo[0].entries[0], 0x60640020u);
  EXPECT_EQ(calls.front(), Key(0x1400, 1));
  EXPECT_EQ(calls.back(), Key(0x1A00, 1));
}

TEST(PdoMappingReader, ReportsReadFailureFromInjectedAsyncReadFn) {
  auto reader = std::make_shared<PdoMappingReader>();

  std::mutex mtx;
  std::condition_variable cv;
  bool finished = false;
  bool ok = true;
  std::string error;

  reader->Start(
      [&](uint16_t idx, uint8_t sub, bool,
          PdoMappingReader::ReadValueCallback cb) {
        if (idx == 0x1600 && sub == 0) {
          cb(false, 0, "busy");
          return;
        }
        cb(true, 0, std::string());
      },
      [&](bool read_ok, const std::string& read_error, const PdoMapping&) {
        std::lock_guard<std::mutex> lk(mtx);
        finished = true;
        ok = read_ok;
        error = read_error;
        cv.notify_all();
      },
      std::chrono::milliseconds(200));

  {
    std::unique_lock<std::mutex> lk(mtx);
    ASSERT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(200),
                            [&]() { return finished; }));
  }

  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("0x1600:0"), std::string::npos);
  EXPECT_NE(error.find("busy"), std::string::npos);
}

}  // namespace
}  // namespace canopen_hw
