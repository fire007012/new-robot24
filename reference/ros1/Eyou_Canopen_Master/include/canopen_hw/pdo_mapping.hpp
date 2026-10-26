#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <condition_variable>

namespace canopen_hw {

struct PdoChannelMapping {
  uint32_t cob_id = 0;
  std::vector<uint32_t> entries;
};

struct PdoMapping {
  std::array<PdoChannelMapping, 4> rpdo{};
  std::array<PdoChannelMapping, 4> tpdo{};
};

bool LoadExpectedPdoMappingFromDcf(const std::string& path, PdoMapping* mapping,
                                   std::string* error);

bool DiffPdoMapping(const PdoMapping& expected, const PdoMapping& actual,
                    std::vector<std::string>* diffs);

// 线程模型说明:
// - Start() 只能在 Lely 事件线程调用
// - ScheduleNext() / SDO 回调在 Lely 事件线程执行
// - 超时线程是唯一的非 Lely 线程入口
// - Finish() 可被上述两条路径调用，内部用 finish_mtx_ 串行化，
//   保证同一时刻只有一个路径执行完成流程
// - 调用者（AxisDriver）不得在回调内部 reset() reader，
//   应在回调外部安全时机释放
class PdoMappingReader : public std::enable_shared_from_this<PdoMappingReader> {
 public:
  using ReadValueCallback =
      std::function<void(bool, uint32_t, const std::string&)>;
  using AsyncReadFn =
      std::function<void(uint16_t, uint8_t, bool, ReadValueCallback)>;
  using DoneCallback =
      std::function<void(bool, const std::string&, const PdoMapping&)>;

  ~PdoMappingReader();

  void Start(AsyncReadFn read_fn, DoneCallback cb,
             std::chrono::milliseconds timeout);

 private:
  struct ReadStep {
    uint16_t idx = 0;
    uint8_t sub = 0;
    bool is_u8 = false;
    std::function<void(uint32_t)> on_value;
  };

  void BuildHeaderSteps();
  void BuildEntrySteps();
  void ScheduleNext();
  void Finish(bool ok, const std::string& error);

  AsyncReadFn read_fn_;
  DoneCallback done_cb_;
  std::vector<ReadStep> steps_;
  std::array<uint8_t, 4> rpdo_counts_{};
  std::array<uint8_t, 4> tpdo_counts_{};
  std::size_t step_index_ = 0;
  bool phase_entries_ = false;

  // finish_mtx_ 保护 finished_ 和 Finish() 整体流程的串行化，
  // 确保 SDO 回调路径与超时线程路径互斥执行。
  std::mutex finish_mtx_;
  bool finished_ = false;

  std::atomic<bool> timeout_stop_{false};
  std::thread timeout_thread_;
  std::mutex timeout_mtx_;
  std::condition_variable timeout_cv_;
  PdoMapping mapping_{};
  std::string error_;
};

}  // namespace canopen_hw
