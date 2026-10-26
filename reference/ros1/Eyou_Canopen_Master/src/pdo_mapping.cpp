#include "canopen_hw/pdo_mapping.hpp"

#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>

#include <lely/co/dcf.h>
#include <lely/co/dev.h>

namespace canopen_hw {

namespace {

bool GetU8(const co_dev_t* dev, uint16_t idx, uint8_t sub, uint8_t* out,
           std::string* error) {
  if (!co_dev_find_sub(dev, idx, sub)) {
    if (error) {
      std::ostringstream oss;
      oss << "missing object 0x" << std::hex << idx << ":" << std::dec
          << static_cast<int>(sub);
      *error = oss.str();
    }
    return false;
  }
  *out = co_dev_get_val_u8(dev, idx, sub);
  return true;
}

bool GetU32(const co_dev_t* dev, uint16_t idx, uint8_t sub, uint32_t* out,
            std::string* error) {
  if (!co_dev_find_sub(dev, idx, sub)) {
    if (error) {
      std::ostringstream oss;
      oss << "missing object 0x" << std::hex << idx << ":" << std::dec
          << static_cast<int>(sub);
      *error = oss.str();
    }
    return false;
  }
  *out = co_dev_get_val_u32(dev, idx, sub);
  return true;
}

std::string FormatEntry(uint32_t entry) {
  const uint16_t idx = static_cast<uint16_t>(entry & 0xFFFFu);
  const uint8_t sub = static_cast<uint8_t>((entry >> 16) & 0xFFu);
  const uint8_t bits = static_cast<uint8_t>((entry >> 24) & 0xFFu);
  std::ostringstream oss;
  oss << std::hex << idx << ":" << std::dec << static_cast<int>(sub) << "/"
      << static_cast<int>(bits);
  return oss.str();
}

}  // namespace

bool LoadExpectedPdoMappingFromDcf(const std::string& path, PdoMapping* mapping,
                                   std::string* error) {
  if (!mapping) {
    if (error) {
      *error = "mapping is null";
    }
    return false;
  }

  co_dev_t* dev = co_dev_create_from_dcf_file(path.c_str());
  if (!dev) {
    if (error) {
      *error = "failed to load dcf file";
    }
    return false;
  }

  bool ok = true;
  for (uint16_t i = 0; i < 4 && ok; ++i) {
    uint32_t cob_id = 0;
    ok = GetU32(dev, static_cast<uint16_t>(0x1400 + i), 1, &cob_id, error);
    if (!ok) {
      break;
    }
    mapping->rpdo[i].cob_id = cob_id;

    uint8_t count = 0;
    ok = GetU8(dev, static_cast<uint16_t>(0x1600 + i), 0, &count, error);
    if (!ok) {
      break;
    }
    mapping->rpdo[i].entries.clear();
    mapping->rpdo[i].entries.reserve(count);
    for (uint8_t sub = 1; sub <= count && ok; ++sub) {
      uint32_t entry = 0;
      ok = GetU32(dev, static_cast<uint16_t>(0x1600 + i), sub, &entry, error);
      if (!ok) {
        break;
      }
      mapping->rpdo[i].entries.push_back(entry);
    }
  }

  for (uint16_t i = 0; i < 4 && ok; ++i) {
    uint32_t cob_id = 0;
    ok = GetU32(dev, static_cast<uint16_t>(0x1800 + i), 1, &cob_id, error);
    if (!ok) {
      break;
    }
    mapping->tpdo[i].cob_id = cob_id;

    uint8_t count = 0;
    ok = GetU8(dev, static_cast<uint16_t>(0x1A00 + i), 0, &count, error);
    if (!ok) {
      break;
    }
    mapping->tpdo[i].entries.clear();
    mapping->tpdo[i].entries.reserve(count);
    for (uint8_t sub = 1; sub <= count && ok; ++sub) {
      uint32_t entry = 0;
      ok = GetU32(dev, static_cast<uint16_t>(0x1A00 + i), sub, &entry, error);
      if (!ok) {
        break;
      }
      mapping->tpdo[i].entries.push_back(entry);
    }
  }

  co_dev_destroy(dev);
  return ok;
}

bool DiffPdoMapping(const PdoMapping& expected, const PdoMapping& actual,
                    std::vector<std::string>* diffs) {
  if (diffs) {
    diffs->clear();
  }

  auto record = [&](const std::string& msg) {
    if (diffs) {
      diffs->push_back(msg);
    }
  };

  for (std::size_t i = 0; i < 4; ++i) {
    if (expected.rpdo[i].cob_id != actual.rpdo[i].cob_id) {
      std::ostringstream oss;
      oss << "RPDO" << (i + 1) << " COB-ID mismatch: expected 0x" << std::hex
          << expected.rpdo[i].cob_id << " actual 0x" << actual.rpdo[i].cob_id;
      record(oss.str());
    }
    if (expected.rpdo[i].entries.size() != actual.rpdo[i].entries.size()) {
      std::ostringstream oss;
      oss << "RPDO" << (i + 1) << " entry count mismatch: expected "
          << expected.rpdo[i].entries.size() << " actual "
          << actual.rpdo[i].entries.size();
      record(oss.str());
    }
    const std::size_t count =
        std::min(expected.rpdo[i].entries.size(),
                 actual.rpdo[i].entries.size());
    for (std::size_t j = 0; j < count; ++j) {
      if (expected.rpdo[i].entries[j] != actual.rpdo[i].entries[j]) {
        std::ostringstream oss;
        oss << "RPDO" << (i + 1) << " entry[" << j << "] mismatch: expected "
            << FormatEntry(expected.rpdo[i].entries[j]) << " actual "
            << FormatEntry(actual.rpdo[i].entries[j]);
        record(oss.str());
      }
    }
  }

  for (std::size_t i = 0; i < 4; ++i) {
    if (expected.tpdo[i].cob_id != actual.tpdo[i].cob_id) {
      std::ostringstream oss;
      oss << "TPDO" << (i + 1) << " COB-ID mismatch: expected 0x" << std::hex
          << expected.tpdo[i].cob_id << " actual 0x" << actual.tpdo[i].cob_id;
      record(oss.str());
    }
    if (expected.tpdo[i].entries.size() != actual.tpdo[i].entries.size()) {
      std::ostringstream oss;
      oss << "TPDO" << (i + 1) << " entry count mismatch: expected "
          << expected.tpdo[i].entries.size() << " actual "
          << actual.tpdo[i].entries.size();
      record(oss.str());
    }
    const std::size_t count =
        std::min(expected.tpdo[i].entries.size(),
                 actual.tpdo[i].entries.size());
    for (std::size_t j = 0; j < count; ++j) {
      if (expected.tpdo[i].entries[j] != actual.tpdo[i].entries[j]) {
        std::ostringstream oss;
        oss << "TPDO" << (i + 1) << " entry[" << j << "] mismatch: expected "
            << FormatEntry(expected.tpdo[i].entries[j]) << " actual "
            << FormatEntry(actual.tpdo[i].entries[j]);
        record(oss.str());
      }
    }
  }

  return !diffs || diffs->empty();
}

PdoMappingReader::~PdoMappingReader() {
  // 通知超时线程退出
  timeout_stop_.store(true);
  timeout_cv_.notify_all();
  // 析构函数不可能从超时线程自身调用（因为超时线程持有 shared_ptr，
  // 析构只会发生在最后一个 shared_ptr 释放时，此时超时线程已退出），
  // 因此可以安全 join。
  if (timeout_thread_.joinable()) {
    timeout_thread_.join();
  }
}

void PdoMappingReader::Start(AsyncReadFn read_fn, DoneCallback cb,
                             std::chrono::milliseconds timeout) {
  {
    std::lock_guard<std::mutex> lk(finish_mtx_);
    if (finished_) {
      return;
    }
  }
  read_fn_ = std::move(read_fn);
  done_cb_ = std::move(cb);
  BuildHeaderSteps();
  ScheduleNext();

  if (timeout.count() > 0) {
    timeout_stop_.store(false);
    // 持有 shared_from_this() 的拷贝（而非 weak_ptr），
    // 确保对象在超时线程执行期间不会被析构。
    // 超时线程退出后释放 shared_ptr，允许对象正常析构。
    std::shared_ptr<PdoMappingReader> self = shared_from_this();
    timeout_thread_ = std::thread([self, timeout]() {
      std::unique_lock<std::mutex> lk(self->timeout_mtx_);
      const bool stopped = self->timeout_cv_.wait_for(
          lk, timeout, [self]() { return self->timeout_stop_.load(); });
      lk.unlock();
      if (!stopped) {
        self->Finish(false, "PDO verify timeout");
      }
      // 超时线程在此之后不再访问任何成员，
      // self 析构时若为最后引用则安全销毁对象。
    });
  }
}

void PdoMappingReader::BuildHeaderSteps() {
  steps_.clear();
  step_index_ = 0;
  phase_entries_ = false;

  for (uint16_t i = 0; i < 4; ++i) {
    ReadStep rpdo_cob;
    rpdo_cob.idx = static_cast<uint16_t>(0x1400 + i);
    rpdo_cob.sub = 1;
    rpdo_cob.on_value = [this, i](uint32_t value) {
      mapping_.rpdo[i].cob_id = value;
    };
    steps_.push_back(rpdo_cob);

    ReadStep rpdo_count;
    rpdo_count.idx = static_cast<uint16_t>(0x1600 + i);
    rpdo_count.sub = 0;
    rpdo_count.is_u8 = true;
    rpdo_count.on_value = [this, i](uint32_t value) {
      rpdo_counts_[i] = static_cast<uint8_t>(value);
    };
    steps_.push_back(rpdo_count);

    ReadStep tpdo_cob;
    tpdo_cob.idx = static_cast<uint16_t>(0x1800 + i);
    tpdo_cob.sub = 1;
    tpdo_cob.on_value = [this, i](uint32_t value) {
      mapping_.tpdo[i].cob_id = value;
    };
    steps_.push_back(tpdo_cob);

    ReadStep tpdo_count;
    tpdo_count.idx = static_cast<uint16_t>(0x1A00 + i);
    tpdo_count.sub = 0;
    tpdo_count.is_u8 = true;
    tpdo_count.on_value = [this, i](uint32_t value) {
      tpdo_counts_[i] = static_cast<uint8_t>(value);
    };
    steps_.push_back(tpdo_count);
  }
}

void PdoMappingReader::BuildEntrySteps() {
  steps_.clear();
  step_index_ = 0;
  phase_entries_ = true;

  for (uint16_t i = 0; i < 4; ++i) {
    mapping_.rpdo[i].entries.clear();
    mapping_.rpdo[i].entries.reserve(rpdo_counts_[i]);
    for (uint8_t sub = 1; sub <= rpdo_counts_[i]; ++sub) {
      ReadStep step;
      step.idx = static_cast<uint16_t>(0x1600 + i);
      step.sub = sub;
      step.on_value = [this, i](uint32_t value) {
        mapping_.rpdo[i].entries.push_back(value);
      };
      steps_.push_back(step);
    }
  }

  for (uint16_t i = 0; i < 4; ++i) {
    mapping_.tpdo[i].entries.clear();
    mapping_.tpdo[i].entries.reserve(tpdo_counts_[i]);
    for (uint8_t sub = 1; sub <= tpdo_counts_[i]; ++sub) {
      ReadStep step;
      step.idx = static_cast<uint16_t>(0x1A00 + i);
      step.sub = sub;
      step.on_value = [this, i](uint32_t value) {
        mapping_.tpdo[i].entries.push_back(value);
      };
      steps_.push_back(step);
    }
  }
}

void PdoMappingReader::ScheduleNext() {
  {
    std::lock_guard<std::mutex> lk(finish_mtx_);
    if (finished_) {
      return;
    }
  }
  if (step_index_ >= steps_.size()) {
    if (!phase_entries_) {
      BuildEntrySteps();
      ScheduleNext();
      return;
    }
    Finish(true, std::string());
    return;
  }

  const ReadStep step = steps_[step_index_];
  if (!read_fn_) {
    Finish(false, "PDO reader not initialized");
    return;
  }

  try {
    read_fn_(
        step.idx, step.sub, step.is_u8,
        [this, step](bool ok, uint32_t value, const std::string& error) {
          if (!ok) {
            std::ostringstream oss;
            oss << "SDO read failed at 0x" << std::hex << step.idx << ":"
                << std::dec << static_cast<int>(step.sub);
            if (!error.empty()) {
              oss << ": " << error;
            }
            Finish(false, oss.str());
            return;
          }
          if (step.on_value) {
            step.on_value(value);
          }
          ++step_index_;
          ScheduleNext();
        });
  } catch (const std::exception& e) {
    std::ostringstream oss;
    oss << "SDO read dispatch failed at 0x" << std::hex << step.idx << ":"
        << std::dec << static_cast<int>(step.sub) << ": " << e.what();
    Finish(false, oss.str());
  } catch (...) {
    std::ostringstream oss;
    oss << "SDO read dispatch failed at 0x" << std::hex << step.idx << ":"
        << std::dec << static_cast<int>(step.sub) << ": unknown exception";
    Finish(false, oss.str());
  }
}

void PdoMappingReader::Finish(bool ok, const std::string& error) {
  DoneCallback cb;
  PdoMapping mapping_copy;
  std::string error_copy;

  {
    std::lock_guard<std::mutex> lk(finish_mtx_);
    if (finished_) {
      return;
    }
    finished_ = true;

    // 通知超时线程退出（如果从 SDO 回调路径进入）。
    // 不在此处 join——超时线程可能正是调用者，也可能被阻塞等待 finish_mtx_。
    // join 留给析构函数处理，此时 timeout_stop_ 已为 true，
    // 超时线程会立即退出。
    timeout_stop_.store(true);
    timeout_cv_.notify_all();

    error_ = error;
    // 拷贝回调和数据到局部变量，在锁外执行回调，
    // 避免回调中可能触发的 reset() 导致死锁或生命周期问题。
    cb = done_cb_;
    mapping_copy = mapping_;
    error_copy = error_;
  }

  if (cb) {
    cb(ok, error_copy, mapping_copy);
  }
}

}  // namespace canopen_hw
