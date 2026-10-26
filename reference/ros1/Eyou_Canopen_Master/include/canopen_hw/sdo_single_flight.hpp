#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

namespace canopen_hw {

// 单飞任务队列：任意时刻仅允许一个任务处于活动状态。
// 后续任务按 FIFO 排队，并在前一任务调用 completion 后自动推进。
class SdoSingleFlightQueue {
 public:
  using Completion = std::function<void()>;
  using Task = std::function<void(Completion)>;

  void Enqueue(Task task) {
    if (!task) {
      return;
    }

    Task next_task;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      queue_.push_back(std::move(task));
      if (in_flight_) {
        return;
      }
      in_flight_ = true;
      next_task = std::move(queue_.front());
      queue_.pop_front();
    }

    Dispatch(std::move(next_task));
  }

  bool WaitForIdle(std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lk(mtx_);
    return idle_cv_.wait_for(
        lk, timeout, [this]() { return !in_flight_ && queue_.empty(); });
  }

  bool IsIdle() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return !in_flight_ && queue_.empty();
  }

  std::size_t PendingCount() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return queue_.size() + (in_flight_ ? 1u : 0u);
  }

 private:
  struct CompletionState {
    explicit CompletionState(SdoSingleFlightQueue* owner) : owner(owner) {}

    void Finish() {
      bool expected = false;
      if (!finished.compare_exchange_strong(expected, true)) {
        return;
      }
      owner->CompleteAndPump();
    }

    SdoSingleFlightQueue* owner = nullptr;
    std::atomic<bool> finished{false};
  };

  void Dispatch(Task task) {
    auto state = std::make_shared<CompletionState>(this);
    task([state]() { state->Finish(); });
  }

  void CompleteAndPump() {
    Task next_task;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!queue_.empty()) {
        next_task = std::move(queue_.front());
        queue_.pop_front();
      } else {
        in_flight_ = false;
        idle_cv_.notify_all();
        return;
      }
    }

    Dispatch(std::move(next_task));
  }

  mutable std::mutex mtx_;
  mutable std::condition_variable idle_cv_;
  std::deque<Task> queue_;
  bool in_flight_ = false;
};

}  // namespace canopen_hw
