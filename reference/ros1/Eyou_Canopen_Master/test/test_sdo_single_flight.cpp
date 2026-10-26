#include <gtest/gtest.h>

#include <condition_variable>
#include <mutex>
#include <vector>

#include "canopen_hw/sdo_single_flight.hpp"

namespace canopen_hw {
namespace {

bool WaitForSize(std::condition_variable& cv, std::mutex& mtx,
                 const std::vector<int>& values, std::size_t expected) {
  std::unique_lock<std::mutex> lk(mtx);
  return cv.wait_for(lk, std::chrono::milliseconds(200), [&values, expected]() {
    return values.size() >= expected;
  });
}

TEST(SdoSingleFlightQueue, RunsTasksOneAtATimeInFifoOrder) {
  SdoSingleFlightQueue queue;

  std::mutex mtx;
  std::condition_variable cv;
  std::vector<int> started;
  std::vector<SdoSingleFlightQueue::Completion> completions(3);

  for (int i = 0; i < 3; ++i) {
    queue.Enqueue([&, i](SdoSingleFlightQueue::Completion done) {
      {
        std::lock_guard<std::mutex> lk(mtx);
        started.push_back(i);
        completions[static_cast<std::size_t>(i)] = std::move(done);
      }
      cv.notify_all();
    });
  }

  ASSERT_TRUE(WaitForSize(cv, mtx, started, 1u));
  EXPECT_EQ(started, std::vector<int>({0}));
  EXPECT_FALSE(queue.IsIdle());

  completions[0]();
  ASSERT_TRUE(WaitForSize(cv, mtx, started, 2u));
  EXPECT_EQ(started, std::vector<int>({0, 1}));

  completions[1]();
  ASSERT_TRUE(WaitForSize(cv, mtx, started, 3u));
  EXPECT_EQ(started, std::vector<int>({0, 1, 2}));

  completions[2]();
  EXPECT_TRUE(queue.WaitForIdle(std::chrono::milliseconds(200)));
  EXPECT_TRUE(queue.IsIdle());
}

TEST(SdoSingleFlightQueue, DuplicateCompletionDoesNotAdvanceTwice) {
  SdoSingleFlightQueue queue;

  std::mutex mtx;
  std::condition_variable cv;
  std::vector<int> started;
  SdoSingleFlightQueue::Completion first_done;

  queue.Enqueue([&](SdoSingleFlightQueue::Completion done) {
    {
      std::lock_guard<std::mutex> lk(mtx);
      started.push_back(1);
      first_done = std::move(done);
    }
    cv.notify_all();
  });
  queue.Enqueue([&](SdoSingleFlightQueue::Completion done) {
    {
      std::lock_guard<std::mutex> lk(mtx);
      started.push_back(2);
    }
    done();
    cv.notify_all();
  });

  ASSERT_TRUE(WaitForSize(cv, mtx, started, 1u));
  ASSERT_EQ(started, std::vector<int>({1}));

  first_done();
  first_done();

  ASSERT_TRUE(WaitForSize(cv, mtx, started, 2u));
  EXPECT_EQ(started, std::vector<int>({1, 2}));
  EXPECT_TRUE(queue.WaitForIdle(std::chrono::milliseconds(200)));
}

TEST(SdoSingleFlightQueue, WaitForIdleTimesOutWhileTaskIsActive) {
  SdoSingleFlightQueue queue;

  SdoSingleFlightQueue::Completion done;
  queue.Enqueue([&](SdoSingleFlightQueue::Completion completion) {
    done = std::move(completion);
  });

  EXPECT_FALSE(queue.WaitForIdle(std::chrono::milliseconds(20)));
  ASSERT_TRUE(done != nullptr);
  done();
  EXPECT_TRUE(queue.WaitForIdle(std::chrono::milliseconds(200)));
}

}  // namespace
}  // namespace canopen_hw
