// Copyright 2026 bburda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/capture_thread_pool.hpp"

using ros2_medkit_fault_manager::CaptureThreadPool;
using ros2_medkit_fault_manager::EnqueueResult;
using ros2_medkit_fault_manager::QueueFullPolicy;

namespace {

rclcpp::Logger test_logger() {
  return rclcpp::get_logger("test_capture_thread_pool");
}

// Capture_fn that records every invocation and blocks each job until released,
// so tests can deterministically hold workers busy and inspect concurrency.
class GatedCallback {
 public:
  std::function<void(const std::string &)> fn() {
    return [this](const std::string & code) {
      run(code);
    };
  }

  void run(const std::string & code) {
    {
      std::lock_guard<std::mutex> lock(m_);
      ++active_;
      max_active_ = std::max(max_active_, active_);
      started_.push_back(code);
      executed_.insert(code);
    }
    entered_cv_.notify_all();
    {
      std::unique_lock<std::mutex> lock(m_);
      release_cv_.wait(lock, [this] {
        return released_;
      });
      --active_;
    }
  }

  void wait_until_active(std::size_t n) {
    std::unique_lock<std::mutex> lock(m_);
    entered_cv_.wait(lock, [&] {
      return active_ >= n;
    });
  }

  void wait_until_started(std::size_t n) {
    std::unique_lock<std::mutex> lock(m_);
    entered_cv_.wait(lock, [&] {
      return started_.size() >= n;
    });
  }

  void release() {
    {
      std::lock_guard<std::mutex> lock(m_);
      released_ = true;
    }
    release_cv_.notify_all();
  }

  std::size_t max_active() {
    std::lock_guard<std::mutex> lock(m_);
    return max_active_;
  }
  std::vector<std::string> started() {
    std::lock_guard<std::mutex> lock(m_);
    return started_;
  }
  std::set<std::string> executed() {
    std::lock_guard<std::mutex> lock(m_);
    return executed_;
  }

 private:
  std::mutex m_;
  std::condition_variable entered_cv_;
  std::condition_variable release_cv_;
  std::size_t active_{0};
  std::size_t max_active_{0};
  bool released_{false};
  std::vector<std::string> started_;
  std::set<std::string> executed_;
};

class CaptureThreadPoolTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

}  // namespace

// Concurrency reaches AND never exceeds pool_size.
TEST_F(CaptureThreadPoolTest, BoundsAndReachesPoolSize) {
  GatedCallback gc;
  CaptureThreadPool pool(2, 8, QueueFullPolicy::kRejectNewest, test_logger(), gc.fn());
  for (int i = 0; i < 6; ++i) {
    pool.enqueue("j" + std::to_string(i));
  }
  gc.wait_until_active(2);         // prove 2 ran concurrently
  EXPECT_EQ(gc.max_active(), 2u);  // never more than pool_size
  gc.release();
  pool.shutdown();
  EXPECT_EQ(gc.max_active(), 2u);
}

TEST_F(CaptureThreadPoolTest, PoolSizeOneSerializes) {
  GatedCallback gc;
  CaptureThreadPool pool(1, 8, QueueFullPolicy::kRejectNewest, test_logger(), gc.fn());
  pool.enqueue("a");
  pool.enqueue("b");
  gc.wait_until_active(1);
  EXPECT_EQ(gc.max_active(), 1u);
  gc.release();
  pool.shutdown();
  EXPECT_EQ(gc.max_active(), 1u);
}

TEST_F(CaptureThreadPoolTest, RejectNewestDropsExcessExactly) {
  GatedCallback gc;
  CaptureThreadPool pool(2, 8, QueueFullPolicy::kRejectNewest, test_logger(), gc.fn());
  pool.enqueue("blk0");
  pool.enqueue("blk1");
  gc.wait_until_active(2);       // both workers busy; queue empties
  for (int i = 0; i < 8; ++i) {  // fill pending queue (queue_depth)
    EXPECT_EQ(pool.enqueue("f" + std::to_string(i)).result, EnqueueResult::kAccepted);
  }
  for (int i = 0; i < 5; ++i) {  // overflow
    EXPECT_EQ(pool.enqueue("x" + std::to_string(i)).result, EnqueueResult::kDroppedNewest);
  }
  EXPECT_EQ(pool.dropped_captures(), 5u);
  gc.release();
  pool.shutdown();
  for (int i = 0; i < 5; ++i) {  // dropped jobs never ran
    EXPECT_EQ(gc.executed().count("x" + std::to_string(i)), 0u);
  }
}

TEST_F(CaptureThreadPoolTest, DropOldestEvictsOldestPending) {
  GatedCallback gc;
  CaptureThreadPool pool(2, 8, QueueFullPolicy::kDropOldest, test_logger(), gc.fn());
  pool.enqueue("blk0");
  pool.enqueue("blk1");
  gc.wait_until_active(2);
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(pool.enqueue("f" + std::to_string(i)).result, EnqueueResult::kAccepted);
  }
  for (int i = 0; i < 5; ++i) {  // each overflow evicts current oldest f0..f4
    auto outcome = pool.enqueue("x" + std::to_string(i));
    EXPECT_EQ(outcome.result, EnqueueResult::kEvictedOldest);
    ASSERT_TRUE(outcome.evicted_code.has_value());
    EXPECT_EQ(*outcome.evicted_code, "f" + std::to_string(i));
  }
  EXPECT_EQ(pool.dropped_captures(), 5u);
  gc.release();
  gc.wait_until_started(10);  // blk0, blk1 + f5,f6,f7 + x0..x4 (evicted f0..f4 never queued)
  pool.shutdown();
  auto ran = gc.executed();
  for (int i = 0; i < 5; ++i) {  // evicted absent
    EXPECT_EQ(ran.count("f" + std::to_string(i)), 0u);
  }
  for (int i = 0; i < 5; ++i) {  // newest present
    EXPECT_EQ(ran.count("x" + std::to_string(i)), 1u);
  }
}

TEST_F(CaptureThreadPoolTest, FifoOrder) {
  GatedCallback gc;
  CaptureThreadPool pool(1, 8, QueueFullPolicy::kRejectNewest, test_logger(), gc.fn());
  pool.enqueue("A");
  gc.wait_until_active(1);
  pool.enqueue("B");
  pool.enqueue("C");
  gc.release();
  gc.wait_until_started(3);  // A, B, C must all run before shutdown discards queue
  pool.shutdown();
  std::vector<std::string> expected{"A", "B", "C"};
  EXPECT_EQ(gc.started(), expected);
}

TEST_F(CaptureThreadPoolTest, CallbackThrowKeepsWorkerAlive) {
  std::mutex m;
  std::condition_variable cv;
  int ran = 0;
  auto fn = [&](const std::string & code) {
    if (code == "boom") {
      throw std::runtime_error("boom");
    }
    {
      std::lock_guard<std::mutex> lock(m);
      ++ran;
    }
    cv.notify_all();
  };
  CaptureThreadPool pool(1, 8, QueueFullPolicy::kRejectNewest, test_logger(), fn);
  pool.enqueue("boom");
  pool.enqueue("ok1");
  pool.enqueue("ok2");
  {
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [&] {
      return ran >= 2;
    });
  }
  pool.shutdown();
  EXPECT_EQ(ran, 2);
}

TEST_F(CaptureThreadPoolTest, EnqueueAfterShutdownRejected) {
  CaptureThreadPool pool(1, 8, QueueFullPolicy::kRejectNewest, test_logger(), [](const std::string &) {});
  pool.shutdown();
  auto outcome = pool.enqueue("x");
  EXPECT_EQ(outcome.result, EnqueueResult::kRejectedShuttingDown);
  EXPECT_EQ(pool.dropped_captures(), 0u);
}

TEST_F(CaptureThreadPoolTest, ShutdownDiscardsPendingCompletesInFlight) {
  GatedCallback gc;
  CaptureThreadPool pool(1, 8, QueueFullPolicy::kRejectNewest, test_logger(), gc.fn());
  EXPECT_EQ(pool.enqueue("inflight").result, EnqueueResult::kAccepted);
  gc.wait_until_active(1);
  EXPECT_EQ(pool.enqueue("pending").result, EnqueueResult::kAccepted);
  EXPECT_EQ(pool.pending_size(), 1u);

  std::thread shut([&] {
    pool.shutdown();
  });
  while (pool.pending_size() != 0u) {  // wait until shutdown set stop_ and cleared the queue
    std::this_thread::yield();
  }
  gc.release();  // now safe: stop_ is set, worker discards "pending" on re-loop
  shut.join();

  auto ran = gc.executed();
  EXPECT_EQ(ran.count("inflight"), 1u);
  EXPECT_EQ(ran.count("pending"), 0u);
}

TEST_F(CaptureThreadPoolTest, ShutdownIsIdempotent) {
  CaptureThreadPool pool(2, 8, QueueFullPolicy::kRejectNewest, test_logger(), [](const std::string &) {});
  pool.shutdown();
  pool.shutdown();  // must not throw, hang, or double-join
  SUCCEED();
}

// shutdown() releases the captured callback (and the shared_ptrs it holds) after
// joining - the load-bearing invariant behind the #441 destructor reorder.
TEST_F(CaptureThreadPoolTest, ReleasesCaptureFnAfterShutdown) {
  auto sentinel = std::make_shared<int>(42);
  {
    std::weak_ptr<int> weak = sentinel;
    CaptureThreadPool pool(2, 8, QueueFullPolicy::kRejectNewest, test_logger(), [held = sentinel](const std::string &) {
      (void)held;
    });
    sentinel.reset();  // pool's capture_fn now holds the only strong ref
    EXPECT_FALSE(weak.expired());
    pool.shutdown();
    EXPECT_TRUE(weak.expired());  // capture_fn released after join -> ref dropped
  }
}
