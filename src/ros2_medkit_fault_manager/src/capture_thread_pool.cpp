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

#include "ros2_medkit_fault_manager/capture_thread_pool.hpp"

#include <system_error>
#include <utility>

#include "rclcpp/logging.hpp"

namespace ros2_medkit_fault_manager {

CaptureThreadPool::CaptureThreadPool(std::size_t pool_size, std::size_t queue_depth, QueueFullPolicy full_policy,
                                     rclcpp::Logger logger, std::function<void(const std::string &)> capture_fn)
  : queue_depth_(queue_depth == 0 ? 1 : queue_depth)
  , full_policy_(full_policy)
  , logger_(std::move(logger))
  , capture_fn_(std::move(capture_fn)) {
  // Defensive clamp: this is a public class. Callers (the node) validate to >= 1,
  // but a stray 0 must not UB. With 0 workers nothing drains the queue; with
  // queue_depth 0 + kDropOldest, enqueue() would call front() on an empty deque.
  // queue_depth_ is clamped in the initializer above.
  const std::size_t worker_count = pool_size == 0 ? 1 : pool_size;
  workers_.reserve(worker_count);
  for (std::size_t i = 0; i < worker_count; ++i) {
    workers_.emplace_back([this] {
      worker_loop();
    });
  }
}

CaptureThreadPool::~CaptureThreadPool() {
  shutdown();
}

EnqueueOutcome CaptureThreadPool::enqueue(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (stop_) {
    return {EnqueueResult::kRejectedShuttingDown, std::nullopt};
  }
  if (queue_.size() < queue_depth_) {
    queue_.push_back(fault_code);
    cv_.notify_one();
    return {EnqueueResult::kAccepted, std::nullopt};
  }
  // Queue full.
  if (full_policy_ == QueueFullPolicy::kRejectNewest) {
    dropped_captures_.fetch_add(1, std::memory_order_relaxed);
    return {EnqueueResult::kDroppedNewest, std::nullopt};
  }
  // kDropOldest: evict the oldest pending job.
  std::string evicted = std::move(queue_.front());
  queue_.pop_front();
  queue_.push_back(fault_code);
  dropped_captures_.fetch_add(1, std::memory_order_relaxed);
  cv_.notify_one();
  return {EnqueueResult::kEvictedOldest, std::move(evicted)};
}

void CaptureThreadPool::shutdown() noexcept {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_) {
      return;
    }
    stop_ = true;
    queue_.clear();  // discard pending jobs atomically with setting stop_
  }
  cv_.notify_all();
  for (auto & worker : workers_) {
    if (worker.joinable()) {
      // join() only throws std::system_error, and only for a self-join
      // (resource_deadlock_would_occur). shutdown() runs on the owner/destructor
      // thread, never on a worker, so that cannot happen here. The catch keeps
      // shutdown() noexcept and, in the impossible-in-practice case that one join
      // fails, still lets us join the remaining workers instead of terminating.
      try {
        worker.join();
      } catch (const std::system_error & e) {
        RCLCPP_ERROR(logger_, "CaptureThreadPool worker join failed: %s", e.what());
      }
    }
  }
  // Release captured shared_ptrs deterministically, before owning objects vanish.
  capture_fn_ = nullptr;
}

std::size_t CaptureThreadPool::pending_size() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return queue_.size();
}

void CaptureThreadPool::worker_loop() {
  for (;;) {
    std::string job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this] {
        return stop_ || !queue_.empty();
      });
      if (stop_) {
        return;  // check stop BEFORE dispatch -> pending discarded, not drained
      }
      job = std::move(queue_.front());
      queue_.pop_front();
    }  // release lock BEFORE running the (multi-second) job
    try {
      if (capture_fn_) {
        capture_fn_(job);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Capture job for '%s' threw: %s", job.c_str(), e.what());
    } catch (...) {
      RCLCPP_ERROR(logger_, "Capture job for '%s' threw unknown exception", job.c_str());
    }
  }
}

}  // namespace ros2_medkit_fault_manager
