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

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/logger.hpp"

namespace ros2_medkit_fault_manager {

/// Policy applied when the bounded pending queue is full.
enum class QueueFullPolicy {
  kRejectNewest,  ///< Drop the incoming job.
  kDropOldest     ///< Evict the oldest pending job, queue the incoming one.
};

/// Result of an enqueue attempt.
enum class EnqueueResult {
  kAccepted,             ///< Queued; will run.
  kDroppedNewest,        ///< Queue full (reject_newest): incoming dropped.
  kEvictedOldest,        ///< Queue full (drop_oldest): oldest evicted, incoming queued.
  kRejectedShuttingDown  ///< Pool is shutting down: not queued, will not run.
};

struct EnqueueOutcome {
  EnqueueResult result;
  std::optional<std::string> evicted_code;  ///< Set only for kEvictedOldest.
};

/// Bounded worker pool that runs fault-capture jobs off the service thread.
///
/// Caps concurrent in-flight captures at pool_size and bounds pending work at
/// queue_depth with a configurable full-queue policy, giving peak memory and
/// thread count a hard upper bound under a fault storm. See issue #441.
class CaptureThreadPool {
 public:
  /// @param pool_size  Number of worker threads (caller must pass >= 1).
  /// @param queue_depth  Max pending jobs (caller must pass >= 1).
  /// @param full_policy  Behavior when the pending queue is full.
  /// @param logger  Logger for worker-side capture failures.
  /// @param capture_fn  Invoked per job on a worker thread. Must be thread-safe
  ///        for pool_size concurrent calls. Exceptions are caught and logged.
  CaptureThreadPool(std::size_t pool_size, std::size_t queue_depth, QueueFullPolicy full_policy, rclcpp::Logger logger,
                    std::function<void(const std::string &)> capture_fn);
  ~CaptureThreadPool();

  CaptureThreadPool(const CaptureThreadPool &) = delete;
  CaptureThreadPool & operator=(const CaptureThreadPool &) = delete;
  CaptureThreadPool(CaptureThreadPool &&) = delete;
  CaptureThreadPool & operator=(CaptureThreadPool &&) = delete;

  /// Enqueue a capture job. Non-blocking. Thread-safe.
  EnqueueOutcome enqueue(const std::string & fault_code);

  /// Stop accepting work, let in-flight jobs finish, discard pending, join all
  /// workers. Idempotent and noexcept. Called by the destructor.
  void shutdown() noexcept;

  /// Lifetime total of storm drops (kDroppedNewest + kEvictedOldest).
  uint64_t dropped_captures() const {
    return dropped_captures_.load();
  }

  /// Current number of pending (not yet started) jobs. For tests/observability.
  std::size_t pending_size() const;

 private:
  void worker_loop();

  const std::size_t queue_depth_;
  const QueueFullPolicy full_policy_;
  rclcpp::Logger logger_;
  std::function<void(const std::string &)> capture_fn_;

  mutable std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::deque<std::string> queue_;
  bool stop_{false};

  std::atomic<uint64_t> dropped_captures_{0};
  std::vector<std::thread> workers_;
};

}  // namespace ros2_medkit_fault_manager
