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

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_medkit_fault_manager/core/log_sink.hpp"
#include "ros2_medkit_fault_manager/core/rosbag_retention.hpp"

namespace ros2_medkit_fault_manager {

/// Runs the file system work that follows a rosbag row change, such as removing a directory
/// no row names any more.
///
/// kThread: one thread runs the posted jobs in FIFO order. kInline: post() runs the job at once
/// on the calling thread. In both modes a job that throws is logged as an ERROR, and every job
/// ends with the DEBUG line `retention job done: <kind>`. post() after drain_and_stop() logs a
/// WARN and drops the job.
class RetentionWorker {
 public:
  enum class Mode { kThread, kInline };

  RetentionWorker(core::LogSink & log_sink, std::string logger_name, Mode mode);
  /// Runs drain_and_stop().
  ~RetentionWorker();

  RetentionWorker(const RetentionWorker &) = delete;
  RetentionWorker & operator=(const RetentionWorker &) = delete;
  RetentionWorker(RetentionWorker &&) = delete;
  RetentionWorker & operator=(RetentionWorker &&) = delete;

  void post(std::string kind, std::function<void()> job);

  /// Remove the directory of @p recording on the calling thread, then log the removal with its
  /// reason: `Recording removed: <id> (max_bags_per_fault)` or `(fault cleared)`, main's
  /// `Deleted old bag '<path>' to enforce storage limit` for the quota, nothing for a directory
  /// that was already missing. A failure logs a WARN with the path, the reason and the error.
  /// @return true when the directory is gone
  bool remove_recording(const core::FreedRecording & recording);

  /// Post one job that runs remove_recording() for each of @p recordings. Posts nothing when
  /// @p recordings is empty.
  void post_removals(std::string kind, std::vector<core::FreedRecording> recordings);

  /// Run every job posted so far, then stop the thread. Idempotent.
  void drain_and_stop();

 private:
  struct Job {
    std::string kind;
    std::function<void()> run;
  };

  void thread_loop();
  void run_job(const Job & job);

  core::LogSink & log_sink_;
  const std::string logger_name_;
  const Mode mode_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Job> queue_;
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace ros2_medkit_fault_manager
