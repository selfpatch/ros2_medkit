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

#include "ros2_medkit_fault_manager/retention_worker.hpp"

#include <exception>
#include <filesystem>
#include <system_error>

namespace ros2_medkit_fault_manager {

namespace {

/// Why a recording went, as its log lines name it.
const char * removal_reason_name(core::RemovalReason reason) {
  switch (reason) {
    case core::RemovalReason::kCap:
      return "max_bags_per_fault";
    case core::RemovalReason::kQuota:
      return "max_total_storage_mb";
    case core::RemovalReason::kCleared:
      return "fault cleared";
    case core::RemovalReason::kStale:
      return "directory missing";
  }
  return "unknown";
}

}  // namespace

RetentionWorker::RetentionWorker(core::LogSink & log_sink, std::string logger_name, Mode mode)
  : log_sink_(log_sink), logger_name_(std::move(logger_name)), mode_(mode) {
  if (mode_ == Mode::kThread) {
    thread_ = std::thread([this]() {
      thread_loop();
    });
  }
}

RetentionWorker::~RetentionWorker() {
  // A destructor must not throw, and join() can.
  try {
    drain_and_stop();
  } catch (...) {
  }
}

void RetentionWorker::post(std::string kind, std::function<void()> job) {
  Job entry{std::move(kind), std::move(job)};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      core::logf(log_sink_, core::LogLevel::kWarn, logger_name_, "Retention worker stopped, dropped job: %s",
                 entry.kind.c_str());
      return;
    }
    if (mode_ == Mode::kThread) {
      queue_.push_back(std::move(entry));
      cv_.notify_one();
      return;
    }
  }
  run_job(entry);
}

void RetentionWorker::drain_and_stop() {
  std::thread worker;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    worker = std::move(thread_);
  }
  cv_.notify_all();
  if (worker.joinable()) {
    worker.join();
  }
}

void RetentionWorker::thread_loop() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    cv_.wait(lock, [this]() {
      return !queue_.empty() || stopping_;
    });
    if (queue_.empty()) {
      return;
    }
    Job job = std::move(queue_.front());
    queue_.pop_front();
    lock.unlock();
    run_job(job);
    lock.lock();
  }
}

void RetentionWorker::run_job(const Job & job) {
  try {
    job.run();
  } catch (const std::exception & e) {
    core::logf(log_sink_, core::LogLevel::kError, logger_name_, "Retention job failed: %s: %s", job.kind.c_str(),
               e.what());
  } catch (...) {
    core::logf(log_sink_, core::LogLevel::kError, logger_name_, "Retention job failed: %s: unknown exception",
               job.kind.c_str());
  }
  core::logf(log_sink_, core::LogLevel::kDebug, logger_name_, "retention job done: %s", job.kind.c_str());
}

bool RetentionWorker::remove_recording(const core::FreedRecording & recording) {
  std::error_code ec;
  std::filesystem::remove_all(recording.path, ec);
  if (ec) {
    core::logf(log_sink_, core::LogLevel::kWarn, logger_name_, "Failed to remove recording '%s' (%s): %s",
               recording.path.c_str(), removal_reason_name(recording.reason), ec.message().c_str());
    return false;
  }
  switch (recording.reason) {
    case core::RemovalReason::kQuota:
      core::logf(log_sink_, core::LogLevel::kInfo, logger_name_, "Deleted old bag '%s' to enforce storage limit",
                 recording.path.c_str());
      break;
    case core::RemovalReason::kStale:
      // The directory was already missing: there was nothing to remove.
      break;
    case core::RemovalReason::kCap:
    case core::RemovalReason::kCleared:
      core::logf(log_sink_, core::LogLevel::kInfo, logger_name_, "Recording removed: %s (%s)",
                 recording.recording_id.c_str(), removal_reason_name(recording.reason));
      break;
  }
  return true;
}

void RetentionWorker::post_removals(std::string kind, std::vector<core::FreedRecording> recordings) {
  if (recordings.empty()) {
    return;
  }
  // The job runs on this worker, so the worker outlives it.
  post(std::move(kind), [this, recordings = std::move(recordings)]() {
    for (const auto & recording : recordings) {
      remove_recording(recording);
    }
  });
}

}  // namespace ros2_medkit_fault_manager
