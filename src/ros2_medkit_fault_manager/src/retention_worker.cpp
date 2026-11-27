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

void post_unlinks(RetentionWorker & worker, std::string kind, std::vector<core::FreedRecording> recordings) {
  if (recordings.empty()) {
    return;
  }
  worker.post(std::move(kind), [recordings = std::move(recordings)]() {
    for (const auto & recording : recordings) {
      // A directory that is already gone needs no removal.
      std::error_code ec;
      std::filesystem::remove_all(recording.path, ec);
    }
  });
}

}  // namespace ros2_medkit_fault_manager
