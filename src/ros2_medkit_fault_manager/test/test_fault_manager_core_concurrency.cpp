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

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_startup.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace fd = ros2_medkit::fault_detection;
namespace core = ros2_medkit_fault_manager::core;

namespace {

constexpr int kThreads = 8;
constexpr int kReportsPerThread = 500;
constexpr int kCodes = 4;
constexpr int kClearEvery = 50;
const char * const kClearedCode = "CLEARED_CODE";

class NullLogSink : public core::LogSink {
 public:
  void log(core::LogLevel /*level*/, const std::string & /*logger*/, const std::string & /*message*/) override {
  }
};

class CountingEventSink : public core::EventSink {
 public:
  void publish(const fd::FaultEventRecord & event) override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++counts_[{event.fault.fault_code, event.event_type}];
  }
  std::map<std::pair<std::string, std::string>, int> counts() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return counts_;
  }

 private:
  mutable std::mutex mutex_;
  std::map<std::pair<std::string, std::string>, int> counts_;
};

class NoCaptureHooks : public core::CaptureHooks {
 public:
  core::CaptureResult capture_now(const std::string & /*code*/) override {
    return {};
  }
  void on_prefailed(const std::string & /*code*/) override {
  }
  void on_cleared(const std::string & /*code*/) override {
  }
};

class FixedClock : public core::WallClock {
 public:
  int64_t now_ns() const override {
    return 1000000000LL;
  }
};

/// In-memory storage that yields inside the calls the report sequence makes. The read hands back
/// a result that is stale by the time the caller acts on it, so a caller without mutual exclusion
/// overlaps with others on every first report of a code.
class YieldingStorage : public ros2_medkit_fault_manager::InMemoryFaultStorage {
 public:
  bool report_fault_event(const std::string & fault_code, fd::ReportKind kind, uint8_t severity,
                          const std::string & description, const std::string & source_id, int64_t timestamp_ns,
                          const ros2_medkit_fault_manager::DebounceConfig & config) override {
    pause();
    return InMemoryFaultStorage::report_fault_event(fault_code, kind, severity, description, source_id, timestamp_ns,
                                                    config);
  }
  std::optional<fd::FaultRecord> get_fault(const std::string & fault_code) const override {
    auto result = InMemoryFaultStorage::get_fault(fault_code);
    pause();
    return result;
  }

 private:
  static void pause() {
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
};

/// One core on in-memory storage with threshold -1, plus the recorders it reports to.
struct Fixture {
  NullLogSink log;
  CountingEventSink events;
  NoCaptureHooks hooks;
  FixedClock clock;
  core::FaultManagerCore core;

  static core::FaultManagerConfig make_config() {
    core::FaultManagerConfig config;
    config.storage_type = "memory";
    config.confirmation_threshold = -1;
    return config;
  }

  static core::CoreParts make_parts(const core::FaultManagerConfig & config, core::LogSink & log) {
    auto parts = core::build_core_parts(config, log);
    parts.storage = std::make_unique<YieldingStorage>();
    return parts;
  }

  explicit Fixture(const core::FaultManagerConfig & config = make_config())
    : core(config, make_parts(config, log), events, hooks, clock, log) {
    fd::FaultReport seed;
    seed.fault_code = kClearedCode;
    seed.severity = fd::severity::kError;
    seed.source_id = "/seed";
    EXPECT_TRUE(core.report(seed).accepted);
  }
};

std::string code_name(int index) {
  return "CODE_" + std::to_string(index);
}

/// The sequence one thread runs: FAILED reports spread over the codes, a clear every kClearEvery.
void run_thread_sequence(core::FaultManagerCore & core, int thread_index) {
  for (int i = 0; i < kReportsPerThread; ++i) {
    fd::FaultReport report;
    report.fault_code = code_name((thread_index + i) % kCodes);
    report.kind = fd::ReportKind::kFailed;
    report.severity = fd::severity::kError;
    report.description = "load";
    report.source_id = "/thread_" + std::to_string(thread_index);
    EXPECT_TRUE(core.report(report).accepted);
    if (i % kClearEvery == 0) {
      EXPECT_TRUE(core.clear(kClearedCode, true).success);
    }
  }
}

}  // namespace

TEST(FaultManagerCoreConcurrency, ParallelReportsKeepCounts) {
  // The replay: the same per-thread sequences, one thread at a time.
  Fixture replay;
  for (int t = 0; t < kThreads; ++t) {
    run_thread_sequence(replay.core, t);
  }

  Fixture parallel;
  std::atomic<bool> go{false};
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&parallel, &go, t]() {
      while (!go.load()) {
        std::this_thread::yield();
      }
      run_thread_sequence(parallel.core, t);
    });
  }
  go.store(true);
  for (auto & thread : threads) {
    thread.join();
  }

  // Event counts per (code, type): every event once, none lost or duplicated.
  EXPECT_EQ(parallel.events.counts(), replay.events.counts());
  for (int c = 0; c < kCodes; ++c) {
    const auto code = code_name(c);
    const auto key = std::make_pair(code, std::string(fd::fault_event_type::kConfirmed));
    const auto counts = parallel.events.counts();
    ASSERT_EQ(counts.count(key), 1u) << code;
    EXPECT_EQ(counts.at(key), 1) << code << " confirmed more than once";
    // Every FAILED report after the confirming one is an update.
    const auto updated = std::make_pair(code, std::string(fd::fault_event_type::kUpdated));
    const int reports_per_code = kThreads * kReportsPerThread / kCodes;
    ASSERT_EQ(counts.count(updated), 1u) << code;
    EXPECT_EQ(counts.at(updated), reports_per_code - 1) << code;
  }

  // Per code occurrence_count and status equal the replay's.
  for (int c = 0; c < kCodes; ++c) {
    const auto code = code_name(c);
    const auto got = parallel.core.get(code);
    const auto want = replay.core.get(code);
    ASSERT_TRUE(got.success) << code;
    ASSERT_TRUE(want.success) << code;
    EXPECT_EQ(got.fault.occurrence_count, want.fault.occurrence_count) << code;
    EXPECT_EQ(got.fault.status, want.fault.status) << code;
  }
  const int expected_clears = kThreads * ((kReportsPerThread + kClearEvery - 1) / kClearEvery);
  const auto counts = parallel.events.counts();
  EXPECT_EQ(counts.at({kClearedCode, fd::fault_event_type::kCleared}), expected_clears);
}
