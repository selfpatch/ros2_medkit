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
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/core/rosbag_retention.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "test_log_sink.hpp"
#include "test_rosbag_rows.hpp"

namespace {

using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::RosbagApplyResult;
using ros2_medkit_fault_manager::RosbagChangeSet;
using ros2_medkit_fault_manager::RosbagFileInfo;
using ros2_medkit_fault_manager::SqliteFaultStorage;
using ros2_medkit_fault_manager::core::RemovalReason;
using ros2_medkit_fault_manager::core::RosbagRetention;
using ros2_medkit_fault_manager::core::RosbagRetentionLimits;
using ros2_medkit_fault_manager::core::StoreResult;

constexpr const char * kLogger = "test_rosbag_retention";
constexpr uint8_t kSeverityError = 2;

/// SQLite storage whose fault read holds at a gate after the real read has returned.
class GatedFaultReadStorage : public SqliteFaultStorage {
 public:
  using SqliteFaultStorage::SqliteFaultStorage;

  std::optional<fd::FaultRecord> get_fault(const std::string & fault_code) const override {
    auto fault = SqliteFaultStorage::get_fault(fault_code);
    std::unique_lock<std::mutex> lock(gate_mutex_);
    if (armed_) {
      entered_ = true;
      gate_cv_.notify_all();
      gate_cv_.wait_for(lock, std::chrono::seconds(30), [this]() {
        return released_;
      });
    }
    return fault;
  }

  void arm() {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    armed_ = true;
  }

  bool wait_entered(std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lock(gate_mutex_);
    return gate_cv_.wait_for(lock, timeout, [this]() {
      return entered_;
    });
  }

  void release() {
    std::lock_guard<std::mutex> lock(gate_mutex_);
    released_ = true;
    gate_cv_.notify_all();
  }

 private:
  mutable std::mutex gate_mutex_;
  mutable std::condition_variable gate_cv_;
  bool armed_{false};
  mutable bool entered_{false};
  bool released_{false};
};

/// Opens the gate when the test ends, so a failed assertion cannot leave a store waiting.
class GateRelease {
 public:
  explicit GateRelease(GatedFaultReadStorage & storage) : storage_(storage) {
  }
  ~GateRelease() {
    storage_.release();
  }
  GateRelease(const GateRelease &) = delete;
  GateRelease & operator=(const GateRelease &) = delete;
  GateRelease(GateRelease &&) = delete;
  GateRelease & operator=(GateRelease &&) = delete;

 private:
  GatedFaultReadStorage & storage_;
};

/// Storage whose per-fault rosbag read fails.
class FailingCapReadStorage : public InMemoryFaultStorage {
 public:
  std::vector<RosbagFileInfo> get_rosbag_files(const std::string & /*fault_code*/) const override {
    ++reads;
    throw std::runtime_error("cap read failed");
  }

  mutable size_t reads = 0;
};

/// Storage whose rosbag total fails.
class FailingQuotaReadStorage : public InMemoryFaultStorage {
 public:
  size_t get_total_rosbag_storage_bytes() const override {
    ++reads;
    throw std::runtime_error("quota read failed");
  }

  mutable size_t reads = 0;
};

/// Storage whose removal of one recording fails.
class FailingEvictionStorage : public InMemoryFaultStorage {
 public:
  explicit FailingEvictionStorage(std::string failing_recording) : failing_recording_(std::move(failing_recording)) {
  }

  RosbagApplyResult apply_rosbag_changes(const RosbagChangeSet & changes) override {
    const auto & ids = changes.remove_recordings;
    if (std::find(ids.begin(), ids.end(), failing_recording_) != ids.end()) {
      ++failures;
      throw std::runtime_error("eviction failed");
    }
    return InMemoryFaultStorage::apply_rosbag_changes(changes);
  }

  size_t failures = 0;

 private:
  std::string failing_recording_;
};

class RosbagRetentionStore : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::temp_directory_path() /
           ("medkit_retention_" + std::to_string(reinterpret_cast<uintptr_t>(this)));
    std::filesystem::remove_all(dir_);
    std::filesystem::create_directories(dir_);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  RosbagFileInfo row(const std::string & code, const std::string & bag_name, int64_t created_ns,
                     size_t size_bytes = 1024) {
    const auto path = dir_ / bag_name;
    std::filesystem::create_directories(path);
    std::ofstream(path / "metadata.yaml") << "version: 9\n";
    RosbagFileInfo info;
    info.fault_code = code;
    info.file_path = path.string();
    info.recording_id = bag_name;
    info.format = "mcap";
    info.duration_sec = 1.0;
    info.size_bytes = size_bytes;
    info.created_at_ns = created_ns;
    return info;
  }

  static RosbagRetentionLimits limits(size_t max_bags_per_fault, size_t max_total_bytes = 500ULL * 1024 * 1024) {
    RosbagRetentionLimits limits;
    limits.max_bags_per_fault = max_bags_per_fault;
    limits.max_total_bytes = max_total_bytes;
    return limits;
  }

  std::filesystem::path dir_;
};

TEST_F(RosbagRetentionStore, ClearBlocksWhileTheStoreChecks) {
  // A clear either shows as cleared in the store's check, or its removal runs after the insert
  // and takes the row. The gate holds the store between the check and the insert.
  GatedFaultReadStorage storage((dir_ / "faults.db").string(), test_log_sink());
  storage.report_fault_event("CLEAR_RACE", fd::ReportKind::kFailed, kSeverityError, "race", "/node", 1000,
                             DebounceConfig{});
  // auto_cleanup on and a cap of 1: the store reads the fault before it inserts.
  RosbagRetention retention(storage, limits(1), test_log_sink(), kLogger);
  const auto bag = row("CLEAR_RACE", "fault_CLEAR_RACE_1", 1000);

  storage.arm();
  auto stored = std::async(std::launch::async, [&retention, &bag]() {
    return retention.store({bag});
  });
  std::future<ros2_medkit_fault_manager::core::RemovalOutcome> cleared;
  GateRelease release(storage);
  ASSERT_TRUE(storage.wait_entered(std::chrono::seconds(10))) << "the store never read the fault";

  // The read returned "not cleared". The clear marks the fault first, as the core does.
  ASSERT_TRUE(storage.clear_fault("CLEAR_RACE"));
  cleared = std::async(std::launch::async, [&retention]() {
    return retention.on_fault_cleared("CLEAR_RACE");
  });
  // Bounded negative wait: a clear blocked on the retention mutex gives no sign of it, so the
  // test can only see that it has not returned within the window.
  EXPECT_EQ(cleared.wait_for(std::chrono::milliseconds(500)), std::future_status::timeout)
      << "the clear ran while the store was between its check and its insert";

  storage.release();
  const auto store_outcome = stored.get();
  const auto removal = cleared.get();
  EXPECT_EQ(store_outcome.result, StoreResult::kStored);
  EXPECT_TRUE(storage.get_rosbag_files("CLEAR_RACE").empty()) << "a cleared fault kept its row";
  EXPECT_EQ(removal.removed_rows, 1u);
  expect_unlinkable(removal.unlink_paths);
  EXPECT_TRUE(listed(removal.unlink_paths, bag.file_path)) << "the clear did not hand over the directory";
}

TEST_F(RosbagRetentionStore, NoCapReadWhenTheCapIsZero) {
  FailingCapReadStorage storage;
  RosbagRetention retention(storage, limits(0), test_log_sink(), kLogger);
  const auto bag = row("NO_CAP", "fault_NO_CAP_1", 1000);

  const auto outcome = retention.store({bag});

  EXPECT_EQ(outcome.result, StoreResult::kStored) << outcome.error;
  EXPECT_EQ(storage.reads, 0u) << "a cap of 0 read the fault's recordings";
  EXPECT_TRUE(storage.get_rosbag_file("NO_CAP").has_value());
  EXPECT_TRUE(outcome.unlink_paths.empty());
}

TEST_F(RosbagRetentionStore, FailingCapReadIsAStoreFailure) {
  FailingCapReadStorage storage;
  RosbagRetention retention(storage, limits(1), test_log_sink(), kLogger);
  const auto bag = row("CAP_READ", "fault_CAP_READ_1", 1000);

  const auto outcome = retention.store({bag});

  EXPECT_EQ(outcome.result, StoreResult::kStoreFailed);
  EXPECT_EQ(storage.reads, 1u) << "a failed cap read is not retried";
  EXPECT_NE(outcome.error.find("cap read failed"), std::string::npos) << outcome.error;
  EXPECT_FALSE(storage.get_rosbag_file("CAP_READ").has_value());
  EXPECT_TRUE(outcome.unlink_paths.empty()) << "the caller discards the bag, not the worker";
}

TEST_F(RosbagRetentionStore, FailingQuotaReadKeepsTheBag) {
  FailingQuotaReadStorage storage;
  RosbagRetention retention(storage, limits(1), test_log_sink(), kLogger);
  const auto bag = row("QUOTA_READ", "fault_QUOTA_READ_1", 1000);

  const auto outcome = retention.store({bag});

  EXPECT_EQ(outcome.result, StoreResult::kStored);
  EXPECT_TRUE(outcome.quota_failed);
  EXPECT_EQ(storage.reads, 1u);
  EXPECT_NE(outcome.error.find("quota read failed"), std::string::npos) << outcome.error;
  EXPECT_TRUE(storage.get_rosbag_file("QUOTA_READ").has_value()) << "a failed quota took the stored row";
  EXPECT_TRUE(outcome.unlink_paths.empty());
}

TEST_F(RosbagRetentionStore, EvictionStopsAtTheFirstFailure) {
  FailingEvictionStorage storage("fault_O2_2000");
  const auto o1 = row("O1", "fault_O1_1000", 1000, 500);
  const auto o2 = row("O2", "fault_O2_2000", 2000, 500);
  seed_rosbag_rows(storage, {o1, o2});
  RosbagRetention retention(storage, limits(0, 600), test_log_sink(), kLogger);
  const auto fresh = row("NEW", "fault_NEW_3000", 3000, 500);

  const auto outcome = retention.store({fresh});

  EXPECT_EQ(outcome.result, StoreResult::kStored);
  EXPECT_TRUE(outcome.quota_failed);
  EXPECT_NE(outcome.error.find("eviction failed"), std::string::npos) << outcome.error;
  EXPECT_EQ(storage.failures, 1u);
  expect_unlinkable(outcome.unlink_paths);
  EXPECT_TRUE(storage.get_rosbag_files("O1").empty()) << "the eviction before the failure was undone";
  EXPECT_TRUE(listed(outcome.unlink_paths, o1.file_path));
  EXPECT_EQ(storage.get_rosbag_files("O2").size(), 1u);
  EXPECT_FALSE(listed(outcome.unlink_paths, o2.file_path));
  EXPECT_EQ(storage.get_rosbag_files("NEW").size(), 1u);
  EXPECT_FALSE(listed(outcome.unlink_paths, fresh.file_path));
}

TEST_F(RosbagRetentionStore, QuotaRunsWhenTheCapRemovedEveryNewRow) {
  InMemoryFaultStorage storage;
  const auto other = row("OTHER", "fault_OTHER_1000", 1000, 500);
  const auto newer = row("CAPPED", "fault_CAPPED_9000", 9000, 500);
  seed_rosbag_rows(storage, {other, newer});
  RosbagRetention retention(storage, limits(1, 600), test_log_sink(), kLogger);
  // Older than the stored recording, so it loses the cap of 1.
  const auto loser = row("CAPPED", "fault_CAPPED_2000", 2000, 500);

  const auto outcome = retention.store({loser});

  EXPECT_EQ(outcome.result, StoreResult::kStored);
  EXPECT_FALSE(outcome.quota_failed) << outcome.error;
  const auto capped = storage.get_rosbag_files("CAPPED");
  ASSERT_EQ(capped.size(), 1u);
  EXPECT_EQ(capped[0].file_path, newer.file_path);
  expect_unlinkable(outcome.unlink_paths);
  EXPECT_TRUE(listed(outcome.unlink_paths, loser.file_path)) << "no row names the new recording";
  EXPECT_TRUE(storage.get_rosbag_files("OTHER").empty()) << "the quota did not run";
  EXPECT_TRUE(listed(outcome.unlink_paths, other.file_path));
  EXPECT_FALSE(listed(outcome.unlink_paths, newer.file_path));
  // Each freed directory carries the reason its removal is logged with.
  for (const auto & recording : outcome.unlink_paths) {
    if (recording.path == loser.file_path) {
      EXPECT_EQ(recording.reason, RemovalReason::kCap);
      EXPECT_EQ(recording.recording_id, "fault_CAPPED_2000");
    } else {
      EXPECT_EQ(recording.reason, RemovalReason::kQuota);
      EXPECT_EQ(recording.recording_id, "fault_OTHER_1000");
    }
  }
}

TEST_F(RosbagRetentionStore, QuotaStopsAtTheLimit) {
  InMemoryFaultStorage storage;
  const auto o1 = row("O1", "fault_O1_1000", 1000, 300);
  const auto o2 = row("O2", "fault_O2_2000", 2000, 300);
  seed_rosbag_rows(storage, {o1, o2});
  RosbagRetention retention(storage, limits(0, 1000), test_log_sink(), kLogger);

  // 300 + 300 + 400 is the limit exactly.
  const auto at_limit = retention.store({row("N1", "fault_N1_3000", 3000, 400)});
  EXPECT_EQ(at_limit.result, StoreResult::kStored);
  EXPECT_TRUE(at_limit.unlink_paths.empty()) << "a total equal to the limit evicted a recording";
  EXPECT_EQ(storage.get_rosbag_files("O1").size(), 1u);

  // 1300 is over: O1 goes, and the eviction stops at the limit with O2 still stored.
  const auto over = retention.store({row("N2", "fault_N2_4000", 4000, 300)});
  EXPECT_EQ(over.result, StoreResult::kStored);
  expect_unlinkable(over.unlink_paths);
  EXPECT_EQ(paths_of(over.unlink_paths), std::vector<std::string>{o1.file_path});
  EXPECT_TRUE(storage.get_rosbag_files("O1").empty());
  EXPECT_EQ(storage.get_rosbag_files("O2").size(), 1u) << "the eviction went past the limit";
}

TEST_F(RosbagRetentionStore, AutoCleanupCountsRemovedRows) {
  InMemoryFaultStorage storage;
  const auto own = row("CLEARED", "fault_CLEARED_1000", 1000);
  auto shared = row("CLEARED", "fault_SHARED_2000", 2000);
  seed_rosbag_rows(storage, {own, shared});
  shared.fault_code = "SIBLING";
  seed_rosbag_rows(storage, {shared});
  RosbagRetention retention(storage, limits(1), test_log_sink(), kLogger);

  const auto removal = retention.on_fault_cleared("CLEARED");
  EXPECT_EQ(removal.removed_rows, 2u);
  expect_unlinkable(removal.unlink_paths);
  EXPECT_EQ(paths_of(removal.unlink_paths), std::vector<std::string>{own.file_path})
      << "SIBLING still names the shared bag";
  EXPECT_TRUE(storage.get_rosbag_files("CLEARED").empty());
  EXPECT_EQ(storage.get_rosbag_files("SIBLING").size(), 1u);

  const auto none = retention.on_fault_cleared("NEVER_RECORDED");
  EXPECT_EQ(none.removed_rows, 0u);
  EXPECT_TRUE(none.unlink_paths.empty());
}

}  // namespace
