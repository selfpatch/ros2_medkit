// Copyright 2025 mfaferek93
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

#include <cstdio>
#include <filesystem>
#include <memory>
#include <random>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::SqliteFaultStorage;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::srv::ReportFault;

class SqliteFaultStorageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a unique temp file for each test using random_device for better entropy
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    temp_db_path_ = std::filesystem::temp_directory_path() / ("test_faults_" + std::to_string(dist(gen)) + ".db");
    storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());
  }

  void TearDown() override {
    storage_.reset();
    // Clean up temp file
    std::filesystem::remove(temp_db_path_);
    // Also remove WAL and SHM files if they exist
    std::filesystem::remove(temp_db_path_.string() + "-wal");
    std::filesystem::remove(temp_db_path_.string() + "-shm");
  }

  std::filesystem::path temp_db_path_;
  std::unique_ptr<SqliteFaultStorage> storage_;
};

TEST_F(SqliteFaultStorageTest, ReportNewFaultEvent) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new =
      storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                   "Motor temperature exceeded threshold", "/powertrain/motor", timestamp);

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_->size(), 1u);
  EXPECT_TRUE(storage_->contains("MOTOR_OVERHEAT"));
}

TEST_F(SqliteFaultStorageTest, PassedEventForNonExistentFaultIgnored) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_->report_fault_event("NON_EXISTENT", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR,
                                             "Test", "/node1", timestamp);

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_->size(), 0u);
}

TEST_F(SqliteFaultStorageTest, ReportExistingFaultEventUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Initial report", "/powertrain/motor1", timestamp1);

  bool is_new = storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED,
                                             Fault::SEVERITY_ERROR, "Second report", "/powertrain/motor2", timestamp2);

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_->size(), 1u);

  auto fault = storage_->get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 2u);
  EXPECT_EQ(fault->severity, Fault::SEVERITY_ERROR);  // Updated to higher severity
  EXPECT_EQ(fault->reporting_sources.size(), 2u);
}

TEST_F(SqliteFaultStorageTest, ListFaultsDefaultReturnsConfirmedOnly) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, single report confirms immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp);

  // Default query should return the CONFIRMED fault
  auto faults = storage_->list_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ListFaultsWithPrefailedStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Set threshold to -3 to test PREFAILED status
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp);

  // Query with PREFAILED status
  auto faults = storage_->list_faults(false, 0, {Fault::STATUS_PREFAILED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PREFAILED);
}

TEST_F(SqliteFaultStorageTest, ListFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  storage_->report_fault_event("FAULT_INFO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Info", "/node1",
                               timestamp);
  storage_->report_fault_event("FAULT_ERROR", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Error",
                               "/node1", timestamp);

  // Filter by ERROR severity (query CONFIRMED since that's the default status now)
  auto faults = storage_->list_faults(true, Fault::SEVERITY_ERROR, {Fault::STATUS_CONFIRMED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(SqliteFaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                               "/node1", timestamp);

  bool cleared = storage_->clear_fault("MOTOR_OVERHEAT");
  EXPECT_TRUE(cleared);

  auto fault = storage_->get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);
}

TEST_F(SqliteFaultStorageTest, ClearNonExistentFault) {
  bool cleared = storage_->clear_fault("NON_EXISTENT");
  EXPECT_FALSE(cleared);
}

TEST_F(SqliteFaultStorageTest, GetClearedFaults) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp);
  storage_->clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_->list_faults(false, 0, {Fault::STATUS_CLEARED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(SqliteFaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, fault is immediately CONFIRMED
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp);

  // Query with invalid status - defaults to CONFIRMED, which now matches our fault
  auto faults = storage_->list_faults(false, 0, {"INVALID_STATUS"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

// SQLite-specific persistence test
TEST_F(SqliteFaultStorageTest, PersistenceAcrossRestarts) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  // Report some faults
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Persistent fault 1", "/node1", timestamp);
  storage_->report_fault_event("FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Persistent fault 2", "/node2", timestamp);
  storage_->clear_fault("FAULT_2");

  // Verify initial state
  EXPECT_EQ(storage_->size(), 2u);

  // Close the storage
  storage_.reset();

  // Reopen the storage
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  // Verify faults persisted
  EXPECT_EQ(storage_->size(), 2u);
  EXPECT_TRUE(storage_->contains("FAULT_1"));
  EXPECT_TRUE(storage_->contains("FAULT_2"));

  auto fault1 = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault1.has_value());
  EXPECT_EQ(fault1->severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(fault1->status, Fault::STATUS_CONFIRMED);  // Immediately confirmed with threshold=-1
  EXPECT_EQ(fault1->description, "Persistent fault 1");

  auto fault2 = storage_->get_fault("FAULT_2");
  ASSERT_TRUE(fault2.has_value());
  EXPECT_EQ(fault2->status, Fault::STATUS_CLEARED);
}

// Test timestamp precision
TEST_F(SqliteFaultStorageTest, TimestampPrecision) {
  // Create a timestamp with nanosecond precision
  int64_t test_ns = 1735312456123456789LL;  // Specific nanosecond timestamp
  rclcpp::Time timestamp(test_ns, RCL_SYSTEM_TIME);

  storage_->report_fault_event("FAULT_TS", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Timestamp test",
                               "/node1", timestamp);

  auto fault = storage_->get_fault("FAULT_TS");
  ASSERT_TRUE(fault.has_value());

  // Convert builtin_interfaces::msg::Time back to rclcpp::Time for comparison
  rclcpp::Time first_ts(fault->first_occurred);
  rclcpp::Time last_ts(fault->last_occurred);

  // Verify nanosecond precision is preserved
  EXPECT_EQ(first_ts.nanoseconds(), test_ns);
  EXPECT_EQ(last_ts.nanoseconds(), test_ns);
}

// Test in-memory SQLite database
TEST(SqliteInMemoryTest, InMemoryDatabase) {
  SqliteFaultStorage storage(":memory:");
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage.report_fault_event("MEM_FAULT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN, "In-memory test",
                             "/test", timestamp);

  EXPECT_EQ(storage.size(), 1u);
  EXPECT_TRUE(storage.contains("MEM_FAULT"));
}

// Test reporting sources JSON handling
TEST_F(SqliteFaultStorageTest, ReportingSourcesJsonHandling) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Add multiple sources for the same fault
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/node/path/with/slashes", timestamp);
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/another/node", timestamp);
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/special\"chars", timestamp);

  auto fault = storage_->get_fault("MULTI_SRC");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->reporting_sources.size(), 3u);

  // Verify all sources are present (order may vary due to set)
  std::set<std::string> sources(fault->reporting_sources.begin(), fault->reporting_sources.end());
  EXPECT_TRUE(sources.count("/node/path/with/slashes") > 0);
  EXPECT_TRUE(sources.count("/another/node") > 0);
  EXPECT_TRUE(sources.count("/special\"chars") > 0);
}

// Test database path accessor
TEST_F(SqliteFaultStorageTest, DbPathAccessor) {
  EXPECT_EQ(storage_->db_path(), temp_db_path_.string());
}

// Debounce config tests for SQLite storage
TEST_F(SqliteFaultStorageTest, DefaultDebounceConfig) {
  auto config = storage_->get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
  EXPECT_FALSE(config.healing_enabled);
  EXPECT_EQ(config.healing_threshold, 3);
  EXPECT_TRUE(config.critical_immediate_confirm);
}

TEST_F(SqliteFaultStorageTest, SetDebounceConfig) {
  DebounceConfig config;
  config.confirmation_threshold = -5;
  config.healing_enabled = true;
  config.healing_threshold = 5;
  config.critical_immediate_confirm = false;

  storage_->set_debounce_config(config);
  auto retrieved = storage_->get_debounce_config();

  EXPECT_EQ(retrieved.confirmation_threshold, -5);
  EXPECT_TRUE(retrieved.healing_enabled);
  EXPECT_EQ(retrieved.healing_threshold, 5);
  EXPECT_FALSE(retrieved.critical_immediate_confirm);
}

TEST_F(SqliteFaultStorageTest, FaultStaysPrefailedAboveThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (2 FAILED events should stay PREFAILED)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 2u);
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(SqliteFaultStorageTest, FaultConfirmsAtThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (3 FAILED events should confirm)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 3u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ImmediateConfirmationWithThresholdZero) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = 0;  // Immediate confirmation
  storage_->set_debounce_config(config);

  // Single report should confirm immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, CriticalSeverityBypassesDebounce) {
  rclcpp::Clock clock;

  // CRITICAL severity should confirm immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                               "/node1", clock.now());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, ClearedFaultCanBeReactivated) {
  rclcpp::Clock clock;

  // Report to confirm (with default threshold=-1, single report confirms)
  bool is_new = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                             "Initial", "/node1", clock.now());
  EXPECT_TRUE(is_new);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->occurrence_count, 1u);

  // Clear the fault
  storage_->clear_fault("FAULT_1");
  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);

  // Report again - should reactivate
  is_new = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                        "Reactivated", "/node2", clock.now());
  EXPECT_TRUE(is_new);  // Should return true like a new fault

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Should be reconfirmed
  EXPECT_EQ(fault->occurrence_count, 2u);             // Should increment
  EXPECT_EQ(fault->reporting_sources.size(), 2u);     // Both sources
  EXPECT_EQ(fault->description, "Reactivated");       // Updated description
}

TEST_F(SqliteFaultStorageTest, PassedEventForClearedFaultIgnored) {
  rclcpp::Clock clock;

  // Report and confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());

  // Clear the fault
  storage_->clear_fault("FAULT_1");

  // PASSED event should be ignored for CLEARED fault
  bool result =
      storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
  EXPECT_FALSE(result);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
}

TEST_F(SqliteFaultStorageTest, ClearedFaultReactivationRestartsDebounce) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  // Report 3 times to confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Clear the fault
  storage_->clear_fault("FAULT_1");

  // Reactivate - should start in PREFAILED with counter=-1
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node4",
                               clock.now());

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);  // Not yet confirmed, needs 2 more FAILED

  // Report 2 more times to re-confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node5",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node6",
                               clock.now());

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Now confirmed
}

TEST_F(SqliteFaultStorageTest, ConfirmationPersistsAfterReopen) {
  rclcpp::Clock clock;

  // Report 3 times to confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now());

  // Close and reopen storage
  storage_.reset();
  storage_ = std::make_unique<SqliteFaultStorage>(temp_db_path_.string());

  // Verify status persisted
  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, PassedEventIncrementsCounter) {
  rclcpp::Clock clock;

  // Report 2 FAILED events
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now());

  // Report 3 PASSED events
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREPASSED);  // Counter > 0
}

TEST_F(SqliteFaultStorageTest, HealingWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_->set_debounce_config(config);

  // Report 1 FAILED event
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());

  // Report 4 PASSED events (counter = -1 + 4 = +3, reaches healing threshold)
  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
  }

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

TEST_F(SqliteFaultStorageTest, TimeBasedConfirmationWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;  // Need debounce so fault stays PREFAILED
  config.auto_confirm_after_sec = 10.0;
  storage_->set_debounce_config(config);

  auto now = clock.now();
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               now);

  // Check before timeout - should not confirm
  auto before_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(5e9));
  size_t confirmed_early = storage_->check_time_based_confirmation(before_timeout);
  EXPECT_EQ(confirmed_early, 0u);

  // Check after timeout - should confirm
  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  size_t confirmed = storage_->check_time_based_confirmation(after_timeout);
  EXPECT_EQ(confirmed, 1u);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

// Snapshot storage tests
// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, StoreAndRetrieveSnapshot) {
  using ros2_medkit_fault_manager::SnapshotData;

  // First, create a fault to associate the snapshot with
  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now());

  // Store a snapshot
  SnapshotData snapshot;
  snapshot.fault_code = "MOTOR_OVERHEAT";
  snapshot.topic = "/motor/temperature";
  snapshot.message_type = "sensor_msgs/msg/Temperature";
  snapshot.data = R"({"temperature": 85.5, "variance": 0.1})";
  snapshot.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot);

  // Retrieve snapshots
  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");
  ASSERT_EQ(snapshots.size(), 1u);

  EXPECT_EQ(snapshots[0].fault_code, "MOTOR_OVERHEAT");
  EXPECT_EQ(snapshots[0].topic, "/motor/temperature");
  EXPECT_EQ(snapshots[0].message_type, "sensor_msgs/msg/Temperature");
  EXPECT_EQ(snapshots[0].data, R"({"temperature": 85.5, "variance": 0.1})");
  EXPECT_EQ(snapshots[0].captured_at_ns, snapshot.captured_at_ns);
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, MultipleSnapshotsForSameFault) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now());

  // Store multiple snapshots for the same fault
  SnapshotData snapshot1;
  snapshot1.fault_code = "MOTOR_OVERHEAT";
  snapshot1.topic = "/motor/temperature";
  snapshot1.message_type = "sensor_msgs/msg/Temperature";
  snapshot1.data = R"({"temperature": 85.5})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();

  SnapshotData snapshot2;
  snapshot2.fault_code = "MOTOR_OVERHEAT";
  snapshot2.topic = "/motor/rpm";
  snapshot2.message_type = "std_msgs/msg/Float64";
  snapshot2.data = R"({"data": 5500.0})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot1);
  storage_->store_snapshot(snapshot2);

  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");
  EXPECT_EQ(snapshots.size(), 2u);
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, FilterSnapshotsByTopic) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now());

  SnapshotData snapshot1;
  snapshot1.fault_code = "MOTOR_OVERHEAT";
  snapshot1.topic = "/motor/temperature";
  snapshot1.message_type = "sensor_msgs/msg/Temperature";
  snapshot1.data = R"({"temperature": 85.5})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();

  SnapshotData snapshot2;
  snapshot2.fault_code = "MOTOR_OVERHEAT";
  snapshot2.topic = "/motor/rpm";
  snapshot2.message_type = "std_msgs/msg/Float64";
  snapshot2.data = R"({"data": 5500.0})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot1);
  storage_->store_snapshot(snapshot2);

  // Filter by topic
  auto filtered = storage_->get_snapshots("MOTOR_OVERHEAT", "/motor/temperature");
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_EQ(filtered[0].topic, "/motor/temperature");
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, NoSnapshotsForUnknownFault) {
  auto snapshots = storage_->get_snapshots("UNKNOWN_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

// @verifies REQ_INTEROP_088
TEST_F(SqliteFaultStorageTest, ClearFaultDeletesAssociatedSnapshots) {
  using ros2_medkit_fault_manager::SnapshotData;
  rclcpp::Clock clock;

  // Create a fault using report_fault_event
  storage_->report_fault_event("SNAPSHOT_CLEAR_TEST", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Test fault for snapshot cleanup", "/test_node", clock.now());

  // Store snapshots for this fault
  SnapshotData snapshot1;
  snapshot1.fault_code = "SNAPSHOT_CLEAR_TEST";
  snapshot1.topic = "/test/topic1";
  snapshot1.message_type = "std_msgs/msg/String";
  snapshot1.data = R"({"data": "test1"})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot1);

  SnapshotData snapshot2;
  snapshot2.fault_code = "SNAPSHOT_CLEAR_TEST";
  snapshot2.topic = "/test/topic2";
  snapshot2.message_type = "std_msgs/msg/String";
  snapshot2.data = R"({"data": "test2"})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot2);

  // Verify snapshots exist
  auto snapshots_before = storage_->get_snapshots("SNAPSHOT_CLEAR_TEST");
  ASSERT_EQ(snapshots_before.size(), 2u);

  // Clear the fault
  bool cleared = storage_->clear_fault("SNAPSHOT_CLEAR_TEST");
  EXPECT_TRUE(cleared);

  // Verify snapshots are deleted
  auto snapshots_after = storage_->get_snapshots("SNAPSHOT_CLEAR_TEST");
  EXPECT_TRUE(snapshots_after.empty());
}

// Rosbag entity-scoped listing tests

// @verifies REQ_INTEROP_071
TEST_F(SqliteFaultStorageTest, ListRosbagsForEntityFiltersCorrectly) {
  using ros2_medkit_fault_manager::RosbagFileInfo;
  rclcpp::Clock clock;

  // Create fault with reporting source for entity
  storage_->report_fault_event("ENTITY_FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Fault from entity", "/powertrain/motor", clock.now());

  // Create another fault with different reporting source
  storage_->report_fault_event("ENTITY_FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Fault from other entity", "/chassis/brake", clock.now());

  // Store rosbags for both faults
  RosbagFileInfo info1;
  info1.fault_code = "ENTITY_FAULT_1";
  info1.file_path = "/tmp/entity1.mcap";
  info1.format = "mcap";
  info1.duration_sec = 5.0;
  info1.size_bytes = 1024;
  info1.created_at_ns = clock.now().nanoseconds();
  storage_->store_rosbag_file(info1);

  RosbagFileInfo info2;
  info2.fault_code = "ENTITY_FAULT_2";
  info2.file_path = "/tmp/entity2.mcap";
  info2.format = "mcap";
  info2.duration_sec = 3.0;
  info2.size_bytes = 512;
  info2.created_at_ns = clock.now().nanoseconds();
  storage_->store_rosbag_file(info2);

  // Get rosbags for motor entity
  auto rosbags = storage_->list_rosbags_for_entity("/powertrain/motor");
  ASSERT_EQ(rosbags.size(), 1u);
  EXPECT_EQ(rosbags[0].fault_code, "ENTITY_FAULT_1");

  // Get rosbags for brake entity
  auto brake_rosbags = storage_->list_rosbags_for_entity("/chassis/brake");
  ASSERT_EQ(brake_rosbags.size(), 1u);
  EXPECT_EQ(brake_rosbags[0].fault_code, "ENTITY_FAULT_2");

  // Get rosbags for unknown entity
  auto unknown_rosbags = storage_->list_rosbags_for_entity("/unknown/entity");
  EXPECT_TRUE(unknown_rosbags.empty());
}

// @verifies REQ_INTEROP_073
TEST_F(SqliteFaultStorageTest, GetAllRosbagFilesReturnsSortedByCreatedAt) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  RosbagFileInfo info1;
  info1.fault_code = "FAULT_A";
  info1.file_path = "/tmp/a.mcap";
  info1.format = "mcap";
  info1.duration_sec = 1.0;
  info1.size_bytes = 100;
  info1.created_at_ns = 1000;
  storage_->store_rosbag_file(info1);

  RosbagFileInfo info2;
  info2.fault_code = "FAULT_B";
  info2.file_path = "/tmp/b.mcap";
  info2.format = "mcap";
  info2.duration_sec = 2.0;
  info2.size_bytes = 200;
  info2.created_at_ns = 2000;
  storage_->store_rosbag_file(info2);

  auto all_rosbags = storage_->get_all_rosbag_files();
  ASSERT_EQ(all_rosbags.size(), 2u);

  // Should be sorted by created_at_ns (oldest first)
  EXPECT_EQ(all_rosbags[0].fault_code, "FAULT_A");
  EXPECT_EQ(all_rosbags[1].fault_code, "FAULT_B");
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
