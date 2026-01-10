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

TEST_F(SqliteFaultStorageTest, GetFaultsDefaultReturnsConfirmedOnly) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, single report confirms immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp);

  // Default query should return the CONFIRMED fault
  auto faults = storage_->get_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(SqliteFaultStorageTest, GetFaultsWithPrefailedStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Set threshold to -3 to test PREFAILED status
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp);

  // Query with PREFAILED status
  auto faults = storage_->get_faults(false, 0, {Fault::STATUS_PREFAILED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PREFAILED);
}

TEST_F(SqliteFaultStorageTest, GetFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  storage_->report_fault_event("FAULT_INFO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Info", "/node1",
                               timestamp);
  storage_->report_fault_event("FAULT_ERROR", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Error",
                               "/node1", timestamp);

  // Filter by ERROR severity (query CONFIRMED since that's the default status now)
  auto faults = storage_->get_faults(true, Fault::SEVERITY_ERROR, {Fault::STATUS_CONFIRMED});
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
  auto faults = storage_->get_faults(false, 0, {Fault::STATUS_CLEARED});
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
  auto faults = storage_->get_faults(false, 0, {"INVALID_STATUS"});
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

TEST_F(SqliteFaultStorageTest, ClearedFaultNotReconfirmed) {
  rclcpp::Clock clock;

  // Report 3 times to confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now());

  // Clear the fault
  storage_->clear_fault("FAULT_1");

  // Report again
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node4",
                               clock.now());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
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

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
