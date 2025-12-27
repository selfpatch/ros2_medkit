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

using ros2_medkit_fault_manager::SqliteFaultStorage;
using ros2_medkit_msgs::msg::Fault;

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

TEST_F(SqliteFaultStorageTest, ReportNewFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_->report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Motor temperature exceeded threshold",
                                       "/powertrain/motor", timestamp);

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_->size(), 1u);
  EXPECT_TRUE(storage_->contains("MOTOR_OVERHEAT"));
}

TEST_F(SqliteFaultStorageTest, ReportExistingFaultUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_->report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_WARN, "Initial report", "/powertrain/motor1", timestamp1);

  bool is_new = storage_->report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Second report", "/powertrain/motor2",
                                       timestamp2);

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

  // Report a fault (starts as PENDING)
  storage_->report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

  // Default query should return empty (only PENDING exists)
  auto faults = storage_->get_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 0u);
}

TEST_F(SqliteFaultStorageTest, GetFaultsWithPendingStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

  // Query with PENDING status
  auto faults = storage_->get_faults(false, 0, {"PENDING"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PENDING);
}

TEST_F(SqliteFaultStorageTest, GetFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault("FAULT_INFO", Fault::SEVERITY_INFO, "Info", "/node1", timestamp);
  storage_->report_fault("FAULT_ERROR", Fault::SEVERITY_ERROR, "Error", "/node1", timestamp);

  // Filter by ERROR severity
  auto faults = storage_->get_faults(true, Fault::SEVERITY_ERROR, {"PENDING"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(SqliteFaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

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

  storage_->report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);
  storage_->clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_->get_faults(false, 0, {"CLEARED"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(SqliteFaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

  // Query with invalid status - defaults to CONFIRMED (fault is PENDING, so no matches)
  auto faults = storage_->get_faults(false, 0, {"INVALID_STATUS"});
  EXPECT_EQ(faults.size(), 0u);
}

// SQLite-specific persistence test
TEST_F(SqliteFaultStorageTest, PersistenceAcrossRestarts) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Report some faults
  storage_->report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Persistent fault 1", "/node1", timestamp);
  storage_->report_fault("FAULT_2", Fault::SEVERITY_WARN, "Persistent fault 2", "/node2", timestamp);
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
  EXPECT_EQ(fault1->status, Fault::STATUS_PENDING);
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

  storage_->report_fault("FAULT_TS", Fault::SEVERITY_INFO, "Timestamp test", "/node1", timestamp);

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

  storage.report_fault("MEM_FAULT", Fault::SEVERITY_WARN, "In-memory test", "/test", timestamp);

  EXPECT_EQ(storage.size(), 1u);
  EXPECT_TRUE(storage.contains("MEM_FAULT"));
}

// Test reporting sources JSON handling
TEST_F(SqliteFaultStorageTest, ReportingSourcesJsonHandling) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Add multiple sources for the same fault
  storage_->report_fault("MULTI_SRC", Fault::SEVERITY_ERROR, "Multi-source", "/node/path/with/slashes", timestamp);
  storage_->report_fault("MULTI_SRC", Fault::SEVERITY_ERROR, "Multi-source", "/another/node", timestamp);
  storage_->report_fault("MULTI_SRC", Fault::SEVERITY_ERROR, "Multi-source", "/special\"chars", timestamp);

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

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
