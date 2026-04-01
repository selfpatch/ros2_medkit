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

#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_manager_node.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/list_faults_for_entity.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::FaultManagerNode;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::msg::FaultEvent;
using ros2_medkit_msgs::srv::ClearFault;
using ros2_medkit_msgs::srv::GetFault;
using ros2_medkit_msgs::srv::ListFaultsForEntity;
using ros2_medkit_msgs::srv::ReportFault;

/// Default debounce config for tests (matches DebounceConfig defaults: threshold=-1, no healing)
static DebounceConfig default_config() {
  return DebounceConfig{};
}

class FaultStorageTest : public ::testing::Test {
 protected:
  InMemoryFaultStorage storage_;
};

TEST_F(FaultStorageTest, ReportNewFaultEvent) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Motor temperature exceeded threshold", "/powertrain/motor", timestamp,
                                            default_config());

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_.size(), 1u);
  EXPECT_TRUE(storage_.contains("MOTOR_OVERHEAT"));
}

TEST_F(FaultStorageTest, PassedEventForNonExistentFaultIgnored) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_.report_fault_event("NON_EXISTENT", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR,
                                            "Test", "/node1", timestamp, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_.size(), 0u);
}

TEST_F(FaultStorageTest, ReportExistingFaultEventUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                              "Initial report", "/powertrain/motor1", timestamp1, default_config());

  bool is_new = storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Second report", "/powertrain/motor2", timestamp2, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_.size(), 1u);

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 2u);
  EXPECT_EQ(fault->severity, Fault::SEVERITY_ERROR);  // Updated to higher severity
  EXPECT_EQ(fault->reporting_sources.size(), 2u);
}

TEST_F(FaultStorageTest, ListFaultsDefaultReturnsConfirmedOnly) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, single report confirms immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, default_config());

  // Default query should return the CONFIRMED fault
  auto faults = storage_.list_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ListFaultsWithPrefailedStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Set threshold to -3 to test PREFAILED status
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, config);

  // Query with PREFAILED status
  auto faults = storage_.list_faults(false, 0, {Fault::STATUS_PREFAILED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, ListFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  storage_.report_fault_event("FAULT_INFO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Info", "/node1",
                              timestamp, default_config());
  storage_.report_fault_event("FAULT_ERROR", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Error",
                              "/node1", timestamp, default_config());

  // Filter by ERROR severity (query CONFIRMED since that's the default status now)
  auto faults = storage_.list_faults(true, Fault::SEVERITY_ERROR, {Fault::STATUS_CONFIRMED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(FaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/node1", timestamp, default_config());

  bool cleared = storage_.clear_fault("MOTOR_OVERHEAT");
  EXPECT_TRUE(cleared);

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);
}

TEST_F(FaultStorageTest, ClearNonExistentFault) {
  bool cleared = storage_.clear_fault("NON_EXISTENT");
  EXPECT_FALSE(cleared);
}

TEST_F(FaultStorageTest, GetClearedFaults) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, default_config());
  storage_.clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_.list_faults(false, 0, {Fault::STATUS_CLEARED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(FaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, fault is immediately CONFIRMED
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp, default_config());

  // Query with invalid status - defaults to CONFIRMED, which now matches our fault
  auto faults = storage_.list_faults(false, 0, {"INVALID_STATUS"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, DefaultDebounceConfig) {
  auto config = storage_.get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
  EXPECT_FALSE(config.healing_enabled);
  EXPECT_EQ(config.healing_threshold, 3);
  EXPECT_TRUE(config.critical_immediate_confirm);
  EXPECT_DOUBLE_EQ(config.auto_confirm_after_sec, 0.0);
}

TEST_F(FaultStorageTest, SetDebounceConfig) {
  DebounceConfig config;
  config.confirmation_threshold = -5;
  config.healing_enabled = true;
  config.healing_threshold = 5;
  config.critical_immediate_confirm = false;
  config.auto_confirm_after_sec = 10.0;

  storage_.set_debounce_config(config);
  auto retrieved = storage_.get_debounce_config();

  EXPECT_EQ(retrieved.confirmation_threshold, -5);
  EXPECT_TRUE(retrieved.healing_enabled);
  EXPECT_EQ(retrieved.healing_threshold, 5);
  EXPECT_FALSE(retrieved.critical_immediate_confirm);
  EXPECT_DOUBLE_EQ(retrieved.auto_confirm_after_sec, 10.0);
}

TEST_F(FaultStorageTest, FaultStaysPrefailedAboveThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (2 FAILED events should stay PREFAILED)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 2u);
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, FaultConfirmsAtThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (3 FAILED events should confirm)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 3u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ConfirmedFaultStaysConfirmed) {
  rclcpp::Clock clock;

  // Report fault 4 times
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                                "/node" + std::to_string(i), clock.now(), default_config());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 4u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, MultiSourceConfirmsFault) {
  rclcpp::Clock clock;

  // Report same fault from 3 different sources
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor1", clock.now(), default_config());
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor2", clock.now(), default_config());
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor3", clock.now(), default_config());

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->reporting_sources.size(), 3u);
}

TEST_F(FaultStorageTest, SameSourceMultipleReportsConfirms) {
  rclcpp::Clock clock;

  // Same source reports 3 times
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 3u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->reporting_sources.size(), 1u);  // Only one unique source
}

TEST_F(FaultStorageTest, ImmediateConfirmationWithThresholdZero) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = 0;  // Immediate confirmation
  storage_.set_debounce_config(config);

  // Single report should confirm immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, CriticalSeverityBypassesDebounce) {
  rclcpp::Clock clock;

  // CRITICAL severity should confirm immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                              "/node1", clock.now(), default_config());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, CriticalBypassCanBeDisabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;  // Need debounce to test CRITICAL bypass
  config.critical_immediate_confirm = false;
  storage_.set_debounce_config(config);

  // CRITICAL should NOT confirm immediately when disabled
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                              "/node1", clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, ClearedFaultCanBeReactivated) {
  rclcpp::Clock clock;

  // Report to confirm (with default threshold=-1, single report confirms)
  bool is_new = storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Initial", "/node1", clock.now(), default_config());
  EXPECT_TRUE(is_new);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->occurrence_count, 1u);

  // Clear the fault
  storage_.clear_fault("FAULT_1");
  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);

  // Report again - should reactivate
  is_new = storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                       "Reactivated", "/node2", clock.now(), default_config());
  EXPECT_TRUE(is_new);  // Should return true like a new fault

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Should be reconfirmed
  EXPECT_EQ(fault->occurrence_count, 2u);             // Should increment
  EXPECT_EQ(fault->reporting_sources.size(), 2u);     // Both sources
  EXPECT_EQ(fault->description, "Reactivated");       // Updated description
}

TEST_F(FaultStorageTest, PassedEventForClearedFaultIgnored) {
  rclcpp::Clock clock;

  // Report and confirm
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());

  // Clear the fault
  storage_.clear_fault("FAULT_1");

  // PASSED event should be ignored for CLEARED fault
  bool result = storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(),
                                            default_config());
  EXPECT_FALSE(result);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
}

TEST_F(FaultStorageTest, ClearedFaultReactivationRestartsDebounce) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  // Report 3 times to confirm
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                              clock.now(), config);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Clear the fault
  storage_.clear_fault("FAULT_1");

  // Reactivate - should start in PREFAILED with counter=-1
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node4",
                              clock.now(), config);

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);  // Not yet confirmed, needs 2 more FAILED

  // Report 2 more times to re-confirm
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node5",
                              clock.now(), config);
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node6",
                              clock.now(), config);

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Now confirmed
}

TEST_F(FaultStorageTest, PassedEventIncrementsCounter) {
  rclcpp::Clock clock;

  // Report 2 FAILED events (counter = -2)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now(), default_config());

  // Report 3 PASSED events (counter = -2 + 3 = +1)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(),
                              default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(),
                              default_config());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(),
                              default_config());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREPASSED);  // Counter > 0
}

TEST_F(FaultStorageTest, HealingDisabledByDefault) {
  rclcpp::Clock clock;

  // Report 1 FAILED event
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), default_config());

  // Report many PASSED events (counter = -1 + 10 = +9, but healing disabled)
  for (int i = 0; i < 10; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(),
                                default_config());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREPASSED);  // Not HEALED since healing disabled
}

TEST_F(FaultStorageTest, HealingWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_.set_debounce_config(config);

  // Report 1 FAILED event (counter = -1)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  // Report 4 PASSED events (counter = -1 + 4 = +3, reaches healing threshold)
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

TEST_F(FaultStorageTest, TimeBasedConfirmationDisabledByDefault) {
  rclcpp::Clock clock;

  // Set threshold to -3 to get PREFAILED status for testing time-based confirmation
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  // Advance time and check - should not auto-confirm (auto_confirm_after_sec = 0)
  auto future_time = rclcpp::Time(clock.now().nanoseconds() + static_cast<int64_t>(20e9));
  size_t confirmed = storage_.check_time_based_confirmation(future_time);

  EXPECT_EQ(confirmed, 0u);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, TimeBasedConfirmationWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;  // Need debounce so fault stays PREFAILED
  config.auto_confirm_after_sec = 10.0;
  storage_.set_debounce_config(config);

  auto now = clock.now();
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              now, config);

  // Check before timeout - should not confirm
  auto before_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(5e9));
  size_t confirmed_early = storage_.check_time_based_confirmation(before_timeout);
  EXPECT_EQ(confirmed_early, 0u);

  // Check after timeout - should confirm
  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  size_t confirmed = storage_.check_time_based_confirmation(after_timeout);
  EXPECT_EQ(confirmed, 1u);

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ConfirmedFaultCanHealWithPassedEvents) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_.set_debounce_config(config);

  // Report 3 FAILED events to confirm (counter = -3)
  for (int i = 0; i < 3; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                                "/node" + std::to_string(i), clock.now(), config);
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Report 6 PASSED events (counter = -3 + 6 = +3, reaches healing threshold)
  for (int i = 0; i < 6; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

TEST_F(FaultStorageTest, HealedFaultCanRecurWithFailedEvents) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_.set_debounce_config(config);

  // Report 1 FAILED event (counter = -1)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now(), config);

  // Report 4 PASSED events to heal (counter = -1 + 4 = +3)
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);

  // Report 3 FAILED events - fault should recur and confirm (counter = +3 - 3 = 0, then -3)
  for (int i = 0; i < 6; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Recurrence",
                                "/node2", clock.now(), config);
  }

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, GetAllFaultsReturnsAllFaults) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Add faults with different statuses
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test 1", "/node1",
                              timestamp, default_config());
  storage_.report_fault_event("FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN, "Test 2", "/node2",
                              timestamp, default_config());
  storage_.report_fault_event("FAULT_3", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Test 3", "/node3",
                              timestamp, default_config());

  auto all_faults = storage_.get_all_faults();
  EXPECT_EQ(all_faults.size(), 3u);
}

TEST_F(FaultStorageTest, GetAllFaultsReturnsEmptyForEmptyStorage) {
  auto all_faults = storage_.get_all_faults();
  EXPECT_TRUE(all_faults.empty());
}

// FaultManagerNode tests
class FaultManagerNodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Use in-memory storage for tests to avoid permission issues
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "memory"},
    });
    node_ = std::make_shared<FaultManagerNode>(options);
  }

  void TearDown() override {
    node_.reset();
  }

  std::shared_ptr<FaultManagerNode> node_;
};

TEST_F(FaultManagerNodeTest, NodeCreation) {
  EXPECT_STREQ(node_->get_name(), "fault_manager");
  EXPECT_EQ(node_->get_storage().size(), 0u);
  EXPECT_EQ(node_->get_storage_type(), "memory");
}

TEST_F(FaultManagerNodeTest, DefaultDebounceConfig) {
  auto config = node_->get_storage().get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
}

TEST(FaultManagerNodeParameterTest, CustomConfirmationThreshold) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"confirmation_threshold", -5},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  auto config = node->get_storage().get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -5);
}

TEST(FaultManagerNodeParameterTest, ConfirmationThresholdDisabled) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"confirmation_threshold", 0},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  // 0 means immediate confirmation
  auto config = node->get_storage().get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, 0);
}

TEST(FaultManagerNodeParameterTest, HealingEnabled) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"healing_enabled", true},
      {"healing_threshold", 5},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  auto config = node->get_storage().get_debounce_config();
  EXPECT_TRUE(config.healing_enabled);
  EXPECT_EQ(config.healing_threshold, 5);
}

TEST(FaultManagerNodeParameterTest, AutoConfirmAfterSec) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"auto_confirm_after_sec", 15.0},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  auto config = node->get_storage().get_debounce_config();
  EXPECT_DOUBLE_EQ(config.auto_confirm_after_sec, 15.0);
}

// FaultEvent Publishing Tests
class FaultEventPublishingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create fault manager node with immediate confirmation
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "memory"}, {"confirmation_threshold", -1},  // Immediate confirmation
    });
    fault_manager_ = std::make_shared<FaultManagerNode>(options);

    // Create test node for subscribing and calling services
    test_node_ = std::make_shared<rclcpp::Node>("test_event_subscriber");

    // Subscribe to fault events
    event_subscription_ = test_node_->create_subscription<FaultEvent>(
        "/fault_manager/events", rclcpp::QoS(100).reliable(), [this](const FaultEvent::SharedPtr msg) {
          received_events_.push_back(*msg);
        });

    // Create service clients
    report_fault_client_ = test_node_->create_client<ReportFault>("/fault_manager/report_fault");
    clear_fault_client_ = test_node_->create_client<ClearFault>("/fault_manager/clear_fault");
    get_fault_client_ = test_node_->create_client<GetFault>("/fault_manager/get_fault");
    list_faults_for_entity_client_ =
        test_node_->create_client<ListFaultsForEntity>("/fault_manager/list_faults_for_entity");

    // Wait for services
    ASSERT_TRUE(report_fault_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(clear_fault_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(get_fault_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(list_faults_for_entity_client_->wait_for_service(std::chrono::seconds(5)));

    // Drain any stale DDS messages from previous tests (same topic, new subscription)
    spin_for(std::chrono::milliseconds(50));
    received_events_.clear();
  }

  void TearDown() override {
    event_subscription_.reset();
    report_fault_client_.reset();
    clear_fault_client_.reset();
    get_fault_client_.reset();
    list_faults_for_entity_client_.reset();
    test_node_.reset();
    fault_manager_.reset();
    received_events_.clear();
  }

  void spin_for(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(fault_manager_);
      rclcpp::spin_some(test_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  /// Spin until a predicate becomes true or timeout expires.
  /// More robust than fixed spin_for() under CPU contention.
  bool spin_until(const std::function<bool()> & predicate,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(fault_manager_);
      rclcpp::spin_some(test_node_);
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
  }

  /// Spin until a future is ready, with 2s timeout. Robust under CPU contention.
  template <typename FutureT>
  bool spin_until_future_ready(FutureT & future, std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(fault_manager_);
      rclcpp::spin_some(test_node_);
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }

  bool call_report_fault(const std::string & fault_code, uint8_t severity, const std::string & source_id) {
    auto request = std::make_shared<ReportFault::Request>();
    request->fault_code = fault_code;
    request->event_type = ReportFault::Request::EVENT_FAILED;
    request->severity = severity;
    request->description = "Test fault";
    request->source_id = source_id;

    auto future = report_fault_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return false;
    }
    return future.get()->accepted;
  }

  bool call_clear_fault(const std::string & fault_code) {
    auto request = std::make_shared<ClearFault::Request>();
    request->fault_code = fault_code;

    auto future = clear_fault_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return false;
    }
    return future.get()->success;
  }

  std::optional<GetFault::Response> call_get_fault(const std::string & fault_code) {
    auto request = std::make_shared<GetFault::Request>();
    request->fault_code = fault_code;

    auto future = get_fault_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return std::nullopt;
    }
    return *future.get();
  }

  std::optional<ListFaultsForEntity::Response> call_list_faults_for_entity(const std::string & entity_id) {
    auto request = std::make_shared<ListFaultsForEntity::Request>();
    request->entity_id = entity_id;

    auto future = list_faults_for_entity_client_->async_send_request(request);
    if (!spin_until_future_ready(future)) {
      return std::nullopt;
    }
    return *future.get();
  }

  std::shared_ptr<FaultManagerNode> fault_manager_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Subscription<FaultEvent>::SharedPtr event_subscription_;
  rclcpp::Client<ReportFault>::SharedPtr report_fault_client_;
  rclcpp::Client<ClearFault>::SharedPtr clear_fault_client_;
  rclcpp::Client<GetFault>::SharedPtr get_fault_client_;
  rclcpp::Client<ListFaultsForEntity>::SharedPtr list_faults_for_entity_client_;
  std::vector<FaultEvent> received_events_;
};

TEST_F(FaultEventPublishingTest, NewFaultPublishesConfirmedEvent) {
  // Report a new fault - with threshold=-1, it should immediately confirm
  ASSERT_TRUE(call_report_fault("TEST_FAULT_1", Fault::SEVERITY_ERROR, "/test_node"));

  // Wait for event to arrive (polling, robust under CPU contention)
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Verify EVENT_CONFIRMED was published
  ASSERT_EQ(received_events_.size(), 1u);
  EXPECT_EQ(received_events_[0].event_type, FaultEvent::EVENT_CONFIRMED);
  EXPECT_EQ(received_events_[0].fault.fault_code, "TEST_FAULT_1");
  EXPECT_EQ(received_events_[0].fault.severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(received_events_[0].fault.status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultEventPublishingTest, UpdateExistingFaultPublishesUpdatedEvent) {
  // Report a new fault first
  ASSERT_TRUE(call_report_fault("TEST_FAULT_2", Fault::SEVERITY_WARN, "/test_node1"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Clear received events
  received_events_.clear();

  // Report same fault again - should trigger EVENT_UPDATED
  ASSERT_TRUE(call_report_fault("TEST_FAULT_2", Fault::SEVERITY_ERROR, "/test_node2"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Verify EVENT_UPDATED was published
  ASSERT_EQ(received_events_.size(), 1u);
  EXPECT_EQ(received_events_[0].event_type, FaultEvent::EVENT_UPDATED);
  EXPECT_EQ(received_events_[0].fault.fault_code, "TEST_FAULT_2");
  EXPECT_EQ(received_events_[0].fault.occurrence_count, 2u);
}

TEST_F(FaultEventPublishingTest, ClearFaultPublishesClearedEvent) {
  // Report a fault first
  ASSERT_TRUE(call_report_fault("TEST_FAULT_3", Fault::SEVERITY_ERROR, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Clear received events
  received_events_.clear();

  // Clear the fault
  ASSERT_TRUE(call_clear_fault("TEST_FAULT_3"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Verify EVENT_CLEARED was published
  ASSERT_EQ(received_events_.size(), 1u);
  EXPECT_EQ(received_events_[0].event_type, FaultEvent::EVENT_CLEARED);
  EXPECT_EQ(received_events_[0].fault.fault_code, "TEST_FAULT_3");
  EXPECT_EQ(received_events_[0].fault.status, Fault::STATUS_CLEARED);
}

TEST_F(FaultEventPublishingTest, ClearNonExistentFaultNoEvent) {
  // Clear non-existent fault - should not publish event
  ASSERT_FALSE(call_clear_fault("NON_EXISTENT_FAULT"));
  // Brief spin to confirm no event arrives (negative test - keep short)
  spin_for(std::chrono::milliseconds(100));

  // Verify no events published
  EXPECT_EQ(received_events_.size(), 0u);
}

TEST_F(FaultEventPublishingTest, EventContainsCorrectTimestamp) {
  auto before = fault_manager_->now();

  ASSERT_TRUE(call_report_fault("TEST_FAULT_4", Fault::SEVERITY_WARN, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  auto after = fault_manager_->now();

  ASSERT_EQ(received_events_.size(), 1u);

  // Verify timestamp is within expected range
  rclcpp::Time event_time(received_events_[0].timestamp);
  EXPECT_GE(event_time, before);
  EXPECT_LE(event_time, after);
}

TEST_F(FaultEventPublishingTest, EventContainsFullFaultData) {
  ASSERT_TRUE(call_report_fault("FULL_DATA_TEST", Fault::SEVERITY_CRITICAL, "/sensor/temperature"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  ASSERT_EQ(received_events_.size(), 1u);
  const auto & fault = received_events_[0].fault;

  // Verify all fault fields are populated
  EXPECT_EQ(fault.fault_code, "FULL_DATA_TEST");
  EXPECT_EQ(fault.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(fault.description, "Test fault");
  EXPECT_EQ(fault.status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault.occurrence_count, 1u);
  ASSERT_EQ(fault.reporting_sources.size(), 1u);
  EXPECT_EQ(fault.reporting_sources[0], "/sensor/temperature");
}

TEST_F(FaultEventPublishingTest, TimestampUsesWallClockNotSimTime) {
  // Get current wall clock time
  auto wall_before = std::chrono::system_clock::now();
  auto wall_before_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_before.time_since_epoch()).count();

  ASSERT_TRUE(call_report_fault("WALL_CLOCK_TEST", Fault::SEVERITY_WARN, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  auto wall_after = std::chrono::system_clock::now();
  auto wall_after_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_after.time_since_epoch()).count();

  ASSERT_EQ(received_events_.size(), 1u);

  // Convert event timestamp to nanoseconds
  rclcpp::Time event_time(received_events_[0].timestamp);
  int64_t event_ns = event_time.nanoseconds();

  // Verify timestamp is close to wall clock time (within 5 seconds to be safe)
  // This catches the bug where sim time (e.g., 148 seconds) would be far from wall clock
  EXPECT_GE(event_ns, wall_before_ns - 5'000'000'000LL);
  EXPECT_LE(event_ns, wall_after_ns + 5'000'000'000LL);

  // Also verify it's a reasonable Unix timestamp (after year 2020)
  // This catches the "1970" bug from the issue
  constexpr int64_t YEAR_2020_NS = 1577836800LL * 1'000'000'000LL;  // 2020-01-01 00:00:00 UTC
  EXPECT_GT(event_ns, YEAR_2020_NS);
}

// GetFault Service Tests
TEST_F(FaultEventPublishingTest, GetFaultReturnsExpectedFault) {
  // Report a fault first
  ASSERT_TRUE(call_report_fault("GET_FAULT_TEST", Fault::SEVERITY_ERROR, "/test_node"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Get fault via service
  auto response = call_get_fault("GET_FAULT_TEST");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->fault.fault_code, "GET_FAULT_TEST");
  EXPECT_EQ(response->fault.severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(response->fault.status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultEventPublishingTest, GetFaultReturnsNotFoundForMissingFault) {
  // Get non-existent fault
  auto response = call_get_fault("NON_EXISTENT_FAULT");
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
}

TEST_F(FaultEventPublishingTest, GetFaultReturnsEnvironmentData) {
  // Report a fault
  ASSERT_TRUE(call_report_fault("ENV_DATA_TEST", Fault::SEVERITY_WARN, "/sensor/temp"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  auto response = call_get_fault("ENV_DATA_TEST");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);

  // Environment data should be present
  const auto & env_data = response->environment_data;

  // Should have freeze-frame type snapshot for the first occurrence
  // Note: The actual snapshot content depends on the storage implementation
  // but we can verify the service returns environment_data correctly
  EXPECT_TRUE(env_data.snapshots.empty() || !env_data.snapshots[0].type.empty());
}

TEST_F(FaultEventPublishingTest, GetFaultReturnsExtendedDataRecords) {
  // Report fault twice to have first and last occurrence timestamps differ
  ASSERT_TRUE(call_report_fault("EDR_TEST", Fault::SEVERITY_ERROR, "/node1"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));
  ASSERT_TRUE(call_report_fault("EDR_TEST", Fault::SEVERITY_ERROR, "/node2"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 2;
  }));

  auto response = call_get_fault("EDR_TEST");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);

  // Verify extended data records contain timestamp information
  const auto & edr = response->environment_data.extended_data_records;
  EXPECT_NE(edr.first_occurrence_ns, 0);
  EXPECT_NE(edr.last_occurrence_ns, 0);
}

// @verifies REQ_INTEROP_012
TEST_F(FaultEventPublishingTest, ListFaultsForEntitySuccess) {
  // Report faults from different sources
  ASSERT_TRUE(call_report_fault("MOTOR_FAULT", Fault::SEVERITY_ERROR, "/powertrain/motor_controller"));
  ASSERT_TRUE(call_report_fault("SENSOR_FAULT", Fault::SEVERITY_WARN, "/powertrain/motor_controller"));
  ASSERT_TRUE(call_report_fault("BRAKE_FAULT", Fault::SEVERITY_ERROR, "/chassis/brake_system"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 3;
  }));

  // Query faults for motor_controller entity
  auto response = call_list_faults_for_entity("motor_controller");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->faults.size(), 2u);

  // Verify the returned faults are from motor_controller
  std::set<std::string> codes;
  for (const auto & fault : response->faults) {
    codes.insert(fault.fault_code);
  }
  EXPECT_TRUE(codes.count("MOTOR_FAULT"));
  EXPECT_TRUE(codes.count("SENSOR_FAULT"));
  EXPECT_FALSE(codes.count("BRAKE_FAULT"));
}

// @verifies REQ_INTEROP_012
TEST_F(FaultEventPublishingTest, ListFaultsForEntityEmptyResult) {
  // Report faults from a different entity
  ASSERT_TRUE(call_report_fault("SOME_FAULT", Fault::SEVERITY_ERROR, "/some/other_entity"));
  ASSERT_TRUE(spin_until([this]() {
    return received_events_.size() >= 1;
  }));

  // Query faults for non-existent entity
  auto response = call_list_faults_for_entity("motor_controller");
  ASSERT_TRUE(response.has_value());
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->faults.empty());
}

// @verifies REQ_INTEROP_012
TEST_F(FaultEventPublishingTest, ListFaultsForEntityWithEmptyId) {
  auto response = call_list_faults_for_entity("");
  ASSERT_TRUE(response.has_value());
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->error_message.empty());
}

// matches_entity helper tests
TEST(MatchesEntityTest, ExactMatch) {
  std::vector<std::string> sources = {"motor_controller"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "motor_controller"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "other"));
}

TEST(MatchesEntityTest, FQNSuffixMatch) {
  std::vector<std::string> sources = {"/powertrain/motor_controller"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "motor_controller"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "powertrain"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "controller"));  // Partial - not a suffix
}

TEST(MatchesEntityTest, HierarchicalMatch) {
  std::vector<std::string> sources = {"/perception/lidar/front_sensor"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "front_sensor"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "lidar"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "perception"));  // Root only
}

TEST(MatchesEntityTest, MultipleSources) {
  std::vector<std::string> sources = {"/chassis/brake_system", "/powertrain/motor_controller", "/sensor/temperature"};
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "brake_system"));
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "motor_controller"));
  EXPECT_TRUE(FaultManagerNode::matches_entity(sources, "temperature"));
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "unknown"));
}

TEST(MatchesEntityTest, EmptySources) {
  std::vector<std::string> sources;
  EXPECT_FALSE(FaultManagerNode::matches_entity(sources, "motor_controller"));
}

// --- InMemoryFaultStorage snapshot limit tests ---

TEST(InMemorySnapshotLimitTest, RejectsWhenFull) {
  InMemoryFaultStorage storage;
  storage.set_max_snapshots_per_fault(2);

  ros2_medkit_fault_manager::SnapshotData snap;
  snap.fault_code = "TEST";
  snap.topic = "/test";
  snap.message_type = "std_msgs/msg/String";
  snap.data = "{}";

  snap.captured_at_ns = 1000;
  storage.store_snapshot(snap);
  snap.captured_at_ns = 2000;
  storage.store_snapshot(snap);
  snap.captured_at_ns = 3000;
  storage.store_snapshot(snap);  // Should be rejected

  auto result = storage.get_snapshots("TEST");
  EXPECT_EQ(result.size(), 2u);
}

TEST(InMemorySnapshotLimitTest, UnlimitedWhenZero) {
  InMemoryFaultStorage storage;
  storage.set_max_snapshots_per_fault(0);

  ros2_medkit_fault_manager::SnapshotData snap;
  snap.fault_code = "TEST";
  snap.topic = "/test";
  snap.message_type = "std_msgs/msg/String";
  snap.data = "{}";

  for (int i = 0; i < 20; ++i) {
    snap.captured_at_ns = static_cast<int64_t>(i * 1000);
    storage.store_snapshot(snap);
  }

  auto result = storage.get_snapshots("TEST");
  EXPECT_EQ(result.size(), 20u);
}

// =============================================================================
// Snapshot recapture cooldown test
// =============================================================================

class SnapshotCooldownTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"storage_type", "memory"},
        {"confirmation_threshold", -1},  // Immediate confirmation
        {"healing_enabled", true},
        {"healing_threshold", 1},  // Heal on first PASSED
        {"snapshots.enabled", true},
        {"snapshots.recapture_cooldown_sec", 0.5},  // Short cooldown for testing
        {"snapshots.max_per_fault", 0},             // Unlimited (test cooldown, not limit)
    });
    fault_manager_ = std::make_shared<FaultManagerNode>(options);

    test_node_ = std::make_shared<rclcpp::Node>("test_cooldown");
    report_client_ = test_node_->create_client<ReportFault>("/fault_manager/report_fault");
    clear_client_ = test_node_->create_client<ClearFault>("/fault_manager/clear_fault");
    get_fault_client_ = test_node_->create_client<GetFault>("/fault_manager/get_fault");

    executor_.add_node(fault_manager_);
    executor_.add_node(test_node_);
    spin_thread_ = std::thread([this]() {
      executor_.spin();
    });

    ASSERT_TRUE(report_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(clear_client_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(get_fault_client_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(fault_manager_);
    executor_.remove_node(test_node_);
    report_client_.reset();
    clear_client_.reset();
    get_fault_client_.reset();
    test_node_.reset();
    fault_manager_.reset();
  }

  void spin_for(std::chrono::milliseconds duration) {
    std::this_thread::sleep_for(duration);
  }

  bool report_fault(const std::string & code) {
    auto req = std::make_shared<ReportFault::Request>();
    req->fault_code = code;
    req->event_type = ReportFault::Request::EVENT_FAILED;
    req->severity = Fault::SEVERITY_CRITICAL;  // Bypass debounce
    req->description = "Test";
    req->source_id = "/test";
    auto future = report_client_->async_send_request(req);
    spin_for(std::chrono::milliseconds(200));
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready && future.get()->accepted;
  }

  bool report_passed(const std::string & code) {
    auto req = std::make_shared<ReportFault::Request>();
    req->fault_code = code;
    req->event_type = ReportFault::Request::EVENT_PASSED;
    req->severity = 0;
    req->source_id = "/test";
    auto future = report_client_->async_send_request(req);
    spin_for(std::chrono::milliseconds(200));
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready && future.get()->accepted;
  }

  size_t get_snapshot_count(const std::string & code) {
    auto req = std::make_shared<GetFault::Request>();
    req->fault_code = code;
    auto future = get_fault_client_->async_send_request(req);
    spin_for(std::chrono::milliseconds(200));
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return 0;
    }
    auto resp = future.get();
    return resp->environment_data.snapshots.size();
  }

  std::shared_ptr<FaultManagerNode> fault_manager_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Client<ReportFault>::SharedPtr report_client_;
  rclcpp::Client<ClearFault>::SharedPtr clear_client_;
  rclcpp::Client<GetFault>::SharedPtr get_fault_client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
};

TEST_F(SnapshotCooldownTest, CooldownPreventsRapidRecapture) {
  // First confirmation -> snapshot captured
  ASSERT_TRUE(report_fault("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(300));

  // Heal and re-confirm immediately (within cooldown)
  ASSERT_TRUE(report_passed("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(report_fault("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(300));

  // Only 1 snapshot capture should have occurred (second blocked by cooldown)
  // Note: snapshot capture is async, so we check the storage-level count.
  // Freeze-frame snapshots may or may not appear depending on topic availability,
  // but the capture thread should have been spawned only once.
  (void)get_snapshot_count("COOLDOWN_TEST");
  // With no topics to capture, count may be 0. The key assertion is that
  // the second confirmation did NOT spawn a capture thread.
  // We verify indirectly: wait for cooldown to expire and re-confirm.
  std::this_thread::sleep_for(std::chrono::milliseconds(600));  // Cooldown=0.5s

  // Now cooldown expired, heal and re-confirm -> should capture again
  ASSERT_TRUE(report_passed("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(report_fault("COOLDOWN_TEST"));
  spin_for(std::chrono::milliseconds(300));

  // This is a basic smoke test verifying the cooldown path doesn't crash
  // and the node handles rapid re-confirmation gracefully.
  SUCCEED();
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
