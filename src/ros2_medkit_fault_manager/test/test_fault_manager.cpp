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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_manager_node.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::FaultManagerNode;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::srv::ReportFault;

class FaultStorageTest : public ::testing::Test {
 protected:
  InMemoryFaultStorage storage_;
};

TEST_F(FaultStorageTest, ReportNewFaultEvent) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Motor temperature exceeded threshold", "/powertrain/motor", timestamp);

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_.size(), 1u);
  EXPECT_TRUE(storage_.contains("MOTOR_OVERHEAT"));
}

TEST_F(FaultStorageTest, PassedEventForNonExistentFaultIgnored) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_.report_fault_event("NON_EXISTENT", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR,
                                            "Test", "/node1", timestamp);

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_.size(), 0u);
}

TEST_F(FaultStorageTest, ReportExistingFaultEventUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                              "Initial report", "/powertrain/motor1", timestamp1);

  bool is_new = storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                            "Second report", "/powertrain/motor2", timestamp2);

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_.size(), 1u);

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 2u);
  EXPECT_EQ(fault->severity, Fault::SEVERITY_ERROR);  // Updated to higher severity
  EXPECT_EQ(fault->reporting_sources.size(), 2u);
}

TEST_F(FaultStorageTest, GetFaultsDefaultReturnsConfirmedOnly) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, single report confirms immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp);

  // Default query should return the CONFIRMED fault
  auto faults = storage_.get_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, GetFaultsWithPrefailedStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Set threshold to -3 to test PREFAILED status
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_.set_debounce_config(config);

  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp);

  // Query with PREFAILED status
  auto faults = storage_.get_faults(false, 0, {Fault::STATUS_PREFAILED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, GetFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  storage_.report_fault_event("FAULT_INFO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Info", "/node1",
                              timestamp);
  storage_.report_fault_event("FAULT_ERROR", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Error",
                              "/node1", timestamp);

  // Filter by ERROR severity (query CONFIRMED since that's the default status now)
  auto faults = storage_.get_faults(true, Fault::SEVERITY_ERROR, {Fault::STATUS_CONFIRMED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(FaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/node1", timestamp);

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
                              timestamp);
  storage_.clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_.get_faults(false, 0, {Fault::STATUS_CLEARED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(FaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, fault is immediately CONFIRMED
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              timestamp);

  // Query with invalid status - defaults to CONFIRMED, which now matches our fault
  auto faults = storage_.get_faults(false, 0, {"INVALID_STATUS"});
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
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now());

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
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                              clock.now());

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
                                "/node" + std::to_string(i), clock.now());
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
                              "/sensor1", clock.now());
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor2", clock.now());
  storage_.report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                              "/sensor3", clock.now());

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->reporting_sources.size(), 3u);
}

TEST_F(FaultStorageTest, SameSourceMultipleReportsConfirms) {
  rclcpp::Clock clock;

  // Same source reports 3 times
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now());

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
                              clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, CriticalSeverityBypassesDebounce) {
  rclcpp::Clock clock;

  // CRITICAL severity should confirm immediately
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                              "/node1", clock.now());

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
                              "/node1", clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(FaultStorageTest, ClearedFaultNotReconfirmed) {
  rclcpp::Clock clock;

  // Report 3 times to confirm
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                              clock.now());

  // Clear the fault
  storage_.clear_fault("FAULT_1");

  // Report again
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node4",
                              clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
}

TEST_F(FaultStorageTest, PassedEventIncrementsCounter) {
  rclcpp::Clock clock;

  // Report 2 FAILED events (counter = -2)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                              clock.now());

  // Report 3 PASSED events (counter = -2 + 3 = +1)
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREPASSED);  // Counter > 0
}

TEST_F(FaultStorageTest, HealingDisabledByDefault) {
  rclcpp::Clock clock;

  // Report 1 FAILED event
  storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                              clock.now());

  // Report many PASSED events (counter = -1 + 10 = +9, but healing disabled)
  for (int i = 0; i < 10; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
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
                              clock.now());

  // Report 4 PASSED events (counter = -1 + 4 = +3, reaches healing threshold)
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
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
                              clock.now());

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
                              now);

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
                                "/node" + std::to_string(i), clock.now());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Report 6 PASSED events (counter = -3 + 6 = +3, reaches healing threshold)
  for (int i = 0; i < 6; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
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
                              clock.now());

  // Report 4 PASSED events to heal (counter = -1 + 4 = +3)
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);

  // Report 3 FAILED events - fault should recur and confirm (counter = +3 - 3 = 0, then -3)
  for (int i = 0; i < 6; ++i) {
    storage_.report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Recurrence",
                                "/node2", clock.now());
  }

  fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
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

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
