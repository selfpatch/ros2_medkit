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

using ros2_medkit_fault_manager::FaultManagerNode;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_msgs::msg::Fault;

class FaultStorageTest : public ::testing::Test {
 protected:
  InMemoryFaultStorage storage_;
};

TEST_F(FaultStorageTest, ReportNewFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_.report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Motor temperature exceeded threshold",
                                      "/powertrain/motor", timestamp);

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_.size(), 1u);
  EXPECT_TRUE(storage_.contains("MOTOR_OVERHEAT"));
}

TEST_F(FaultStorageTest, ReportExistingFaultUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_.report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_WARN, "Initial report", "/powertrain/motor1", timestamp1);

  bool is_new =
      storage_.report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Second report", "/powertrain/motor2", timestamp2);

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_.size(), 1u);

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 2u);
  EXPECT_EQ(fault->severity, Fault::SEVERITY_ERROR);  // Updated to higher severity
  EXPECT_EQ(fault->reporting_sources.size(), 2u);
}

TEST_F(FaultStorageTest, GetFaultsDefaultReturnsPendingAndConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Report a fault (starts as PENDING)
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

  // Default query should return PENDING faults (PENDING + CONFIRMED by default)
  auto faults = storage_.get_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_PENDING);
}

TEST_F(FaultStorageTest, GetFaultsWithPendingStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

  // Query with PENDING status
  auto faults = storage_.get_faults(false, 0, {"PENDING"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PENDING);
}

TEST_F(FaultStorageTest, GetFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault("FAULT_INFO", Fault::SEVERITY_INFO, "Info", "/node1", timestamp);
  storage_.report_fault("FAULT_ERROR", Fault::SEVERITY_ERROR, "Error", "/node1", timestamp);

  // Filter by ERROR severity
  auto faults = storage_.get_faults(true, Fault::SEVERITY_ERROR, {"PENDING"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(FaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

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

  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);
  storage_.clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_.get_faults(false, 0, {"CLEARED"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(FaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", timestamp);

  // Query with invalid status - defaults to CONFIRMED (fault is PENDING, so no matches)
  auto faults = storage_.get_faults(false, 0, {"INVALID_STATUS"});
  EXPECT_EQ(faults.size(), 0u);
}

TEST_F(FaultStorageTest, ConfirmationThresholdDefaultsToZero) {
  // Default threshold should be 0 (disabled)
  EXPECT_EQ(storage_.get_confirmation_threshold(), 0u);
}

TEST_F(FaultStorageTest, SetConfirmationThreshold) {
  storage_.set_confirmation_threshold(5);
  EXPECT_EQ(storage_.get_confirmation_threshold(), 5u);

  storage_.set_confirmation_threshold(0);
  EXPECT_EQ(storage_.get_confirmation_threshold(), 0u);
}

TEST_F(FaultStorageTest, FaultStaysPendingBelowThreshold) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(3);

  // Report fault twice (below threshold of 3)
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", clock.now());
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node2", clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 2u);
  EXPECT_EQ(fault->status, Fault::STATUS_PENDING);
}

TEST_F(FaultStorageTest, FaultConfirmsAtThreshold) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(3);

  // Report fault 3 times (reaches threshold)
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", clock.now());
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node2", clock.now());
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node3", clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 3u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, ConfirmedFaultStaysConfirmed) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(3);

  // Report fault 4 times
  for (int i = 0; i < 4; ++i) {
    storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node" + std::to_string(i), clock.now());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 4u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(FaultStorageTest, MultiSourceConfirmsFault) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(3);

  // Report same fault from 3 different sources
  storage_.report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Test", "/sensor1", clock.now());
  storage_.report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Test", "/sensor2", clock.now());
  storage_.report_fault("MOTOR_OVERHEAT", Fault::SEVERITY_ERROR, "Test", "/sensor3", clock.now());

  auto fault = storage_.get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->reporting_sources.size(), 3u);
}

TEST_F(FaultStorageTest, SameSourceMultipleReportsConfirms) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(3);

  // Same source reports 3 times
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", clock.now());
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", clock.now());
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 3u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->reporting_sources.size(), 1u);  // Only one unique source
}

TEST_F(FaultStorageTest, ThresholdDisabledKeepsPending) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(0);  // Disabled

  // Report many times - should stay PENDING
  for (int i = 0; i < 10; ++i) {
    storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node" + std::to_string(i), clock.now());
  }

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 10u);
  EXPECT_EQ(fault->status, Fault::STATUS_PENDING);
}

TEST_F(FaultStorageTest, ClearedFaultNotReconfirmed) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(3);

  // Report 3 times to confirm
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", clock.now());
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node2", clock.now());
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node3", clock.now());

  // Clear the fault
  storage_.clear_fault("FAULT_1");

  // Report again
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node4", clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
}

TEST_F(FaultStorageTest, ThresholdOneConfirmsImmediately) {
  rclcpp::Clock clock;
  storage_.set_confirmation_threshold(1);

  // Single report should confirm immediately
  storage_.report_fault("FAULT_1", Fault::SEVERITY_ERROR, "Test", "/node1", clock.now());

  auto fault = storage_.get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
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

TEST_F(FaultManagerNodeTest, DefaultConfirmationThreshold) {
  // Default confirmation threshold should be 3
  EXPECT_EQ(node_->get_storage().get_confirmation_threshold(), 3u);
}

TEST(FaultManagerNodeParameterTest, CustomConfirmationThreshold) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"confirmation_threshold", 5},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  EXPECT_EQ(node->get_storage().get_confirmation_threshold(), 5u);
}

TEST(FaultManagerNodeParameterTest, ConfirmationThresholdDisabled) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"storage_type", "memory"},
      {"confirmation_threshold", 0},
  });
  auto node = std::make_shared<FaultManagerNode>(options);

  // 0 means disabled (no auto-confirmation)
  EXPECT_EQ(node->get_storage().get_confirmation_threshold(), 0u);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
