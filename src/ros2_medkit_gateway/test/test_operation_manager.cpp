// Copyright 2025 bburda
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
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/discovery_manager.hpp"
#include "ros2_medkit_gateway/operation_manager.hpp"

using namespace ros2_medkit_gateway;

class TestOperationManager : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Use short timeout for tests to avoid long waits on nonexistent services
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("service_call_timeout_sec", static_cast<int64_t>(1))});
    node_ = std::make_shared<rclcpp::Node>("test_operation_manager_node", options);
    discovery_manager_ = std::make_unique<DiscoveryManager>(node_.get());
    operation_manager_ = std::make_unique<OperationManager>(node_.get(), discovery_manager_.get());
  }

  void TearDown() override {
    operation_manager_.reset();
    discovery_manager_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<DiscoveryManager> discovery_manager_;
  std::unique_ptr<OperationManager> operation_manager_;
};

// ==================== TYPE VALIDATION TESTS ====================

TEST_F(TestOperationManager, test_is_valid_message_type_service) {
  // Valid service types
  EXPECT_TRUE(OperationManager::is_valid_message_type("std_srvs/srv/Trigger"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("example_interfaces/srv/AddTwoInts"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("my_package/srv/MyService"));
}

TEST_F(TestOperationManager, test_is_valid_message_type_action) {
  // Valid action types
  EXPECT_TRUE(OperationManager::is_valid_message_type("example_interfaces/action/Fibonacci"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("nav2_msgs/action/NavigateToPose"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("my_package/action/MyAction"));
}

TEST_F(TestOperationManager, test_is_valid_message_type_message) {
  // Valid message types
  EXPECT_TRUE(OperationManager::is_valid_message_type("std_msgs/msg/String"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("sensor_msgs/msg/Image"));
  EXPECT_TRUE(OperationManager::is_valid_message_type("geometry_msgs/msg/Twist"));
}

TEST_F(TestOperationManager, test_is_valid_message_type_invalid) {
  // Invalid types
  EXPECT_FALSE(OperationManager::is_valid_message_type(""));
  EXPECT_FALSE(OperationManager::is_valid_message_type("invalid"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("package/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("package/invalid/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("/srv/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("123package/srv/Type"));
  EXPECT_FALSE(OperationManager::is_valid_message_type("package/srv/"));
}

TEST_F(TestOperationManager, test_is_service_type) {
  EXPECT_TRUE(OperationManager::is_service_type("std_srvs/srv/Trigger"));
  EXPECT_TRUE(OperationManager::is_service_type("example_interfaces/srv/AddTwoInts"));

  EXPECT_FALSE(OperationManager::is_service_type("example_interfaces/action/Fibonacci"));
  EXPECT_FALSE(OperationManager::is_service_type("std_msgs/msg/String"));
  EXPECT_FALSE(OperationManager::is_service_type(""));
}

TEST_F(TestOperationManager, test_is_action_type) {
  EXPECT_TRUE(OperationManager::is_action_type("example_interfaces/action/Fibonacci"));
  EXPECT_TRUE(OperationManager::is_action_type("nav2_msgs/action/NavigateToPose"));

  EXPECT_FALSE(OperationManager::is_action_type("std_srvs/srv/Trigger"));
  EXPECT_FALSE(OperationManager::is_action_type("std_msgs/msg/String"));
  EXPECT_FALSE(OperationManager::is_action_type(""));
}

// ==================== UUID VALIDATION TESTS ====================

TEST_F(TestOperationManager, test_is_valid_uuid_hex_valid) {
  // Valid 32-character hex strings
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("00000000000000000000000000000000"));
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("ffffffffffffffffffffffffffffffff"));
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("0123456789abcdef0123456789abcdef"));
  EXPECT_TRUE(OperationManager::is_valid_uuid_hex("ABCDEF0123456789ABCDEF0123456789"));
}

TEST_F(TestOperationManager, test_is_valid_uuid_hex_invalid) {
  // Invalid UUIDs
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex(""));
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0"));
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0000000000000000000000000000000"));     // 31 chars
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("000000000000000000000000000000000"));   // 33 chars
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0000000000000000000000000000000g"));    // invalid char
  EXPECT_FALSE(OperationManager::is_valid_uuid_hex("0000-0000-0000-0000-0000-0000-0000"));  // dashes
}

// ==================== ACTION STATUS CONVERSION TESTS ====================

TEST_F(TestOperationManager, test_action_status_to_string) {
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::UNKNOWN), "unknown");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::ACCEPTED), "accepted");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::EXECUTING), "executing");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::CANCELING), "canceling");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::SUCCEEDED), "succeeded");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::CANCELED), "canceled");
  EXPECT_EQ(action_status_to_string(ActionGoalStatus::ABORTED), "aborted");
}

// ==================== GOAL TRACKING TESTS ====================

TEST_F(TestOperationManager, test_list_tracked_goals_empty) {
  auto goals = operation_manager_->list_tracked_goals();
  EXPECT_TRUE(goals.empty());
}

TEST_F(TestOperationManager, test_get_tracked_goal_not_found) {
  auto goal = operation_manager_->get_tracked_goal("nonexistent_goal_id_12345678");
  EXPECT_FALSE(goal.has_value());
}

TEST_F(TestOperationManager, test_get_goals_for_action_empty) {
  auto goals = operation_manager_->get_goals_for_action("/test/action");
  EXPECT_TRUE(goals.empty());
}

TEST_F(TestOperationManager, test_get_latest_goal_for_action_empty) {
  auto goal = operation_manager_->get_latest_goal_for_action("/test/action");
  EXPECT_FALSE(goal.has_value());
}

TEST_F(TestOperationManager, test_cleanup_old_goals_empty) {
  // Should not throw on empty goals
  EXPECT_NO_THROW(operation_manager_->cleanup_old_goals(std::chrono::seconds(0)));
}

// ==================== SERVICE CALL TESTS ====================

TEST_F(TestOperationManager, test_call_service_unavailable) {
  // Call a service that doesn't exist
  auto result = operation_manager_->call_service("/nonexistent/service", "std_srvs/srv/Trigger", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestOperationManager, test_call_component_service_invalid_type) {
  auto result = operation_manager_->call_component_service("/test", "operation", "invalid_type", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid service type format") != std::string::npos);
}

TEST_F(TestOperationManager, test_call_component_service_wrong_type) {
  // Trying to call a service with an action type
  auto result = operation_manager_->call_component_service("/test", "operation", "example_interfaces/action/Fibonacci",
                                                           nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not a service type") != std::string::npos);
}

// ==================== ACTION CALL TESTS ====================

TEST_F(TestOperationManager, test_send_component_action_goal_invalid_type) {
  auto result = operation_manager_->send_component_action_goal("/test", "operation", "invalid_type", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid action type format") != std::string::npos);
}

TEST_F(TestOperationManager, test_send_component_action_goal_wrong_type) {
  // Trying to send action goal with a service type
  auto result =
      operation_manager_->send_component_action_goal("/test", "operation", "std_srvs/srv/Trigger", nlohmann::json{});

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not an action type") != std::string::npos);
}

TEST_F(TestOperationManager, test_cancel_action_goal_invalid_uuid) {
  auto result = operation_manager_->cancel_action_goal("/test/action", "invalid_uuid");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid goal_id format") != std::string::npos);
}

TEST_F(TestOperationManager, test_get_action_result_invalid_uuid) {
  auto result = operation_manager_->get_action_result("/test/action", "example_interfaces/action/Fibonacci", "invalid");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("Invalid goal_id format") != std::string::npos);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
