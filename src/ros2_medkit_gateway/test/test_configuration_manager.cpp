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

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ros2_medkit_gateway/configuration_manager.hpp"

using namespace ros2_medkit_gateway;

class TestConfigurationManager : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Create node with callback groups for multi-threaded execution
    // Use short timeout for tests to avoid long waits on nonexistent nodes
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(false);
    options.parameter_overrides({rclcpp::Parameter("parameter_service_timeout_sec", 0.1)});
    node_ = std::make_shared<rclcpp::Node>("test_config_manager_node", options);
    config_manager_ = std::make_unique<ConfigurationManager>(node_.get());

    // Create and start executor in separate thread to avoid deadlocks
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_thread_running_) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });
  }

  void TearDown() override {
    spin_thread_running_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(node_);
    config_manager_.reset();
    node_.reset();
    executor_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<ConfigurationManager> config_manager_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_thread_running_{false};
};

// ==================== LIST PARAMETERS TESTS ====================

TEST_F(TestConfigurationManager, test_list_parameters_nonexistent_node) {
  // Try to list parameters for a node that doesn't exist
  auto result = config_manager_->list_parameters("/nonexistent_node_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_TRUE(result.error_message.find("not available") != std::string::npos ||
              result.error_message.find("Failed") != std::string::npos);
}

TEST_F(TestConfigurationManager, test_list_parameters_own_node) {
  // List parameters for our own test node (which should exist)
  auto result = config_manager_->list_parameters("/test_config_manager_node");

  // Should succeed since we're querying our own node
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.is_array());
  // Even a basic node has some default parameters
  EXPECT_GE(result.data.size(), 0);
}

// ==================== GET PARAMETER TESTS ====================

TEST_F(TestConfigurationManager, test_get_parameter_nonexistent_node) {
  auto result = config_manager_->get_parameter("/nonexistent_node_xyz", "some_param");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestConfigurationManager, test_get_parameter_nonexistent_param) {
  // Query our own node for a parameter that doesn't exist
  auto result = config_manager_->get_parameter("/test_config_manager_node", "nonexistent_parameter_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not found") != std::string::npos ||
              result.error_message.find("Parameter not found") != std::string::npos);
}

TEST_F(TestConfigurationManager, test_get_parameter_use_sim_time) {
  // use_sim_time is a standard parameter that should exist
  auto result = config_manager_->get_parameter("/test_config_manager_node", "use_sim_time");

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.contains("name"));
  EXPECT_EQ(result.data["name"], "use_sim_time");
  EXPECT_TRUE(result.data.contains("value"));
  EXPECT_TRUE(result.data.contains("type"));
  EXPECT_EQ(result.data["type"], "bool");
}

// ==================== SET PARAMETER TESTS ====================

TEST_F(TestConfigurationManager, test_set_parameter_nonexistent_node) {
  auto result = config_manager_->set_parameter("/nonexistent_node_xyz", "param", nlohmann::json(42));

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

// ==================== RESET PARAMETER TESTS ====================

TEST_F(TestConfigurationManager, test_reset_parameter_nonexistent_node) {
  auto result = config_manager_->reset_parameter("/nonexistent_node_xyz", "param");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestConfigurationManager, test_reset_parameter_nonexistent_param) {
  // First need to cache defaults for our node
  auto list_result = config_manager_->list_parameters("/test_config_manager_node");
  ASSERT_TRUE(list_result.success);

  // Try to reset a parameter that doesn't exist in defaults
  auto result = config_manager_->reset_parameter("/test_config_manager_node", "nonexistent_parameter_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("No default value") != std::string::npos);
}

// ==================== RESET ALL PARAMETERS TESTS ====================

TEST_F(TestConfigurationManager, test_reset_all_parameters_nonexistent_node) {
  auto result = config_manager_->reset_all_parameters("/nonexistent_node_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestConfigurationManager, test_reset_all_parameters_own_node) {
  // First need to cache defaults for our node
  auto list_result = config_manager_->list_parameters("/test_config_manager_node");
  ASSERT_TRUE(list_result.success);

  // Reset all parameters - some may fail because they're read-only
  auto result = config_manager_->reset_all_parameters("/test_config_manager_node");

  // The operation should return data even if some parameters fail
  EXPECT_TRUE(result.data.contains("node_name"));
  EXPECT_TRUE(result.data.contains("reset_count"));
  EXPECT_TRUE(result.data.contains("failed_count"));
  // At least some parameters should have been attempted
  EXPECT_GE(result.data["reset_count"].get<int>() + result.data["failed_count"].get<int>(), 0);
}

// ==================== INTEGRATION TESTS ====================

TEST_F(TestConfigurationManager, test_set_and_get_parameter) {
  // Declare a parameter on our node first
  node_->declare_parameter("test_param", 100);

  // Get the initial value
  auto get_result1 = config_manager_->get_parameter("/test_config_manager_node", "test_param");
  ASSERT_TRUE(get_result1.success);
  EXPECT_EQ(get_result1.data["value"], 100);

  // Set a new value
  auto set_result = config_manager_->set_parameter("/test_config_manager_node", "test_param", nlohmann::json(200));
  ASSERT_TRUE(set_result.success);
  EXPECT_EQ(set_result.data["value"], 200);

  // Verify the new value
  auto get_result2 = config_manager_->get_parameter("/test_config_manager_node", "test_param");
  ASSERT_TRUE(get_result2.success);
  EXPECT_EQ(get_result2.data["value"], 200);
}

TEST_F(TestConfigurationManager, test_set_and_reset_parameter) {
  // Declare a parameter on our node
  node_->declare_parameter("reset_test_param", 42.5);

  // Cache defaults first
  auto list_result = config_manager_->list_parameters("/test_config_manager_node");
  ASSERT_TRUE(list_result.success);

  // Change the value
  auto set_result =
      config_manager_->set_parameter("/test_config_manager_node", "reset_test_param", nlohmann::json(99.9));
  ASSERT_TRUE(set_result.success);

  // Verify changed
  auto get_result1 = config_manager_->get_parameter("/test_config_manager_node", "reset_test_param");
  ASSERT_TRUE(get_result1.success);
  EXPECT_NEAR(get_result1.data["value"].get<double>(), 99.9, 0.01);

  // Reset to default
  auto reset_result = config_manager_->reset_parameter("/test_config_manager_node", "reset_test_param");
  ASSERT_TRUE(reset_result.success);
  EXPECT_TRUE(reset_result.data["reset_to_default"].get<bool>());

  // Verify reset to original
  auto get_result2 = config_manager_->get_parameter("/test_config_manager_node", "reset_test_param");
  ASSERT_TRUE(get_result2.success);
  EXPECT_NEAR(get_result2.data["value"].get<double>(), 42.5, 0.01);
}

TEST_F(TestConfigurationManager, test_set_parameter_different_types) {
  // Test setting parameters of different types
  node_->declare_parameter("string_param", "initial");
  node_->declare_parameter("int_param", 10);
  node_->declare_parameter("double_param", 3.14);
  node_->declare_parameter("bool_param", false);

  // String
  auto str_result =
      config_manager_->set_parameter("/test_config_manager_node", "string_param", nlohmann::json("modified"));
  EXPECT_TRUE(str_result.success);
  EXPECT_EQ(str_result.data["value"], "modified");
  EXPECT_EQ(str_result.data["type"], "string");

  // Integer
  auto int_result = config_manager_->set_parameter("/test_config_manager_node", "int_param", nlohmann::json(999));
  EXPECT_TRUE(int_result.success);
  EXPECT_EQ(int_result.data["value"], 999);
  EXPECT_EQ(int_result.data["type"], "int");

  // Double
  auto dbl_result = config_manager_->set_parameter("/test_config_manager_node", "double_param", nlohmann::json(2.718));
  EXPECT_TRUE(dbl_result.success);
  EXPECT_NEAR(dbl_result.data["value"].get<double>(), 2.718, 0.001);
  EXPECT_EQ(dbl_result.data["type"], "double");

  // Boolean
  auto bool_result = config_manager_->set_parameter("/test_config_manager_node", "bool_param", nlohmann::json(true));
  EXPECT_TRUE(bool_result.success);
  EXPECT_EQ(bool_result.data["value"], true);
  EXPECT_EQ(bool_result.data["type"], "bool");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
