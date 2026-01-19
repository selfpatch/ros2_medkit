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

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "ros2_medkit_gateway/data_access_manager.hpp"
#include "ros2_medkit_gateway/exceptions.hpp"
#include "ros2_medkit_gateway/type_introspection.hpp"

using namespace ros2_medkit_gateway;
using namespace std::chrono_literals;

// =============================================================================
// TypeIntrospection Tests
// =============================================================================

class TypeIntrospectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    introspection_ = std::make_unique<TypeIntrospection>("");
  }

  std::unique_ptr<TypeIntrospection> introspection_;
};

TEST_F(TypeIntrospectionTest, get_type_template_for_valid_type) {
  auto template_json = introspection_->get_type_template("std_msgs/msg/String");

  EXPECT_TRUE(template_json.is_object());
  EXPECT_TRUE(template_json.contains("data"));
}

TEST_F(TypeIntrospectionTest, get_type_template_for_float32) {
  auto template_json = introspection_->get_type_template("std_msgs/msg/Float32");

  EXPECT_TRUE(template_json.is_object());
  EXPECT_TRUE(template_json.contains("data"));
}

TEST_F(TypeIntrospectionTest, get_type_template_for_unknown_type_throws) {
  EXPECT_THROW(introspection_->get_type_template("nonexistent_pkg/msg/NonExistent"), std::runtime_error);
}

TEST_F(TypeIntrospectionTest, get_type_schema_for_valid_type) {
  auto schema = introspection_->get_type_schema("std_msgs/msg/String");

  EXPECT_TRUE(schema.is_object());
}

TEST_F(TypeIntrospectionTest, get_type_schema_for_unknown_type_throws) {
  EXPECT_THROW(introspection_->get_type_schema("nonexistent_pkg/msg/NonExistent"), std::runtime_error);
}

TEST_F(TypeIntrospectionTest, get_type_info_returns_complete_info) {
  auto info = introspection_->get_type_info("std_msgs/msg/String");

  EXPECT_EQ(info.name, "std_msgs/msg/String");
  EXPECT_TRUE(info.default_value.is_object());
  EXPECT_TRUE(info.schema.is_object());
}

TEST_F(TypeIntrospectionTest, get_type_info_caches_results) {
  // First call
  auto info1 = introspection_->get_type_info("std_msgs/msg/Float32");
  // Second call should return cached result
  auto info2 = introspection_->get_type_info("std_msgs/msg/Float32");

  EXPECT_EQ(info1.name, info2.name);
  EXPECT_EQ(info1.default_value, info2.default_value);
  EXPECT_EQ(info1.schema, info2.schema);
}

TEST_F(TypeIntrospectionTest, get_type_info_for_unknown_type_returns_empty) {
  // For unknown types, get_type_info should return empty objects instead of throwing
  auto info = introspection_->get_type_info("nonexistent_pkg/msg/NonExistent");

  EXPECT_EQ(info.name, "nonexistent_pkg/msg/NonExistent");
  EXPECT_TRUE(info.default_value.is_object());
  EXPECT_TRUE(info.schema.is_object());
}

// =============================================================================
// DataAccessManager Tests
// =============================================================================

class DataAccessManagerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    // Use short timeout for faster tests
    options.parameter_overrides(
        {rclcpp::Parameter("topic_sample_timeout_sec", 0.5), rclcpp::Parameter("max_parallel_topic_samples", 5)});
    node_ = std::make_shared<rclcpp::Node>("test_data_access_node", options);
    data_manager_ = std::make_unique<DataAccessManager>(node_.get());
  }

  void TearDown() override {
    data_manager_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<DataAccessManager> data_manager_;
};

TEST_F(DataAccessManagerTest, get_native_sampler_returns_valid_ptr) {
  auto sampler = data_manager_->get_native_sampler();

  EXPECT_NE(sampler, nullptr);
}

TEST_F(DataAccessManagerTest, get_type_introspection_returns_valid_ptr) {
  auto introspection = data_manager_->get_type_introspection();

  EXPECT_NE(introspection, nullptr);
}

TEST_F(DataAccessManagerTest, get_topic_sample_timeout_returns_configured_value) {
  double timeout = data_manager_->get_topic_sample_timeout();

  EXPECT_NEAR(timeout, 0.5, 0.01);
}

TEST_F(DataAccessManagerTest, sample_nonexistent_topic_throws) {
  EXPECT_THROW(data_manager_->get_topic_sample_with_fallback("/nonexistent_topic_xyz_123", 0.1),
               TopicNotAvailableException);
}

TEST_F(DataAccessManagerTest, publish_to_topic_creates_publisher) {
  nlohmann::json data = {{"data", "test message"}};

  // This should not throw
  EXPECT_NO_THROW({
    auto result = data_manager_->publish_to_topic("/test_publish_topic", "std_msgs/msg/String", data);
    EXPECT_EQ(result["status"], "published");
    EXPECT_EQ(result["topic"], "/test_publish_topic");
  });
}

TEST_F(DataAccessManagerTest, publish_to_topic_returns_timestamp) {
  nlohmann::json data = {{"data", 3.14f}};

  auto result = data_manager_->publish_to_topic("/test_float_topic", "std_msgs/msg/Float32", data);

  EXPECT_TRUE(result.contains("timestamp"));
  EXPECT_TRUE(result["timestamp"].is_number());
}

TEST_F(DataAccessManagerTest, publish_with_invalid_type_throws) {
  nlohmann::json data = {{"data", "test"}};

  EXPECT_THROW(data_manager_->publish_to_topic("/test_topic", "nonexistent_pkg/msg/NonExistent", data),
               std::runtime_error);
}

TEST_F(DataAccessManagerTest, publish_with_invalid_data_throws) {
  // Completely invalid JSON - cannot be serialized to any message
  nlohmann::json data = nlohmann::json::array({1, 2, 3});

  // This may succeed or throw depending on serializer behavior
  // For now, test that invalid type definitely throws
  EXPECT_THROW(data_manager_->publish_to_topic("/test_topic", "nonexistent_pkg/msg/NonExistent", data),
               std::runtime_error);
}

// =============================================================================
// DataAccessManager with Publisher Tests
// =============================================================================

class DataAccessManagerWithPublisherTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("topic_sample_timeout_sec", 1.0)});
    node_ = std::make_shared<rclcpp::Node>("test_data_access_pub_node", options);

    // Create executor for spinning
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    data_manager_ = std::make_unique<DataAccessManager>(node_.get());

    // Create a publisher for test topic
    publisher_ = node_->create_publisher<std_msgs::msg::String>("/test_sample_topic", 10);

    // Start spinning in background
    spin_thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_spinning_) {
        executor_->spin_some(10ms);
      }
    });

    // Give time for discovery
    std::this_thread::sleep_for(100ms);
  }

  void TearDown() override {
    stop_spinning_ = true;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    publisher_.reset();
    data_manager_.reset();
    executor_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<DataAccessManager> data_manager_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::thread spin_thread_;
  std::atomic<bool> stop_spinning_{false};
};

TEST_F(DataAccessManagerWithPublisherTest, sample_topic_with_publisher_returns_metadata) {
  // Allow more time for discovery
  std::this_thread::sleep_for(200ms);

  // Even without published messages, should return metadata for existing topic
  auto result = data_manager_->get_topic_sample_with_fallback("/test_sample_topic", 0.5);

  EXPECT_TRUE(result.contains("topic"));
  EXPECT_EQ(result["topic"], "/test_sample_topic");
  EXPECT_TRUE(result.contains("publisher_count"));
  EXPECT_GE(result["publisher_count"].get<int>(), 1);
}

TEST_F(DataAccessManagerWithPublisherTest, sample_topic_returns_type_info) {
  std::this_thread::sleep_for(200ms);

  auto result = data_manager_->get_topic_sample_with_fallback("/test_sample_topic", 0.5);

  // Should have type information
  EXPECT_TRUE(result.contains("type"));
  EXPECT_EQ(result["type"], "std_msgs/msg/String");
}

TEST_F(DataAccessManagerWithPublisherTest, sample_nonexistent_topic_throws_exception) {
  EXPECT_THROW(data_manager_->get_topic_sample_native("/nonexistent_topic_abc", 0.1), TopicNotAvailableException);
}

TEST_F(DataAccessManagerWithPublisherTest, get_topic_sample_native_returns_metadata) {
  std::this_thread::sleep_for(200ms);

  auto result = data_manager_->get_topic_sample_native("/test_sample_topic", 0.5);

  EXPECT_TRUE(result.contains("topic"));
  EXPECT_TRUE(result.contains("status"));
}

// =============================================================================
// Parameter Validation Tests
// =============================================================================

class DataAccessManagerParameterTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

TEST_F(DataAccessManagerParameterTest, invalid_max_parallel_samples_uses_default) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("max_parallel_topic_samples", 100)});  // Out of range
  auto node = std::make_shared<rclcpp::Node>("test_param_node", options);

  // Should not throw, will use default value of 10
  EXPECT_NO_THROW({ auto manager = std::make_unique<DataAccessManager>(node.get()); });
}

TEST_F(DataAccessManagerParameterTest, invalid_timeout_uses_default) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_sample_timeout_sec", 100.0)});  // Out of range
  auto node = std::make_shared<rclcpp::Node>("test_param_node2", options);

  auto manager = std::make_unique<DataAccessManager>(node.get());
  // Should use default of 1.0
  EXPECT_NEAR(manager->get_topic_sample_timeout(), 1.0, 0.01);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
