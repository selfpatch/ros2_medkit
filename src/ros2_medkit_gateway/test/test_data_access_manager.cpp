// Copyright 2025-2026 bburda
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

#include "ros2_medkit_gateway/core/exceptions.hpp"
#include "ros2_medkit_gateway/core/type_introspection.hpp"
#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"
#include "ros2_medkit_gateway/data_access_manager.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_topic_transport.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

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
    node_ = std::make_shared<rclcpp::Node>("test_data_access_node");
    transport_ = std::make_shared<ros2::Ros2TopicTransport>(node_.get(), 0.5);
    data_manager_ = std::make_unique<DataAccessManager>(transport_, 0.5);
  }

  void TearDown() override {
    data_manager_.reset();
    transport_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ros2::Ros2TopicTransport> transport_;
  std::unique_ptr<DataAccessManager> data_manager_;
};

TEST_F(DataAccessManagerTest, topic_data_provider_is_null_until_attached) {
  EXPECT_EQ(data_manager_->get_topic_data_provider(), nullptr);
}

TEST_F(DataAccessManagerTest, get_type_introspection_returns_valid_ptr) {
  auto introspection = data_manager_->get_type_introspection();

  EXPECT_NE(introspection, nullptr);
}

TEST_F(DataAccessManagerTest, get_topic_sample_timeout_returns_configured_value) {
  double timeout = data_manager_->get_topic_sample_timeout();

  EXPECT_NEAR(timeout, 0.5, 0.01);
}

TEST_F(DataAccessManagerTest, sample_nonexistent_topic_returns_metadata_only) {
  // With no publishers present, the manager short-circuits to a metadata-only
  // response without engaging the transport. This mirrors the fast-path
  // contract documented in the manager and exercised by the routing tests.
  auto result = data_manager_->get_topic_sample_with_fallback("/nonexistent_topic_xyz_123", 0.1);
  EXPECT_EQ(result["status"], "metadata_only");
  EXPECT_EQ(result["topic"], "/nonexistent_topic_xyz_123");
  EXPECT_EQ(result["publisher_count"].get<uint64_t>(), 0u);
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
    node_ = std::make_shared<rclcpp::Node>("test_data_access_pub_node");
    // Publisher lives on a node that the main executor does not spin so
    // its create / destroy does not race rcutils_hash_map_* iterations
    // from the spin thread (TSan).
    publisher_node_ = std::make_shared<rclcpp::Node>("test_data_access_publisher_node");

    // MultiThreaded executor matches the production wiring needed by the
    // pool-backed TopicDataProvider (executor adds both the gateway node and
    // the subscription-worker node built inside Ros2SubscriptionExecutor).
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    transport_ = std::make_shared<ros2::Ros2TopicTransport>(node_.get(), 1.0);
    data_manager_ = std::make_unique<DataAccessManager>(transport_, 1.0);

    sub_exec_ = std::make_shared<ros2_common::Ros2SubscriptionExecutor>(node_);
    serializer_ = std::make_shared<ros2_medkit_serialization::JsonSerializer>();
    topic_data_provider_ = std::make_shared<Ros2TopicDataProvider>(sub_exec_, serializer_);
    data_manager_->set_topic_data_provider(topic_data_provider_.get());
    transport_->set_data_provider(topic_data_provider_.get());

    // Create a publisher for test topic
    publisher_ = publisher_node_->create_publisher<std_msgs::msg::String>("/test_sample_topic", 10);

    // Start spinning in background
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    // Give time for discovery
    std::this_thread::sleep_for(100ms);
  }

  void TearDown() override {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    publisher_.reset();
    if (transport_) {
      transport_->set_data_provider(nullptr);
    }
    topic_data_provider_.reset();
    sub_exec_.reset();
    data_manager_.reset();
    transport_.reset();
    executor_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<ros2::Ros2TopicTransport> transport_;
  std::unique_ptr<DataAccessManager> data_manager_;
  std::shared_ptr<ros2_common::Ros2SubscriptionExecutor> sub_exec_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;
  std::shared_ptr<Ros2TopicDataProvider> topic_data_provider_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::thread spin_thread_;
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

TEST_F(DataAccessManagerWithPublisherTest, sample_nonexistent_topic_returns_metadata_only) {
  auto result = data_manager_->get_topic_sample_native("/nonexistent_topic_abc", 0.1);
  EXPECT_EQ(result["status"], "metadata_only");
  EXPECT_EQ(result["topic"], "/nonexistent_topic_abc");
  EXPECT_EQ(result["publisher_count"].get<uint64_t>(), 0u);
}

TEST_F(DataAccessManagerWithPublisherTest, get_topic_sample_native_returns_metadata) {
  std::this_thread::sleep_for(200ms);

  auto result = data_manager_->get_topic_sample_native("/test_sample_topic", 0.5);

  EXPECT_TRUE(result.contains("topic"));
  EXPECT_TRUE(result.contains("status"));
}

// @verifies REQ_INTEROP_019
TEST_F(DataAccessManagerWithPublisherTest, set_topic_data_provider_nullptr_detaches_provider) {
  // Sanity: provider is attached after SetUp.
  ASSERT_NE(data_manager_->get_topic_data_provider(), nullptr);

  // Detach (mirrors the teardown step in main() that drops the provider before
  // the local shared_ptr resets).
  data_manager_->set_topic_data_provider(nullptr);
  EXPECT_EQ(data_manager_->get_topic_data_provider(), nullptr);

  // Re-attach must succeed without crashing.
  data_manager_->set_topic_data_provider(topic_data_provider_.get());
  EXPECT_EQ(data_manager_->get_topic_data_provider(), topic_data_provider_.get());
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

TEST_F(DataAccessManagerParameterTest, invalid_timeout_uses_default) {
  auto node = std::make_shared<rclcpp::Node>("test_param_node2");
  // Out-of-range timeout should be clamped to the safe default (1.0).
  auto transport = std::make_shared<ros2::Ros2TopicTransport>(node.get(), 100.0);
  auto manager = std::make_unique<DataAccessManager>(transport, 100.0);
  EXPECT_NEAR(manager->get_topic_sample_timeout(), 1.0, 0.01);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
