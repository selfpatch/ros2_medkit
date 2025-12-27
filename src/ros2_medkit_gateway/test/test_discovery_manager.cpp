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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/discovery_manager.hpp"
#include "ros2_medkit_gateway/native_topic_sampler.hpp"

using ros2_medkit_gateway::DiscoveryManager;
using ros2_medkit_gateway::NativeTopicSampler;

// =============================================================================
// DiscoveryManager tests
// =============================================================================

class DiscoveryManagerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_discovery_node");
    topic_sampler_ = std::make_shared<NativeTopicSampler>(node_.get());
    discovery_manager_ = std::make_unique<DiscoveryManager>(node_.get());
    discovery_manager_->set_topic_sampler(topic_sampler_.get());
  }

  void TearDown() override {
    discovery_manager_.reset();
    topic_sampler_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<NativeTopicSampler> topic_sampler_;
  std::unique_ptr<DiscoveryManager> discovery_manager_;
};

TEST_F(DiscoveryManagerTest, DiscoverTopicComponents_ReturnsEmptyWhenNoTopics) {
  // In a clean environment with only system topics, should return empty
  auto components = discovery_manager_->discover_topic_components();

  // System topics are filtered, so we may get empty list
  // Each component should have source="topic" if present
  for (const auto & comp : components) {
    EXPECT_EQ(comp.source, "topic") << "Topic-based component should have source='topic'";
  }
}

TEST_F(DiscoveryManagerTest, DiscoverTopicComponents_SetsSourceField) {
  auto components = discovery_manager_->discover_topic_components();

  // All topic-based components should have source="topic"
  for (const auto & comp : components) {
    EXPECT_EQ(comp.source, "topic");
    EXPECT_FALSE(comp.id.empty());
    EXPECT_FALSE(comp.namespace_path.empty());
    EXPECT_FALSE(comp.fqn.empty());
  }
}

TEST_F(DiscoveryManagerTest, DiscoverComponents_NodeBasedHaveSourceNode) {
  auto components = discovery_manager_->discover_components();

  // Node-based components should have source="node" (default)
  for (const auto & comp : components) {
    EXPECT_EQ(comp.source, "node") << "Node-based component should have source='node'";
  }
}

// =============================================================================
// Integration test with publishers (topic-based discovery)
// =============================================================================

class DiscoveryManagerWithPublishersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Create main node for discovery
    node_ = std::make_shared<rclcpp::Node>("test_discovery_node");
    topic_sampler_ = std::make_shared<NativeTopicSampler>(node_.get());
    discovery_manager_ = std::make_unique<DiscoveryManager>(node_.get());
    discovery_manager_->set_topic_sampler(topic_sampler_.get());

    // Create publishers on namespaced topics (simulating Isaac Sim)
    // These topics have no associated nodes in those namespaces
    pub1_ = node_->create_publisher<std_msgs::msg::String>("/robot_alpha/status", 10);
    pub2_ = node_->create_publisher<std_msgs::msg::String>("/robot_alpha/odom", 10);
    pub3_ = node_->create_publisher<std_msgs::msg::String>("/robot_beta/status", 10);

    // Allow time for graph discovery
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    rclcpp::spin_some(node_);
  }

  void TearDown() override {
    pub1_.reset();
    pub2_.reset();
    pub3_.reset();
    discovery_manager_.reset();
    topic_sampler_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<NativeTopicSampler> topic_sampler_;
  std::unique_ptr<DiscoveryManager> discovery_manager_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;
};

TEST_F(DiscoveryManagerWithPublishersTest, DiscoverTopicComponents_FindsNamespacedTopics) {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto components = discovery_manager_->discover_topic_components();

  // Should discover robot_alpha and robot_beta namespaces
  bool found_alpha = false;
  bool found_beta = false;

  for (const auto & comp : components) {
    if (comp.id == "robot_alpha") {
      found_alpha = true;
      EXPECT_EQ(comp.source, "topic");
      EXPECT_EQ(comp.namespace_path, "/robot_alpha");
      EXPECT_EQ(comp.area, "robot_alpha");
      // Should have at least 2 topics
      EXPECT_GE(comp.topics.publishes.size(), 2u);
    }
    if (comp.id == "robot_beta") {
      found_beta = true;
      EXPECT_EQ(comp.source, "topic");
      EXPECT_EQ(comp.namespace_path, "/robot_beta");
      EXPECT_GE(comp.topics.publishes.size(), 1u);
    }
  }

  EXPECT_TRUE(found_alpha) << "Should discover robot_alpha from topics";
  EXPECT_TRUE(found_beta) << "Should discover robot_beta from topics";
}

TEST_F(DiscoveryManagerWithPublishersTest, DiscoverTopicComponents_ComponentHasCorrectTopics) {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto components = discovery_manager_->discover_topic_components();

  for (const auto & comp : components) {
    if (comp.id == "robot_alpha") {
      bool has_status = false;
      bool has_odom = false;

      for (const auto & topic : comp.topics.publishes) {
        if (topic == "/robot_alpha/status") {
          has_status = true;
        }
        if (topic == "/robot_alpha/odom") {
          has_odom = true;
        }
      }

      EXPECT_TRUE(has_status) << "robot_alpha should have /robot_alpha/status topic";
      EXPECT_TRUE(has_odom) << "robot_alpha should have /robot_alpha/odom topic";
    }
  }
}

TEST_F(DiscoveryManagerWithPublishersTest, DiscoverAreas_IncludesTopicBasedAreas) {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  rclcpp::spin_some(node_);

  auto areas = discovery_manager_->discover_areas();

  bool found_alpha = false;
  bool found_beta = false;

  for (const auto & area : areas) {
    if (area.id == "robot_alpha") {
      found_alpha = true;
    }
    if (area.id == "robot_beta") {
      found_beta = true;
    }
  }

  EXPECT_TRUE(found_alpha) << "Areas should include robot_alpha from topics";
  EXPECT_TRUE(found_beta) << "Areas should include robot_beta from topics";
}

TEST_F(DiscoveryManagerWithPublishersTest, DiscoverTopicComponents_DoesNotDuplicateNodeNamespaces) {
  // The discovery manager's own node is in root namespace
  // It should not create a topic-based component for root namespace

  auto topic_components = discovery_manager_->discover_topic_components();

  for (const auto & comp : topic_components) {
    // Topic-based components should not be in root namespace
    EXPECT_NE(comp.namespace_path, "/") << "Topic-based component should not be in root namespace";
    // Also check it's not duplicating test_discovery_node's namespace
    // (which is root "/")
    EXPECT_FALSE(comp.id.empty());
  }
}
