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
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "ros2_medkit_serialization/json_serializer.hpp"

#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"

using ros2_medkit_gateway::DiscoveryManager;
using ros2_medkit_gateway::Ros2TopicDataProvider;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;

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
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });
    sub_exec_ = std::make_shared<Ros2SubscriptionExecutor>(node_);
    serializer_ = std::make_shared<ros2_medkit_serialization::JsonSerializer>();
    topic_provider_ = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_);
    discovery_manager_ = std::make_unique<DiscoveryManager>(node_.get());
    discovery_manager_->set_topic_data_provider(topic_provider_.get());
  }

  void TearDown() override {
    // Cancel + join the main executor BEFORE dropping sub_exec / provider so
    // ~Ros2SubscriptionExecutor does not race the spin thread on the
    // subscription node's rcl internals (TSan flags rcutils_array_list_fini
    // concurrently read from both threads otherwise).
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    discovery_manager_.reset();
    topic_provider_.reset();
    sub_exec_.reset();
    executor_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<Ros2SubscriptionExecutor> sub_exec_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;
  std::unique_ptr<Ros2TopicDataProvider> topic_provider_;
  std::unique_ptr<DiscoveryManager> discovery_manager_;
};

// @verifies REQ_INTEROP_018
TEST_F(DiscoveryManagerTest, SetTopicDataProviderNullptrIsAcceptedAfterAttach) {
  // The discovery manager has a provider attached in SetUp (line 57). The
  // teardown sequence in main() (commit ab558e0d) calls
  // GatewayNode::set_topic_data_provider(nullptr), which propagates through
  // the discovery manager. Confirm that path does not throw and that
  // subsequent discovery calls still work (no provider just means topic-map
  // enrichment is skipped).
  discovery_manager_->set_topic_data_provider(nullptr);
  EXPECT_NO_THROW({
    auto components = discovery_manager_->discover_components();
    (void)components;
  });
  // Re-attach must work too.
  discovery_manager_->set_topic_data_provider(topic_provider_.get());
  EXPECT_NO_THROW({
    auto components = discovery_manager_->discover_components();
    (void)components;
  });
}

TEST_F(DiscoveryManagerTest, DiscoverComponents_RuntimeOnlyReturnsHostComponent) {
  // With default config (host info provider enabled), should return single host component
  ros2_medkit_gateway::DiscoveryConfig config;
  config.runtime.default_component_enabled = true;
  discovery_manager_->initialize(config);

  auto components = discovery_manager_->discover_components();
  EXPECT_EQ(components.size(), 1u) << "Should return exactly one host-derived component";
  if (!components.empty()) {
    EXPECT_EQ(components[0].source, "runtime") << "Component should come from HostInfoProvider with source='runtime'";
  }
}

TEST_F(DiscoveryManagerTest, DiscoverComponents_EmptyWhenHostInfoDisabled) {
  // With host info provider disabled, no components in runtime mode
  ros2_medkit_gateway::DiscoveryConfig config;
  config.runtime.default_component_enabled = false;
  discovery_manager_->initialize(config);

  auto components = discovery_manager_->discover_components();
  EXPECT_TRUE(components.empty()) << "Components should be empty when host info provider is disabled";
}

TEST_F(DiscoveryManagerTest, DiscoverAreas_AlwaysEmptyInRuntimeMode) {
  auto areas = discovery_manager_->discover_areas();
  EXPECT_TRUE(areas.empty()) << "Areas should always be empty in runtime mode - Areas come from manifest only";
}

TEST_F(DiscoveryManagerTest, DiscoverFunctions_CreatedFromNamespaces) {
  // Default config has create_functions_from_namespaces=true
  auto functions = discovery_manager_->discover_functions();
  // Should find at least "root" function from the discovery node's namespace
  bool found_root = false;
  for (const auto & func : functions) {
    if (func.id == "root") {
      found_root = true;
      EXPECT_EQ(func.source, "runtime");
    }
  }
  EXPECT_TRUE(found_root) << "Should discover 'root' function from namespace grouping";
}
