// Copyright 2026 bburda
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

#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ros2_medkit_gateway/discovery/runtime_discovery.hpp"

using ros2_medkit_gateway::discovery::RuntimeDiscoveryStrategy;

// =============================================================================
// RuntimeDiscoveryStrategy - Function from namespace tests
// =============================================================================

class RuntimeDiscoveryTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Create node in a namespace to test namespace grouping
    rclcpp::NodeOptions options;
    node_ = std::make_shared<rclcpp::Node>("test_node", "/test_ns", options);
    strategy_ = std::make_unique<RuntimeDiscoveryStrategy>(node_.get());
  }

  void TearDown() override {
    strategy_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<RuntimeDiscoveryStrategy> strategy_;
};

// -----------------------------------------------------------------------------
// discover_areas() - always returns empty (Areas come from manifest only)
// -----------------------------------------------------------------------------

TEST_F(RuntimeDiscoveryTest, DiscoverAreas_AlwaysReturnsEmpty) {
  auto areas = strategy_->discover_areas();
  EXPECT_TRUE(areas.empty()) << "Areas should always be empty - Areas come from manifest only";
}

// -----------------------------------------------------------------------------
// discover_components() - always returns empty (Components come from
// HostInfoProvider or manifest)
// -----------------------------------------------------------------------------

TEST_F(RuntimeDiscoveryTest, DiscoverComponents_AlwaysReturnsEmpty) {
  auto components = strategy_->discover_components();
  EXPECT_TRUE(components.empty())
      << "Components should always be empty - Components come from HostInfoProvider or manifest";
}

// -----------------------------------------------------------------------------
// discover_functions() - namespace grouping
// -----------------------------------------------------------------------------

TEST_F(RuntimeDiscoveryTest, DiscoverFunctions_DefaultCreatesFromNamespaces) {
  // Default config has create_functions_from_namespaces=true
  auto functions = strategy_->discover_functions();

  // Should find at least "test_ns" function from our node's namespace
  bool found_test_ns = false;
  for (const auto & func : functions) {
    if (func.id == "test_ns") {
      found_test_ns = true;
      EXPECT_EQ(func.name, "test_ns");
      EXPECT_EQ(func.source, "runtime");
      EXPECT_FALSE(func.hosts.empty()) << "Function should have at least one host app";
      // The host should be our test node
      bool found_test_node = false;
      for (const auto & host_id : func.hosts) {
        if (host_id == "test_node") {
          found_test_node = true;
        }
      }
      EXPECT_TRUE(found_test_node) << "Function hosts should include 'test_node'";
    }
  }
  EXPECT_TRUE(found_test_ns) << "Should discover 'test_ns' function from namespace grouping";
}

TEST_F(RuntimeDiscoveryTest, DiscoverFunctions_ReturnsEmptyWhenDisabled) {
  RuntimeDiscoveryStrategy::RuntimeConfig config;
  config.create_functions_from_namespaces = false;
  strategy_->set_config(config);

  auto functions = strategy_->discover_functions();
  EXPECT_TRUE(functions.empty()) << "Functions should be empty when create_functions_from_namespaces=false";
}

TEST_F(RuntimeDiscoveryTest, DiscoverFunctions_HostsPointToAppIds) {
  auto functions = strategy_->discover_functions();
  auto apps = strategy_->discover_apps();

  // All host IDs in functions should correspond to actual app IDs
  for (const auto & func : functions) {
    for (const auto & host_id : func.hosts) {
      bool found = false;
      for (const auto & app : apps) {
        if (app.id == host_id) {
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Function '" << func.id << "' host '" << host_id << "' should be a valid app ID";
    }
  }
}

TEST_F(RuntimeDiscoveryTest, DiscoverFunctions_OnlyCreatesNonEmptyFunctions) {
  // All discovered functions should have at least one host
  auto functions = strategy_->discover_functions();
  for (const auto & func : functions) {
    EXPECT_FALSE(func.hosts.empty()) << "Function '" << func.id << "' should have at least one host app";
  }
}

// -----------------------------------------------------------------------------
// RuntimeConfig defaults
// -----------------------------------------------------------------------------

TEST_F(RuntimeDiscoveryTest, DefaultConfig_HasCorrectDefaults) {
  RuntimeDiscoveryStrategy::RuntimeConfig config;
  EXPECT_TRUE(config.create_functions_from_namespaces) << "create_functions_from_namespaces should default to true";
}

// =============================================================================
// Tests with multiple namespaces
// =============================================================================

class RuntimeDiscoveryMultiNsTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Create nodes in different namespaces
    node_sensors_ = std::make_shared<rclcpp::Node>("camera", "/sensors");
    node_nav_ = std::make_shared<rclcpp::Node>("planner", "/navigation");
    // Create discovery node (the one that queries the graph)
    node_discovery_ = std::make_shared<rclcpp::Node>("discovery_node");
    strategy_ = std::make_unique<RuntimeDiscoveryStrategy>(node_discovery_.get());
  }

  void TearDown() override {
    strategy_.reset();
    node_discovery_.reset();
    node_nav_.reset();
    node_sensors_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_sensors_;
  std::shared_ptr<rclcpp::Node> node_nav_;
  std::shared_ptr<rclcpp::Node> node_discovery_;
  std::unique_ptr<RuntimeDiscoveryStrategy> strategy_;
};

TEST_F(RuntimeDiscoveryMultiNsTest, DiscoverFunctions_GroupsByNamespace) {
  auto functions = strategy_->discover_functions();

  bool found_sensors = false;
  bool found_navigation = false;
  bool found_root = false;

  for (const auto & func : functions) {
    if (func.id == "sensors") {
      found_sensors = true;
      EXPECT_EQ(func.source, "runtime");
      // "camera" node should be in sensors
      auto it = std::find(func.hosts.begin(), func.hosts.end(), "camera");
      EXPECT_NE(it, func.hosts.end()) << "sensors function should host 'camera' app";
    }
    if (func.id == "navigation") {
      found_navigation = true;
      EXPECT_EQ(func.source, "runtime");
      // "planner" node should be in navigation
      auto it = std::find(func.hosts.begin(), func.hosts.end(), "planner");
      EXPECT_NE(it, func.hosts.end()) << "navigation function should host 'planner' app";
    }
    if (func.id == "root") {
      found_root = true;
      // "discovery_node" is in root namespace
      auto it = std::find(func.hosts.begin(), func.hosts.end(), "discovery_node");
      EXPECT_NE(it, func.hosts.end()) << "root function should host 'discovery_node' app";
    }
  }

  EXPECT_TRUE(found_sensors) << "Should create 'sensors' function from /sensors namespace";
  EXPECT_TRUE(found_navigation) << "Should create 'navigation' function from /navigation namespace";
  EXPECT_TRUE(found_root) << "Should create 'root' function from root namespace nodes";
}

TEST_F(RuntimeDiscoveryMultiNsTest, DiscoverAreas_AlwaysEmpty) {
  auto areas = strategy_->discover_areas();
  EXPECT_TRUE(areas.empty()) << "Areas should always be empty - Areas come from manifest only";
}

TEST_F(RuntimeDiscoveryMultiNsTest, DiscoverComponents_AlwaysEmpty) {
  auto components = strategy_->discover_components();
  EXPECT_TRUE(components.empty())
      << "Components should always be empty - Components come from HostInfoProvider or manifest";
}
