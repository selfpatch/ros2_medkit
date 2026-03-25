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

#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"

using namespace ros2_medkit_gateway::discovery;
using namespace ros2_medkit_gateway;

class RuntimeLinkerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    linker_ = std::make_unique<RuntimeLinker>(nullptr);
  }

  // Helper to create a test App with ROS binding
  App create_app(const std::string & id, const std::string & node_name, const std::string & ns_pattern = "*") {
    App app;
    app.id = id;
    app.name = id;
    app.source = "manifest";

    App::RosBinding binding;
    binding.node_name = node_name;
    binding.namespace_pattern = ns_pattern;
    app.ros_binding = binding;

    return app;
  }

  // Helper to create a test App with topic namespace binding
  App create_topic_app(const std::string & id, const std::string & topic_ns) {
    App app;
    app.id = id;
    app.name = id;
    app.source = "manifest";

    App::RosBinding binding;
    binding.topic_namespace = topic_ns;
    app.ros_binding = binding;

    return app;
  }

  // Helper to create a runtime App (representing a discovered ROS 2 node)
  App create_runtime_app(const std::string & id, const std::string & ns = "/") {
    App app;
    app.id = id;
    app.name = id;
    app.source = "heuristic";
    app.is_online = true;
    app.bound_fqn = ns == "/" ? "/" + id : ns + "/" + id;
    return app;
  }

  std::unique_ptr<RuntimeLinker> linker_;
  ManifestConfig config_;
};

// =============================================================================
// Exact Match Tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST_F(RuntimeLinkerTest, ExactMatch_NodeNameAndNamespace) {
  std::vector<App> apps = {create_app("controller_app", "controller", "/nav")};
  std::vector<App> runtime_apps = {create_runtime_app("controller", "/nav")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_EQ(result.linked_apps.size(), 1);
  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/nav/controller");
  EXPECT_EQ(result.app_to_node["controller_app"], "/nav/controller");
  EXPECT_EQ(result.node_to_app["/nav/controller"], "controller_app");
  EXPECT_TRUE(result.unlinked_app_ids.empty());
  EXPECT_TRUE(result.orphan_nodes.empty());
}

TEST_F(RuntimeLinkerTest, ExactMatch_RootNamespace) {
  std::vector<App> apps = {create_app("my_app", "my_node", "/")};
  std::vector<App> runtime_apps = {create_runtime_app("my_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_EQ(result.linked_apps.size(), 1);
  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/my_node");
}

TEST_F(RuntimeLinkerTest, NoMatch_DifferentNodeName) {
  std::vector<App> apps = {create_app("app1", "controller", "/nav")};
  std::vector<App> runtime_apps = {create_runtime_app("planner", "/nav")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_EQ(result.linked_apps.size(), 1);
  EXPECT_FALSE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.unlinked_app_ids.size(), 1);
  EXPECT_EQ(result.unlinked_app_ids[0], "app1");
  EXPECT_EQ(result.orphan_nodes.size(), 1);
}

TEST_F(RuntimeLinkerTest, NoMatch_DifferentNamespace) {
  std::vector<App> apps = {create_app("app1", "controller", "/navigation")};
  std::vector<App> runtime_apps = {create_runtime_app("controller", "/planning")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_FALSE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.unlinked_app_ids.size(), 1);
}

// =============================================================================
// Wildcard Namespace Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, WildcardNamespace_MatchesAny) {
  std::vector<App> apps = {create_app("app1", "controller", "*")};
  std::vector<App> runtime_apps = {
      create_runtime_app("controller", "/ns1"),
      create_runtime_app("controller", "/ns2"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  // Should match the first one found
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/ns1/controller");
  // Second node should be orphan since app is already matched
  EXPECT_EQ(result.orphan_nodes.size(), 1);
}

TEST_F(RuntimeLinkerTest, WildcardNamespace_MatchesRootNamespace) {
  std::vector<App> apps = {create_app("app1", "my_node", "*")};
  std::vector<App> runtime_apps = {create_runtime_app("my_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/my_node");
}

TEST_F(RuntimeLinkerTest, WildcardNamespace_MatchesNestedNamespace) {
  std::vector<App> apps = {create_app("app1", "controller", "*")};
  std::vector<App> runtime_apps = {create_runtime_app("controller", "/robot/nav/local")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/robot/nav/local/controller");
}

// =============================================================================
// Topic Namespace Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, TopicNamespace_MatchesByPublisher) {
  App app;
  app.id = "app1";
  app.name = "App 1";
  App::RosBinding binding;
  binding.topic_namespace = "/sensor_data";
  app.ros_binding = binding;

  App rt_app = create_runtime_app("sensor_driver", "/");
  rt_app.topics.publishes = {"/sensor_data/imu", "/sensor_data/gps"};

  std::vector<App> apps = {app};
  std::vector<App> runtime_apps = {rt_app};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/sensor_driver");
}

TEST_F(RuntimeLinkerTest, TopicNamespace_MatchesBySubscriber) {
  App app;
  app.id = "app1";
  app.name = "App 1";
  App::RosBinding binding;
  binding.topic_namespace = "/cmd";
  app.ros_binding = binding;

  App rt_app = create_runtime_app("motor_driver", "/");
  rt_app.topics.subscribes = {"/cmd/velocity", "/cmd/position"};

  std::vector<App> apps = {app};
  std::vector<App> runtime_apps = {rt_app};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
}

TEST_F(RuntimeLinkerTest, TopicNamespace_NoMatch) {
  App app;
  app.id = "app1";
  app.name = "App 1";
  App::RosBinding binding;
  binding.topic_namespace = "/navigation";
  app.ros_binding = binding;

  App rt_app = create_runtime_app("sensor_driver", "/");
  rt_app.topics.publishes = {"/sensor/imu"};

  std::vector<App> apps = {app};
  std::vector<App> runtime_apps = {rt_app};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_FALSE(result.linked_apps[0].is_online);
}

// =============================================================================
// Orphan Node Detection Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, OrphanNodes_DetectedCorrectly) {
  std::vector<App> apps = {create_app("app1", "controller", "/nav")};
  std::vector<App> runtime_apps = {
      create_runtime_app("controller", "/nav"),
      create_runtime_app("planner", "/nav"),
      create_runtime_app("mapper", "/map"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_EQ(result.orphan_nodes.size(), 2);
  EXPECT_TRUE(std::find(result.orphan_nodes.begin(), result.orphan_nodes.end(), "/nav/planner") !=
              result.orphan_nodes.end());
  EXPECT_TRUE(std::find(result.orphan_nodes.begin(), result.orphan_nodes.end(), "/map/mapper") !=
              result.orphan_nodes.end());
}

TEST_F(RuntimeLinkerTest, OrphanNodes_AllMatched) {
  std::vector<App> apps = {
      create_app("app1", "node1", "*"),
      create_app("app2", "node2", "*"),
  };
  std::vector<App> runtime_apps = {
      create_runtime_app("node1", "/"),
      create_runtime_app("node2", "/"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.orphan_nodes.empty());
  EXPECT_EQ(result.app_to_node.size(), 2);
}

// =============================================================================
// Orphan Policy Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, OrphanPolicy_Error_ReportsError) {
  config_.unmanifested_nodes = ManifestConfig::UnmanifestedNodePolicy::ERROR;

  std::vector<App> apps = {};
  std::vector<App> runtime_apps = {create_runtime_app("orphan_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.has_errors(config_.unmanifested_nodes));
  EXPECT_EQ(result.orphan_nodes.size(), 1);
}

TEST_F(RuntimeLinkerTest, OrphanPolicy_Ignore_NoError) {
  config_.unmanifested_nodes = ManifestConfig::UnmanifestedNodePolicy::IGNORE;

  std::vector<App> apps = {};
  std::vector<App> runtime_apps = {create_runtime_app("orphan_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_FALSE(result.has_errors(config_.unmanifested_nodes));
}

// @verifies REQ_INTEROP_003
TEST_F(RuntimeLinkerTest, OrphanPolicy_Warn_NoError) {
  config_.unmanifested_nodes = ManifestConfig::UnmanifestedNodePolicy::WARN;

  std::vector<App> apps = {};
  std::vector<App> runtime_apps = {create_runtime_app("orphan_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  // WARN logs warnings but does not fail
  EXPECT_FALSE(result.has_errors(config_.unmanifested_nodes));
  EXPECT_EQ(result.orphan_nodes.size(), 1u);
}

// @verifies REQ_INTEROP_003
TEST_F(RuntimeLinkerTest, OrphanPolicy_IncludeAsOrphan_NoError) {
  config_.unmanifested_nodes = ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN;

  std::vector<App> apps = {};
  std::vector<App> runtime_apps = {create_runtime_app("orphan_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  // INCLUDE_AS_ORPHAN includes orphans but does not fail
  EXPECT_FALSE(result.has_errors(config_.unmanifested_nodes));
  EXPECT_EQ(result.orphan_nodes.size(), 1u);
}

// =============================================================================
// App Enrichment Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, EnrichApp_CopiesTopics) {
  std::vector<App> apps = {create_app("app1", "sensor", "*")};

  App rt_app = create_runtime_app("sensor", "/");
  rt_app.topics.publishes = {"/sensor/data", "/sensor/status"};
  rt_app.topics.subscribes = {"/sensor/config"};

  std::vector<App> runtime_apps = {rt_app};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].topics.publishes.size(), 2);
  EXPECT_EQ(result.linked_apps[0].topics.subscribes.size(), 1);
}

TEST_F(RuntimeLinkerTest, EnrichApp_CopiesServices) {
  std::vector<App> apps = {create_app("app1", "server", "*")};

  App rt_app = create_runtime_app("server", "/");
  rt_app.services = {{"srv1", "/srv1", "std_srvs/srv/Trigger", std::nullopt},
                     {"srv2", "/srv2", "std_srvs/srv/Empty", std::nullopt}};

  std::vector<App> runtime_apps = {rt_app};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].services.size(), 2);
}

TEST_F(RuntimeLinkerTest, EnrichApp_CopiesActions) {
  std::vector<App> apps = {create_app("app1", "action_server", "*")};

  App rt_app = create_runtime_app("action_server", "/");
  rt_app.actions = {{"nav", "/nav", "nav2_msgs/action/NavigateToPose", std::nullopt}};

  std::vector<App> runtime_apps = {rt_app};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].actions.size(), 1);
}

// =============================================================================
// External App Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, ExternalApp_NotLinked) {
  App app;
  app.id = "external_app";
  app.name = "External App";
  app.external = true;

  std::vector<App> apps = {app};
  std::vector<App> runtime_apps = {create_runtime_app("some_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_EQ(result.linked_apps.size(), 1);
  EXPECT_FALSE(result.linked_apps[0].is_online);
  EXPECT_TRUE(result.unlinked_app_ids.empty());  // External apps don't go to unlinked
  EXPECT_EQ(result.orphan_nodes.size(), 1);      // Node is orphan since external app ignored
}

// =============================================================================
// No ROS Binding Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, NoBinding_GoesToUnlinked) {
  App app;
  app.id = "app_no_binding";
  app.name = "App Without Binding";
  // No ros_binding set

  std::vector<App> apps = {app};
  std::vector<App> runtime_apps = {create_runtime_app("some_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_FALSE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.unlinked_app_ids.size(), 1);
  EXPECT_EQ(result.unlinked_app_ids[0], "app_no_binding");
}

TEST_F(RuntimeLinkerTest, EmptyBinding_GoesToUnlinked) {
  App app;
  app.id = "app_empty_binding";
  app.name = "App With Empty Binding";
  app.ros_binding = App::RosBinding{};  // Empty binding

  std::vector<App> apps = {app};
  std::vector<App> runtime_apps = {create_runtime_app("some_node", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_FALSE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.unlinked_app_ids.size(), 1);
}

// =============================================================================
// Lookup Methods Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, IsAppOnline_AfterLinking) {
  std::vector<App> apps = {
      create_app("online_app", "online_node", "*"),
      create_app("offline_app", "missing_node", "*"),
  };
  std::vector<App> runtime_apps = {create_runtime_app("online_node", "/")};

  linker_->link(apps, runtime_apps, config_);

  EXPECT_TRUE(linker_->is_app_online("online_app"));
  EXPECT_FALSE(linker_->is_app_online("offline_app"));
  EXPECT_FALSE(linker_->is_app_online("nonexistent_app"));
}

TEST_F(RuntimeLinkerTest, GetBoundNode_ReturnsCorrectFqn) {
  std::vector<App> apps = {create_app("app1", "my_node", "/ns")};
  std::vector<App> runtime_apps = {create_runtime_app("my_node", "/ns")};

  linker_->link(apps, runtime_apps, config_);

  auto bound = linker_->get_bound_node("app1");
  ASSERT_TRUE(bound.has_value());
  EXPECT_EQ(bound.value(), "/ns/my_node");

  EXPECT_FALSE(linker_->get_bound_node("nonexistent").has_value());
}

TEST_F(RuntimeLinkerTest, GetAppForNode_ReturnsCorrectId) {
  std::vector<App> apps = {create_app("app1", "my_node", "/ns")};
  std::vector<App> runtime_apps = {create_runtime_app("my_node", "/ns")};

  linker_->link(apps, runtime_apps, config_);

  auto app_id = linker_->get_app_for_node("/ns/my_node");
  ASSERT_TRUE(app_id.has_value());
  EXPECT_EQ(app_id.value(), "app1");

  EXPECT_FALSE(linker_->get_app_for_node("/unknown/node").has_value());
}

// =============================================================================
// Multiple Apps Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, MultipleApps_AllLinked) {
  std::vector<App> apps = {
      create_app("app1", "node1", "*"),
      create_app("app2", "node2", "*"),
      create_app("app3", "node3", "*"),
  };
  std::vector<App> runtime_apps = {
      create_runtime_app("node1", "/"),
      create_runtime_app("node2", "/"),
      create_runtime_app("node3", "/"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_EQ(result.linked_apps.size(), 3);
  for (const auto & app : result.linked_apps) {
    EXPECT_TRUE(app.is_online);
  }
  EXPECT_TRUE(result.unlinked_app_ids.empty());
  EXPECT_TRUE(result.orphan_nodes.empty());
}

TEST_F(RuntimeLinkerTest, MultipleApps_SomeUnlinked) {
  std::vector<App> apps = {
      create_app("app1", "node1", "*"),
      create_app("app2", "missing1", "*"),
      create_app("app3", "node3", "*"),
      create_app("app4", "missing2", "*"),
  };
  std::vector<App> runtime_apps = {
      create_runtime_app("node1", "/"),
      create_runtime_app("node3", "/"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);

  EXPECT_EQ(result.linked_apps.size(), 4);
  EXPECT_EQ(result.app_to_node.size(), 2);
  EXPECT_EQ(result.unlinked_app_ids.size(), 2);
}

// =============================================================================
// Result Summary Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, ResultSummary_FormatsCorrectly) {
  std::vector<App> apps = {
      create_app("app1", "node1", "*"),
      create_app("app2", "missing", "*"),
  };
  std::vector<App> runtime_apps = {
      create_runtime_app("node1", "/"),
      create_runtime_app("orphan", "/"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);

  std::string summary = result.summary();
  EXPECT_TRUE(summary.find("1 linked") != std::string::npos);
  EXPECT_TRUE(summary.find("1 unlinked") != std::string::npos);
  EXPECT_TRUE(summary.find("1 orphan") != std::string::npos);
}

TEST_F(RuntimeLinkerTest, GetLastResult_ReturnsLatest) {
  std::vector<App> apps = {create_app("app1", "node1", "*")};
  std::vector<App> runtime_apps = {create_runtime_app("node1", "/")};

  linker_->link(apps, runtime_apps, config_);

  const auto & last = linker_->get_last_result();
  EXPECT_EQ(last.app_to_node.size(), 1);
}

// =============================================================================
// Namespace Matching Determinism Tests (Task 16)
// =============================================================================

TEST_F(RuntimeLinkerTest, NamespaceMatch_RejectsStringPrefix) {
  // "/nav" should NOT match node in "/navigation" namespace
  std::vector<App> apps = {create_app("nav_app", "navigator", "/nav")};
  std::vector<App> runtime_apps = {create_runtime_app("navigator", "/navigation")};

  auto result = linker_->link(apps, runtime_apps, config_);
  EXPECT_FALSE(result.linked_apps[0].is_online);
}

TEST_F(RuntimeLinkerTest, NamespaceMatch_AcceptsPathPrefix) {
  // "/nav" SHOULD match node in "/nav/sub" namespace
  std::vector<App> apps = {create_app("nav_app", "planner", "/nav")};
  std::vector<App> runtime_apps = {create_runtime_app("planner", "/nav/sub")};

  auto result = linker_->link(apps, runtime_apps, config_);
  EXPECT_TRUE(result.linked_apps[0].is_online);
}

TEST_F(RuntimeLinkerTest, NodeName_ExactLastSegmentOnly) {
  // Binding for "map" should NOT match node "map_server" (FQN contains "/map")
  std::vector<App> apps = {create_app("mapper", "map", "/")};
  std::vector<App> runtime_apps = {create_runtime_app("map_server", "/")};

  auto result = linker_->link(apps, runtime_apps, config_);
  EXPECT_FALSE(result.linked_apps[0].is_online);
}

TEST_F(RuntimeLinkerTest, Wildcard_DeterministicMultiMatch) {
  // Wildcard: two nodes match by name, deterministic winner (alphabetical FQN)
  std::vector<App> apps = {create_app("ctrl_app", "controller", "*")};
  std::vector<App> runtime_apps = {
      create_runtime_app("controller", "/beta"),
      create_runtime_app("controller", "/alpha"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);
  EXPECT_TRUE(result.linked_apps[0].is_online);
  // Deterministic: alphabetically first FQN wins
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/alpha/controller");
}

TEST_F(RuntimeLinkerTest, TopicNamespace_RejectsStringPrefix) {
  // Topic namespace "/state" should NOT match topic "/statement/data"
  auto app = create_topic_app("state_app", "/state");
  App rt_app = create_runtime_app("some_node", "/");
  rt_app.topics.publishes = {"/statement/data"};

  auto result = linker_->link({app}, {rt_app}, config_);
  EXPECT_FALSE(result.linked_apps[0].is_online);
}

TEST_F(RuntimeLinkerTest, TopicNamespace_AcceptsPathPrefix) {
  // Topic namespace "/state" SHOULD match topic "/state/machine"
  auto app = create_topic_app("state_app", "/state");
  App rt_app = create_runtime_app("some_node", "/");
  rt_app.topics.publishes = {"/state/machine"};

  auto result = linker_->link({app}, {rt_app}, config_);
  EXPECT_TRUE(result.linked_apps[0].is_online);
}

// =============================================================================
// Multi-match and Binding Conflict Tests (Task 17)
// =============================================================================

TEST_F(RuntimeLinkerTest, TwoAppsCompeteForSameNode) {
  // Two manifest apps bind to the same runtime node
  std::vector<App> apps = {
      create_app("app1", "controller", "/nav"),
      create_app("app2", "controller", "/nav"),
  };
  std::vector<App> runtime_apps = {create_runtime_app("controller", "/nav")};

  auto result = linker_->link(apps, runtime_apps, config_);

  // First app wins (insertion order = priority)
  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/nav/controller");

  // Second app is unlinked (node already taken)
  EXPECT_FALSE(result.linked_apps[1].is_online);
  EXPECT_EQ(result.unlinked_app_ids.size(), 1u);

  // Conflict reported
  EXPECT_EQ(result.binding_conflicts, 1u);
}

TEST_F(RuntimeLinkerTest, LinkingReportSummaryIncludesConflicts) {
  std::vector<App> apps = {
      create_app("app1", "controller", "/nav"),
      create_app("app2", "controller", "/nav"),
  };
  std::vector<App> runtime_apps = {create_runtime_app("controller", "/nav")};

  auto result = linker_->link(apps, runtime_apps, config_);
  auto summary = result.summary();
  EXPECT_TRUE(summary.find("conflict") != std::string::npos);
}

TEST_F(RuntimeLinkerTest, WildcardMultiMatchCounted) {
  std::vector<App> apps = {create_app("ctrl_app", "controller", "*")};
  std::vector<App> runtime_apps = {
      create_runtime_app("controller", "/alpha"),
      create_runtime_app("controller", "/beta"),
  };

  auto result = linker_->link(apps, runtime_apps, config_);
  EXPECT_EQ(result.wildcard_multi_match, 1u);
}

// =============================================================================
// Duplicate Suppression Tests (#307)
// =============================================================================

TEST_F(RuntimeLinkerTest, MergedInput_SuppressesRuntimeDuplicates) {
  // Bug #307: merge_entities() produces BOTH manifest and runtime apps for the
  // same node with different IDs. The merge pipeline passes the merged set as
  // the first arg to link(). After link(), linked_apps must NOT contain the
  // runtime-origin duplicates - only manifest-source apps should remain.

  // Manifest app with ros_binding targeting the node
  App manifest_app;
  manifest_app.id = "lidar-sim";
  manifest_app.name = "LiDAR Simulator";
  manifest_app.source = "manifest";
  manifest_app.ros_binding = App::RosBinding{"lidar_sim", "/sensors", ""};

  // Runtime app representing the same node (from runtime discovery, different ID)
  App runtime_origin_app;
  runtime_origin_app.id = "lidar_sim";
  runtime_origin_app.name = "lidar_sim";
  runtime_origin_app.source = "heuristic";
  runtime_origin_app.is_online = true;
  runtime_origin_app.bound_fqn = "/sensors/lidar_sim";
  runtime_origin_app.topics.publishes = {"/sensors/lidar_sim/scan"};

  // A second manifest app with a different binding
  App manifest_app2;
  manifest_app2.id = "imu-driver";
  manifest_app2.name = "IMU Driver";
  manifest_app2.source = "manifest";
  manifest_app2.ros_binding = App::RosBinding{"imu_node", "/sensors", ""};

  // Runtime app for the second node
  App runtime_origin_app2;
  runtime_origin_app2.id = "imu_node";
  runtime_origin_app2.name = "imu_node";
  runtime_origin_app2.source = "heuristic";
  runtime_origin_app2.is_online = true;
  runtime_origin_app2.bound_fqn = "/sensors/imu_node";

  // An orphan runtime app (no manifest counterpart) - should be preserved
  App orphan_runtime_app;
  orphan_runtime_app.id = "camera_node";
  orphan_runtime_app.name = "camera_node";
  orphan_runtime_app.source = "heuristic";
  orphan_runtime_app.is_online = true;
  orphan_runtime_app.bound_fqn = "/sensors/camera_node";

  // Simulate merge_pipeline: first arg is the merged set (manifest + runtime apps)
  // second arg is the raw runtime apps used for binding lookups
  std::vector<App> merged_apps = {manifest_app, runtime_origin_app, manifest_app2, runtime_origin_app2,
                                  orphan_runtime_app};
  std::vector<App> runtime_apps = {runtime_origin_app, runtime_origin_app2, orphan_runtime_app};

  auto result = linker_->link(merged_apps, runtime_apps, config_);

  // Both manifest apps should be linked
  EXPECT_EQ(result.app_to_node.size(), 2u);
  EXPECT_EQ(result.app_to_node["lidar-sim"], "/sensors/lidar_sim");
  EXPECT_EQ(result.app_to_node["imu-driver"], "/sensors/imu_node");

  // linked_apps should contain exactly 2 manifest apps + 1 orphan runtime app
  // The 2 runtime duplicates (lidar_sim, imu_node) whose bound_fqn matches
  // a linked manifest app must be suppressed.
  EXPECT_EQ(result.linked_apps.size(), 3u);

  // Verify: manifest apps are present and linked
  size_t manifest_count = 0;
  size_t orphan_count = 0;
  for (const auto & app : result.linked_apps) {
    if (app.source == "manifest") {
      manifest_count++;
      EXPECT_TRUE(app.is_online) << "Manifest app should be online: " << app.id;
    } else {
      orphan_count++;
      // The only non-manifest app should be the orphan (camera_node)
      EXPECT_EQ(app.id, "camera_node");
    }
  }
  EXPECT_EQ(manifest_count, 2u);
  EXPECT_EQ(orphan_count, 1u);

  // The orphan runtime app should be detected as orphan node
  EXPECT_EQ(result.orphan_nodes.size(), 1u);
  EXPECT_EQ(result.orphan_nodes[0], "/sensors/camera_node");
}

TEST_F(RuntimeLinkerTest, MergedInput_PreservesOrphanRuntimeApps) {
  // When runtime apps don't match any manifest app's binding, they should NOT
  // be suppressed (they are genuine orphans, not duplicates).

  App manifest_app;
  manifest_app.id = "nav-controller";
  manifest_app.name = "Navigation Controller";
  manifest_app.source = "manifest";
  manifest_app.ros_binding = App::RosBinding{"controller", "/nav", ""};

  // Runtime app matching the manifest binding (duplicate - should be suppressed)
  App rt_matching;
  rt_matching.id = "controller";
  rt_matching.name = "controller";
  rt_matching.source = "heuristic";
  rt_matching.is_online = true;
  rt_matching.bound_fqn = "/nav/controller";

  // Runtime app NOT matching any manifest binding (orphan - should be preserved)
  App rt_orphan;
  rt_orphan.id = "planner";
  rt_orphan.name = "planner";
  rt_orphan.source = "heuristic";
  rt_orphan.is_online = true;
  rt_orphan.bound_fqn = "/nav/planner";

  // Simulate merge_pipeline: first arg is merged set
  std::vector<App> merged_apps = {manifest_app, rt_matching, rt_orphan};
  std::vector<App> runtime_apps = {rt_matching, rt_orphan};

  auto result = linker_->link(merged_apps, runtime_apps, config_);

  // Manifest app linked
  EXPECT_EQ(result.app_to_node.size(), 1u);

  // linked_apps: 1 manifest app + 1 orphan runtime app (duplicate suppressed)
  EXPECT_EQ(result.linked_apps.size(), 2u);

  // Verify the manifest app is present and linked
  bool found_manifest = false;
  bool found_orphan = false;
  for (const auto & app : result.linked_apps) {
    if (app.id == "nav-controller") {
      found_manifest = true;
      EXPECT_EQ(app.source, "manifest");
      EXPECT_TRUE(app.is_online);
    } else if (app.id == "planner") {
      found_orphan = true;
      EXPECT_EQ(app.source, "heuristic");
    }
  }
  EXPECT_TRUE(found_manifest) << "Manifest app nav-controller not found in linked_apps";
  EXPECT_TRUE(found_orphan) << "Orphan app planner not found in linked_apps";

  // Orphan still detected in orphan_nodes
  EXPECT_EQ(result.orphan_nodes.size(), 1u);
  EXPECT_EQ(result.orphan_nodes[0], "/nav/planner");
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
