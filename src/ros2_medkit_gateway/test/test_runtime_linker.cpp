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

  // Helper to create a test Component (representing a runtime node)
  Component create_component(const std::string & id, const std::string & ns = "/") {
    Component comp;
    comp.id = id;
    comp.name = id;
    comp.namespace_path = ns;
    comp.fqn = ns == "/" ? "/" + id : ns + "/" + id;
    comp.source = "node";
    return comp;
  }

  std::unique_ptr<RuntimeLinker> linker_;
  ManifestConfig config_;
};

// =============================================================================
// Exact Match Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, ExactMatch_NodeNameAndNamespace) {
  std::vector<App> apps = {create_app("controller_app", "controller", "/nav")};
  std::vector<Component> components = {create_component("controller", "/nav")};

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {create_component("my_node", "/")};

  auto result = linker_->link(apps, components, config_);

  EXPECT_EQ(result.linked_apps.size(), 1);
  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/my_node");
}

TEST_F(RuntimeLinkerTest, NoMatch_DifferentNodeName) {
  std::vector<App> apps = {create_app("app1", "controller", "/nav")};
  std::vector<Component> components = {create_component("planner", "/nav")};

  auto result = linker_->link(apps, components, config_);

  EXPECT_EQ(result.linked_apps.size(), 1);
  EXPECT_FALSE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.unlinked_app_ids.size(), 1);
  EXPECT_EQ(result.unlinked_app_ids[0], "app1");
  EXPECT_EQ(result.orphan_nodes.size(), 1);
}

TEST_F(RuntimeLinkerTest, NoMatch_DifferentNamespace) {
  std::vector<App> apps = {create_app("app1", "controller", "/navigation")};
  std::vector<Component> components = {create_component("controller", "/planning")};

  auto result = linker_->link(apps, components, config_);

  EXPECT_FALSE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.unlinked_app_ids.size(), 1);
}

// =============================================================================
// Wildcard Namespace Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, WildcardNamespace_MatchesAny) {
  std::vector<App> apps = {create_app("app1", "controller", "*")};
  std::vector<Component> components = {
      create_component("controller", "/ns1"),
      create_component("controller", "/ns2"),
  };

  auto result = linker_->link(apps, components, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  // Should match the first one found
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/ns1/controller");
  // Second node should be orphan since app is already matched
  EXPECT_EQ(result.orphan_nodes.size(), 1);
}

TEST_F(RuntimeLinkerTest, WildcardNamespace_MatchesRootNamespace) {
  std::vector<App> apps = {create_app("app1", "my_node", "*")};
  std::vector<Component> components = {create_component("my_node", "/")};

  auto result = linker_->link(apps, components, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].bound_fqn, "/my_node");
}

TEST_F(RuntimeLinkerTest, WildcardNamespace_MatchesNestedNamespace) {
  std::vector<App> apps = {create_app("app1", "controller", "*")};
  std::vector<Component> components = {create_component("controller", "/robot/nav/local")};

  auto result = linker_->link(apps, components, config_);

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

  Component comp = create_component("sensor_driver", "/");
  comp.topics.publishes = {"/sensor_data/imu", "/sensor_data/gps"};

  std::vector<App> apps = {app};
  std::vector<Component> components = {comp};

  auto result = linker_->link(apps, components, config_);

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

  Component comp = create_component("motor_driver", "/");
  comp.topics.subscribes = {"/cmd/velocity", "/cmd/position"};

  std::vector<App> apps = {app};
  std::vector<Component> components = {comp};

  auto result = linker_->link(apps, components, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
}

TEST_F(RuntimeLinkerTest, TopicNamespace_NoMatch) {
  App app;
  app.id = "app1";
  app.name = "App 1";
  App::RosBinding binding;
  binding.topic_namespace = "/navigation";
  app.ros_binding = binding;

  Component comp = create_component("sensor_driver", "/");
  comp.topics.publishes = {"/sensor/imu"};

  std::vector<App> apps = {app};
  std::vector<Component> components = {comp};

  auto result = linker_->link(apps, components, config_);

  EXPECT_FALSE(result.linked_apps[0].is_online);
}

// =============================================================================
// Orphan Node Detection Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, OrphanNodes_DetectedCorrectly) {
  std::vector<App> apps = {create_app("app1", "controller", "/nav")};
  std::vector<Component> components = {
      create_component("controller", "/nav"),
      create_component("planner", "/nav"),
      create_component("mapper", "/map"),
  };

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {
      create_component("node1", "/"),
      create_component("node2", "/"),
  };

  auto result = linker_->link(apps, components, config_);

  EXPECT_TRUE(result.orphan_nodes.empty());
  EXPECT_EQ(result.app_to_node.size(), 2);
}

// =============================================================================
// Orphan Policy Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, OrphanPolicy_Error_ReportsError) {
  config_.unmanifested_nodes = ManifestConfig::UnmanifestedNodePolicy::ERROR;

  std::vector<App> apps = {};
  std::vector<Component> components = {create_component("orphan_node", "/")};

  auto result = linker_->link(apps, components, config_);

  EXPECT_TRUE(result.has_errors(config_.unmanifested_nodes));
  EXPECT_EQ(result.orphan_nodes.size(), 1);
}

TEST_F(RuntimeLinkerTest, OrphanPolicy_Ignore_NoError) {
  config_.unmanifested_nodes = ManifestConfig::UnmanifestedNodePolicy::IGNORE;

  std::vector<App> apps = {};
  std::vector<Component> components = {create_component("orphan_node", "/")};

  auto result = linker_->link(apps, components, config_);

  EXPECT_FALSE(result.has_errors(config_.unmanifested_nodes));
}

// =============================================================================
// App Enrichment Tests
// =============================================================================

TEST_F(RuntimeLinkerTest, EnrichApp_CopiesTopics) {
  std::vector<App> apps = {create_app("app1", "sensor", "*")};

  Component comp = create_component("sensor", "/");
  comp.topics.publishes = {"/sensor/data", "/sensor/status"};
  comp.topics.subscribes = {"/sensor/config"};

  std::vector<Component> components = {comp};

  auto result = linker_->link(apps, components, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].topics.publishes.size(), 2);
  EXPECT_EQ(result.linked_apps[0].topics.subscribes.size(), 1);
}

TEST_F(RuntimeLinkerTest, EnrichApp_CopiesServices) {
  std::vector<App> apps = {create_app("app1", "server", "*")};

  Component comp = create_component("server", "/");
  comp.services = {
      {"srv1", "/srv1", "std_srvs/srv/Trigger", std::nullopt},
      {"srv2", "/srv2", "std_srvs/srv/Empty", std::nullopt}};

  std::vector<Component> components = {comp};

  auto result = linker_->link(apps, components, config_);

  EXPECT_TRUE(result.linked_apps[0].is_online);
  EXPECT_EQ(result.linked_apps[0].services.size(), 2);
}

TEST_F(RuntimeLinkerTest, EnrichApp_CopiesActions) {
  std::vector<App> apps = {create_app("app1", "action_server", "*")};

  Component comp = create_component("action_server", "/");
  comp.actions = {{"nav", "/nav", "nav2_msgs/action/NavigateToPose", std::nullopt}};

  std::vector<Component> components = {comp};

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {create_component("some_node", "/")};

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {create_component("some_node", "/")};

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {create_component("some_node", "/")};

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {create_component("online_node", "/")};

  linker_->link(apps, components, config_);

  EXPECT_TRUE(linker_->is_app_online("online_app"));
  EXPECT_FALSE(linker_->is_app_online("offline_app"));
  EXPECT_FALSE(linker_->is_app_online("nonexistent_app"));
}

TEST_F(RuntimeLinkerTest, GetBoundNode_ReturnsCorrectFqn) {
  std::vector<App> apps = {create_app("app1", "my_node", "/ns")};
  std::vector<Component> components = {create_component("my_node", "/ns")};

  linker_->link(apps, components, config_);

  auto bound = linker_->get_bound_node("app1");
  ASSERT_TRUE(bound.has_value());
  EXPECT_EQ(bound.value(), "/ns/my_node");

  EXPECT_FALSE(linker_->get_bound_node("nonexistent").has_value());
}

TEST_F(RuntimeLinkerTest, GetAppForNode_ReturnsCorrectId) {
  std::vector<App> apps = {create_app("app1", "my_node", "/ns")};
  std::vector<Component> components = {create_component("my_node", "/ns")};

  linker_->link(apps, components, config_);

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
  std::vector<Component> components = {
      create_component("node1", "/"),
      create_component("node2", "/"),
      create_component("node3", "/"),
  };

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {
      create_component("node1", "/"),
      create_component("node3", "/"),
  };

  auto result = linker_->link(apps, components, config_);

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
  std::vector<Component> components = {
      create_component("node1", "/"),
      create_component("orphan", "/"),
  };

  auto result = linker_->link(apps, components, config_);

  std::string summary = result.summary();
  EXPECT_TRUE(summary.find("1 linked") != std::string::npos);
  EXPECT_TRUE(summary.find("1 unlinked") != std::string::npos);
  EXPECT_TRUE(summary.find("1 orphan") != std::string::npos);
}

TEST_F(RuntimeLinkerTest, GetLastResult_ReturnsLatest) {
  std::vector<App> apps = {create_app("app1", "node1", "*")};
  std::vector<Component> components = {create_component("node1", "/")};

  linker_->link(apps, components, config_);

  const auto & last = linker_->get_last_result();
  EXPECT_EQ(last.app_to_node.size(), 1);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
