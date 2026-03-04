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

#include "ros2_medkit_gateway/discovery/discovery_layer.hpp"
#include "ros2_medkit_gateway/discovery/layers/manifest_layer.hpp"
#include "ros2_medkit_gateway/discovery/layers/plugin_layer.hpp"
#include "ros2_medkit_gateway/discovery/layers/runtime_layer.hpp"
#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"
#include "ros2_medkit_gateway/discovery/merge_pipeline.hpp"
#include "ros2_medkit_gateway/discovery/merge_types.hpp"

#include <gtest/gtest.h>

#include <algorithm>

using namespace ros2_medkit_gateway::discovery;
using namespace ros2_medkit_gateway;

// Concrete test layer for unit testing
class TestLayer : public DiscoveryLayer {
 public:
  TestLayer(std::string name, LayerOutput output, std::unordered_map<FieldGroup, MergePolicy> policies = {})
    : name_(std::move(name)), output_(std::move(output)), policies_(std::move(policies)) {
  }

  std::string name() const override {
    return name_;
  }
  LayerOutput discover() override {
    return output_;
  }

  MergePolicy policy_for(FieldGroup group) const override {
    auto it = policies_.find(group);
    if (it != policies_.end()) {
      return it->second;
    }
    return MergePolicy::ENRICHMENT;  // default
  }

 private:
  std::string name_;
  LayerOutput output_;
  std::unordered_map<FieldGroup, MergePolicy> policies_;
};

TEST(MergeTypesTest, MergePolicyValues) {
  // Verify enum values exist and are distinct
  EXPECT_NE(static_cast<int>(MergePolicy::AUTHORITATIVE), static_cast<int>(MergePolicy::ENRICHMENT));
  EXPECT_NE(static_cast<int>(MergePolicy::ENRICHMENT), static_cast<int>(MergePolicy::FALLBACK));
}

TEST(MergeTypesTest, FieldGroupValues) {
  // Verify all 5 field groups exist
  EXPECT_NE(static_cast<int>(FieldGroup::IDENTITY), static_cast<int>(FieldGroup::HIERARCHY));
  EXPECT_NE(static_cast<int>(FieldGroup::LIVE_DATA), static_cast<int>(FieldGroup::STATUS));
  auto metadata = FieldGroup::METADATA;
  (void)metadata;
}

TEST(MergeTypesTest, MergeReportDefaultEmpty) {
  MergeReport report;
  EXPECT_TRUE(report.conflicts.empty());
  EXPECT_TRUE(report.entity_source.empty());
  EXPECT_EQ(report.total_entities, 0u);
  EXPECT_EQ(report.enriched_count, 0u);
  EXPECT_EQ(report.conflict_count, 0u);
  EXPECT_EQ(report.id_collision_count, 0u);
}

TEST(MergeTypesTest, MergeReportToJson) {
  MergeReport report;
  report.layers = {"manifest", "runtime"};
  report.total_entities = 10;
  report.enriched_count = 7;
  report.conflict_count = 2;
  report.id_collision_count = 0;

  auto j = report.to_json();
  EXPECT_EQ(j["total_entities"], 10);
  EXPECT_EQ(j["enriched_count"], 7);
  EXPECT_EQ(j["conflict_count"], 2);
  EXPECT_EQ(j["id_collisions"], 0);
  EXPECT_EQ(j["layers"].size(), 2u);
}

TEST(DiscoveryLayerTest, TestLayerReturnsConfiguredOutput) {
  LayerOutput output;
  Area area;
  area.id = "powertrain";
  area.name = "Powertrain";
  output.areas.push_back(area);

  TestLayer layer("test", output);
  EXPECT_EQ(layer.name(), "test");

  auto result = layer.discover();
  ASSERT_EQ(result.areas.size(), 1u);
  EXPECT_EQ(result.areas[0].id, "powertrain");
}

TEST(DiscoveryLayerTest, PolicyForReturnsConfiguredPolicy) {
  TestLayer layer("test", {},
                  {{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE}, {FieldGroup::LIVE_DATA, MergePolicy::FALLBACK}});

  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::LIVE_DATA), MergePolicy::FALLBACK);
  EXPECT_EQ(layer.policy_for(FieldGroup::STATUS), MergePolicy::ENRICHMENT);  // default
}

// Helper to create test entities
namespace {

Area make_area(const std::string & id, const std::string & name = "") {
  Area a;
  a.id = id;
  a.name = name.empty() ? id : name;
  a.namespace_path = "/" + id;
  return a;
}

Component make_component(const std::string & id, const std::string & area = "", const std::string & ns = "/") {
  Component c;
  c.id = id;
  c.name = id;
  c.area = area;
  c.namespace_path = ns;
  c.fqn = ns == "/" ? "/" + id : ns + "/" + id;
  c.source = "test";
  return c;
}

App make_app(const std::string & id, const std::string & component_id = "") {
  App a;
  a.id = id;
  a.name = id;
  a.component_id = component_id;
  a.source = "test";
  return a;
}

}  // namespace

class MergePipelineTest : public ::testing::Test {
 protected:
  MergePipeline pipeline_;
};

TEST_F(MergePipelineTest, EmptyPipelineReturnsEmptyResult) {
  auto result = pipeline_.execute();
  EXPECT_TRUE(result.areas.empty());
  EXPECT_TRUE(result.components.empty());
  EXPECT_TRUE(result.apps.empty());
  EXPECT_TRUE(result.functions.empty());
  EXPECT_EQ(result.report.total_entities, 0u);
}

TEST_F(MergePipelineTest, SingleLayerPassthrough) {
  LayerOutput output;
  output.areas.push_back(make_area("powertrain"));
  output.components.push_back(make_component("engine", "powertrain", "/powertrain"));
  output.apps.push_back(make_app("engine_app", "engine"));

  pipeline_.add_layer(std::make_unique<TestLayer>("manifest", output));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.areas.size(), 1u);
  EXPECT_EQ(result.areas[0].id, "powertrain");
  ASSERT_EQ(result.components.size(), 1u);
  EXPECT_EQ(result.components[0].id, "engine");
  ASSERT_EQ(result.apps.size(), 1u);
  EXPECT_EQ(result.apps[0].id, "engine_app");
  EXPECT_EQ(result.report.total_entities, 3u);
  EXPECT_EQ(result.report.conflict_count, 0u);
}

TEST_F(MergePipelineTest, MultipleLayersDisjointEntities) {
  LayerOutput manifest_output;
  manifest_output.areas.push_back(make_area("powertrain"));

  LayerOutput runtime_output;
  runtime_output.areas.push_back(make_area("chassis"));

  pipeline_.add_layer(std::make_unique<TestLayer>("manifest", manifest_output));
  pipeline_.add_layer(std::make_unique<TestLayer>("runtime", runtime_output));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.areas.size(), 2u);
  EXPECT_EQ(result.report.total_entities, 2u);
  EXPECT_EQ(result.report.conflict_count, 0u);
}

TEST_F(MergePipelineTest, AuthoritativeWinsOverEnrichment) {
  // Manifest: IDENTITY=AUTH, LIVE_DATA=ENRICH
  // Runtime: IDENTITY=FALLBACK, LIVE_DATA=AUTH
  // Same component in both - manifest name wins, runtime topics win

  Component manifest_comp = make_component("engine", "powertrain", "/powertrain");
  manifest_comp.name = "Engine ECU";
  manifest_comp.source = "manifest";

  Component runtime_comp = make_component("engine", "powertrain", "/powertrain");
  runtime_comp.name = "engine";
  runtime_comp.source = "node";
  runtime_comp.topics.publishes = {"/powertrain/engine/rpm"};

  LayerOutput manifest_out;
  manifest_out.components.push_back(manifest_comp);

  LayerOutput runtime_out;
  runtime_out.components.push_back(runtime_comp);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT}}));

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", runtime_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);
  EXPECT_EQ(result.components[0].name, "Engine ECU");           // manifest IDENTITY wins
  EXPECT_EQ(result.components[0].topics.publishes.size(), 1u);  // runtime LIVE_DATA wins
  EXPECT_EQ(result.components[0].source, "manifest");           // higher priority source
}

TEST_F(MergePipelineTest, EnrichmentFillsEmptyFields) {
  // Manifest has name but no topics
  // Runtime has topics
  // Both declare ENRICHMENT for LIVE_DATA
  // Result: topics filled from runtime

  Component manifest_comp = make_component("engine", "powertrain", "/powertrain");
  manifest_comp.source = "manifest";

  Component runtime_comp = make_component("engine", "powertrain", "/powertrain");
  runtime_comp.topics.publishes = {"/powertrain/engine/rpm"};
  runtime_comp.topics.subscribes = {"/powertrain/engine/throttle"};

  LayerOutput manifest_out;
  manifest_out.components.push_back(manifest_comp);

  LayerOutput runtime_out;
  runtime_out.components.push_back(runtime_comp);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", runtime_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);
  EXPECT_FALSE(result.components[0].topics.publishes.empty());
}

TEST_F(MergePipelineTest, AuthoritativeVsAuthoritativeHigherPriorityWins) {
  // Both layers claim AUTHORITATIVE for IDENTITY
  // Higher priority (first added) wins, conflict logged

  Component manifest_comp = make_component("engine", "powertrain", "/powertrain");
  manifest_comp.name = "Manifest Engine";

  Component runtime_comp = make_component("engine", "powertrain", "/powertrain");
  runtime_comp.name = "Runtime Engine";

  LayerOutput manifest_out;
  manifest_out.components.push_back(manifest_comp);
  LayerOutput runtime_out;
  runtime_out.components.push_back(runtime_comp);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", runtime_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);
  EXPECT_EQ(result.components[0].name, "Manifest Engine");  // higher priority wins
  EXPECT_GE(result.report.conflict_count, 1u);              // conflict recorded
}

TEST_F(MergePipelineTest, CollectionFieldsUnionOnEnrichment) {
  // Both layers provide services for the same component with ENRICHMENT
  // Result: union of services (deduped by full_path)

  Component layer1_comp = make_component("engine", "", "/powertrain");
  layer1_comp.services.push_back(
      ServiceInfo{"calibrate", "/powertrain/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt});

  Component layer2_comp = make_component("engine", "", "/powertrain");
  layer2_comp.services.push_back(
      ServiceInfo{"calibrate", "/powertrain/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt});
  layer2_comp.services.push_back(
      ServiceInfo{"reset", "/powertrain/engine/reset", "std_srvs/srv/Trigger", std::nullopt});

  LayerOutput out1, out2;
  out1.components.push_back(layer1_comp);
  out2.components.push_back(layer2_comp);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "layer1", out1, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "layer2", out2, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);
  // Union: calibrate (deduped) + reset = 2 services
  EXPECT_EQ(result.components[0].services.size(), 2u);
}

// --- ManifestLayer and RuntimeLayer tests ---

TEST(ManifestLayerTest, DefaultPolicies) {
  ManifestLayer layer(nullptr);
  EXPECT_EQ(layer.name(), "manifest");
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::HIERARCHY), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::LIVE_DATA), MergePolicy::ENRICHMENT);
  EXPECT_EQ(layer.policy_for(FieldGroup::STATUS), MergePolicy::FALLBACK);
  EXPECT_EQ(layer.policy_for(FieldGroup::METADATA), MergePolicy::AUTHORITATIVE);
}

TEST(ManifestLayerTest, PolicyOverride) {
  ManifestLayer layer(nullptr);
  layer.set_policy(FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::LIVE_DATA), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::AUTHORITATIVE);
}

TEST(ManifestLayerTest, DiscoverReturnsEmptyWhenNoManifest) {
  ManifestLayer layer(nullptr);
  auto output = layer.discover();
  EXPECT_TRUE(output.areas.empty());
  EXPECT_TRUE(output.components.empty());
  EXPECT_TRUE(output.apps.empty());
  EXPECT_TRUE(output.functions.empty());
}

TEST(RuntimeLayerTest, DefaultPolicies) {
  RuntimeLayer layer(nullptr);
  EXPECT_EQ(layer.name(), "runtime");
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::FALLBACK);
  EXPECT_EQ(layer.policy_for(FieldGroup::HIERARCHY), MergePolicy::FALLBACK);
  EXPECT_EQ(layer.policy_for(FieldGroup::LIVE_DATA), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::STATUS), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::METADATA), MergePolicy::ENRICHMENT);
}

TEST(RuntimeLayerTest, DiscoverReturnsEmptyWhenNoStrategy) {
  RuntimeLayer layer(nullptr);
  auto output = layer.discover();
  EXPECT_TRUE(output.areas.empty());
  EXPECT_TRUE(output.components.empty());
}

// --- PluginLayer tests ---

class MockIntrospectionProvider : public IntrospectionProvider {
 public:
  IntrospectionResult introspect(const IntrospectionInput & input) override {
    (void)input;
    return result_;
  }

  IntrospectionResult result_;
};

TEST(PluginLayerTest, DefaultPolicies) {
  auto provider = std::make_shared<MockIntrospectionProvider>();
  PluginLayer layer("lidar_mapper", provider.get());
  EXPECT_EQ(layer.name(), "lidar_mapper");
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::ENRICHMENT);
  EXPECT_EQ(layer.policy_for(FieldGroup::METADATA), MergePolicy::AUTHORITATIVE);
}

TEST(PluginLayerTest, MapsNewEntitiesToLayerOutput) {
  auto provider = std::make_shared<MockIntrospectionProvider>();

  Component new_comp;
  new_comp.id = "lidar_unit";
  new_comp.name = "LiDAR Processing Unit";
  new_comp.source = "plugin";
  provider->result_.new_entities.components.push_back(new_comp);

  PluginLayer layer("lidar_mapper", provider.get());
  auto output = layer.discover();
  ASSERT_EQ(output.components.size(), 1u);
  EXPECT_EQ(output.components[0].id, "lidar_unit");
}

TEST(PluginLayerTest, MetadataPassedThrough) {
  auto provider = std::make_shared<MockIntrospectionProvider>();
  provider->result_.metadata["engine"] = {{"x-medkit-temperature", 85}};

  PluginLayer layer("sensor_plugin", provider.get());
  auto output = layer.discover();
  ASSERT_EQ(output.entity_metadata.size(), 1u);
  EXPECT_TRUE(output.entity_metadata.count("engine"));
  EXPECT_EQ(layer.get_metadata().at("engine")["x-medkit-temperature"], 85);
}

TEST(PluginLayerTest, DiscoverReturnsEmptyWhenNoProvider) {
  PluginLayer layer("broken_plugin", nullptr);
  auto output = layer.discover();
  EXPECT_TRUE(output.areas.empty());
  EXPECT_TRUE(output.components.empty());
  EXPECT_TRUE(output.apps.empty());
  EXPECT_TRUE(output.entity_metadata.empty());
}

// --- GapFillConfig tests ---

TEST(GapFillConfigTest, DefaultAllowsAll) {
  GapFillConfig config;
  EXPECT_TRUE(config.allow_heuristic_areas);
  EXPECT_TRUE(config.allow_heuristic_components);
  EXPECT_TRUE(config.allow_heuristic_apps);
  EXPECT_FALSE(config.allow_heuristic_functions);
}

TEST(GapFillConfigTest, NamespaceBlacklist) {
  GapFillConfig config;
  config.namespace_blacklist = {"/rosout", "/parameter_events"};
  EXPECT_EQ(config.namespace_blacklist.size(), 2u);
  EXPECT_EQ(config.namespace_blacklist[0], "/rosout");
}

TEST(RuntimeLayerTest, GapFillFilterBlocksApps) {
  GapFillConfig gap_fill;
  gap_fill.allow_heuristic_apps = false;

  RuntimeLayer layer(nullptr);
  layer.set_gap_fill_config(gap_fill);
  auto output = layer.discover();
  EXPECT_TRUE(output.apps.empty());
}

// --- Post-merge linking tests ---

TEST_F(MergePipelineTest, PostMergeLinkingSetsAppOnlineStatus) {
  // Manifest provides app with ros_binding
  App manifest_app = make_app("controller_app", "nav_comp");
  manifest_app.source = "manifest";
  App::RosBinding binding;
  binding.node_name = "controller";
  binding.namespace_pattern = "/nav";
  manifest_app.ros_binding = binding;

  // Runtime provides matching app (node)
  App runtime_node;
  runtime_node.id = "controller";
  runtime_node.name = "controller";
  runtime_node.source = "heuristic";
  runtime_node.is_online = true;
  runtime_node.bound_fqn = "/nav/controller";

  LayerOutput manifest_out;
  manifest_out.apps.push_back(manifest_app);

  LayerOutput runtime_out;
  runtime_out.apps.push_back(runtime_node);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::STATUS, MergePolicy::FALLBACK}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", runtime_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::STATUS, MergePolicy::AUTHORITATIVE}}));

  // Enable linking
  ManifestConfig manifest_config;
  pipeline_.set_linker(std::make_unique<RuntimeLinker>(nullptr), manifest_config);

  auto result = pipeline_.execute();
  // Both apps in result (different IDs). Linker matches controller_app's binding
  // to runtime controller's bound_fqn.
  auto it = std::find_if(result.apps.begin(), result.apps.end(), [](const App & a) {
    return a.id == "controller_app";
  });
  ASSERT_NE(it, result.apps.end());
  EXPECT_TRUE(it->is_online);
  EXPECT_EQ(it->bound_fqn, "/nav/controller");
}

TEST_F(MergePipelineTest, PostMergeLinkingReportsOrphanNodes) {
  // Runtime provides an app (node) with no matching manifest app
  App orphan_node;
  orphan_node.id = "orphan_node";
  orphan_node.name = "orphan_node";
  orphan_node.source = "heuristic";
  orphan_node.is_online = true;
  orphan_node.bound_fqn = "/test/orphan_node";

  LayerOutput runtime_out;
  runtime_out.apps.push_back(orphan_node);

  pipeline_.add_layer(std::make_unique<TestLayer>("runtime", runtime_out));

  ManifestConfig manifest_config;
  pipeline_.set_linker(std::make_unique<RuntimeLinker>(nullptr), manifest_config);

  auto result = pipeline_.execute();
  auto & linking = pipeline_.get_linking_result();
  EXPECT_FALSE(linking.orphan_nodes.empty());
}
