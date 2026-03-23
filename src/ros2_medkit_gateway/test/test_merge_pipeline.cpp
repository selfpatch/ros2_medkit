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
  bool provides_runtime_apps() const override {
    return name_ == "runtime";
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

// @verifies REQ_INTEROP_003
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

Function make_function(const std::string & id, const std::string & name = "") {
  Function f;
  f.id = id;
  f.name = name.empty() ? id : name;
  return f;
}

}  // namespace

// @verifies REQ_INTEROP_003
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

// @verifies REQ_INTEROP_003
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

// @verifies REQ_INTEROP_003
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
    last_input_ = input;
    introspect_called_ = true;
    return result_;
  }

  IntrospectionResult result_;
  IntrospectionInput last_input_;
  bool introspect_called_{false};
};

// @verifies REQ_INTEROP_003
TEST(PluginLayerTest, DefaultPolicies) {
  auto provider = std::make_shared<MockIntrospectionProvider>();
  PluginLayer layer("lidar_mapper", provider.get());
  EXPECT_EQ(layer.name(), "lidar_mapper");
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::ENRICHMENT);
  EXPECT_EQ(layer.policy_for(FieldGroup::METADATA), MergePolicy::ENRICHMENT);
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

// @verifies REQ_INTEROP_003
TEST_F(MergePipelineTest, PluginReceivesDiscoveryContext) {
  // Manifest layer provides a component
  LayerOutput manifest_out;
  manifest_out.components.push_back(make_component("engine", "powertrain", "/powertrain"));
  manifest_out.apps.push_back(make_app("engine_ecu", "engine"));
  pipeline_.add_layer(std::make_unique<TestLayer>("manifest", manifest_out));

  // Runtime layer provides an area
  LayerOutput runtime_out;
  runtime_out.areas.push_back(make_area("powertrain"));
  pipeline_.add_layer(std::make_unique<TestLayer>("runtime", runtime_out));

  // Plugin layer should see entities from manifest + runtime
  auto provider = std::make_shared<MockIntrospectionProvider>();
  auto plugin = std::make_unique<PluginLayer>("test_plugin", provider.get());
  pipeline_.add_layer(std::move(plugin));

  pipeline_.execute();

  // Plugin's introspect() should have received the context from previous layers
  ASSERT_TRUE(provider->introspect_called_);
  EXPECT_EQ(provider->last_input_.components.size(), 1u);
  EXPECT_EQ(provider->last_input_.components[0].id, "engine");
  EXPECT_EQ(provider->last_input_.apps.size(), 1u);
  EXPECT_EQ(provider->last_input_.apps[0].id, "engine_ecu");
  EXPECT_EQ(provider->last_input_.areas.size(), 1u);
  EXPECT_EQ(provider->last_input_.areas[0].id, "powertrain");
}

TEST_F(MergePipelineTest, PluginLayerMapsFunctionsThroughPipeline) {
  auto provider = std::make_shared<MockIntrospectionProvider>();
  Function func;
  func.id = "engine-monitoring";
  func.name = "Engine Monitoring";
  func.hosts = {"engine_temp_sensor"};
  provider->result_.new_entities.functions = {func};

  auto plugin_layer = std::make_unique<PluginLayer>("test_beacon", provider.get());
  pipeline_.add_layer(std::move(plugin_layer));

  auto result = pipeline_.execute();
  auto functions = result.functions;
  ASSERT_EQ(functions.size(), 1u);
  EXPECT_EQ(functions[0].id, "engine-monitoring");
  EXPECT_EQ(functions[0].hosts.size(), 1u);
}

TEST_F(MergePipelineTest, PluginFunctionEnrichesExistingFunction) {
  // Manifest layer provides a function with identity only
  Function manifest_func = make_function("engine-monitoring", "Engine Monitoring");
  manifest_func.source = "manifest";

  LayerOutput manifest_out;
  manifest_out.functions.push_back(manifest_func);
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::HIERARCHY, MergePolicy::ENRICHMENT}}));

  // Plugin layer enriches the same function with hosts
  auto provider = std::make_shared<MockIntrospectionProvider>();
  Function plugin_func;
  plugin_func.id = "engine-monitoring";
  plugin_func.name = "engine-monitoring-plugin";
  plugin_func.hosts = {"engine_temp_sensor", "oil_pressure_sensor"};
  plugin_func.source = "plugin";
  provider->result_.new_entities.functions = {plugin_func};

  auto plugin_layer = std::make_unique<PluginLayer>("test_beacon", provider.get());
  pipeline_.add_layer(std::move(plugin_layer));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.functions.size(), 1u);
  // AUTHORITATIVE manifest identity wins
  EXPECT_EQ(result.functions[0].name, "Engine Monitoring");
  // ENRICHMENT from plugin fills hosts
  EXPECT_EQ(result.functions[0].hosts.size(), 2u);
}

// --- GapFillConfig tests ---

// @verifies REQ_INTEROP_003
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

TEST(MergeReportTest, FilteredByGapFillInJson) {
  MergeReport report;
  report.layers = {"manifest", "runtime"};
  report.total_entities = 5;
  report.filtered_by_gap_fill = 3;

  auto j = report.to_json();
  EXPECT_EQ(j["filtered_by_gap_fill"], 3);
}

TEST(RuntimeLayerTest, FilteredCountTracked) {
  // RuntimeLayer with no strategy returns 0 filtered
  RuntimeLayer layer(nullptr);
  auto output = layer.discover();
  EXPECT_EQ(layer.last_filtered_count(), 0u);
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
  auto linking = pipeline_.get_linking_result();
  EXPECT_FALSE(linking.orphan_nodes.empty());
}

// --- M4: Cross-type entity ID collision detection ---

TEST_F(MergePipelineTest, CrossTypeIdCollisionDetected) {
  // Same ID used for both an Area and a Component - should detect collision
  LayerOutput output;
  output.areas.push_back(make_area("shared_id"));
  output.components.push_back(make_component("shared_id"));

  pipeline_.add_layer(std::make_unique<TestLayer>("layer1", output));

  auto result = pipeline_.execute();
  EXPECT_EQ(result.report.id_collision_count, 1u);
}

TEST_F(MergePipelineTest, SameTypeIdIsNotCollision) {
  // Same ID in two layers of the same type - normal merge, not a collision
  LayerOutput out1;
  out1.areas.push_back(make_area("powertrain"));
  LayerOutput out2;
  out2.areas.push_back(make_area("powertrain"));

  pipeline_.add_layer(std::make_unique<TestLayer>("layer1", out1));
  pipeline_.add_layer(std::make_unique<TestLayer>("layer2", out2));

  auto result = pipeline_.execute();
  EXPECT_EQ(result.report.id_collision_count, 0u);
  EXPECT_EQ(result.areas.size(), 1u);  // merged, not duplicated
}

// --- M5: FALLBACK policy behavior ---

TEST_F(MergePipelineTest, FallbackVsFallbackKeepsTargetWhenBothHaveData) {
  // Both layers declare FALLBACK for IDENTITY, both have name set
  // Higher priority target keeps its value (first non-empty)
  Component comp1 = make_component("engine", "powertrain", "/powertrain");
  comp1.name = "First Layer Engine";

  Component comp2 = make_component("engine", "powertrain", "/powertrain");
  comp2.name = "Second Layer Engine";

  LayerOutput out1, out2;
  out1.components.push_back(comp1);
  out2.components.push_back(comp2);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "layer1", out1, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "layer2", out2, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);
  EXPECT_EQ(result.components[0].name, "First Layer Engine");
  EXPECT_EQ(result.report.conflict_count, 0u);
}

TEST_F(MergePipelineTest, FallbackVsFallbackFillsEmptyFields) {
  // Target has empty description, source has it - FALLBACK should fill the gap
  Component comp1 = make_component("engine", "powertrain", "/powertrain");
  comp1.description = "";

  Component comp2 = make_component("engine", "powertrain", "/powertrain");
  comp2.description = "Engine control unit";

  LayerOutput out1, out2;
  out1.components.push_back(comp1);
  out2.components.push_back(comp2);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "layer1", out1, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "layer2", out2, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);
  EXPECT_EQ(result.components[0].description, "Engine control unit");
}

TEST_F(MergePipelineTest, FallbackVsEnrichmentEnrichmentWins) {
  // ENRICHMENT > FALLBACK
  Component comp1 = make_component("engine", "powertrain", "/powertrain");
  comp1.name = "Fallback Name";

  Component comp2 = make_component("engine", "powertrain", "/powertrain");
  comp2.name = "Enrichment Name";

  LayerOutput out1, out2;
  out1.components.push_back(comp1);
  out2.components.push_back(comp2);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "fallback", out1, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "enrich", out2, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::ENRICHMENT}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);
  EXPECT_EQ(result.components[0].name, "Enrichment Name");  // ENRICHMENT wins over FALLBACK
}

// --- M6: MergeConflict struct fields ---

TEST_F(MergePipelineTest, MergeConflictStructPopulated) {
  // AUTH vs AUTH on same entity - verify conflict fields
  Component comp1 = make_component("engine", "powertrain", "/powertrain");
  comp1.name = "Manifest Engine";

  Component comp2 = make_component("engine", "powertrain", "/powertrain");
  comp2.name = "Runtime Engine";

  LayerOutput out1, out2;
  out1.components.push_back(comp1);
  out2.components.push_back(comp2);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", out1,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", out2,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE}}));

  auto result = pipeline_.execute();
  ASSERT_GE(result.report.conflicts.size(), 1u);

  // Find the IDENTITY conflict for "engine"
  bool found = false;
  for (const auto & c : result.report.conflicts) {
    if (c.entity_id == "engine" && c.field_group == FieldGroup::IDENTITY) {
      EXPECT_EQ(c.winning_layer, "manifest");
      EXPECT_EQ(c.losing_layer, "runtime");
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected IDENTITY conflict for entity 'engine'";
}

// --- m9: MergeReport::to_json() includes conflicts ---

TEST(MergeTypesTest, MergeReportToJsonIncludesConflicts) {
  MergeReport report;
  report.layers = {"manifest", "runtime"};
  report.conflict_count = 1;
  report.conflicts.push_back({"engine", FieldGroup::IDENTITY, "manifest", "runtime"});

  auto j = report.to_json();
  ASSERT_TRUE(j.contains("conflicts"));
  ASSERT_EQ(j["conflicts"].size(), 1u);
  EXPECT_EQ(j["conflicts"][0]["entity_id"], "engine");
  EXPECT_EQ(j["conflicts"][0]["field_group"], "IDENTITY");
  EXPECT_EQ(j["conflicts"][0]["winning_layer"], "manifest");
  EXPECT_EQ(j["conflicts"][0]["losing_layer"], "runtime");
}

// --- Three-layer merge (manifest + runtime + plugin) ---

// @verifies REQ_INTEROP_003
TEST_F(MergePipelineTest, ThreeLayerMerge_PluginEnrichesManifestEntity) {
  // Manifest: component "engine" with AUTHORITATIVE identity
  Component manifest_comp = make_component("engine", "powertrain", "/powertrain");
  manifest_comp.name = "Engine ECU";
  manifest_comp.description = "Main engine controller";

  // Runtime: component "engine" with FALLBACK identity, AUTHORITATIVE live_data (topics)
  Component runtime_comp = make_component("engine", "powertrain", "/powertrain");
  runtime_comp.name = "engine";  // heuristic name, should not override manifest
  ServiceInfo svc;
  svc.name = "get_status";
  svc.full_path = "/powertrain/get_status";
  svc.type = "std_srvs/srv/Trigger";
  runtime_comp.services.push_back(svc);

  // Plugin: component "engine" with ENRICHMENT metadata - adds vendor extension
  Component plugin_comp = make_component("engine", "powertrain", "/powertrain");
  plugin_comp.description = "Plugin-enriched description";  // IDENTITY: won't override manifest AUTH
  plugin_comp.source = "vendor-plugin";                     // METADATA: will fill empty field

  LayerOutput manifest_out, runtime_out, plugin_out;
  manifest_out.components.push_back(manifest_comp);
  runtime_out.components.push_back(runtime_comp);
  plugin_out.components.push_back(plugin_comp);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT},
                                                  {FieldGroup::METADATA, MergePolicy::AUTHORITATIVE}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", runtime_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "plugin", plugin_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::ENRICHMENT},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT},
                                                  {FieldGroup::METADATA, MergePolicy::ENRICHMENT}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);

  const auto & merged = result.components[0];
  // Name from manifest (AUTHORITATIVE IDENTITY wins over plugin ENRICHMENT)
  EXPECT_EQ(merged.name, "Engine ECU");
  // Description from manifest (AUTHORITATIVE beats ENRICHMENT)
  EXPECT_EQ(merged.description, "Main engine controller");
  // Services from runtime (AUTHORITATIVE LIVE_DATA)
  EXPECT_EQ(merged.services.size(), 1u);
  // entity_source tracks the first layer that introduced this entity
  EXPECT_EQ(result.report.entity_source["engine"], "manifest");
}

// --- App STATUS field group merge ---

// @verifies REQ_INTEROP_003
TEST_F(MergePipelineTest, AppStatusMerge_BoolOrSemantics) {
  // Two layers provide the same App with different is_online values
  App app1 = make_app("controller", "nav_comp");
  app1.is_online = false;

  App app2 = make_app("controller", "nav_comp");
  app2.is_online = true;
  app2.bound_fqn = "/nav/controller";

  LayerOutput out1, out2;
  out1.apps.push_back(app1);
  out2.apps.push_back(app2);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", out1, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::STATUS, MergePolicy::FALLBACK}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", out2, std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::STATUS, MergePolicy::AUTHORITATIVE}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.apps.size(), 1u);
  // Runtime STATUS is AUTHORITATIVE, so is_online=true and bound_fqn set
  EXPECT_TRUE(result.apps[0].is_online);
  EXPECT_EQ(result.apps[0].bound_fqn, "/nav/controller");
}

// @verifies REQ_INTEROP_003
TEST_F(MergePipelineTest, ThreeLayerMerge_PerFieldGroupOwnerTracking) {
  // Regression test: verify that when Runtime wins STATUS (AUTH) over Manifest (FALLBACK),
  // a Plugin (ENRICH) cannot override Runtime's authoritative STATUS.
  // Previously, owner_layer_idx was fixed to first layer, so Plugin compared against
  // Manifest's FALLBACK policy instead of Runtime's AUTH, incorrectly winning STATUS.
  App manifest_app = make_app("controller", "nav_comp");
  manifest_app.is_online = false;

  App runtime_app = make_app("controller", "nav_comp");
  runtime_app.is_online = true;
  runtime_app.bound_fqn = "/nav/controller";

  App plugin_app = make_app("controller", "nav_comp");
  plugin_app.is_online = false;  // Plugin says offline - should NOT override Runtime's AUTH
  plugin_app.bound_fqn = std::nullopt;

  LayerOutput manifest_out, runtime_out, plugin_out;
  manifest_out.apps.push_back(manifest_app);
  runtime_out.apps.push_back(runtime_app);
  plugin_out.apps.push_back(plugin_app);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::STATUS, MergePolicy::FALLBACK},
                                                  {FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", runtime_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::STATUS, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::IDENTITY, MergePolicy::FALLBACK}}));
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "plugin", plugin_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::STATUS, MergePolicy::ENRICHMENT},
                                                  {FieldGroup::IDENTITY, MergePolicy::ENRICHMENT}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.apps.size(), 1u);
  // Runtime's AUTH should win STATUS - plugin's ENRICH cannot override
  EXPECT_TRUE(result.apps[0].is_online);
  EXPECT_EQ(result.apps[0].bound_fqn, "/nav/controller");
}

// --- GapFillConfig namespace filtering ---
// These tests verify the namespace matching semantics used by RuntimeLayer.
// Since filter_by_namespace is internal to runtime_layer.cpp, we replicate the
// matching logic here to test the *semantics* of GapFillConfig lists.

namespace {
// Mirrors the is_namespace_allowed() semantics in runtime_layer.cpp:
// path-segment boundary matching (ns == w || ns starts with w + "/").
bool is_namespace_allowed(const std::string & ns, const GapFillConfig & config) {
  if (!config.namespace_whitelist.empty()) {
    bool found =
        std::any_of(config.namespace_whitelist.begin(), config.namespace_whitelist.end(), [&ns](const std::string & w) {
          return ns == w || ns.find(w + "/") == 0;
        });
    if (!found) {
      return false;
    }
  }
  for (const auto & b : config.namespace_blacklist) {
    if (ns == b || ns.find(b + "/") == 0) {
      return false;
    }
  }
  return true;
}
}  // namespace

// @verifies REQ_INTEROP_003
TEST(GapFillConfigTest, NamespaceWhitelistFiltersAreas) {
  GapFillConfig config;
  config.namespace_whitelist = {"/robot"};

  // Exact match passes
  EXPECT_TRUE(is_namespace_allowed("/robot", config));
  // Child namespace passes (path-segment boundary)
  EXPECT_TRUE(is_namespace_allowed("/robot/nav", config));
  // Different namespace blocked
  EXPECT_FALSE(is_namespace_allowed("/sensor", config));
  // Prefix that is NOT a path segment boundary should be blocked
  EXPECT_FALSE(is_namespace_allowed("/robotics", config));
}

// @verifies REQ_INTEROP_003
TEST(GapFillConfigTest, NamespaceBlacklistFiltersAreas) {
  GapFillConfig config;
  config.namespace_blacklist = {"/rosout"};

  // Not blacklisted
  EXPECT_TRUE(is_namespace_allowed("/robot", config));
  // Exact match blocked
  EXPECT_FALSE(is_namespace_allowed("/rosout", config));
  // Child namespace blocked (path-segment boundary)
  EXPECT_FALSE(is_namespace_allowed("/rosout/sub", config));
  // Prefix that is NOT a path segment boundary should pass
  EXPECT_TRUE(is_namespace_allowed("/rosoutput", config));
}

// --- Pipeline exception safety ---

namespace {
class ThrowingLayer : public DiscoveryLayer {
 public:
  std::string name() const override {
    return "throwing";
  }
  LayerOutput discover() override {
    throw std::runtime_error("plugin crash");
  }
  MergePolicy policy_for(FieldGroup /*group*/) const override {
    return MergePolicy::ENRICHMENT;
  }
};
}  // namespace

// @verifies REQ_INTEROP_003
TEST_F(MergePipelineTest, LayerExceptionDoesNotCrashPipeline) {
  // A good layer followed by a throwing layer - good layer's data should survive
  LayerOutput good_output;
  good_output.areas.push_back(make_area("powertrain"));
  pipeline_.add_layer(std::make_unique<TestLayer>("good", good_output));
  pipeline_.add_layer(std::make_unique<ThrowingLayer>());

  auto result = pipeline_.execute();
  // Good layer's data should be present despite the throwing layer
  ASSERT_EQ(result.areas.size(), 1u);
  EXPECT_EQ(result.areas[0].id, "powertrain");
}

TEST_F(MergePipelineTest, FunctionMerge_HostsAndIdentity) {
  // Layer 1 (AUTH for IDENTITY): function with name, no hosts
  Function auth_func = make_function("diagnostics", "Diagnostics Suite");
  auth_func.source = "manifest";

  LayerOutput auth_output;
  auth_output.functions.push_back(auth_func);

  // Layer 2 (ENRICHMENT): function with hosts, different name
  Function enrich_func = make_function("diagnostics", "diag_runtime");
  enrich_func.hosts = {"engine_ecu", "brake_controller"};
  enrich_func.source = "runtime";

  LayerOutput enrich_output;
  enrich_output.functions.push_back(enrich_func);

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", auth_output,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::HIERARCHY, MergePolicy::ENRICHMENT}}));

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", enrich_output,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK},
                                                  {FieldGroup::HIERARCHY, MergePolicy::ENRICHMENT}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.functions.size(), 1u);
  EXPECT_EQ(result.functions[0].name, "Diagnostics Suite");  // AUTH identity wins
  EXPECT_EQ(result.functions[0].hosts.size(), 2u);           // ENRICHMENT fills hosts
  EXPECT_EQ(result.functions[0].source, "manifest");         // higher priority source
}

// --- field_group_from_string / merge_policy_from_string parsing ---

TEST(MergeTypesParsingTest, FieldGroupFromStringValid) {
  EXPECT_EQ(field_group_from_string("identity"), FieldGroup::IDENTITY);
  EXPECT_EQ(field_group_from_string("hierarchy"), FieldGroup::HIERARCHY);
  EXPECT_EQ(field_group_from_string("live_data"), FieldGroup::LIVE_DATA);
  EXPECT_EQ(field_group_from_string("status"), FieldGroup::STATUS);
  EXPECT_EQ(field_group_from_string("metadata"), FieldGroup::METADATA);
}

TEST(MergeTypesParsingTest, FieldGroupFromStringInvalid) {
  EXPECT_EQ(field_group_from_string(""), std::nullopt);
  EXPECT_EQ(field_group_from_string("IDENTITY"), std::nullopt);
  EXPECT_EQ(field_group_from_string("Identity"), std::nullopt);
  EXPECT_EQ(field_group_from_string("unknown"), std::nullopt);
  EXPECT_EQ(field_group_from_string("live-data"), std::nullopt);
}

TEST(MergeTypesParsingTest, MergePolicyFromStringValid) {
  EXPECT_EQ(merge_policy_from_string("authoritative"), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(merge_policy_from_string("enrichment"), MergePolicy::ENRICHMENT);
  EXPECT_EQ(merge_policy_from_string("fallback"), MergePolicy::FALLBACK);
}

TEST(MergeTypesParsingTest, MergePolicyFromStringInvalid) {
  EXPECT_EQ(merge_policy_from_string(""), std::nullopt);
  EXPECT_EQ(merge_policy_from_string("AUTHORITATIVE"), std::nullopt);
  EXPECT_EQ(merge_policy_from_string("auth"), std::nullopt);
  EXPECT_EQ(merge_policy_from_string("unknown"), std::nullopt);
}

// --- ManifestLayer / RuntimeLayer set_policy override ---

TEST(LayerPolicyOverrideTest, ManifestLayerSetPolicyOverridesDefault) {
  ManifestLayer layer(nullptr);

  // Default: LIVE_DATA = ENRICHMENT
  EXPECT_EQ(layer.policy_for(FieldGroup::LIVE_DATA), MergePolicy::ENRICHMENT);

  // Override to AUTHORITATIVE
  layer.set_policy(FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::LIVE_DATA), MergePolicy::AUTHORITATIVE);

  // Other policies unchanged
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::STATUS), MergePolicy::FALLBACK);
}

TEST(LayerPolicyOverrideTest, RuntimeLayerSetPolicyOverridesDefault) {
  RuntimeLayer layer(nullptr);

  // Default: IDENTITY = FALLBACK
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::FALLBACK);

  // Override to AUTHORITATIVE
  layer.set_policy(FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::AUTHORITATIVE);

  // Other policies unchanged
  EXPECT_EQ(layer.policy_for(FieldGroup::LIVE_DATA), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::METADATA), MergePolicy::ENRICHMENT);
}

TEST(LayerPolicyOverrideTest, PluginLayerSetPolicyOverridesDefault) {
  PluginLayer layer("test_plugin", nullptr);

  // Default: all ENRICHMENT
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::ENRICHMENT);

  // Override IDENTITY to AUTHORITATIVE
  layer.set_policy(FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::IDENTITY), MergePolicy::AUTHORITATIVE);
  EXPECT_EQ(layer.policy_for(FieldGroup::HIERARCHY), MergePolicy::ENRICHMENT);
}

// --- Policy override affects merge behavior end-to-end ---

TEST_F(MergePipelineTest, PolicyOverrideChangedMergeBehavior) {
  // Manifest layer with LIVE_DATA overridden from ENRICHMENT to AUTHORITATIVE
  // This means manifest topics should win over runtime topics
  LayerOutput manifest_output;
  manifest_output.areas.push_back(make_area("powertrain"));

  Component manifest_comp;
  manifest_comp.id = "engine";
  manifest_comp.area = "powertrain";
  manifest_comp.source = "manifest";
  manifest_comp.topics.publishes = {"manifest_topic"};
  manifest_output.components.push_back(manifest_comp);

  // Runtime layer provides same component with different topics
  Component runtime_comp;
  runtime_comp.id = "engine";
  runtime_comp.area = "powertrain";
  runtime_comp.source = "runtime";
  runtime_comp.topics.publishes = {"runtime_topic"};

  LayerOutput runtime_output;
  runtime_output.components.push_back(runtime_comp);

  // Use TestLayers with actual ManifestLayer/RuntimeLayer default policies + override
  pipeline_.add_layer(std::make_unique<TestLayer>(
      "manifest", manifest_output,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::HIERARCHY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE},  // overridden!
                                                  {FieldGroup::STATUS, MergePolicy::FALLBACK},
                                                  {FieldGroup::METADATA, MergePolicy::AUTHORITATIVE}}));

  pipeline_.add_layer(std::make_unique<TestLayer>(
      "runtime", runtime_output,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK},
                                                  {FieldGroup::HIERARCHY, MergePolicy::FALLBACK},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::STATUS, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::METADATA, MergePolicy::ENRICHMENT}}));

  auto result = pipeline_.execute();
  ASSERT_EQ(result.components.size(), 1u);

  // With both layers AUTH for LIVE_DATA, manifest (higher priority) wins
  // This means we get a conflict but manifest topics survive
  EXPECT_EQ(result.components[0].source, "manifest");
  EXPECT_GE(result.report.conflict_count, 1u);
}
