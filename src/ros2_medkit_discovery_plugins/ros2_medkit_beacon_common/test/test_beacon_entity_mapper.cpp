// Copyright 2026 selfpatch GmbH
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
#include <chrono>
#include <string>
#include <vector>

#include "ros2_medkit_beacon_common/beacon_entity_mapper.hpp"

using ros2_medkit_beacon::BeaconEntityMapper;
using ros2_medkit_beacon::BeaconHint;
using ros2_medkit_beacon::BeaconHintStore;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::Function;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_gateway::IntrospectionResult;
using HintStatus = BeaconHintStore::HintStatus;

namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
BeaconHintStore::StoredHint make_stored_hint(const std::string & entity_id, HintStatus status = HintStatus::ACTIVE,
                                             std::chrono::seconds age = std::chrono::seconds(1)) {
  BeaconHint h;
  h.entity_id = entity_id;
  h.received_at = std::chrono::steady_clock::now();

  BeaconHintStore::StoredHint sh;
  sh.hint = h;
  sh.status = status;
  sh.last_seen = std::chrono::steady_clock::now() - age;
  return sh;
}

IntrospectionInput make_base_input() {
  IntrospectionInput input;

  App app1;
  app1.id = "lidar_driver";
  app1.name = "Lidar Driver";
  app1.component_id = "sensors";
  input.apps.push_back(app1);

  App app2;
  app2.id = "camera_node";
  app2.name = "Camera Node";
  app2.component_id = "sensors";
  input.apps.push_back(app2);

  Component comp;
  comp.id = "sensors";
  comp.name = "Sensors";
  comp.area = "perception";
  input.components.push_back(comp);

  Function func1;
  func1.id = "autonomous_nav";
  func1.name = "Autonomous Navigation";
  func1.hosts = {"lidar_driver"};
  input.functions.push_back(func1);

  Function func2;
  func2.id = "localization";
  func2.name = "Localization";
  input.functions.push_back(func2);

  return input;
}

}  // namespace

// ---------------------------------------------------------------------------
// EnrichesExistingApp
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, EnrichesExistingApp) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.display_name = "Enriched Lidar";
  sh.hint.transport_type = "ros2";

  auto result = mapper.map({sh}, input);

  // Shadow app should be created in new_entities
  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  EXPECT_EQ(result.new_entities.apps[0].id, "lidar_driver");
  EXPECT_EQ(result.new_entities.apps[0].name, "Enriched Lidar");

  // Metadata should be set
  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  EXPECT_EQ(meta["x-medkit-beacon-status"], "active");
  EXPECT_TRUE(meta.contains("x-medkit-beacon-age-sec"));
  EXPECT_EQ(meta["x-medkit-beacon-transport-type"], "ros2");
}

// ---------------------------------------------------------------------------
// EnrichesExistingComponent
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, EnrichesExistingComponent) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("sensors");
  sh.hint.display_name = "Enriched Sensors";
  sh.hint.transport_type = "zenoh";

  auto result = mapper.map({sh}, input);

  // Shadow component should be created
  ASSERT_EQ(result.new_entities.components.size(), 1u);
  EXPECT_EQ(result.new_entities.components[0].id, "sensors");
  EXPECT_EQ(result.new_entities.components[0].name, "Enriched Sensors");

  // No apps should be added (it matched a component, not an app)
  EXPECT_TRUE(result.new_entities.apps.empty());

  // Metadata present
  ASSERT_TRUE(result.metadata.count("sensors"));
  EXPECT_EQ(result.metadata.at("sensors")["x-medkit-beacon-transport-type"], "zenoh");
}

// ---------------------------------------------------------------------------
// UnknownEntityIgnoredWhenDisabled
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, UnknownEntityIgnoredWhenDisabled) {
  auto input = make_base_input();
  BeaconEntityMapper::Config cfg;
  cfg.allow_new_entities = false;
  BeaconEntityMapper mapper(cfg);

  auto sh = make_stored_hint("unknown_app");

  auto result = mapper.map({sh}, input);

  EXPECT_TRUE(result.new_entities.apps.empty());
  EXPECT_TRUE(result.new_entities.components.empty());
  EXPECT_TRUE(result.metadata.empty());
}

// ---------------------------------------------------------------------------
// UnknownEntityCreatesAppWhenEnabled
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, UnknownEntityCreatesAppWhenEnabled) {
  auto input = make_base_input();
  BeaconEntityMapper::Config cfg;
  cfg.allow_new_entities = true;
  BeaconEntityMapper mapper(cfg);

  auto sh = make_stored_hint("brand_new_app");
  sh.hint.display_name = "Brand New App";
  sh.hint.component_id = "compute";

  auto result = mapper.map({sh}, input);

  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  EXPECT_EQ(result.new_entities.apps[0].id, "brand_new_app");
  EXPECT_EQ(result.new_entities.apps[0].name, "Brand New App");
  EXPECT_EQ(result.new_entities.apps[0].component_id, "compute");
  EXPECT_EQ(result.new_entities.apps[0].source, "beacon");

  ASSERT_TRUE(result.metadata.count("brand_new_app"));
}

// ---------------------------------------------------------------------------
// FunctionIdsReverseMapAddsToHosts
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, FunctionIdsReverseMapAddsToHosts) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.function_ids = {"autonomous_nav", "localization"};

  auto result = mapper.map({sh}, input);

  // Both functions should appear in result with lidar_driver as host
  ASSERT_EQ(result.new_entities.functions.size(), 2u);

  for (const auto & func : result.new_entities.functions) {
    auto it = std::find(func.hosts.begin(), func.hosts.end(), "lidar_driver");
    EXPECT_NE(it, func.hosts.end()) << "Function " << func.id << " missing lidar_driver host";
  }
}

// ---------------------------------------------------------------------------
// FunctionIdsNonExistentFunctionLogsWarning
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, FunctionIdsNonExistentFunctionLogsWarning) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.function_ids = {"nonexistent_func"};

  auto result = mapper.map({sh}, input);

  // Nonexistent function should NOT be created in result functions
  EXPECT_TRUE(result.new_entities.functions.empty());

  // But metadata should still have the function_ids recorded
  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  ASSERT_TRUE(meta.contains("x-medkit-beacon-functions"));
  EXPECT_EQ(meta["x-medkit-beacon-functions"].size(), 1u);
  EXPECT_EQ(meta["x-medkit-beacon-functions"][0], "nonexistent_func");
}

// ---------------------------------------------------------------------------
// EmptyFunctionIdsIsNoop
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, EmptyFunctionIdsIsNoop) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.function_ids = {};

  auto result = mapper.map({sh}, input);

  EXPECT_TRUE(result.new_entities.functions.empty());

  // Metadata should not contain x-medkit-beacon-functions when empty
  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  EXPECT_FALSE(result.metadata.at("lidar_driver").contains("x-medkit-beacon-functions"));
}

// ---------------------------------------------------------------------------
// ComponentIdSetsParent
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, ComponentIdSetsParent) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.component_id = "compute_module";

  auto result = mapper.map({sh}, input);

  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  EXPECT_EQ(result.new_entities.apps[0].component_id, "compute_module");
}

// ---------------------------------------------------------------------------
// DisplayNameSetsName
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, DisplayNameSetsName) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.display_name = "Custom Display Name";

  auto result = mapper.map({sh}, input);

  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  EXPECT_EQ(result.new_entities.apps[0].name, "Custom Display Name");
}

// ---------------------------------------------------------------------------
// DependsOnMapped
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, DependsOnMapped) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.depends_on = {"camera_node", "imu_driver"};

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  ASSERT_TRUE(meta.contains("x-medkit-beacon-depends-on"));
  auto deps = meta["x-medkit-beacon-depends-on"];
  ASSERT_EQ(deps.size(), 2u);
  EXPECT_EQ(deps[0], "camera_node");
  EXPECT_EQ(deps[1], "imu_driver");
}

// ---------------------------------------------------------------------------
// MetadataPrefixedCorrectly
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, MetadataPrefixedCorrectly) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.metadata = {{"firmware_version", "2.1.0"}, {"calibration_date", "2026-01-15"}};

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  EXPECT_EQ(meta["x-medkit-beacon-firmware_version"], "2.1.0");
  EXPECT_EQ(meta["x-medkit-beacon-calibration_date"], "2026-01-15");
}

// ---------------------------------------------------------------------------
// ProcessDiagnosticsInMetadata
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, ProcessDiagnosticsInMetadata) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.transport_type = "zenoh";
  sh.hint.process_id = 12345;
  sh.hint.process_name = "lidar_node";
  sh.hint.hostname = "robot-01";

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  EXPECT_EQ(meta["x-medkit-beacon-transport-type"], "zenoh");
  EXPECT_EQ(meta["x-medkit-beacon-process-id"], 12345);
  EXPECT_EQ(meta["x-medkit-beacon-process-name"], "lidar_node");
  EXPECT_EQ(meta["x-medkit-beacon-hostname"], "robot-01");
}

// ---------------------------------------------------------------------------
// ActiveHintMetadataCorrect
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, ActiveHintMetadataCorrect) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver", HintStatus::ACTIVE, std::chrono::seconds(5));

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  EXPECT_EQ(meta["x-medkit-beacon-status"], "active");
  // Age should be approximately 5 seconds (allow some tolerance)
  double age = meta["x-medkit-beacon-age-sec"].get<double>();
  EXPECT_GE(age, 4.0);
  EXPECT_LE(age, 7.0);
}

// ---------------------------------------------------------------------------
// StaleHintMetadataCorrect
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, StaleHintMetadataCorrect) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver", HintStatus::STALE, std::chrono::seconds(30));

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  EXPECT_EQ(meta["x-medkit-beacon-status"], "stale");
  double age = meta["x-medkit-beacon-age-sec"].get<double>();
  EXPECT_GE(age, 29.0);
  EXPECT_LE(age, 32.0);
}

// ---------------------------------------------------------------------------
// MultipleHintsProcessedIndependently
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, MultipleHintsProcessedIndependently) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh1 = make_stored_hint("lidar_driver");
  sh1.hint.display_name = "Lidar";
  auto sh2 = make_stored_hint("camera_node");
  sh2.hint.display_name = "Camera";

  auto result = mapper.map({sh1, sh2}, input);

  ASSERT_EQ(result.new_entities.apps.size(), 2u);
  EXPECT_EQ(result.metadata.size(), 2u);
  EXPECT_TRUE(result.metadata.count("lidar_driver"));
  EXPECT_TRUE(result.metadata.count("camera_node"));
}

// ---------------------------------------------------------------------------
// StableIdInMetadata
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, StableIdInMetadata) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.stable_id = "abc-123-stable";

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  EXPECT_EQ(result.metadata.at("lidar_driver")["x-medkit-beacon-stable-id"], "abc-123-stable");
}

// ---------------------------------------------------------------------------
// NegotiatedFormatInMetadata
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, NegotiatedFormatInMetadata) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  sh.hint.negotiated_format = "cdr";

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  EXPECT_EQ(result.metadata.at("lidar_driver")["x-medkit-beacon-negotiated-format"], "cdr");
}

// ---------------------------------------------------------------------------
// EmptyOptionalFieldsOmitted
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, EmptyOptionalFieldsOmitted) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  // Minimal hint - only entity_id
  auto sh = make_stored_hint("lidar_driver");

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");

  // Required fields always present
  EXPECT_TRUE(meta.contains("x-medkit-beacon-status"));
  EXPECT_TRUE(meta.contains("x-medkit-beacon-age-sec"));

  // Optional fields should be absent when not provided
  EXPECT_FALSE(meta.contains("x-medkit-beacon-transport-type"));
  EXPECT_FALSE(meta.contains("x-medkit-beacon-negotiated-format"));
  EXPECT_FALSE(meta.contains("x-medkit-beacon-process-id"));
  EXPECT_FALSE(meta.contains("x-medkit-beacon-process-name"));
  EXPECT_FALSE(meta.contains("x-medkit-beacon-hostname"));
  EXPECT_FALSE(meta.contains("x-medkit-beacon-stable-id"));
  EXPECT_FALSE(meta.contains("x-medkit-beacon-depends-on"));
  EXPECT_FALSE(meta.contains("x-medkit-beacon-functions"));
}

// ---------------------------------------------------------------------------
// FunctionDeduplication - same function referenced by multiple hints
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, FunctionDeduplication) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh1 = make_stored_hint("lidar_driver");
  sh1.hint.function_ids = {"autonomous_nav"};

  auto sh2 = make_stored_hint("camera_node");
  sh2.hint.function_ids = {"autonomous_nav"};

  auto result = mapper.map({sh1, sh2}, input);

  // Should have exactly one Function entry for autonomous_nav
  ASSERT_EQ(result.new_entities.functions.size(), 1u);
  EXPECT_EQ(result.new_entities.functions[0].id, "autonomous_nav");

  // Both entities should be in hosts
  auto & hosts = result.new_entities.functions[0].hosts;
  EXPECT_NE(std::find(hosts.begin(), hosts.end(), "lidar_driver"), hosts.end());
  EXPECT_NE(std::find(hosts.begin(), hosts.end(), "camera_node"), hosts.end());
}

// ---------------------------------------------------------------------------
// EmptyHintsVector
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, EmptyHintsVector) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto result = mapper.map({}, input);

  EXPECT_TRUE(result.new_entities.apps.empty());
  EXPECT_TRUE(result.new_entities.components.empty());
  EXPECT_TRUE(result.new_entities.functions.empty());
  EXPECT_TRUE(result.metadata.empty());
}

// ---------------------------------------------------------------------------
// DisplayNameEmptyDoesNotOverrideName
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, DisplayNameEmptyDoesNotOverrideName) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  // display_name is empty by default

  auto result = mapper.map({sh}, input);

  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  // Name should remain empty on the shadow entity (no override)
  EXPECT_TRUE(result.new_entities.apps[0].name.empty());
}

// ---------------------------------------------------------------------------
// FreeformMetadataDoesNotOverrideStructuredFields
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, FreeformMetadataDoesNotOverrideStructuredFields) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver", HintStatus::ACTIVE, std::chrono::seconds(1));
  // Attempt to inject a freeform key "status" that collides with x-medkit-beacon-status
  sh.hint.metadata = {{"status", "evil"}, {"age-sec", "999"}};

  auto result = mapper.map({sh}, input);

  ASSERT_TRUE(result.metadata.count("lidar_driver"));
  auto & meta = result.metadata.at("lidar_driver");
  // Structured "active" must win over freeform "evil"
  EXPECT_EQ(meta["x-medkit-beacon-status"], "active");
  // Structured age-sec must be a number (not the freeform string "999")
  EXPECT_TRUE(meta["x-medkit-beacon-age-sec"].is_number());
}

// ---------------------------------------------------------------------------
// ComponentIdEmptyDoesNotOverride
// ---------------------------------------------------------------------------
// @verifies REQ_DISCO_BEACON_01
TEST(BeaconEntityMapper, ComponentIdEmptyDoesNotOverride) {
  auto input = make_base_input();
  BeaconEntityMapper mapper;

  auto sh = make_stored_hint("lidar_driver");
  // component_id is empty by default

  auto result = mapper.map({sh}, input);

  ASSERT_EQ(result.new_entities.apps.size(), 1u);
  // component_id should remain empty on the shadow entity
  EXPECT_TRUE(result.new_entities.apps[0].component_id.empty());
}
