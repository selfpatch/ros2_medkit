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

/**
 * @file test_manifest_parser.cpp
 * @brief Unit tests for manifest YAML parser
 *
 * @verifies REQ_DISCOVERY_003 Manifest parsing
 */

#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "ros2_medkit_gateway/discovery/manifest/manifest_parser.hpp"

using ros2_medkit_gateway::discovery::ManifestConfig;
using ros2_medkit_gateway::discovery::ManifestParser;

// =============================================================================
// Valid Manifest Parsing Tests
// =============================================================================

class ManifestParserTest : public ::testing::Test {
 protected:
  ManifestParser parser_;
};

TEST_F(ManifestParserTest, ParseMinimalManifest) {
  const std::string yaml = R"(
manifest_version: "1.0"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.manifest_version, "1.0");
  EXPECT_TRUE(manifest.is_loaded());
}

TEST_F(ManifestParserTest, ParseMetadata) {
  const std::string yaml = R"(
manifest_version: "1.0"
metadata:
  name: "Test Manifest"
  description: "A test manifest"
  version: "1.0.0"
  created_at: "2025-01-15"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.metadata.name, "Test Manifest");
  EXPECT_EQ(manifest.metadata.description, "A test manifest");
  EXPECT_EQ(manifest.metadata.version, "1.0.0");
  EXPECT_EQ(manifest.metadata.created_at, "2025-01-15");
}

TEST_F(ManifestParserTest, ParseDiscoveryConfig) {
  const std::string yaml = R"(
manifest_version: "1.0"
discovery:
  unmanifested_nodes: "ignore"
  inherit_runtime_resources: false
  allow_manifest_override: true
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_FALSE(manifest.config.inherit_runtime_resources);
  EXPECT_TRUE(manifest.config.allow_manifest_override);
}

TEST_F(ManifestParserTest, ParseDiscoveryConfigDefaultPolicy) {
  const std::string yaml = R"(
manifest_version: "1.0"
discovery:
  unmanifested_nodes: "warn"
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.config.unmanifested_nodes, ManifestConfig::UnmanifestedNodePolicy::WARN);
}

TEST_F(ManifestParserTest, ParseAreas) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "powertrain"
    name: "Powertrain System"
    namespace: "/powertrain"
    description: "Motor and drive systems"
    tags:
      - critical
      - automotive
  - id: "sensors"
    parent_area: "vehicle"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.areas.size(), 2);

  EXPECT_EQ(manifest.areas[0].id, "powertrain");
  EXPECT_EQ(manifest.areas[0].name, "Powertrain System");
  EXPECT_EQ(manifest.areas[0].namespace_path, "/powertrain");
  EXPECT_EQ(manifest.areas[0].description, "Motor and drive systems");
  EXPECT_EQ(manifest.areas[0].tags.size(), 2);

  EXPECT_EQ(manifest.areas[1].id, "sensors");
  EXPECT_EQ(manifest.areas[1].name, "sensors");  // Defaults to id
  EXPECT_EQ(manifest.areas[1].parent_area_id, "vehicle");
}

TEST_F(ManifestParserTest, ParseComponents) {
  const std::string yaml = R"(
manifest_version: "1.0"
components:
  - id: "motor_controller"
    name: "Motor Controller"
    namespace: "/powertrain"
    area: "powertrain"
    description: "Controls the motor"
    variant: "v2"
    tags:
      - actuator
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.components.size(), 1);

  const auto & comp = manifest.components[0];
  EXPECT_EQ(comp.id, "motor_controller");
  EXPECT_EQ(comp.name, "Motor Controller");
  EXPECT_EQ(comp.namespace_path, "/powertrain");
  EXPECT_EQ(comp.fqn, "/powertrain/motor_controller");
  EXPECT_EQ(comp.area, "powertrain");
  EXPECT_EQ(comp.source, "manifest");
  EXPECT_EQ(comp.variant, "v2");
  EXPECT_EQ(comp.tags.size(), 1);
}

TEST_F(ManifestParserTest, ParseApps) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "nav2"
    name: "Navigation 2"
    is_located_on: "navigation_server"
    description: "Navigation stack"
    depends_on:
      - localization
      - mapping
    tags:
      - navigation
    ros_binding:
      node_name: "nav2_controller"
      namespace: "/nav2"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.apps.size(), 1);

  const auto & app = manifest.apps[0];
  EXPECT_EQ(app.id, "nav2");
  EXPECT_EQ(app.name, "Navigation 2");
  EXPECT_EQ(app.component_id, "navigation_server");
  EXPECT_EQ(app.description, "Navigation stack");
  EXPECT_EQ(app.depends_on.size(), 2);
  EXPECT_EQ(app.source, "manifest");

  ASSERT_TRUE(app.ros_binding.has_value());
  EXPECT_EQ(app.ros_binding->node_name, "nav2_controller");
  EXPECT_EQ(app.ros_binding->namespace_pattern, "/nav2");
}

TEST_F(ManifestParserTest, ParseAppExternal) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "external_api"
    name: "External API"
    external: true
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.apps.size(), 1);
  EXPECT_TRUE(manifest.apps[0].external);
}

TEST_F(ManifestParserTest, ParseFunctions) {
  const std::string yaml = R"(
manifest_version: "1.0"
functions:
  - id: "path_planning"
    name: "Path Planning"
    description: "Computes optimal paths"
    hosts:
      - nav2_planner
      - nav2_controller
    depends_on:
      - localization
    tags:
      - planning
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.functions.size(), 1);

  const auto & func = manifest.functions[0];
  EXPECT_EQ(func.id, "path_planning");
  EXPECT_EQ(func.name, "Path Planning");
  EXPECT_EQ(func.hosts.size(), 2);
  EXPECT_EQ(func.depends_on.size(), 1);
  EXPECT_EQ(func.source, "manifest");
}

TEST_F(ManifestParserTest, ParseFullManifest) {
  const std::string yaml = R"(
manifest_version: "1.0"
metadata:
  name: "Robot System"
  version: "1.0.0"
discovery:
  unmanifested_nodes: "include_as_orphan"
areas:
  - id: "navigation"
  - id: "perception"
components:
  - id: "nav_server"
    area: "navigation"
  - id: "lidar_driver"
    area: "perception"
apps:
  - id: "nav2"
    is_located_on: "nav_server"
functions:
  - id: "path_planning"
    hosts:
      - nav_server
)";

  auto manifest = parser_.parse_string(yaml);

  EXPECT_EQ(manifest.areas.size(), 2);
  EXPECT_EQ(manifest.components.size(), 2);
  EXPECT_EQ(manifest.apps.size(), 1);
  EXPECT_EQ(manifest.functions.size(), 1);
}

// =============================================================================
// Script Parsing Tests
// =============================================================================

TEST_F(ManifestParserTest, ParseScripts) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "run-diagnostics"
    name: "Run Diagnostics"
    description: "Check health of all sensors"
    path: "/opt/scripts/run-diagnostics.sh"
    format: "bash"
    timeout_sec: 30
    entity_filter:
      - "components/*"
      - "apps/*"
    env:
      GATEWAY_URL: "http://localhost:8080"
      LOG_LEVEL: "debug"
  - id: "inject-nan"
    name: "Inject NaN Fault"
    path: "/opt/scripts/inject-nan.sh"
    format: "bash"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 2);

  const auto & s1 = manifest.scripts[0];
  EXPECT_EQ(s1.id, "run-diagnostics");
  EXPECT_EQ(s1.name, "Run Diagnostics");
  EXPECT_EQ(s1.description, "Check health of all sensors");
  EXPECT_EQ(s1.path, "/opt/scripts/run-diagnostics.sh");
  EXPECT_EQ(s1.format, "bash");
  EXPECT_EQ(s1.timeout_sec, 30);
  ASSERT_EQ(s1.entity_filter.size(), 2);
  EXPECT_EQ(s1.entity_filter[0], "components/*");
  EXPECT_EQ(s1.entity_filter[1], "apps/*");
  ASSERT_EQ(s1.env.size(), 2);
  EXPECT_EQ(s1.env.at("GATEWAY_URL"), "http://localhost:8080");
  EXPECT_EQ(s1.env.at("LOG_LEVEL"), "debug");

  const auto & s2 = manifest.scripts[1];
  EXPECT_EQ(s2.id, "inject-nan");
  EXPECT_EQ(s2.name, "Inject NaN Fault");
  EXPECT_EQ(s2.path, "/opt/scripts/inject-nan.sh");
  EXPECT_EQ(s2.format, "bash");
  EXPECT_EQ(s2.timeout_sec, 300);  // default
  EXPECT_TRUE(s2.entity_filter.empty());
  EXPECT_TRUE(s2.env.empty());
}

TEST_F(ManifestParserTest, ParseScriptsWithArgs) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "calibrate"
    path: "/opt/scripts/calibrate.py"
    format: "python"
    args:
      - name: "threshold"
        type: "float"
        flag: "--threshold"
      - name: "verbose"
        type: "bool"
        flag: "-v"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 1);
  const auto & s = manifest.scripts[0];
  EXPECT_EQ(s.name, "calibrate");  // defaults to id
  ASSERT_TRUE(s.args.is_array());
  ASSERT_EQ(s.args.size(), 2);
  EXPECT_EQ(s.args[0]["name"], "threshold");
  EXPECT_EQ(s.args[0]["type"], "float");
  EXPECT_EQ(s.args[0]["flag"], "--threshold");
  EXPECT_EQ(s.args[1]["name"], "verbose");
}

TEST_F(ManifestParserTest, ParseScriptsWithParametersSchema) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - id: "test-script"
    path: "/opt/scripts/test.sh"
    format: "bash"
    parameters_schema:
      type: "object"
      required: "threshold"
)";

  auto manifest = parser_.parse_string(yaml);

  ASSERT_EQ(manifest.scripts.size(), 1);
  ASSERT_TRUE(manifest.scripts[0].parameters_schema.has_value());
  EXPECT_EQ(manifest.scripts[0].parameters_schema->at("type"), "object");
}

TEST_F(ManifestParserTest, ParseScriptsMissingId) {
  const std::string yaml = R"(
manifest_version: "1.0"
scripts:
  - name: "No ID Script"
    path: "/opt/scripts/test.sh"
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

TEST_F(ManifestParserTest, ParseNoScriptsSection) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "test-area"
)";

  auto manifest = parser_.parse_string(yaml);
  EXPECT_TRUE(manifest.scripts.empty());
}

// =============================================================================
// Error Cases Tests
// =============================================================================

TEST_F(ManifestParserTest, ThrowsOnMissingVersion) {
  const std::string yaml = R"(
metadata:
  name: "Test"
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

TEST_F(ManifestParserTest, ThrowsOnMalformedYaml) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: unclosed_string
    name: [invalid
)";

  EXPECT_THROW(parser_.parse_string(yaml), std::runtime_error);
}

// =============================================================================
// ManifestConfig Policy Tests
// =============================================================================

TEST(ManifestConfigTest, ParsePolicyStrings) {
  EXPECT_EQ(ManifestConfig::parse_policy("ignore"), ManifestConfig::UnmanifestedNodePolicy::IGNORE);
  EXPECT_EQ(ManifestConfig::parse_policy("warn"), ManifestConfig::UnmanifestedNodePolicy::WARN);
  EXPECT_EQ(ManifestConfig::parse_policy("error"), ManifestConfig::UnmanifestedNodePolicy::ERROR);
  EXPECT_EQ(ManifestConfig::parse_policy("include_as_orphan"),
            ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN);
  // Unknown defaults to WARN
  EXPECT_EQ(ManifestConfig::parse_policy("unknown"), ManifestConfig::UnmanifestedNodePolicy::WARN);
}

TEST(ManifestConfigTest, PolicyToString) {
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::IGNORE), "ignore");
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::WARN), "warn");
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::ERROR), "error");
  EXPECT_EQ(ManifestConfig::policy_to_string(ManifestConfig::UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN),
            "include_as_orphan");
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
