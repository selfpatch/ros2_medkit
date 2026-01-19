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
 * @file test_manifest_validator.cpp
 * @brief Unit tests for manifest validator
 *
 * @verifies REQ_DISCOVERY_003 Manifest validation rules R001-R011
 */

#include <gtest/gtest.h>

#include <string>

#include "ros2_medkit_gateway/discovery/manifest/manifest_parser.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_validator.hpp"

using ros2_medkit_gateway::discovery::ManifestParser;
using ros2_medkit_gateway::discovery::ManifestValidator;
using ros2_medkit_gateway::discovery::ValidationResult;

// =============================================================================
// Test Fixture
// =============================================================================

class ManifestValidatorTest : public ::testing::Test {
 protected:
  ManifestParser parser_;
  ManifestValidator validator_;
};

// =============================================================================
// R001: Version Validation
// =============================================================================

TEST_F(ManifestValidatorTest, R001_ValidVersion) {
  const std::string yaml = R"(
manifest_version: "1.0"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
  EXPECT_FALSE(result.has_errors());
}

TEST_F(ManifestValidatorTest, R001_InvalidVersion) {
  const std::string yaml = R"(
manifest_version: "2.0"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_TRUE(result.has_errors());
  EXPECT_EQ(result.errors[0].rule_id, "R001");
}

// =============================================================================
// R002-R005: Unique ID Validation
// =============================================================================

TEST_F(ManifestValidatorTest, R002_DuplicateAreaId) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "navigation"
  - id: "navigation"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R002");
}

TEST_F(ManifestValidatorTest, R003_DuplicateComponentId) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "nav"
components:
  - id: "nav"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  // Component ID collides with Area ID
  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R003");
}

TEST_F(ManifestValidatorTest, R004_DuplicateAppId) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "nav2"
  - id: "nav2"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R004");
}

TEST_F(ManifestValidatorTest, R005_DuplicateFunctionId) {
  const std::string yaml = R"(
manifest_version: "1.0"
functions:
  - id: "planning"
  - id: "planning"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R005");
}

TEST_F(ManifestValidatorTest, UniqueIdsAcrossEntityTypes) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "area1"
components:
  - id: "comp1"
apps:
  - id: "app1"
functions:
  - id: "func1"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
  EXPECT_FALSE(result.has_errors());
}

// =============================================================================
// R006: Area Reference Validation
// =============================================================================

TEST_F(ManifestValidatorTest, R006_ValidParentAreaReference) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "vehicle"
  - id: "powertrain"
    parent_area: "vehicle"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
}

TEST_F(ManifestValidatorTest, R006_InvalidParentAreaReference) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "powertrain"
    parent_area: "nonexistent"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R006");
}

TEST_F(ManifestValidatorTest, R006_ComponentReferencesValidArea) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: "navigation"
components:
  - id: "nav_server"
    area: "navigation"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
}

TEST_F(ManifestValidatorTest, R006_ComponentReferencesInvalidArea) {
  const std::string yaml = R"(
manifest_version: "1.0"
components:
  - id: "nav_server"
    area: "nonexistent"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R006");
}

// =============================================================================
// R007: App Component Reference Validation
// =============================================================================

TEST_F(ManifestValidatorTest, R007_AppReferencesValidComponent) {
  const std::string yaml = R"(
manifest_version: "1.0"
components:
  - id: "nav_server"
apps:
  - id: "nav2"
    is_located_on: "nav_server"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
}

TEST_F(ManifestValidatorTest, R007_AppReferencesInvalidComponent) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "nav2"
    is_located_on: "nonexistent"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R007");
}

// =============================================================================
// R008: Depends-on Reference Warnings
// =============================================================================

TEST_F(ManifestValidatorTest, R008_AppDependsOnValidApp) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "nav2"
  - id: "planner"
    depends_on:
      - nav2
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
  EXPECT_FALSE(result.has_warnings());
}

TEST_F(ManifestValidatorTest, R008_AppDependsOnNonexistent) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "planner"
    depends_on:
      - nonexistent
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  // Warning, not error
  EXPECT_TRUE(result.is_valid);
  EXPECT_TRUE(result.has_warnings());
  EXPECT_EQ(result.warnings[0].rule_id, "R008");
}

TEST_F(ManifestValidatorTest, R008_FunctionDependsOnNonexistent) {
  const std::string yaml = R"(
manifest_version: "1.0"
functions:
  - id: "planning"
    depends_on:
      - nonexistent
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
  EXPECT_TRUE(result.has_warnings());
  EXPECT_EQ(result.warnings[0].rule_id, "R008");
}

// =============================================================================
// R009: Hosts Reference Warnings
// =============================================================================

TEST_F(ManifestValidatorTest, R009_FunctionHostsValidEntity) {
  const std::string yaml = R"(
manifest_version: "1.0"
components:
  - id: "nav_server"
functions:
  - id: "planning"
    hosts:
      - nav_server
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
  EXPECT_FALSE(result.has_warnings());
}

TEST_F(ManifestValidatorTest, R009_FunctionHostsNonexistent) {
  const std::string yaml = R"(
manifest_version: "1.0"
functions:
  - id: "planning"
    hosts:
      - nonexistent
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
  EXPECT_TRUE(result.has_warnings());
  EXPECT_EQ(result.warnings[0].rule_id, "R009");
}

// =============================================================================
// R010: Duplicate ROS Binding Validation
// =============================================================================

TEST_F(ManifestValidatorTest, R010_UniqueRosBindings) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "app1"
    ros_binding:
      node_name: "node1"
      namespace: "/ns1"
  - id: "app2"
    ros_binding:
      node_name: "node2"
      namespace: "/ns2"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
}

TEST_F(ManifestValidatorTest, R010_DuplicateRosBinding) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "app1"
    ros_binding:
      node_name: "same_node"
      namespace: "/ns"
  - id: "app2"
    ros_binding:
      node_name: "same_node"
      namespace: "/ns"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R010");
}

TEST_F(ManifestValidatorTest, R010_WildcardBindingsAllowed) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "app1"
    ros_binding:
      node_name: "node"
      namespace: "*"
  - id: "app2"
    ros_binding:
      node_name: "node"
      namespace: "*"
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  // Wildcard bindings are allowed to overlap
  EXPECT_TRUE(result.is_valid);
}

// =============================================================================
// R011: Circular Dependency Detection
// =============================================================================

TEST_F(ManifestValidatorTest, R011_NoCircularDependency) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "a"
  - id: "b"
    depends_on:
      - a
  - id: "c"
    depends_on:
      - b
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid);
}

TEST_F(ManifestValidatorTest, R011_DirectCircularDependency) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "a"
    depends_on:
      - b
  - id: "b"
    depends_on:
      - a
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R011");
}

TEST_F(ManifestValidatorTest, R011_IndirectCircularDependency) {
  const std::string yaml = R"(
manifest_version: "1.0"
apps:
  - id: "a"
    depends_on:
      - c
  - id: "b"
    depends_on:
      - a
  - id: "c"
    depends_on:
      - b
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R011");
}

TEST_F(ManifestValidatorTest, R011_FunctionCircularDependency) {
  const std::string yaml = R"(
manifest_version: "1.0"
functions:
  - id: "planning"
    depends_on:
      - control
  - id: "control"
    depends_on:
      - planning
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_FALSE(result.is_valid);
  EXPECT_EQ(result.errors[0].rule_id, "R011");
}

// =============================================================================
// ValidationResult Tests
// =============================================================================

TEST(ValidationResultTest, Summary) {
  ValidationResult result;
  result.add_error("R001", "Test error", "path");
  result.add_warning("R008", "Test warning", "path");

  EXPECT_EQ(result.summary(), "1 errors, 1 warnings");
}

TEST(ValidationResultTest, ErrorToString) {
  ros2_medkit_gateway::discovery::ValidationError error{
      "R001", ros2_medkit_gateway::discovery::ValidationSeverity::ERROR, "Test message", "test/path"};

  EXPECT_EQ(error.to_string(), "[R001] ERROR: Test message (at test/path)");
}

// =============================================================================
// Full Validation Integration Test
// =============================================================================

TEST_F(ManifestValidatorTest, CompleteValidManifest) {
  const std::string yaml = R"(
manifest_version: "1.0"
metadata:
  name: "Robot System"
  version: "1.0.0"
discovery:
  unmanifested_nodes: "warn"
areas:
  - id: "navigation"
    name: "Navigation Area"
  - id: "perception"
    name: "Perception Area"
components:
  - id: "nav_server"
    area: "navigation"
  - id: "lidar_driver"
    area: "perception"
apps:
  - id: "nav2"
    is_located_on: "nav_server"
    ros_binding:
      node_name: "nav2_controller"
      namespace: "/nav2"
  - id: "slam"
    is_located_on: "lidar_driver"
    depends_on:
      - nav2
functions:
  - id: "path_planning"
    hosts:
      - nav_server
  - id: "obstacle_avoidance"
    hosts:
      - nav_server
    depends_on:
      - path_planning
)";

  auto manifest = parser_.parse_string(yaml);
  auto result = validator_.validate(manifest);

  EXPECT_TRUE(result.is_valid) << "Errors: " << result.summary();
  EXPECT_FALSE(result.has_errors());
  EXPECT_FALSE(result.has_warnings());
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
