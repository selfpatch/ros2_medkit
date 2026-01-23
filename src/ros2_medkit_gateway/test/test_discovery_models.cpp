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
 * @file test_discovery_models.cpp
 * @brief Unit tests for SOVD discovery model serialization
 *
 * @verifies REQ_DISCOVERY_002 App/Function model serialization
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::Area;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::Function;
using ros2_medkit_gateway::json;

// =============================================================================
// Area Model Tests
// =============================================================================

class AreaModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    area_.id = "powertrain";
    area_.name = "Powertrain System";
    area_.namespace_path = "/powertrain";
    area_.type = "Area";
    area_.translation_id = "area.powertrain";
    area_.description = "Powertrain control systems";
    area_.tags = {"critical", "automotive"};
    area_.parent_area_id = "vehicle";
  }

  Area area_;
};

TEST_F(AreaModelTest, ToJson_ContainsRequiredFields) {
  json j = area_.to_json();

  EXPECT_EQ(j["id"], "powertrain");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["namespace"], "/powertrain");
  EXPECT_EQ(j["x-medkit"]["entityType"], "Area");
}

TEST_F(AreaModelTest, ToJson_ContainsOptionalFields) {
  json j = area_.to_json();

  EXPECT_EQ(j["name"], "Powertrain System");
  EXPECT_EQ(j["translationId"], "area.powertrain");
  EXPECT_EQ(j["x-medkit"]["description"], "Powertrain control systems");
  EXPECT_EQ(j["tags"].size(), 2);
  EXPECT_EQ(j["x-medkit"]["parentAreaId"], "vehicle");
}

TEST_F(AreaModelTest, ToJson_OmitsEmptyOptionalFields) {
  Area minimal;
  minimal.id = "test";
  minimal.namespace_path = "/test";

  json j = minimal.to_json();

  EXPECT_FALSE(j.contains("name"));
  EXPECT_FALSE(j.contains("translationId"));
  EXPECT_FALSE(j["x-medkit"].contains("description"));
  EXPECT_FALSE(j.contains("tags"));
  EXPECT_FALSE(j["x-medkit"].contains("parentAreaId"));
}

TEST_F(AreaModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = area_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "powertrain");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/areas/powertrain");
  EXPECT_FALSE(j.contains("type"));  // SOVD compliant: no type in EntityReference
}

TEST_F(AreaModelTest, ToCapabilities_ContainsSubResources) {
  json j = area_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "powertrain");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "Area");
  EXPECT_TRUE(j.contains("subareas"));
  EXPECT_TRUE(j.contains("related-components"));
}

// =============================================================================
// Component Model Tests
// =============================================================================

class ComponentModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    comp_.id = "motor_controller";
    comp_.name = "Motor Controller";
    comp_.namespace_path = "/powertrain";
    comp_.fqn = "/powertrain/motor_controller";
    comp_.type = "Component";
    comp_.area = "powertrain";
    comp_.source = "node";
    comp_.translation_id = "comp.motor";
    comp_.description = "Controls the electric motor";
    comp_.variant = "v2";
    comp_.tags = {"actuator"};
  }

  Component comp_;
};

TEST_F(ComponentModelTest, ToJson_ContainsRequiredFields) {
  json j = comp_.to_json();

  EXPECT_EQ(j["id"], "motor_controller");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["namespace"], "/powertrain");
  EXPECT_EQ(j["x-medkit"]["fqn"], "/powertrain/motor_controller");
  EXPECT_EQ(j["x-medkit"]["entityType"], "Component");
  EXPECT_EQ(j["x-medkit"]["area"], "powertrain");
  EXPECT_EQ(j["x-medkit"]["source"], "node");
}

TEST_F(ComponentModelTest, ToJson_ContainsOptionalFields) {
  json j = comp_.to_json();

  EXPECT_EQ(j["name"], "Motor Controller");
  EXPECT_EQ(j["translationId"], "comp.motor");
  EXPECT_EQ(j["x-medkit"]["description"], "Controls the electric motor");
  EXPECT_EQ(j["x-medkit"]["variant"], "v2");
  EXPECT_EQ(j["tags"].size(), 1);
}

TEST_F(ComponentModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = comp_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "motor_controller");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/components/motor_controller");
  EXPECT_FALSE(j.contains("type"));  // SOVD compliant: no type in EntityReference
}

TEST_F(ComponentModelTest, ToCapabilities_ContainsConfigurationsForNodes) {
  json j = comp_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "motor_controller");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "Component");

  // Node-based components should have configurations capability
  EXPECT_TRUE(j.contains("configurations"));
  EXPECT_EQ(j["configurations"], "http://localhost:8080/api/v1/components/motor_controller/configurations");
}

// =============================================================================
// App Model Tests
// =============================================================================

class AppModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    app_.id = "nav2";
    app_.name = "Navigation 2";
    app_.source = "manifest";
    app_.translation_id = "app.nav2";
    app_.description = "Navigation stack for ROS 2";
    app_.tags = {"navigation", "autonomous"};

    // ROS binding
    App::RosBinding binding;
    binding.node_name = "nav2_controller";
    binding.namespace_pattern = "/nav2";
    app_.ros_binding = binding;

    // Runtime state
    app_.bound_fqn = "/nav2/controller_server";
    app_.is_online = true;
    app_.external = false;

    // Component relationship
    app_.component_id = "navigation_server";
    app_.depends_on = {"localization", "mapping"};
  }

  App app_;
};

TEST_F(AppModelTest, ToJson_ContainsRequiredFields) {
  json j = app_.to_json();

  EXPECT_EQ(j["id"], "nav2");
  EXPECT_EQ(j["name"], "Navigation 2");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "App");
  EXPECT_EQ(j["x-medkit"]["source"], "manifest");
}

TEST_F(AppModelTest, ToJson_ContainsOptionalFields) {
  json j = app_.to_json();

  EXPECT_EQ(j["translationId"], "app.nav2");
  EXPECT_EQ(j["x-medkit"]["description"], "Navigation stack for ROS 2");
  EXPECT_EQ(j["tags"].size(), 2);
}

TEST_F(AppModelTest, ToJson_ContainsRosBinding) {
  json j = app_.to_json();

  ASSERT_TRUE(j["x-medkit"].contains("rosBinding"));
  EXPECT_EQ(j["x-medkit"]["rosBinding"]["nodeName"], "nav2_controller");
  EXPECT_EQ(j["x-medkit"]["rosBinding"]["namespace"], "/nav2");
}

TEST_F(AppModelTest, ToJson_ContainsRuntimeState) {
  json j = app_.to_json();

  EXPECT_EQ(j["x-medkit"]["boundFqn"], "/nav2/controller_server");
  EXPECT_EQ(j["x-medkit"]["isOnline"], true);
  // external is only included when true, so should not be present when false
  EXPECT_FALSE(j["x-medkit"].contains("external"));
}

TEST_F(AppModelTest, ToJson_ExternalWhenTrue) {
  app_.external = true;
  json j = app_.to_json();

  EXPECT_EQ(j["x-medkit"]["external"], true);
}

TEST_F(AppModelTest, ToJson_OmitsEmptyOptionalFields) {
  App minimal;
  minimal.id = "test";
  minimal.name = "Test App";
  minimal.source = "manifest";

  json j = minimal.to_json();

  EXPECT_FALSE(j.contains("translationId"));
  EXPECT_FALSE(j["x-medkit"].contains("description"));
  EXPECT_FALSE(j.contains("tags"));
  EXPECT_FALSE(j["x-medkit"].contains("rosBinding"));
  EXPECT_FALSE(j["x-medkit"].contains("boundFqn"));
}

TEST_F(AppModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = app_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "nav2");
  EXPECT_EQ(j["name"], "Navigation 2");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/apps/nav2");
}

TEST_F(AppModelTest, ToCapabilities_ContainsExpectedResources) {
  // Add some topics and services to get data and operations capabilities
  app_.topics.publishes.push_back("/nav2/path");
  ros2_medkit_gateway::ServiceInfo svc;
  svc.name = "get_plan";
  svc.full_path = "/nav2/get_plan";
  svc.type = "nav2_msgs/srv/GetPlan";
  app_.services.push_back(svc);

  json j = app_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "nav2");
  EXPECT_EQ(j["name"], "Navigation 2");
  EXPECT_TRUE(j.contains("data"));
  EXPECT_TRUE(j.contains("operations"));
  EXPECT_TRUE(j.contains("faults"));
}

TEST_F(AppModelTest, ToCapabilities_OmitsDataWithoutTopics) {
  // No topics, no services
  App minimal;
  minimal.id = "test";
  minimal.name = "Test App";
  minimal.source = "manifest";

  json j = minimal.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_FALSE(j.contains("data"));
  EXPECT_FALSE(j.contains("operations"));
  EXPECT_TRUE(j.contains("faults"));
  EXPECT_TRUE(j.contains("configurations"));
  EXPECT_TRUE(j.contains("x-medkit"));
}

// =============================================================================
// Function Model Tests
// =============================================================================

class FunctionModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    func_.id = "path_planning";
    func_.name = "Path Planning";
    func_.source = "manifest";
    func_.translation_id = "func.path_planning";
    func_.description = "Computes optimal paths to goal";
    func_.tags = {"planning", "core"};
    func_.hosts = {"nav2_planner_server"};
    func_.depends_on = {"localization"};
  }

  Function func_;
};

TEST_F(FunctionModelTest, ToJson_ContainsRequiredFields) {
  json j = func_.to_json();

  EXPECT_EQ(j["id"], "path_planning");
  EXPECT_EQ(j["name"], "Path Planning");
  EXPECT_TRUE(j.contains("x-medkit"));
  EXPECT_EQ(j["x-medkit"]["entityType"], "Function");
  EXPECT_EQ(j["x-medkit"]["source"], "manifest");
}

TEST_F(FunctionModelTest, ToJson_ContainsOptionalFields) {
  json j = func_.to_json();

  EXPECT_EQ(j["translationId"], "func.path_planning");
  EXPECT_EQ(j["x-medkit"]["description"], "Computes optimal paths to goal");
  EXPECT_EQ(j["tags"].size(), 2);
  EXPECT_EQ(j["x-medkit"]["hosts"].size(), 1);
  EXPECT_EQ(j["x-medkit"]["dependsOn"].size(), 1);
}

TEST_F(FunctionModelTest, ToJson_OmitsEmptyOptionalFields) {
  Function minimal;
  minimal.id = "test";
  minimal.name = "Test Function";
  minimal.source = "manifest";

  json j = minimal.to_json();

  EXPECT_FALSE(j.contains("translationId"));
  EXPECT_FALSE(j["x-medkit"].contains("description"));
  EXPECT_FALSE(j.contains("tags"));
  EXPECT_FALSE(j["x-medkit"].contains("hosts"));
  EXPECT_FALSE(j["x-medkit"].contains("dependsOn"));
}

TEST_F(FunctionModelTest, ToEntityReference_ContainsRequiredFields) {
  json j = func_.to_entity_reference("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "path_planning");
  EXPECT_EQ(j["name"], "Path Planning");
  EXPECT_EQ(j["href"], "http://localhost:8080/api/v1/functions/path_planning");
  EXPECT_FALSE(j.contains("type"));  // SOVD compliant: no type in EntityReference
}

TEST_F(FunctionModelTest, ToCapabilities_ContainsExpectedResources) {
  json j = func_.to_capabilities("http://localhost:8080/api/v1");

  EXPECT_EQ(j["id"], "path_planning");
  EXPECT_EQ(j["name"], "Path Planning");
  EXPECT_TRUE(j.contains("data"));
  EXPECT_TRUE(j.contains("operations"));
  EXPECT_TRUE(j.contains("faults"));
  EXPECT_TRUE(j.contains("x-medkit"));
}

// =============================================================================
// Common Types Tests
// =============================================================================

TEST(CommonTypesTest, ServiceInfo_ToJson) {
  ros2_medkit_gateway::ServiceInfo service;
  service.name = "set_speed";
  service.full_path = "/motor/set_speed";
  service.type = "std_srvs/srv/SetBool";

  json j = service.to_json();

  EXPECT_EQ(j["name"], "set_speed");
  EXPECT_EQ(j["path"], "/motor/set_speed");
  EXPECT_EQ(j["type"], "std_srvs/srv/SetBool");
  EXPECT_EQ(j["kind"], "service");
}

TEST(CommonTypesTest, ActionInfo_ToJson) {
  ros2_medkit_gateway::ActionInfo action;
  action.name = "navigate_to_pose";
  action.full_path = "/navigate_to_pose";
  action.type = "nav2_msgs/action/NavigateToPose";

  json j = action.to_json();

  EXPECT_EQ(j["name"], "navigate_to_pose");
  EXPECT_EQ(j["path"], "/navigate_to_pose");
  EXPECT_EQ(j["type"], "nav2_msgs/action/NavigateToPose");
  EXPECT_EQ(j["kind"], "action");
}

TEST(CommonTypesTest, ComponentTopics_ToJson) {
  ros2_medkit_gateway::ComponentTopics topics;
  topics.publishes.push_back("/odom");
  topics.subscribes.push_back("/cmd_vel");

  json j = topics.to_json();

  EXPECT_EQ(j["publishes"].size(), 1);
  EXPECT_EQ(j["subscribes"].size(), 1);
  EXPECT_EQ(j["publishes"][0], "/odom");
  EXPECT_EQ(j["subscribes"][0], "/cmd_vel");
}

TEST(CommonTypesTest, QosProfile_ToJson) {
  ros2_medkit_gateway::QosProfile qos;
  qos.reliability = "reliable";
  qos.durability = "volatile";
  qos.history = "keep_last";
  qos.depth = 10;
  qos.liveliness = "automatic";

  json j = qos.to_json();

  EXPECT_EQ(j["reliability"], "reliable");
  EXPECT_EQ(j["durability"], "volatile");
  EXPECT_EQ(j["history"], "keep_last");
  EXPECT_EQ(j["depth"], 10);
  EXPECT_EQ(j["liveliness"], "automatic");
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
