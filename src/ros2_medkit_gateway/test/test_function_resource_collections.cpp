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
#include <string>
#include <vector>

#include "ros2_medkit_gateway/models/aggregation_service.hpp"
#include "ros2_medkit_gateway/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/models/entity_types.hpp"
#include "ros2_medkit_gateway/models/thread_safe_entity_cache.hpp"

using namespace ros2_medkit_gateway;
// nlohmann::json is already aliased as 'json' in the ros2_medkit_gateway namespace

// ============================================================================
// Test Helper Functions - avoid C++20 designated initializers for C++17 compat
// ============================================================================

namespace {

Area make_area(const std::string & id, const std::string & name) {
  Area a;
  a.id = id;
  a.name = name;
  return a;
}

Component make_component(const std::string & id, const std::string & name, const std::string & area) {
  Component c;
  c.id = id;
  c.name = name;
  c.area = area;
  return c;
}

App make_app(const std::string & id, const std::string & name, const std::string & component_id) {
  App a;
  a.id = id;
  a.name = name;
  a.component_id = component_id;
  return a;
}

Function make_function(const std::string & id, const std::string & name, const std::vector<std::string> & hosts) {
  Function f;
  f.id = id;
  f.name = name;
  f.hosts = hosts;
  return f;
}

ServiceInfo make_service(const std::string & name, const std::string & full_path) {
  ServiceInfo s;
  s.name = name;
  s.full_path = full_path;
  return s;
}

}  // namespace

// ============================================================================
// Function Entity Capabilities Tests
// ============================================================================

class FunctionResourceCollections : public ::testing::Test {
 protected:
  void SetUp() override {
    // Build entity hierarchy:
    //
    // Area: perception
    //   Component: sensor_stack
    //     App: camera_driver (services: /camera/start_capture)
    //     App: lidar_proc (services: /lidar/start_scan)
    //
    // Function: sensing (hosts: camera_driver, lidar_proc)

    areas_ = {make_area("perception", "Perception Area")};

    components_.push_back(make_component("sensor_stack", "Sensor Stack", "perception"));

    apps_.push_back(make_app("camera_driver", "Camera Driver", "sensor_stack"));
    apps_[0].services = {make_service("start_capture", "/camera/start_capture")};
    apps_[0].topics.publishes = {"/camera/image", "/camera/info"};
    apps_[0].topics.subscribes = {"/camera/enable"};
    apps_[0].ros_binding = {"camera_driver", "/perception", ""};

    apps_.push_back(make_app("lidar_proc", "LiDAR Processor", "sensor_stack"));
    apps_[1].services = {make_service("start_scan", "/lidar/start_scan")};
    apps_[1].topics.publishes = {"/lidar/points"};
    apps_[1].topics.subscribes = {"/camera/image"};
    apps_[1].ros_binding = {"lidar_proc", "/perception", ""};

    functions_ = {make_function("sensing", "Sensing", {"camera_driver", "lidar_proc"})};

    cache_.update_all(areas_, components_, apps_, functions_);
    service_ = std::make_unique<AggregationService>(&cache_);
  }

  ThreadSafeEntityCache cache_;
  std::unique_ptr<AggregationService> service_;
  std::vector<Area> areas_;
  std::vector<Component> components_;
  std::vector<App> apps_;
  std::vector<Function> functions_;
};

// ============================================================================
// EntityCapabilities: Function supports aggregated collections
// ============================================================================

TEST_F(FunctionResourceCollections, FunctionSupportsFaults) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::FAULTS));
}

TEST_F(FunctionResourceCollections, FunctionSupportsLogs) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::LOGS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::LOGS));
}

TEST_F(FunctionResourceCollections, FunctionSupportsData) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::DATA));
}

TEST_F(FunctionResourceCollections, FunctionSupportsOperations) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::OPERATIONS));
}

TEST_F(FunctionResourceCollections, FunctionSupportsConfigurations) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::CONFIGURATIONS));
}

TEST_F(FunctionResourceCollections, FunctionSupportsBulkData) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::BULK_DATA));
  // Bulk-data uses host-scoped filtering, not aggregation
  EXPECT_FALSE(caps.is_aggregated(ResourceCollection::BULK_DATA));
}

TEST_F(FunctionResourceCollections, FunctionSupportsCyclicSubscriptions) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CYCLIC_SUBSCRIPTIONS));
}

// ============================================================================
// EntityCapabilities: Area supports aggregated collections
// ============================================================================

TEST_F(FunctionResourceCollections, AreaSupportsFaults) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::FAULTS));
}

TEST_F(FunctionResourceCollections, AreaSupportsLogs) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::LOGS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::LOGS));
}

TEST_F(FunctionResourceCollections, AreaSupportsData) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::DATA));
}

TEST_F(FunctionResourceCollections, AreaSupportsOperations) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::OPERATIONS));
}

TEST_F(FunctionResourceCollections, AreaSupportsConfigurations) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::CONFIGURATIONS));
}

TEST_F(FunctionResourceCollections, AreaSupportsBulkData) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::BULK_DATA));
  // Bulk-data uses host-scoped filtering, not aggregation
  EXPECT_FALSE(caps.is_aggregated(ResourceCollection::BULK_DATA));
}

// ============================================================================
// AggregationService: get_child_app_ids
// ============================================================================

TEST_F(FunctionResourceCollections, FunctionChildAppIds) {
  auto ids = service_->get_child_app_ids(SovdEntityType::FUNCTION, "sensing");
  ASSERT_EQ(ids.size(), 2);
  EXPECT_TRUE(std::find(ids.begin(), ids.end(), "camera_driver") != ids.end());
  EXPECT_TRUE(std::find(ids.begin(), ids.end(), "lidar_proc") != ids.end());
}

TEST_F(FunctionResourceCollections, AreaChildAppIds) {
  auto ids = service_->get_child_app_ids(SovdEntityType::AREA, "perception");
  ASSERT_EQ(ids.size(), 2);
  EXPECT_TRUE(std::find(ids.begin(), ids.end(), "camera_driver") != ids.end());
  EXPECT_TRUE(std::find(ids.begin(), ids.end(), "lidar_proc") != ids.end());
}

TEST_F(FunctionResourceCollections, ComponentChildAppIds) {
  auto ids = service_->get_child_app_ids(SovdEntityType::COMPONENT, "sensor_stack");
  ASSERT_EQ(ids.size(), 2);
  EXPECT_TRUE(std::find(ids.begin(), ids.end(), "camera_driver") != ids.end());
  EXPECT_TRUE(std::find(ids.begin(), ids.end(), "lidar_proc") != ids.end());
}

TEST_F(FunctionResourceCollections, AppChildAppIdsReturnsSelf) {
  auto ids = service_->get_child_app_ids(SovdEntityType::APP, "camera_driver");
  ASSERT_EQ(ids.size(), 1);
  EXPECT_EQ(ids[0], "camera_driver");
}

TEST_F(FunctionResourceCollections, UnknownEntityChildAppIdsEmpty) {
  auto ids = service_->get_child_app_ids(SovdEntityType::FUNCTION, "nonexistent");
  EXPECT_TRUE(ids.empty());
}

TEST_F(FunctionResourceCollections, UnknownTypeChildAppIdsEmpty) {
  auto ids = service_->get_child_app_ids(SovdEntityType::UNKNOWN, "sensing");
  EXPECT_TRUE(ids.empty());
}

// ============================================================================
// AggregationService: build_collection_x_medkit
// ============================================================================

TEST_F(FunctionResourceCollections, FunctionCollectionXMedkitHasAggregated) {
  auto x_medkit = service_->build_collection_x_medkit(SovdEntityType::FUNCTION, "sensing");

  EXPECT_TRUE(x_medkit["aggregated"].get<bool>());
  EXPECT_EQ(x_medkit["aggregation_level"], "function");
  ASSERT_TRUE(x_medkit.contains("aggregation_sources"));
  EXPECT_EQ(x_medkit["aggregation_sources"].size(), 2);
}

TEST_F(FunctionResourceCollections, AreaCollectionXMedkitHasAggregated) {
  auto x_medkit = service_->build_collection_x_medkit(SovdEntityType::AREA, "perception");

  EXPECT_TRUE(x_medkit["aggregated"].get<bool>());
  EXPECT_EQ(x_medkit["aggregation_level"], "area");
  ASSERT_TRUE(x_medkit.contains("aggregation_sources"));
  EXPECT_EQ(x_medkit["aggregation_sources"].size(), 2);
}

TEST_F(FunctionResourceCollections, ComponentCollectionXMedkitHasAggregated) {
  auto x_medkit = service_->build_collection_x_medkit(SovdEntityType::COMPONENT, "sensor_stack");

  EXPECT_TRUE(x_medkit["aggregated"].get<bool>());
  EXPECT_EQ(x_medkit["aggregation_level"], "component");
  ASSERT_TRUE(x_medkit.contains("aggregation_sources"));
  EXPECT_EQ(x_medkit["aggregation_sources"].size(), 2);
}

TEST_F(FunctionResourceCollections, AppCollectionXMedkitNotAggregated) {
  auto x_medkit = service_->build_collection_x_medkit(SovdEntityType::APP, "camera_driver");

  EXPECT_FALSE(x_medkit["aggregated"].get<bool>());
  EXPECT_EQ(x_medkit["aggregation_level"], "app");
  // Non-aggregated entities don't have aggregation_sources
  EXPECT_FALSE(x_medkit.contains("aggregation_sources"));
}

TEST_F(FunctionResourceCollections, ServerCollectionXMedkit) {
  auto x_medkit = service_->build_collection_x_medkit(SovdEntityType::SERVER, "server");

  EXPECT_TRUE(x_medkit["aggregated"].get<bool>());
  EXPECT_EQ(x_medkit["aggregation_level"], "server");
}

// ============================================================================
// Function data aggregation via cache
// ============================================================================

TEST_F(FunctionResourceCollections, FunctionDataAggregatesFromHostedApps) {
  auto result = cache_.get_function_data("sensing");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "function");
  EXPECT_GE(result.source_ids.size(), 2ul);

  // Should have topics from both camera_driver and lidar_proc
  // camera_driver: /camera/image, /camera/info (pub), /camera/enable (sub)
  // lidar_proc: /lidar/points (pub), /camera/image (sub - merged to 'both')
  EXPECT_GE(result.topics.size(), 4ul);
}

TEST_F(FunctionResourceCollections, FunctionDataMergesDirections) {
  auto result = cache_.get_function_data("sensing");

  // /camera/image is published by camera_driver and subscribed by lidar_proc
  bool found_camera_image = false;
  for (const auto & topic : result.topics) {
    if (topic.name == "/camera/image") {
      found_camera_image = true;
      EXPECT_EQ(topic.direction, "both") << "/camera/image should be direction=both (pub+sub)";
      break;
    }
  }
  EXPECT_TRUE(found_camera_image) << "/camera/image not found in function data";
}

TEST_F(FunctionResourceCollections, FunctionConfigurationsAggregatesFromHosts) {
  auto result = cache_.get_function_configurations("sensing");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "function");
  EXPECT_GE(result.nodes.size(), 2ul);
}

// ============================================================================
// Area data aggregation via cache
// ============================================================================

TEST_F(FunctionResourceCollections, AreaDataAggregatesFromComponents) {
  auto result = cache_.get_area_data("perception");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "area");
  EXPECT_GE(result.source_ids.size(), 1ul);
  EXPECT_GE(result.topics.size(), 4ul);
}

TEST_F(FunctionResourceCollections, AreaDataIncludesAllAppTopics) {
  auto result = cache_.get_area_data("perception");

  // Check that topics from both apps are present
  bool found_lidar = false;
  bool found_camera = false;
  for (const auto & topic : result.topics) {
    if (topic.name == "/lidar/points") {
      found_lidar = true;
    }
    if (topic.name == "/camera/info") {
      found_camera = true;
    }
  }
  EXPECT_TRUE(found_lidar) << "/lidar/points not found in area data";
  EXPECT_TRUE(found_camera) << "/camera/info not found in area data";
}

// ============================================================================
// get_entity_data auto-detects type and uses aggregation
// ============================================================================

TEST_F(FunctionResourceCollections, EntityDataAutoDetectsFunction) {
  auto result = cache_.get_entity_data("sensing");
  EXPECT_EQ(result.aggregation_level, "function");
  EXPECT_TRUE(result.is_aggregated);
}

TEST_F(FunctionResourceCollections, EntityDataAutoDetectsArea) {
  auto result = cache_.get_entity_data("perception");
  EXPECT_EQ(result.aggregation_level, "area");
  EXPECT_TRUE(result.is_aggregated);
}

TEST_F(FunctionResourceCollections, EntityDataAutoDetectsApp) {
  auto result = cache_.get_entity_data("camera_driver");
  EXPECT_EQ(result.aggregation_level, "app");
  EXPECT_FALSE(result.is_aggregated);
}

// ============================================================================
// Operations aggregation for Function and Area
// ============================================================================

TEST_F(FunctionResourceCollections, FunctionOperationsAggregatesFromHosts) {
  auto result = service_->get_operations(SovdEntityType::FUNCTION, "sensing");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "function");
  // Should have services from both camera_driver and lidar_proc
  EXPECT_EQ(result.services.size(), 2);
  EXPECT_GE(result.source_ids.size(), 2ul);
}

TEST_F(FunctionResourceCollections, AreaOperationsAggregatesFromComponents) {
  auto result = service_->get_operations(SovdEntityType::AREA, "perception");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_EQ(result.aggregation_level, "area");
  EXPECT_GE(result.services.size(), 2ul);
}

TEST_F(FunctionResourceCollections, FunctionOperationsXMedkitCorrect) {
  auto ops = service_->get_operations(SovdEntityType::FUNCTION, "sensing");
  auto x_medkit = AggregationService::build_x_medkit(ops);

  EXPECT_TRUE(x_medkit["aggregated"].get<bool>());
  EXPECT_EQ(x_medkit["aggregation_level"], "function");
  ASSERT_TRUE(x_medkit.contains("aggregation_sources"));
  EXPECT_FALSE(x_medkit["aggregation_sources"].empty());
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_F(FunctionResourceCollections, EmptyFunctionReturnsEmptyAggregation) {
  // Create a function with no hosts
  std::vector<Function> empty_funcs = {make_function("empty_func", "Empty Function", {})};
  cache_.update_all(areas_, components_, apps_, empty_funcs);

  auto ops = service_->get_operations(SovdEntityType::FUNCTION, "empty_func");
  EXPECT_TRUE(ops.services.empty());
  EXPECT_TRUE(ops.actions.empty());

  auto data = cache_.get_function_data("empty_func");
  EXPECT_TRUE(data.topics.empty());

  auto child_ids = service_->get_child_app_ids(SovdEntityType::FUNCTION, "empty_func");
  EXPECT_TRUE(child_ids.empty());
}

TEST_F(FunctionResourceCollections, AreaWithNoComponentsReturnsEmpty) {
  // Create an area with no components
  std::vector<Area> solo_area = {make_area("empty_area", "Empty Area")};
  cache_.update_all(solo_area, {}, {}, {});

  auto data = cache_.get_area_data("empty_area");
  EXPECT_TRUE(data.topics.empty());
  EXPECT_EQ(data.aggregation_level, "area");

  auto child_ids = service_->get_child_app_ids(SovdEntityType::AREA, "empty_area");
  EXPECT_TRUE(child_ids.empty());
}

TEST_F(FunctionResourceCollections, NonexistentFunctionReturnsEmpty) {
  auto ops = service_->get_operations(SovdEntityType::FUNCTION, "nonexistent");
  EXPECT_TRUE(ops.empty());

  auto child_ids = service_->get_child_app_ids(SovdEntityType::FUNCTION, "nonexistent");
  EXPECT_TRUE(child_ids.empty());
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
