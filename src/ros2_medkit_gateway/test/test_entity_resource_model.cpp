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

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/models/aggregation_service.hpp"
#include "ros2_medkit_gateway/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/models/entity_types.hpp"
#include "ros2_medkit_gateway/models/thread_safe_entity_cache.hpp"

using namespace ros2_medkit_gateway;
using namespace std::chrono_literals;

// ============================================================================
// EntityTypes Tests
// ============================================================================

TEST(EntityTypes, ToStringReturnsCorrectValues) {
  EXPECT_EQ(to_string(SovdEntityType::SERVER), "Server");
  EXPECT_EQ(to_string(SovdEntityType::AREA), "Area");
  EXPECT_EQ(to_string(SovdEntityType::COMPONENT), "Component");
  EXPECT_EQ(to_string(SovdEntityType::APP), "App");
  EXPECT_EQ(to_string(SovdEntityType::FUNCTION), "Function");
  EXPECT_EQ(to_string(SovdEntityType::UNKNOWN), "Unknown");
}

TEST(EntityTypes, ResourceCollectionToString) {
  EXPECT_EQ(to_string(ResourceCollection::CONFIGURATIONS), "configurations");
  EXPECT_EQ(to_string(ResourceCollection::DATA), "data");
  EXPECT_EQ(to_string(ResourceCollection::FAULTS), "faults");
  EXPECT_EQ(to_string(ResourceCollection::OPERATIONS), "operations");
  EXPECT_EQ(to_string(ResourceCollection::BULK_DATA), "bulk-data");
  EXPECT_EQ(to_string(ResourceCollection::DATA_LISTS), "data-lists");
}

TEST(EntityTypes, ParseResourceCollection) {
  auto configs = parse_resource_collection("configurations");
  ASSERT_TRUE(configs.has_value());
  EXPECT_EQ(*configs, ResourceCollection::CONFIGURATIONS);

  auto data_lists = parse_resource_collection("data-lists");
  ASSERT_TRUE(data_lists.has_value());
  EXPECT_EQ(*data_lists, ResourceCollection::DATA_LISTS);

  auto invalid = parse_resource_collection("invalid");
  EXPECT_FALSE(invalid.has_value());
}

TEST(EntityTypes, ParseEntityType) {
  EXPECT_EQ(parse_entity_type("Component"), SovdEntityType::COMPONENT);
  EXPECT_EQ(parse_entity_type("component"), SovdEntityType::COMPONENT);
  EXPECT_EQ(parse_entity_type("COMPONENT"), SovdEntityType::COMPONENT);
  EXPECT_EQ(parse_entity_type("App"), SovdEntityType::APP);
  EXPECT_EQ(parse_entity_type("application"), SovdEntityType::APP);
  EXPECT_EQ(parse_entity_type("invalid"), SovdEntityType::UNKNOWN);
}

// ============================================================================
// EntityCapabilities Tests
// ============================================================================

TEST(EntityCapabilities, ServerSupportsAllCollections) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::SERVER);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::FAULTS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
}

TEST(EntityCapabilities, AreaDoesNotSupportCollections) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::FAULTS));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::OPERATIONS));
}

TEST(EntityCapabilities, AreaSupportsContains) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::AREA);
  EXPECT_TRUE(caps.supports_resource("contains"));
  EXPECT_TRUE(caps.supports_resource("subareas"));
  EXPECT_TRUE(caps.supports_resource("docs"));
}

TEST(EntityCapabilities, ComponentSupportsOperations) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::COMPONENT);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.supports_resource("hosts"));
}

TEST(EntityCapabilities, AppSupportsIsLocatedOn) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::APP);
  EXPECT_TRUE(caps.supports_resource("is-located-on"));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_FALSE(caps.supports_resource("hosts"));  // Apps don't host anything
}

TEST(EntityCapabilities, FunctionAggregatesOperations) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::FUNCTION);
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::DATA));
  EXPECT_TRUE(caps.supports_collection(ResourceCollection::OPERATIONS));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::DATA));
  EXPECT_TRUE(caps.is_aggregated(ResourceCollection::OPERATIONS));
  EXPECT_FALSE(caps.supports_collection(ResourceCollection::CONFIGURATIONS));
}

TEST(EntityCapabilities, UnknownTypeHasNoCapabilities) {
  auto caps = EntityCapabilities::for_type(SovdEntityType::UNKNOWN);
  EXPECT_TRUE(caps.collections().empty());
  EXPECT_TRUE(caps.resources().empty());
}

// ============================================================================
// ThreadSafeEntityCache Tests
// ============================================================================

class EntityCacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create test data
    areas_ = {
        Area{.id = "perception", .name = "Perception"},
        Area{.id = "control", .name = "Control"},
    };

    components_ = {
        Component{.id = "lidar", .name = "LiDAR Driver", .area = "perception"},
        Component{.id = "nav2", .name = "Nav2 Stack", .area = "control"},
    };

    apps_ = {
        App{.id = "lidar_app", .name = "LiDAR App", .component_id = "lidar"},
        App{.id = "controller", .name = "Controller", .component_id = "nav2"},
        App{.id = "planner", .name = "Planner", .component_id = "nav2"},
    };

    // Add operations to apps
    apps_[0].services = {{.name = "start_scan", .full_path = "/lidar/start_scan"}};
    apps_[1].services = {{.name = "follow_path", .full_path = "/nav2/follow_path"}};
    apps_[1].actions = {{.name = "navigate", .full_path = "/nav2/navigate"}};
    apps_[2].services = {{.name = "compute_path", .full_path = "/nav2/compute_path"}};
  }

  ThreadSafeEntityCache cache_;
  std::vector<Area> areas_;
  std::vector<Component> components_;
  std::vector<App> apps_;
};

TEST_F(EntityCacheTest, EmptyCacheReturnsEmptyVectors) {
  EXPECT_TRUE(cache_.get_areas().empty());
  EXPECT_TRUE(cache_.get_components().empty());
  EXPECT_TRUE(cache_.get_apps().empty());
}

TEST_F(EntityCacheTest, UpdateAllStoresAllEntities) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_EQ(cache_.get_areas().size(), 2);
  EXPECT_EQ(cache_.get_components().size(), 2);
  EXPECT_EQ(cache_.get_apps().size(), 3);
}

TEST_F(EntityCacheTest, GetByIdReturnsCorrectEntity) {
  cache_.update_all(areas_, components_, apps_, {});

  auto area = cache_.get_area("perception");
  ASSERT_TRUE(area.has_value());
  EXPECT_EQ(area->name, "Perception");

  auto comp = cache_.get_component("nav2");
  ASSERT_TRUE(comp.has_value());
  EXPECT_EQ(comp->name, "Nav2 Stack");

  auto app = cache_.get_app("planner");
  ASSERT_TRUE(app.has_value());
  EXPECT_EQ(app->name, "Planner");
}

TEST_F(EntityCacheTest, GetByIdReturnsNulloptForUnknown) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_FALSE(cache_.get_area("unknown").has_value());
  EXPECT_FALSE(cache_.get_component("unknown").has_value());
  EXPECT_FALSE(cache_.get_app("unknown").has_value());
}

TEST_F(EntityCacheTest, HasEntityReturnsCorrectValue) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_TRUE(cache_.has_component("nav2"));
  EXPECT_FALSE(cache_.has_component("unknown"));
  EXPECT_TRUE(cache_.has_app("controller"));
  EXPECT_FALSE(cache_.has_app("unknown"));
}

TEST_F(EntityCacheTest, FindEntityReturnsCorrectType) {
  cache_.update_all(areas_, components_, apps_, {});

  auto comp_ref = cache_.find_entity("nav2");
  ASSERT_TRUE(comp_ref.has_value());
  EXPECT_EQ(comp_ref->type, SovdEntityType::COMPONENT);

  auto app_ref = cache_.find_entity("controller");
  ASSERT_TRUE(app_ref.has_value());
  EXPECT_EQ(app_ref->type, SovdEntityType::APP);

  auto area_ref = cache_.find_entity("perception");
  ASSERT_TRUE(area_ref.has_value());
  EXPECT_EQ(area_ref->type, SovdEntityType::AREA);

  EXPECT_FALSE(cache_.find_entity("unknown").has_value());
}

TEST_F(EntityCacheTest, GetEntityTypeReturnsCorrectType) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_EQ(cache_.get_entity_type("nav2"), SovdEntityType::COMPONENT);
  EXPECT_EQ(cache_.get_entity_type("controller"), SovdEntityType::APP);
  EXPECT_EQ(cache_.get_entity_type("perception"), SovdEntityType::AREA);
  EXPECT_EQ(cache_.get_entity_type("unknown"), SovdEntityType::UNKNOWN);
}

// ============================================================================
// Relationship Index Tests
// ============================================================================

TEST_F(EntityCacheTest, GetAppsForComponentReturnsCorrectApps) {
  cache_.update_all(areas_, components_, apps_, {});

  auto apps = cache_.get_apps_for_component("nav2");
  ASSERT_EQ(apps.size(), 2);
  EXPECT_TRUE(std::find(apps.begin(), apps.end(), "controller") != apps.end());
  EXPECT_TRUE(std::find(apps.begin(), apps.end(), "planner") != apps.end());
}

TEST_F(EntityCacheTest, GetAppsForComponentReturnsEmptyForUnknown) {
  cache_.update_all(areas_, components_, apps_, {});

  auto apps = cache_.get_apps_for_component("unknown");
  EXPECT_TRUE(apps.empty());
}

TEST_F(EntityCacheTest, GetComponentsForAreaReturnsCorrectComponents) {
  cache_.update_all(areas_, components_, apps_, {});

  auto comps = cache_.get_components_for_area("perception");
  ASSERT_EQ(comps.size(), 1);
  EXPECT_EQ(comps[0], "lidar");
}

// ============================================================================
// Aggregation Tests
// ============================================================================

TEST_F(EntityCacheTest, AppOperationsNotAggregated) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_app_operations("controller");
  EXPECT_EQ(ops.aggregation_level, "app");
  EXPECT_FALSE(ops.is_aggregated);
  EXPECT_EQ(ops.services.size(), 1);
  EXPECT_EQ(ops.actions.size(), 1);
  EXPECT_EQ(ops.source_ids.size(), 1);
}

TEST_F(EntityCacheTest, ComponentOperationsAggregatesFromApps) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_component_operations("nav2");
  EXPECT_EQ(ops.aggregation_level, "component");
  EXPECT_TRUE(ops.is_aggregated);

  // Should have operations from both controller and planner apps
  EXPECT_EQ(ops.services.size(), 2);  // follow_path + compute_path
  EXPECT_EQ(ops.actions.size(), 1);   // navigate

  // Source IDs should include component and both apps
  EXPECT_EQ(ops.source_ids.size(), 3);
}

TEST_F(EntityCacheTest, AreaOperationsAggregatesFromComponents) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_area_operations("control");
  EXPECT_EQ(ops.aggregation_level, "area");
  EXPECT_TRUE(ops.is_aggregated);

  // Should have all operations from nav2 component and its apps
  EXPECT_GE(ops.services.size(), 2ul);
}

TEST_F(EntityCacheTest, AggregationDeduplicatesByPath) {
  // Create apps with duplicate operation paths
  apps_[1].services.push_back({.name = "shared_svc", .full_path = "/shared"});
  apps_[2].services.push_back({.name = "shared_svc", .full_path = "/shared"});

  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_component_operations("nav2");

  // Count occurrences of /shared path
  int shared_count = 0;
  for (const auto & svc : ops.services) {
    if (svc.full_path == "/shared") {
      shared_count++;
    }
  }
  EXPECT_EQ(shared_count, 1) << "Duplicate operation path not deduplicated";
}

TEST_F(EntityCacheTest, EmptyComponentReturnsEmptyAggregation) {
  cache_.update_all(areas_, components_, apps_, {});

  auto ops = cache_.get_component_operations("unknown");
  EXPECT_TRUE(ops.empty());
}

// ============================================================================
// Operation Index Tests
// ============================================================================

TEST_F(EntityCacheTest, FindOperationOwnerReturnsCorrectEntity) {
  cache_.update_all(areas_, components_, apps_, {});

  auto owner = cache_.find_operation_owner("/nav2/navigate");
  ASSERT_TRUE(owner.has_value());
  EXPECT_EQ(owner->type, SovdEntityType::APP);
}

TEST_F(EntityCacheTest, FindOperationOwnerReturnsNulloptForUnknown) {
  cache_.update_all(areas_, components_, apps_, {});

  EXPECT_FALSE(cache_.find_operation_owner("/unknown/op").has_value());
}

// ============================================================================
// Thread-Safety Tests
// ============================================================================

TEST_F(EntityCacheTest, ConcurrentReadsDoNotBlock) {
  cache_.update_all(areas_, components_, apps_, {});

  std::atomic<int> completed{0};
  std::vector<std::thread> readers;

  // Start multiple readers
  for (int i = 0; i < 10; ++i) {
    readers.emplace_back([this, &completed] {
      for (int j = 0; j < 100; ++j) {
        auto components = cache_.get_components();
        auto component = cache_.get_component("nav2");
        (void)components;
        (void)component;
      }
      completed++;
    });
  }

  // All readers should complete quickly
  auto start = std::chrono::high_resolution_clock::now();
  for (auto & t : readers) {
    t.join();
  }
  auto duration = std::chrono::high_resolution_clock::now() - start;

  EXPECT_EQ(completed, 10);
  EXPECT_LT(duration, 1s) << "Concurrent reads took too long - possible blocking";
}

TEST_F(EntityCacheTest, ConcurrentReadsAndWritesDoNotDeadlock) {
  cache_.update_all(areas_, components_, apps_, {});

  std::atomic<bool> keep_running{true};
  std::atomic<int> read_count{0};
  std::atomic<int> write_count{0};

  // Multiple readers
  std::vector<std::thread> readers;
  for (int i = 0; i < 4; ++i) {
    readers.emplace_back([this, &keep_running, &read_count] {
      while (keep_running) {
        auto comps = cache_.get_components();
        (void)comps;
        read_count++;
      }
    });
  }

  // Single writer
  std::thread writer([this, &keep_running, &write_count] {
    while (keep_running) {
      cache_.update_all(areas_, components_, apps_, {});
      write_count++;
    }
  });

  // Run for a short time
  std::this_thread::sleep_for(50ms);
  keep_running = false;

  for (auto & t : readers) {
    t.join();
  }
  writer.join();

  // Ensure both readers and writers made progress
  EXPECT_GT(read_count.load(), 0) << "Readers made no progress - possible deadlock";
  EXPECT_GT(write_count.load(), 0) << "Writer made no progress - possible deadlock";
}

// ============================================================================
// Validation Tests
// ============================================================================

TEST_F(EntityCacheTest, ValidateReturnsEmptyForConsistentCache) {
  cache_.update_all(areas_, components_, apps_, {});

  auto error = cache_.validate();
  EXPECT_TRUE(error.empty()) << "Validation failed: " << error;
}

TEST_F(EntityCacheTest, GetStatsReturnsCorrectCounts) {
  cache_.update_all(areas_, components_, apps_, {});

  auto stats = cache_.get_stats();
  EXPECT_EQ(stats.area_count, 2);
  EXPECT_EQ(stats.component_count, 2);
  EXPECT_EQ(stats.app_count, 3);
  EXPECT_GT(stats.total_operations, 0ul);
}

// ============================================================================
// AggregationService Tests
// ============================================================================

class AggregationServiceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup test entities with operations
    areas_ = {{.id = "navigation", .name = "Navigation"}};

    components_.push_back(Component{.id = "nav_stack", .name = "Nav Stack", .area = "navigation"});
    components_[0].services = {{.name = "get_state", .full_path = "/nav/get_state"}};

    apps_ = {
        App{.id = "controller", .component_id = "nav_stack"},
        App{.id = "planner", .component_id = "nav_stack"},
    };
    apps_[0].services = {{.name = "follow", .full_path = "/nav/follow"}};
    apps_[1].services = {{.name = "plan", .full_path = "/nav/plan"}};
    apps_[1].actions = {{.name = "compute", .full_path = "/nav/compute"}};

    cache_.update_all(areas_, components_, apps_, {});
    service_ = std::make_unique<AggregationService>(&cache_);
  }

  ThreadSafeEntityCache cache_;
  std::unique_ptr<AggregationService> service_;
  std::vector<Area> areas_;
  std::vector<Component> components_;
  std::vector<App> apps_;
};

TEST_F(AggregationServiceTest, AppReturnsOwnOperationsOnly) {
  auto result = service_->get_operations(SovdEntityType::APP, "controller");

  EXPECT_FALSE(result.is_aggregated);
  EXPECT_EQ(result.services.size(), 1);
  EXPECT_EQ(result.services[0].name, "follow");
  EXPECT_EQ(result.source_ids.size(), 1);
}

TEST_F(AggregationServiceTest, ComponentAggregatesFromApps) {
  auto result = service_->get_operations(SovdEntityType::COMPONENT, "nav_stack");

  EXPECT_TRUE(result.is_aggregated);
  // Component's own (get_state) + controller (follow) + planner (plan)
  EXPECT_EQ(result.services.size(), 3);
  EXPECT_EQ(result.actions.size(), 1);       // compute
  EXPECT_GE(result.source_ids.size(), 3ul);  // component + 2 apps
}

TEST_F(AggregationServiceTest, AreaAggregatesFromComponents) {
  auto result = service_->get_operations(SovdEntityType::AREA, "navigation");

  EXPECT_TRUE(result.is_aggregated);
  EXPECT_GE(result.services.size(), 3ul);
  EXPECT_GE(result.source_ids.size(), 1ul);
}

TEST_F(AggregationServiceTest, XMedkitMetadataCorrect) {
  auto result = service_->get_operations(SovdEntityType::COMPONENT, "nav_stack");
  auto json = AggregationService::build_x_medkit(result);

  EXPECT_TRUE(json["aggregated"].get<bool>());
  EXPECT_FALSE(json["aggregation_sources"].empty());
  EXPECT_EQ(json["aggregation_level"], "component");
}

TEST_F(AggregationServiceTest, EmptyEntityReturnsEmptyResult) {
  auto result = service_->get_operations(SovdEntityType::COMPONENT, "unknown");

  EXPECT_TRUE(result.services.empty());
  EXPECT_TRUE(result.actions.empty());
  EXPECT_FALSE(result.is_aggregated);
}

TEST_F(AggregationServiceTest, SupportsOperationsCheckCorrect) {
  EXPECT_TRUE(AggregationService::supports_operations(SovdEntityType::COMPONENT));
  EXPECT_TRUE(AggregationService::supports_operations(SovdEntityType::APP));
  EXPECT_TRUE(AggregationService::supports_operations(SovdEntityType::FUNCTION));
  EXPECT_FALSE(AggregationService::supports_operations(SovdEntityType::AREA));
}

TEST_F(AggregationServiceTest, ShouldAggregateCheckCorrect) {
  EXPECT_TRUE(AggregationService::should_aggregate(SovdEntityType::COMPONENT));
  EXPECT_TRUE(AggregationService::should_aggregate(SovdEntityType::AREA));
  EXPECT_TRUE(AggregationService::should_aggregate(SovdEntityType::FUNCTION));
  EXPECT_FALSE(AggregationService::should_aggregate(SovdEntityType::APP));
}

TEST_F(AggregationServiceTest, GetOperationsByIdAutoDetectsType) {
  auto result = service_->get_operations_by_id("nav_stack");
  EXPECT_EQ(result.aggregation_level, "component");

  result = service_->get_operations_by_id("controller");
  EXPECT_EQ(result.aggregation_level, "app");
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
