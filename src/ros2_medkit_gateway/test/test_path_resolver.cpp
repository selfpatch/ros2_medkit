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

#include "../src/openapi/path_resolver.hpp"

using ros2_medkit_gateway::openapi::PathCategory;
using ros2_medkit_gateway::openapi::PathResolver;

// =============================================================================
// Basic path categories
// =============================================================================

TEST(PathResolverTest, RootPath) {
  // @verifies REQ_OPENAPI_DOCS_ENDPOINT
  auto result = PathResolver::resolve("/");
  EXPECT_EQ(result.category, PathCategory::kRoot);
}

TEST(PathResolverTest, EmptyPath) {
  auto result = PathResolver::resolve("");
  EXPECT_EQ(result.category, PathCategory::kRoot);
}

TEST(PathResolverTest, EntityCollection) {
  // @verifies REQ_OPENAPI_DOCS_ENDPOINT
  auto result = PathResolver::resolve("/areas");
  EXPECT_EQ(result.category, PathCategory::kEntityCollection);
  EXPECT_EQ(result.entity_type, "areas");
}

TEST(PathResolverTest, SpecificEntity) {
  // @verifies REQ_OPENAPI_DOCS_ENDPOINT
  auto result = PathResolver::resolve("/apps/my_app");
  EXPECT_EQ(result.category, PathCategory::kSpecificEntity);
  EXPECT_EQ(result.entity_type, "apps");
  EXPECT_EQ(result.entity_id, "my_app");
}

TEST(PathResolverTest, ResourceCollection) {
  // @verifies REQ_OPENAPI_DOCS_ENDPOINT
  auto result = PathResolver::resolve("/apps/my_app/data");
  EXPECT_EQ(result.category, PathCategory::kResourceCollection);
  EXPECT_EQ(result.entity_id, "my_app");
  EXPECT_EQ(result.resource_collection, "data");
}

TEST(PathResolverTest, SpecificResource) {
  // @verifies REQ_OPENAPI_DOCS_ENDPOINT
  auto result = PathResolver::resolve("/apps/my_app/data/temperature");
  EXPECT_EQ(result.category, PathCategory::kSpecificResource);
  EXPECT_EQ(result.entity_id, "my_app");
  EXPECT_EQ(result.resource_collection, "data");
  EXPECT_EQ(result.resource_id, "temperature");
}

// =============================================================================
// Nested paths with parent chain
// =============================================================================

TEST(PathResolverTest, DeepNestedPath) {
  // @verifies REQ_OPENAPI_DOCS_ENDPOINT
  auto result = PathResolver::resolve("/areas/powertrain/components/engine/apps/ecu/data/temp");
  EXPECT_EQ(result.category, PathCategory::kSpecificResource);
  EXPECT_EQ(result.entity_id, "ecu");
  EXPECT_EQ(result.resource_collection, "data");
  EXPECT_EQ(result.resource_id, "temp");
  ASSERT_EQ(result.parent_chain.size(), 2u);
  EXPECT_EQ(result.parent_chain[0].entity_type, "areas");
  EXPECT_EQ(result.parent_chain[0].entity_id, "powertrain");
  EXPECT_EQ(result.parent_chain[1].entity_type, "components");
  EXPECT_EQ(result.parent_chain[1].entity_id, "engine");
}

TEST(PathResolverTest, NestedEntityCollection) {
  // @verifies REQ_OPENAPI_DOCS_ENDPOINT
  auto result = PathResolver::resolve("/areas/powertrain/components");
  EXPECT_EQ(result.category, PathCategory::kEntityCollection);
  EXPECT_EQ(result.entity_type, "components");
  ASSERT_EQ(result.parent_chain.size(), 1u);
  EXPECT_EQ(result.parent_chain[0].entity_type, "areas");
  EXPECT_EQ(result.parent_chain[0].entity_id, "powertrain");
}

TEST(PathResolverTest, NestedSpecificEntity) {
  auto result = PathResolver::resolve("/areas/powertrain/components/engine");
  EXPECT_EQ(result.category, PathCategory::kSpecificEntity);
  EXPECT_EQ(result.entity_type, "components");
  EXPECT_EQ(result.entity_id, "engine");
  ASSERT_EQ(result.parent_chain.size(), 1u);
  EXPECT_EQ(result.parent_chain[0].entity_type, "areas");
  EXPECT_EQ(result.parent_chain[0].entity_id, "powertrain");
}

TEST(PathResolverTest, NestedResourceCollection) {
  auto result = PathResolver::resolve("/components/engine/apps/ecu/faults");
  EXPECT_EQ(result.category, PathCategory::kResourceCollection);
  EXPECT_EQ(result.entity_type, "apps");
  EXPECT_EQ(result.entity_id, "ecu");
  EXPECT_EQ(result.resource_collection, "faults");
  ASSERT_EQ(result.parent_chain.size(), 1u);
  EXPECT_EQ(result.parent_chain[0].entity_type, "components");
  EXPECT_EQ(result.parent_chain[0].entity_id, "engine");
}

// =============================================================================
// Reserved and invalid paths
// =============================================================================

TEST(PathResolverTest, ReservedDocsSegment) {
  auto result = PathResolver::resolve("/apps/docs");
  EXPECT_EQ(result.category, PathCategory::kError);
  EXPECT_EQ(result.error, "Reserved path segment: docs");
}

TEST(PathResolverTest, InvalidPath) {
  auto result = PathResolver::resolve("/nonexistent/path");
  EXPECT_EQ(result.category, PathCategory::kUnresolved);
}

TEST(PathResolverTest, InvalidPathSingleSegment) {
  auto result = PathResolver::resolve("/foobar");
  EXPECT_EQ(result.category, PathCategory::kUnresolved);
}

// =============================================================================
// All entity type keywords
// =============================================================================

TEST(PathResolverTest, AllEntityTypeKeywords) {
  for (const auto & type : {"areas", "components", "apps", "functions"}) {
    auto result = PathResolver::resolve(std::string("/") + type);
    EXPECT_EQ(result.category, PathCategory::kEntityCollection) << "Failed for: " << type;
    EXPECT_EQ(result.entity_type, type) << "Failed for: " << type;
  }
}

TEST(PathResolverTest, NestedEntityTypeKeywords) {
  auto result = PathResolver::resolve("/areas/powertrain/subareas");
  EXPECT_EQ(result.category, PathCategory::kEntityCollection);
  EXPECT_EQ(result.entity_type, "subareas");

  result = PathResolver::resolve("/functions/diag/hosts");
  EXPECT_EQ(result.category, PathCategory::kEntityCollection);
  EXPECT_EQ(result.entity_type, "hosts");
}

TEST(PathResolverTest, SubcomponentsKeyword) {
  auto result = PathResolver::resolve("/components/engine/subcomponents");
  EXPECT_EQ(result.category, PathCategory::kEntityCollection);
  EXPECT_EQ(result.entity_type, "subcomponents");
}

// =============================================================================
// All resource collection keywords
// =============================================================================

TEST(PathResolverTest, AllResourceCollectionKeywords) {
  for (const auto & coll : {"data", "operations", "faults", "configurations", "logs", "bulk-data",
                            "cyclic-subscriptions", "triggers", "updates", "data-categories", "data-groups"}) {
    auto result = PathResolver::resolve(std::string("/apps/my_app/") + coll);
    EXPECT_EQ(result.category, PathCategory::kResourceCollection) << "Failed for: " << coll;
    EXPECT_EQ(result.resource_collection, coll) << "Failed for: " << coll;
  }
}

// =============================================================================
// Edge cases
// =============================================================================

TEST(PathResolverTest, TrailingSlash) {
  // Trailing slash gets split into empty segment which is filtered out
  auto result = PathResolver::resolve("/apps/");
  EXPECT_EQ(result.category, PathCategory::kEntityCollection);
  EXPECT_EQ(result.entity_type, "apps");
}

TEST(PathResolverTest, EntityTypeAsEntityId) {
  // "areas" as an ID under "components" - should work since ID position context matters
  auto result = PathResolver::resolve("/components/areas");
  EXPECT_EQ(result.category, PathCategory::kSpecificEntity);
  EXPECT_EQ(result.entity_type, "components");
  EXPECT_EQ(result.entity_id, "areas");
}

TEST(PathResolverTest, ReservedSegmentInNestedPath) {
  auto result = PathResolver::resolve("/areas/powertrain/components/docs");
  EXPECT_EQ(result.category, PathCategory::kError);
  EXPECT_EQ(result.error, "Reserved path segment: docs");
}

TEST(PathResolverTest, SpecificResourcePreservesEntityInfo) {
  auto result = PathResolver::resolve("/components/engine/operations/start_motor");
  EXPECT_EQ(result.category, PathCategory::kSpecificResource);
  EXPECT_EQ(result.entity_type, "components");
  EXPECT_EQ(result.entity_id, "engine");
  EXPECT_EQ(result.resource_collection, "operations");
  EXPECT_EQ(result.resource_id, "start_motor");
  EXPECT_TRUE(result.parent_chain.empty());
}

TEST(PathResolverTest, ResourceCollectionPreservesEntityInfo) {
  auto result = PathResolver::resolve("/areas/powertrain/configurations");
  EXPECT_EQ(result.category, PathCategory::kResourceCollection);
  EXPECT_EQ(result.entity_type, "areas");
  EXPECT_EQ(result.entity_id, "powertrain");
  EXPECT_EQ(result.resource_collection, "configurations");
  EXPECT_TRUE(result.parent_chain.empty());
}
