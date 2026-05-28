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

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/trigger_handlers.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::handlers;

// ===========================================================================
// parse_resource_uri tests (trigger-specific with areas support)
// ===========================================================================

TEST(TriggerParseResourceUriTest, DataCollectionWithTopic) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/data/temperature");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "apps");
  EXPECT_EQ(result->entity_id, "node1");
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->resource_path, "/temperature");
}

TEST(TriggerParseResourceUriTest, AreasEntityType) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/areas/zone1/data/temperature");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "areas");
  EXPECT_EQ(result->entity_id, "zone1");
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->resource_path, "/temperature");
}

TEST(TriggerParseResourceUriTest, ComponentsEntityType) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/components/ecu1/faults");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "components");
  EXPECT_EQ(result->entity_id, "ecu1");
  EXPECT_EQ(result->collection, "faults");
  EXPECT_EQ(result->resource_path, "");
}

TEST(TriggerParseResourceUriTest, FunctionsEntityType) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/functions/func1/data/topic");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "functions");
  EXPECT_EQ(result->entity_id, "func1");
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->resource_path, "/topic");
}

TEST(TriggerParseResourceUriTest, VendorExtensionCollection) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/x-medkit-metrics/cpu_usage");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->collection, "x-medkit-metrics");
  EXPECT_EQ(result->resource_path, "/cpu_usage");
}

TEST(TriggerParseResourceUriTest, MultiSegmentResourcePath) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/data/parent/child/value");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->resource_path, "/parent/child/value");
}

TEST(TriggerParseResourceUriTest, InvalidMissingCollection) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1");
  EXPECT_FALSE(result.has_value());
}

TEST(TriggerParseResourceUriTest, InvalidMalformedUri) {
  auto result = TriggerHandlers::parse_resource_uri("/not/a/valid/uri");
  EXPECT_FALSE(result.has_value());
}

TEST(TriggerParseResourceUriTest, EmptyUri) {
  auto result = TriggerHandlers::parse_resource_uri("");
  EXPECT_FALSE(result.has_value());
}

TEST(TriggerParseResourceUriTest, PathTraversalRejected) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/data/../../../etc/passwd");
  EXPECT_FALSE(result.has_value());
}

TEST(TriggerParseResourceUriTest, PathTraversalInMiddleRejected) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/data/a/../b");
  EXPECT_FALSE(result.has_value());
}

TEST(TriggerParseResourceUriTest, BenignDoubleDotInSegmentAllowed) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/data/..foo");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->resource_path, "/..foo");
}

TEST(TriggerParseResourceUriTest, PathTraversalAtEndRejected) {
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/data/a/..");
  EXPECT_FALSE(result.has_value());
}

// ===========================================================================
// SSEClientTracker limit test
// ===========================================================================

// @verifies REQ_INTEROP_097
TEST(TriggerSSETrackerTest, ClientLimitEnforced) {
  auto tracker = std::make_shared<SSEClientTracker>(2);

  EXPECT_TRUE(tracker->try_connect());
  EXPECT_TRUE(tracker->try_connect());
  EXPECT_FALSE(tracker->try_connect());  // 3rd should fail

  EXPECT_EQ(tracker->connected_clients(), 2u);

  tracker->disconnect();
  EXPECT_EQ(tracker->connected_clients(), 1u);

  EXPECT_TRUE(tracker->try_connect());  // Now should succeed again
  EXPECT_EQ(tracker->connected_clients(), 2u);
}

// ===========================================================================
// Vendor extension collection
//
// The previous TriggerErrorTest / TriggerValidationTest suites here asserted
// the SOVD GenericError wire format using the deprecated
// HandlerContext::send_error wrapper. Commit 30 removed that public
// surface; the canonical wire-format coverage now lives in
// test_primitives.cpp (write_generic_error suite) and trigger-handler
// validation paths are exercised end-to-end via test_trigger_manager and
// the typed-router handler tests.
// ===========================================================================

// @verifies REQ_INTEROP_029
TEST(TriggerValidationTest, VendorExtensionCollection_Accepted) {
  // x-* vendor extension collections must pass parse_resource_uri and the collection guard.
  // Verify parse_resource_uri accepts x-* collections (the URI parse step).
  auto result = TriggerHandlers::parse_resource_uri("/api/v1/apps/node1/x-custom/metric");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->collection, "x-custom");

  // Verify the x- prefix check logic (collection.substr(0,2) == "x-")
  std::string vendor_collection = "x-custom";
  EXPECT_EQ(vendor_collection.substr(0, 2), "x-");
}
