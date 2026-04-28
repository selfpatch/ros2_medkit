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
#include "ros2_medkit_gateway/http/handlers/cyclic_subscription_handlers.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::handlers;
// json alias already imported via the `using namespace` above (defined in
// core/auth/auth_models.hpp). A local `using json = nlohmann::json;` would
// shadow it and trip clang-diagnostic-shadow under clang-tidy.

// --- parse_resource_uri tests ---

// @verifies REQ_INTEROP_089
TEST(ParseResourceUriTest, DataCollectionWithTopic) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/data/temperature");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "apps");
  EXPECT_EQ(result->entity_id, "node1");
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->resource_path, "/temperature");
}

TEST(ParseResourceUriTest, FaultsCollectionNoPath) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/components/ecu1/faults");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "components");
  EXPECT_EQ(result->entity_id, "ecu1");
  EXPECT_EQ(result->collection, "faults");
  EXPECT_EQ(result->resource_path, "");
}

TEST(ParseResourceUriTest, FaultsCollectionWithId) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/faults/fault_001");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->collection, "faults");
  EXPECT_EQ(result->resource_path, "/fault_001");
}

TEST(ParseResourceUriTest, ConfigurationsCollection) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/components/ecu1/configurations/param1");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->collection, "configurations");
  EXPECT_EQ(result->resource_path, "/param1");
}

TEST(ParseResourceUriTest, VendorExtensionCollection) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/x-medkit-metrics/cpu_usage");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->collection, "x-medkit-metrics");
  EXPECT_EQ(result->resource_path, "/cpu_usage");
}

TEST(ParseResourceUriTest, FunctionVendorExtensionCollection) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/functions/func1/x-medkit-graph");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "functions");
  EXPECT_EQ(result->entity_id, "func1");
  EXPECT_EQ(result->collection, "x-medkit-graph");
  EXPECT_EQ(result->resource_path, "");
}

TEST(ParseResourceUriTest, MultiSegmentResourcePath) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/data/parent/child/value");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->resource_path, "/parent/child/value");
}

TEST(ParseResourceUriTest, InvalidMissingCollection) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1");
  EXPECT_FALSE(result.has_value());
}

TEST(ParseResourceUriTest, InvalidMalformedUri) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/not/a/valid/uri");
  EXPECT_FALSE(result.has_value());
}

TEST(ParseResourceUriTest, FunctionEntityTypeSupported) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/functions/func1/data/topic");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "functions");
  EXPECT_EQ(result->entity_id, "func1");
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->resource_path, "/topic");
}

TEST(ParseResourceUriTest, PathTraversalRejected) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/data/../../../etc/passwd");
  EXPECT_FALSE(result.has_value());
}

TEST(ParseResourceUriTest, PathTraversalInMiddleRejected) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/data/a/../b");
  EXPECT_FALSE(result.has_value());
}

TEST(ParseResourceUriTest, BenignDoubleDotInSegmentAllowed) {
  // "/..foo" is not a traversal - '..' is part of a larger segment name
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/data/..foo");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->resource_path, "/..foo");
}

TEST(ParseResourceUriTest, PathTraversalAtEndRejected) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/data/a/..");
  EXPECT_FALSE(result.has_value());
}

TEST(ParseResourceUriTest, DataCollectionEmptyResourcePath) {
  // data collection without a topic path - still parses, but handler rejects it
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/apps/node1/data");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->resource_path, "");
}

// --- parse_resource_uri: server-level update status ---

// @verifies REQ_INTEROP_089
TEST(ParseResourceUriTest, UpdateStatusUri) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/updates/my-package/status");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->entity_type, "");
  EXPECT_EQ(result->entity_id, "");
  EXPECT_EQ(result->collection, "updates");
  EXPECT_EQ(result->resource_path, "my-package");
}

TEST(ParseResourceUriTest, UpdateStatusUriWithHyphenatedId) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/updates/ADAS-v2-03-2154/status");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->collection, "updates");
  EXPECT_EQ(result->resource_path, "ADAS-v2-03-2154");
}

TEST(ParseResourceUriTest, UpdateStatusUriMissingStatus) {
  // /api/v1/updates/{id} without /status is not a subscribable resource
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/updates/my-package");
  EXPECT_FALSE(result.has_value());
}

TEST(ParseResourceUriTest, UpdateStatusUriMissingId) {
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/updates//status");
  EXPECT_FALSE(result.has_value());
}

TEST(ParseResourceUriTest, UpdatesListNotSubscribable) {
  // /api/v1/updates (list endpoint) is not subscribable
  auto result = CyclicSubscriptionHandlers::parse_resource_uri("/api/v1/updates");
  EXPECT_FALSE(result.has_value());
}

// --- subscription_to_json ---

// @verifies REQ_INTEROP_089
TEST(CyclicSubscriptionJsonTest, ContainsAllRequiredFields) {
  CyclicSubscriptionInfo info;
  info.id = "sub_001";
  info.entity_id = "temp_sensor";
  info.entity_type = "apps";
  info.resource_uri = "/api/v1/apps/temp_sensor/data/temperature";
  info.protocol = "sse";
  info.interval = CyclicInterval::NORMAL;

  std::string event_source = "/api/v1/apps/temp_sensor/cyclic-subscriptions/sub_001/events";
  auto j = CyclicSubscriptionHandlers::subscription_to_json(info, event_source);

  EXPECT_EQ(j["id"], "sub_001");
  EXPECT_EQ(j["observed_resource"], info.resource_uri);
  EXPECT_EQ(j["event_source"], event_source);
  EXPECT_EQ(j["protocol"], "sse");
  EXPECT_EQ(j["interval"], "normal");
}

TEST(CyclicSubscriptionJsonTest, AllIntervalValuesSerialize) {
  CyclicSubscriptionInfo info;
  info.id = "sub_001";
  info.entity_type = "apps";
  info.entity_id = "e";

  for (const auto & [interval, expected] : std::vector<std::pair<CyclicInterval, std::string>>{
           {CyclicInterval::FAST, "fast"}, {CyclicInterval::NORMAL, "normal"}, {CyclicInterval::SLOW, "slow"}}) {
    info.interval = interval;
    auto j = CyclicSubscriptionHandlers::subscription_to_json(info, "/events");
    EXPECT_EQ(j["interval"], expected);
  }
}

// @verifies REQ_INTEROP_089
TEST(CyclicSubscriptionJsonTest, ServerLevelUpdateResource) {
  CyclicSubscriptionInfo info;
  info.id = "sub_updates_001";
  info.entity_id = "temp_sensor";
  info.entity_type = "apps";
  info.resource_uri = "/api/v1/updates/ADAS-v2/status";
  info.collection = "updates";
  info.resource_path = "ADAS-v2";
  info.protocol = "sse";
  info.interval = CyclicInterval::SLOW;

  std::string event_source = "/api/v1/apps/temp_sensor/cyclic-subscriptions/sub_updates_001/events";
  auto j = CyclicSubscriptionHandlers::subscription_to_json(info, event_source);

  EXPECT_EQ(j["id"], "sub_updates_001");
  EXPECT_EQ(j["observed_resource"], "/api/v1/updates/ADAS-v2/status");
  EXPECT_EQ(j["event_source"], event_source);
  EXPECT_EQ(j["protocol"], "sse");
  EXPECT_EQ(j["interval"], "slow");
}

// --- Error response format (via HandlerContext static helpers) ---

TEST(CyclicSubscriptionErrorTest, InvalidParameterErrorFormat) {
  httplib::Response res;
  HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid interval",
                             {{"parameter", "interval"}, {"value", "turbo"}});
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "invalid-parameter");
  EXPECT_EQ(body["message"], "Invalid interval");
  EXPECT_EQ(body["parameters"]["parameter"], "interval");
  EXPECT_EQ(res.status, 400);
}

TEST(CyclicSubscriptionErrorTest, ResourceNotFoundErrorFormat) {
  httplib::Response res;
  HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                             {{"subscription_id", "sub_999"}});
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "resource-not-found");
  EXPECT_EQ(res.status, 404);
}
