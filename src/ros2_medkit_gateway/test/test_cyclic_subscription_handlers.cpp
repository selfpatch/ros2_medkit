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

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/cyclic_subscription_handlers.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::handlers;
using json = nlohmann::json;

// --- subscription_to_json ---

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

  for (auto [interval, expected] : std::vector<std::pair<CyclicInterval, std::string>>{
           {CyclicInterval::FAST, "fast"}, {CyclicInterval::NORMAL, "normal"}, {CyclicInterval::SLOW, "slow"}}) {
    info.interval = interval;
    auto j = CyclicSubscriptionHandlers::subscription_to_json(info, "/events");
    EXPECT_EQ(j["interval"], expected);
  }
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
