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

#include <string>

#include "ros2_medkit_gateway/http/handlers/sse_transport_provider.hpp"
#include "ros2_medkit_gateway/subscription_manager.hpp"

using namespace ros2_medkit_gateway;

class SseTransportProviderTest : public ::testing::Test {
 protected:
  SubscriptionManager mgr_{10};
  std::shared_ptr<SSEClientTracker> tracker_ = std::make_shared<SSEClientTracker>(5);
  SseTransportProvider provider_{mgr_, tracker_};
};

// @verifies REQ_INTEROP_090
TEST_F(SseTransportProviderTest, ProtocolReturnsSse) {
  EXPECT_EQ(provider_.protocol(), "sse");
}

// @verifies REQ_INTEROP_090
TEST_F(SseTransportProviderTest, StartReturnsEventsUrl) {
  CyclicSubscriptionInfo info;
  info.id = "sub_001";
  info.entity_type = "apps";
  info.entity_id = "node1";

  ResourceSamplerFn sampler = [](const std::string &, const std::string &) {
    return nlohmann::json{{"value", 42}};
  };

  auto result = provider_.start(info, sampler, nullptr);
  ASSERT_TRUE(result.has_value());
  EXPECT_NE(result->find("/apps/node1/cyclic-subscriptions/sub_001/events"), std::string::npos);
}

// @verifies REQ_INTEROP_090
TEST_F(SseTransportProviderTest, StopRemovesStream) {
  CyclicSubscriptionInfo info;
  info.id = "sub_002";
  info.entity_type = "components";
  info.entity_id = "ecu1";

  ResourceSamplerFn sampler = [](const std::string &, const std::string &) {
    return nlohmann::json{};
  };

  auto result = provider_.start(info, sampler, nullptr);
  ASSERT_TRUE(result.has_value());

  // stop should not throw even if subscription doesn't exist in mgr
  provider_.stop("sub_002");

  // Calling stop again on already-removed stream should be a no-op
  provider_.stop("sub_002");
}

TEST_F(SseTransportProviderTest, NotifyUpdateIsNoOp) {
  // SSE transport re-reads each iteration, so notify_update is a no-op
  // Just verify it doesn't crash
  provider_.notify_update("nonexistent");
}
