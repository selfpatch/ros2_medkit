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

#include <stdexcept>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/subscription_transport.hpp"

using namespace ros2_medkit_gateway;

/// Mock transport for testing
class MockTransport : public SubscriptionTransportProvider {
 public:
  explicit MockTransport(std::string proto) : proto_(std::move(proto)) {
  }

  std::string protocol() const override {
    return proto_;
  }

  tl::expected<std::string, std::string> start(const CyclicSubscriptionInfo & /*info*/, ResourceSamplerFn /*sampler*/,
                                               GatewayNode * /*node*/) override {
    started_ids_.push_back("started");
    return "mock://event-source";
  }

  void notify_update(const std::string & /*sub_id*/) override {
  }

  void stop(const std::string & sub_id) override {
    stopped_ids_.push_back(sub_id);
  }

  std::vector<std::string> started_ids_;
  std::vector<std::string> stopped_ids_;

 private:
  std::string proto_;
};

// @verifies REQ_INTEROP_089
TEST(TransportRegistryTest, RegisterAndLookup) {
  TransportRegistry registry;
  auto mock = std::make_unique<MockTransport>("mqtt");
  auto * raw = mock.get();
  registry.register_transport(std::move(mock));

  EXPECT_TRUE(registry.has_transport("mqtt"));
  EXPECT_EQ(registry.get_transport("mqtt"), raw);
}

TEST(TransportRegistryTest, HasTransportReturnsFalseForUnregistered) {
  TransportRegistry registry;
  EXPECT_FALSE(registry.has_transport("nonexistent"));
  EXPECT_EQ(registry.get_transport("nonexistent"), nullptr);
}

TEST(TransportRegistryTest, RejectDuplicateRegistration) {
  TransportRegistry registry;
  registry.register_transport(std::make_unique<MockTransport>("mqtt"));
  EXPECT_THROW(registry.register_transport(std::make_unique<MockTransport>("mqtt")), std::runtime_error);
}

TEST(TransportRegistryTest, ShutdownAllCallsStopForActiveSubscriptions) {
  TransportRegistry registry;
  auto mock = std::make_unique<MockTransport>("sse");
  auto * raw = mock.get();
  registry.register_transport(std::move(mock));

  // Create some subscriptions and wire on_removed to transport
  SubscriptionManager mgr(10);
  mgr.set_on_removed([raw](const CyclicSubscriptionInfo & info) {
    raw->stop(info.id);
  });

  auto r1 = mgr.create("e1", "apps", "/r1", "data", "/topic", "sse", CyclicInterval::FAST, 300);
  auto r2 = mgr.create("e2", "apps", "/r2", "faults", "", "sse", CyclicInterval::NORMAL, 300);
  ASSERT_TRUE(r1.has_value());
  ASSERT_TRUE(r2.has_value());

  registry.shutdown_all(mgr);

  // stop() should have been called for both subscriptions
  EXPECT_EQ(raw->stopped_ids_.size(), 2u);
}
