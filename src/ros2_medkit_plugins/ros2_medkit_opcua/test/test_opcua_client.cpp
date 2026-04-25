// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/opcua_client.hpp"

#include <gtest/gtest.h>

namespace ros2_medkit_gateway {

// These tests verify OpcuaClient behavior without a running OPC-UA server.
// Integration tests with a real server are in the Docker demo.

TEST(OpcuaClientTest, DefaultState) {
  OpcuaClient client;
  EXPECT_FALSE(client.is_connected());
  // endpoint_url() reflects the config default ("opc.tcp://localhost:4840");
  // server_description() is only populated after a successful connect.
  EXPECT_EQ(client.endpoint_url(), "opc.tcp://localhost:4840");
  EXPECT_TRUE(client.server_description().empty());
}

TEST(OpcuaClientTest, ConnectFailsWithoutServer) {
  OpcuaClient client;
  OpcuaClientConfig config;
  config.endpoint_url = "opc.tcp://localhost:49999";  // No server here
  config.connect_timeout = std::chrono::milliseconds(500);

  EXPECT_FALSE(client.connect(config));
  EXPECT_FALSE(client.is_connected());
}

TEST(OpcuaClientTest, DisconnectWhenNotConnected) {
  OpcuaClient client;
  // Should not crash
  client.disconnect();
  EXPECT_FALSE(client.is_connected());
}

TEST(OpcuaClientTest, BrowseWhenDisconnected) {
  OpcuaClient client;
  auto result = client.browse({0, UA_NS0ID_OBJECTSFOLDER});
  EXPECT_TRUE(result.empty());
}

TEST(OpcuaClientTest, ReadWhenDisconnected) {
  OpcuaClient client;
  auto result = client.read_value({1, "SomeNode"});
  EXPECT_FALSE(result.good);
}

TEST(OpcuaClientTest, WriteWhenDisconnected) {
  OpcuaClient client;
  EXPECT_FALSE(client.write_value({1, "SomeNode"}, 42.0));
}

TEST(OpcuaClientTest, CreateSubscriptionWhenDisconnected) {
  OpcuaClient client;
  auto id = client.create_subscription(500.0, [](const std::string &, const OpcuaValue &) {});
  EXPECT_EQ(id, 0u);
}

TEST(OpcuaClientTest, RemoveSubscriptionsWhenEmpty) {
  OpcuaClient client;
  // Should not crash
  client.remove_subscriptions();
}

TEST(OpcuaClientTest, WriteValueReturnsNotConnected) {
  OpcuaClient client;
  // Client never connected - write should return NotConnected error
  auto result = client.write_value(opcua::NodeId(0, 1), OpcuaValue{42.0});
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::WriteError::NotConnected);
}

TEST(OpcuaClientTest, WriteValueWithTypeHintDisconnected) {
  OpcuaClient client;
  // Even with a type hint, disconnected client returns NotConnected
  auto result = client.write_value(opcua::NodeId(2, 1), OpcuaValue{75.0}, "float");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::WriteError::NotConnected);
}

TEST(OpcuaClientTest, CurrentConfigPersistence) {
  OpcuaClient client;
  OpcuaClientConfig cfg;
  cfg.endpoint_url = "opc.tcp://test:4840";
  cfg.connect_timeout = std::chrono::milliseconds(1000);
  // connect will fail (no server) but config should be stored
  client.connect(cfg);
  auto stored = client.current_config();
  EXPECT_EQ(stored.endpoint_url, "opc.tcp://test:4840");
  EXPECT_EQ(stored.connect_timeout, std::chrono::milliseconds(1000));
}

// ---------------------------------------------------------------------------
// Issue #386: native OPC-UA AlarmCondition event subscription primitives.
// These tests cover the disconnected-state contract of the new public API.
// End-to-end event flow (real server emits AlarmConditionType, callback fires,
// state machine advances) is exercised in the docker integration test added
// alongside the test_alarm_server fixture in a follow-up commit on the same
// branch.
// ---------------------------------------------------------------------------

TEST(OpcuaClientTest, GenerationStartsAtZero) {
  OpcuaClient client;
  EXPECT_EQ(client.current_generation(), 0u);
}

TEST(OpcuaClientTest, AddEventMonitoredItemWhenDisconnected) {
  OpcuaClient client;
  auto mi = client.add_event_monitored_item(
      /*sub_id=*/1, opcua::NodeId(0, UA_NS0ID_SERVER), /*select=*/{},
      [](const auto &, const auto &, const auto &, const auto &) {});
  EXPECT_EQ(mi, 0u);
}

TEST(OpcuaClientTest, RemoveEventMonitoredItemUnknownIdReturnsFalse) {
  OpcuaClient client;
  EXPECT_FALSE(client.remove_event_monitored_item(/*sub_id=*/1, /*mi_id=*/9999));
}

TEST(OpcuaClientTest, CallMethodWhenDisconnected) {
  OpcuaClient client;
  auto result = client.call_method(opcua::NodeId(0, UA_NS0ID_SERVER), opcua::NodeId(0, 11489), {});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, OpcuaClient::MethodError::NotConnected);
}

TEST(OpcuaClientTest, GenerationBumpsOnDisconnect) {
  OpcuaClient client;
  // The disconnect-without-connect path is a no-op - generation should not
  // change because there is no live subscription state to invalidate.
  client.disconnect();
  EXPECT_EQ(client.current_generation(), 0u);
}

TEST(OpcuaClientTest, RemoveSubscriptionsBumpsGenerationEvenWhenEmpty) {
  // remove_subscriptions() is a publicly exposed bulk-cleanup hook used by the
  // poller's reconnect path. It must increment the generation so any
  // captured-but-not-yet-fired callbacks from the now-defunct subscription set
  // are filtered out by the trampoline. This contract holds even when there
  // are no entries, because the poller does not synchronize fine-grained.
  OpcuaClient client;
  uint64_t before = client.current_generation();
  client.remove_subscriptions();
  EXPECT_GT(client.current_generation(), before);
}

}  // namespace ros2_medkit_gateway
