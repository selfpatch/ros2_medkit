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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_topic_beacon/topic_beacon_plugin.hpp"

using ros2_medkit_beacon::BeaconHint;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::GatewayPlugin;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_gateway::IntrospectionProvider;
using ros2_medkit_gateway::PLUGIN_API_VERSION;
using ros2_medkit_gateway::PluginContext;
using ros2_medkit_gateway::PluginEntityInfo;
using ros2_medkit_gateway::SovdEntityType;

// Extern "C" plugin exports (defined in topic_beacon_plugin.cpp, linked into test binary)
extern "C" int plugin_api_version();
extern "C" GatewayPlugin * create_plugin();
extern "C" IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin);

// Stubs for PluginContext static methods (normally provided by gateway_lib).
// The test binary doesn't link gateway_lib, so we provide minimal implementations.
namespace ros2_medkit_gateway {
void PluginContext::send_json(httplib::Response & res, const nlohmann::json & data) {
  res.set_content(data.dump(), "application/json");
}
void PluginContext::send_error(httplib::Response & res, int status, const std::string & /*error_code*/,
                               const std::string & message, const nlohmann::json & /*parameters*/) {
  res.status = status;
  nlohmann::json err = {{"error", message}};
  res.set_content(err.dump(), "application/json");
}
}  // namespace ros2_medkit_gateway

// Minimal mock PluginContext for unit testing.
// We only need node() and register_capability() to work.
class MockPluginContext : public PluginContext {
 public:
  explicit MockPluginContext(rclcpp::Node * node) : node_(node) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & /*id*/) const override {
    return std::nullopt;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*component_id*/) const override {
    return {};
  }

  nlohmann::json list_entity_faults(const std::string & /*entity_id*/) const override {
    return nlohmann::json::array();
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const httplib::Request & /*req*/,
                                                            httplib::Response & /*res*/,
                                                            const std::string & /*entity_id*/) const override {
    return std::nullopt;
  }

  void register_capability(SovdEntityType type, const std::string & capability_name) override {
    registered_capabilities_.push_back({type, capability_name});
  }

  void register_entity_capability(const std::string & /*entity_id*/, const std::string & /*capability_name*/) override {
  }

  std::vector<std::string> get_type_capabilities(SovdEntityType /*entity_type*/) const override {
    return {};
  }

  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  ros2_medkit_gateway::ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ros2_medkit_gateway::ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
  void set_trigger_store(std::unique_ptr<ros2_medkit_gateway::TriggerStore>) override {
  }
  void register_trigger_transport(std::shared_ptr<ros2_medkit_gateway::TriggerTransportProvider>) override {
  }

  ros2_medkit_gateway::LockAccessResult check_lock(const std::string &, const std::string &,
                                                   const std::string &) const override {
    return ros2_medkit_gateway::LockAccessResult{true, "", "", ""};
  }
  tl::expected<ros2_medkit_gateway::LockInfo, ros2_medkit_gateway::LockError>
  acquire_lock(const std::string &, const std::string &, const std::vector<std::string> &, int) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{"lock-disabled", "Not available", 503, std::nullopt});
  }
  tl::expected<void, ros2_medkit_gateway::LockError> release_lock(const std::string &, const std::string &) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{"lock-disabled", "Not available", 503, std::nullopt});
  }

  struct CapabilityRegistration {
    SovdEntityType type;
    std::string name;
  };
  std::vector<CapabilityRegistration> registered_capabilities_;

 private:
  rclcpp::Node * node_;
};

class TopicBeaconPluginTest : public ::testing::Test {
 public:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_topic_beacon_plugin_node");
    mock_ctx_ = std::make_unique<MockPluginContext>(node_.get());

    // Create plugin directly
    plugin_ = std::make_unique<TopicBeaconPlugin>();

    nlohmann::json config;
    config["topic"] = "/test_beacon_topic";
    config["beacon_ttl_sec"] = 10.0;
    config["beacon_expiry_sec"] = 300.0;
    config["max_messages_per_second"] = 1000.0;
    plugin_->configure(config);
    plugin_->set_context(*mock_ctx_);

    // Create publisher for sending test messages
    publisher_ = node_->create_publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>("/test_beacon_topic",
                                                                                     rclcpp::QoS(100).reliable());
  }

  void TearDown() override {
    plugin_->shutdown();
    publisher_.reset();
    plugin_.reset();
    mock_ctx_.reset();
    node_.reset();
  }

  /// Spin the node for the given duration to process callbacks
  void spin_for(std::chrono::milliseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  ros2_medkit_msgs::msg::MedkitDiscoveryHint make_hint(const std::string & entity_id) {
    ros2_medkit_msgs::msg::MedkitDiscoveryHint msg;
    msg.entity_id = entity_id;
    msg.stable_id = "stable-" + entity_id;
    msg.display_name = "Display " + entity_id;
    msg.transport_type = "nitros_zero_copy";
    msg.process_id = 1234;
    msg.process_name = "test_process";
    msg.hostname = "test-host";
    return msg;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<MockPluginContext> mock_ctx_;
  std::unique_ptr<TopicBeaconPlugin> plugin_;
  rclcpp::Publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>::SharedPtr publisher_;
};

// @verifies REQ_DISCO_BEACON_01
TEST_F(TopicBeaconPluginTest, MessageCallbackUpdatesStore) {
  auto msg = make_hint("engine_temp_sensor");
  publisher_->publish(msg);

  // Spin to receive the message
  spin_for(std::chrono::milliseconds(500));

  EXPECT_GT(plugin_->store().size(), 0u);

  auto stored = plugin_->store().get("engine_temp_sensor");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->hint.entity_id, "engine_temp_sensor");
  EXPECT_EQ(stored->hint.stable_id, "stable-engine_temp_sensor");
  EXPECT_EQ(stored->hint.transport_type, "nitros_zero_copy");
  EXPECT_EQ(stored->hint.process_id, 1234u);
}

// @verifies REQ_DISCO_BEACON_01
TEST_F(TopicBeaconPluginTest, IntrospectReturnsCorrectResult) {
  // Populate store directly
  BeaconHint hint;
  hint.entity_id = "my_app";
  hint.transport_type = "shared_memory";
  hint.received_at = std::chrono::steady_clock::now();
  plugin_->store().update(hint);

  // Build input with matching app
  IntrospectionInput input;
  App app;
  app.id = "my_app";
  app.name = "My App";
  input.apps.push_back(app);

  auto result = plugin_->introspect(input);
  EXPECT_FALSE(result.metadata.empty());
  EXPECT_GT(result.metadata.count("my_app"), 0u);
  EXPECT_TRUE(result.metadata["my_app"].contains("x-medkit-beacon-status"));
}

// @verifies REQ_DISCO_BEACON_05
TEST_F(TopicBeaconPluginTest, InvalidMessageRejectedByValidator) {
  // Publish message with empty entity_id - should be rejected by validator
  ros2_medkit_msgs::msg::MedkitDiscoveryHint msg;
  msg.entity_id = "";  // Invalid: empty entity_id
  msg.transport_type = "nitros";
  publisher_->publish(msg);

  spin_for(std::chrono::milliseconds(500));

  EXPECT_EQ(plugin_->store().size(), 0u);
}

TEST_F(TopicBeaconPluginTest, ShutdownDestroysSubscription) {
  ASSERT_NE(plugin_->subscription(), nullptr);
  plugin_->shutdown();
  EXPECT_EQ(plugin_->subscription(), nullptr);
}

// @verifies REQ_DISCO_BEACON_01
TEST_F(TopicBeaconPluginTest, MetadataFromMessagePreserved) {
  auto msg = make_hint("sensor_node");
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "gpu_model";
  kv.value = "RTX 4090";
  msg.metadata.push_back(kv);
  publisher_->publish(msg);

  spin_for(std::chrono::milliseconds(500));

  auto stored = plugin_->store().get("sensor_node");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->hint.metadata.at("gpu_model"), "RTX 4090");
}

// @verifies REQ_DISCO_BEACON_04
TEST_F(TopicBeaconPluginTest, PluginNameAndCapabilities) {
  EXPECT_EQ(plugin_->name(), "topic_beacon");

  // Verify capabilities were registered
  ASSERT_EQ(mock_ctx_->registered_capabilities_.size(), 2u);
  EXPECT_EQ(mock_ctx_->registered_capabilities_[0].type, SovdEntityType::APP);
  EXPECT_EQ(mock_ctx_->registered_capabilities_[0].name, "x-medkit-topic-beacon");
  EXPECT_EQ(mock_ctx_->registered_capabilities_[1].type, SovdEntityType::COMPONENT);
  EXPECT_EQ(mock_ctx_->registered_capabilities_[1].name, "x-medkit-topic-beacon");
}

TEST_F(TopicBeaconPluginTest, PluginExportsValid) {
  // Verify extern "C" exports
  EXPECT_EQ(plugin_api_version(), PLUGIN_API_VERSION);

  auto * raw_plugin = create_plugin();
  ASSERT_NE(raw_plugin, nullptr);
  EXPECT_EQ(raw_plugin->name(), "topic_beacon");

  auto * provider = get_introspection_provider(raw_plugin);
  ASSERT_NE(provider, nullptr);

  delete raw_plugin;
}

// --- TokenBucket unit tests ---

TEST(TokenBucketTest, FreshBucketAllowsConsume) {
  TokenBucket bucket(10.0);
  EXPECT_TRUE(bucket.try_consume());
}

TEST(TokenBucketTest, BucketExhaustion) {
  TokenBucket bucket(5.0);
  int consumed = 0;
  for (int i = 0; i < 100; ++i) {
    if (bucket.try_consume()) {
      ++consumed;
    }
  }
  EXPECT_EQ(consumed, 5);  // Initial tokens = max_per_second = 5
}

TEST(TokenBucketTest, DefaultConstructor) {
  TokenBucket bucket;
  EXPECT_TRUE(bucket.try_consume());  // default 100 tokens
}

// --- Rate limiting rejection test ---

// @verifies REQ_DISCO_BEACON_05
TEST_F(TopicBeaconPluginTest, RateLimitingRejectsExcessMessages) {
  // Create a separate plugin with very low rate limit
  auto rate_limited_plugin = std::make_unique<TopicBeaconPlugin>();

  nlohmann::json config;
  config["topic"] = "/test_rate_limit_topic";
  config["beacon_ttl_sec"] = 10.0;
  config["beacon_expiry_sec"] = 300.0;
  config["max_messages_per_second"] = 2.0;  // Only 2 tokens initially
  rate_limited_plugin->configure(config);
  rate_limited_plugin->set_context(*mock_ctx_);

  auto rate_pub = node_->create_publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>("/test_rate_limit_topic",
                                                                                      rclcpp::QoS(100).reliable());

  // Publish 20 messages with unique entity_ids
  const int num_messages = 20;
  for (int i = 0; i < num_messages; ++i) {
    auto msg = make_hint("rate_test_entity_" + std::to_string(i));
    rate_pub->publish(msg);
  }

  // Spin to process all queued messages
  spin_for(std::chrono::milliseconds(500));

  // Rate limiter should have rejected most messages.
  // With rate=2.0, the bucket starts with 2 tokens. Some refill may happen
  // during processing, but far fewer than 20 messages should get through.
  auto accepted = rate_limited_plugin->store().size();
  EXPECT_GT(accepted, 0u) << "At least some messages should be accepted";
  EXPECT_LT(accepted, static_cast<size_t>(num_messages)) << "Rate limiter should reject excess messages";

  rate_limited_plugin->shutdown();
}
