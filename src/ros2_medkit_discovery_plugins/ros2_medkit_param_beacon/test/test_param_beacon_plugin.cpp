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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_param_beacon/param_beacon_plugin.hpp"

using ros2_medkit_beacon::BeaconHint;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::GatewayPlugin;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_gateway::IntrospectionProvider;
using ros2_medkit_gateway::PLUGIN_API_VERSION;
using ros2_medkit_gateway::PluginContext;
using ros2_medkit_gateway::PluginEntityInfo;
using ros2_medkit_gateway::SovdEntityType;
using ros2_medkit_param_beacon::ParameterClientInterface;
using ::testing::_;
using ::testing::Return;

// Extern "C" plugin exports
extern "C" int plugin_api_version();
extern "C" GatewayPlugin * create_plugin();
extern "C" IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin);

// Stubs for PluginContext static methods
namespace ros2_medkit_gateway {
void PluginContext::send_json(httplib::Response & res, const nlohmann::json & data) {
  res.set_content(data.dump(), "application/json");
}
void PluginContext::send_error(httplib::Response & res, int status, const std::string &, const std::string & message,
                               const nlohmann::json &) {
  res.status = status;
  nlohmann::json err = {{"error", message}};
  res.set_content(err.dump(), "application/json");
}
}  // namespace ros2_medkit_gateway

// --- Mocks ---

class MockParameterClient : public ParameterClientInterface {
 public:
  MOCK_METHOD(bool, wait_for_service, (std::chrono::duration<double> timeout), (override));
  MOCK_METHOD(rcl_interfaces::msg::ListParametersResult, list_parameters,
              (const std::vector<std::string> & prefixes, uint64_t depth), (override));
  MOCK_METHOD(std::vector<rclcpp::Parameter>, get_parameters, (const std::vector<std::string> & names), (override));
};

class MockPluginContext : public PluginContext {
 public:
  explicit MockPluginContext(rclcpp::Node * node) : node_(node) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }
  std::optional<PluginEntityInfo> get_entity(const std::string &) const override {
    return std::nullopt;
  }
  std::vector<PluginEntityInfo> get_child_apps(const std::string &) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string &) const override {
    return nlohmann::json::array();
  }
  std::optional<PluginEntityInfo> validate_entity_for_route(const httplib::Request &, httplib::Response &,
                                                            const std::string &) const override {
    return std::nullopt;
  }
  void register_capability(SovdEntityType type, const std::string & name) override {
    registered_capabilities_.push_back({type, name});
  }
  void register_entity_capability(const std::string &, const std::string &) override {
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string &) const override {
    return {};
  }

  struct CapReg {
    SovdEntityType type;
    std::string name;
  };
  std::vector<CapReg> registered_capabilities_;

 private:
  rclcpp::Node * node_;
};

// --- Test Fixture ---

class ParamBeaconPluginTest : public ::testing::Test {
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
    node_ = std::make_shared<rclcpp::Node>("test_param_beacon_node");
    mock_ctx_ = std::make_unique<MockPluginContext>(node_.get());
    mock_client_ = std::make_shared<MockParameterClient>();
  }

  void TearDown() override {
    if (plugin_) {
      plugin_->shutdown();
      plugin_.reset();
    }
    mock_ctx_.reset();
    node_.reset();
  }

  /// Create plugin with mock client factory and fast polling for tests.
  /// Named setup_plugin to avoid collision with extern "C" create_plugin().
  void setup_plugin(double poll_interval = 0.1, double poll_budget = 5.0) {
    auto mock = mock_client_;
    plugin_ = std::make_unique<ParameterBeaconPlugin>([mock](const std::string &) {
      return mock;
    });

    nlohmann::json config;
    config["poll_interval_sec"] = poll_interval;
    config["poll_budget_sec"] = poll_budget;
    config["param_timeout_sec"] = 1.0;
    config["beacon_ttl_sec"] = 10.0;
    config["beacon_expiry_sec"] = 300.0;
    plugin_->configure(config);
    plugin_->set_context(*mock_ctx_);
  }

  /// Helper: create standard list_parameters result with beacon parameter names
  rcl_interfaces::msg::ListParametersResult make_list_result(const std::vector<std::string> & names) {
    rcl_interfaces::msg::ListParametersResult result;
    result.names = names;
    return result;
  }

  /// Helper: create standard parameter set for a beacon hint
  std::vector<rclcpp::Parameter> make_beacon_params(const std::string & entity_id,
                                                    const std::string & transport = "shared_memory") {
    return {
        rclcpp::Parameter("ros2_medkit.discovery.entity_id", entity_id),
        rclcpp::Parameter("ros2_medkit.discovery.transport_type", transport),
        rclcpp::Parameter("ros2_medkit.discovery.process_id", 1234),
        rclcpp::Parameter("ros2_medkit.discovery.hostname", "test-host"),
    };
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<MockPluginContext> mock_ctx_;
  std::shared_ptr<MockParameterClient> mock_client_;
  std::unique_ptr<ParameterBeaconPlugin> plugin_;
};

// --- Tests ---

TEST_F(ParamBeaconPluginTest, PluginNameAndExports) {
  setup_plugin();
  EXPECT_EQ(plugin_->name(), "parameter_beacon");
  EXPECT_EQ(plugin_api_version(), PLUGIN_API_VERSION);

  auto * raw = create_plugin();
  ASSERT_NE(raw, nullptr);
  EXPECT_EQ(raw->name(), "parameter_beacon");
  auto * provider = get_introspection_provider(raw);
  ASSERT_NE(provider, nullptr);
  delete raw;
}

// @verifies REQ_DISCO_BEACON_04
TEST_F(ParamBeaconPluginTest, CapabilitiesRegistered) {
  setup_plugin();
  ASSERT_EQ(mock_ctx_->registered_capabilities_.size(), 2u);
  EXPECT_EQ(mock_ctx_->registered_capabilities_[0].name, "x-medkit-param-beacon");
  EXPECT_EQ(mock_ctx_->registered_capabilities_[1].name, "x-medkit-param-beacon");
}

// @verifies REQ_DISCO_BEACON_02
TEST_F(ParamBeaconPluginTest, PollsNodeAndStoresHint) {
  // Setup mock client expectations
  EXPECT_CALL(*mock_client_, wait_for_service(_)).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_client_, list_parameters(_, _))
      .WillRepeatedly(Return(make_list_result({
          "ros2_medkit.discovery.entity_id",
          "ros2_medkit.discovery.transport_type",
          "ros2_medkit.discovery.process_id",
          "ros2_medkit.discovery.hostname",
      })));
  EXPECT_CALL(*mock_client_, get_parameters(_)).WillRepeatedly(Return(make_beacon_params("my_sensor")));

  setup_plugin(0.05);  // Fast poll for test

  // Provide a node to poll via introspect
  IntrospectionInput input;
  App app;
  app.id = "my_sensor";
  app.name = "My Sensor";
  app.is_online = true;
  app.bound_fqn = "/my_sensor";
  input.apps.push_back(app);
  plugin_->introspect(input);

  // Wait for poll cycle to run
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto stored = plugin_->store().get("my_sensor");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->hint.entity_id, "my_sensor");
  EXPECT_EQ(stored->hint.transport_type, "shared_memory");
  EXPECT_EQ(stored->hint.process_id, 1234u);
  EXPECT_EQ(stored->hint.hostname, "test-host");
}

// @verifies REQ_DISCO_BEACON_02
TEST_F(ParamBeaconPluginTest, SkipsNodeWithoutEntityId) {
  EXPECT_CALL(*mock_client_, wait_for_service(_)).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_client_, list_parameters(_, _))
      .WillRepeatedly(Return(make_list_result({"ros2_medkit.discovery.transport_type"})));
  EXPECT_CALL(*mock_client_, get_parameters(_))
      .WillRepeatedly(Return(std::vector<rclcpp::Parameter>{
          rclcpp::Parameter("ros2_medkit.discovery.transport_type", "dds"),
      }));

  setup_plugin(0.05);

  IntrospectionInput input;
  App app;
  app.id = "no_id_node";
  app.is_online = true;
  app.bound_fqn = "/no_id_node";
  input.apps.push_back(app);
  plugin_->introspect(input);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(plugin_->store().size(), 0u);
}

TEST_F(ParamBeaconPluginTest, SkipsNodeWithEmptyParams) {
  EXPECT_CALL(*mock_client_, wait_for_service(_)).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_client_, list_parameters(_, _)).WillRepeatedly(Return(make_list_result({})));

  setup_plugin(0.05);

  IntrospectionInput input;
  App app;
  app.id = "empty_node";
  app.is_online = true;
  app.bound_fqn = "/empty_node";
  input.apps.push_back(app);
  plugin_->introspect(input);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(plugin_->store().size(), 0u);
}

TEST_F(ParamBeaconPluginTest, BackoffOnTimeout) {
  // First call: service unavailable -> backoff
  EXPECT_CALL(*mock_client_, wait_for_service(_))
      .WillOnce(Return(false))        // 1st cycle: timeout -> skip 1
      .WillOnce(Return(false))        // 3rd cycle (after 1 skip): timeout -> skip 2
      .WillRepeatedly(Return(true));  // Eventually succeeds
  EXPECT_CALL(*mock_client_, list_parameters(_, _))
      .WillRepeatedly(Return(make_list_result({"ros2_medkit.discovery.entity_id"})));
  EXPECT_CALL(*mock_client_, get_parameters(_))
      .WillRepeatedly(Return(std::vector<rclcpp::Parameter>{
          rclcpp::Parameter("ros2_medkit.discovery.entity_id", "backoff_test"),
      }));

  setup_plugin(0.05);

  IntrospectionInput input;
  App app;
  app.id = "backoff_test";
  app.is_online = true;
  app.bound_fqn = "/backoff_node";
  input.apps.push_back(app);
  plugin_->introspect(input);

  // Wait long enough for multiple poll cycles
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Eventually the hint should appear after backoff recovery
  auto stored = plugin_->store().get("backoff_test");
  ASSERT_TRUE(stored.has_value());
}

// @verifies REQ_DISCO_BEACON_02
TEST_F(ParamBeaconPluginTest, MetadataSubParams) {
  EXPECT_CALL(*mock_client_, wait_for_service(_)).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_client_, list_parameters(_, _))
      .WillRepeatedly(Return(make_list_result({
          "ros2_medkit.discovery.entity_id",
          "ros2_medkit.discovery.metadata.gxf_status",
          "ros2_medkit.discovery.metadata.firmware",
      })));
  EXPECT_CALL(*mock_client_, get_parameters(_))
      .WillRepeatedly(Return(std::vector<rclcpp::Parameter>{
          rclcpp::Parameter("ros2_medkit.discovery.entity_id", "meta_node"),
          rclcpp::Parameter("ros2_medkit.discovery.metadata.gxf_status", "STARTED"),
          rclcpp::Parameter("ros2_medkit.discovery.metadata.firmware", "2.1.3"),
      }));

  setup_plugin(0.05);

  IntrospectionInput input;
  App app;
  app.id = "meta_node";
  app.is_online = true;
  app.bound_fqn = "/meta_node";
  input.apps.push_back(app);
  plugin_->introspect(input);

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto stored = plugin_->store().get("meta_node");
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->hint.metadata.at("gxf_status"), "STARTED");
  EXPECT_EQ(stored->hint.metadata.at("firmware"), "2.1.3");
}

TEST_F(ParamBeaconPluginTest, ConfigValidationAutoFixes) {
  auto mock = mock_client_;
  auto plugin = std::make_unique<ParameterBeaconPlugin>([mock](const std::string &) {
    return mock;
  });

  nlohmann::json config;
  config["poll_interval_sec"] = 10.0;
  config["beacon_ttl_sec"] = 5.0;     // BAD: ttl < poll_interval
  config["beacon_expiry_sec"] = 3.0;  // BAD: expiry < ttl
  plugin->configure(config);

  // If configure auto-fixes, no crash. Store should be usable.
  plugin->set_context(*mock_ctx_);
  EXPECT_EQ(plugin->name(), "parameter_beacon");
  plugin->shutdown();
}

// @verifies REQ_DISCO_BEACON_02
TEST_F(ParamBeaconPluginTest, IntrospectReturnsMetadata) {
  // Populate store directly
  BeaconHint hint;
  hint.entity_id = "direct_app";
  hint.transport_type = "tcp";
  hint.received_at = std::chrono::steady_clock::now();

  setup_plugin(100.0);  // Very slow poll - won't fire during test
  plugin_->store().update(hint);

  IntrospectionInput input;
  App app;
  app.id = "direct_app";
  app.name = "Direct App";
  input.apps.push_back(app);

  auto result = plugin_->introspect(input);
  EXPECT_FALSE(result.metadata.empty());
  EXPECT_GT(result.metadata.count("direct_app"), 0u);
}

TEST_F(ParamBeaconPluginTest, ShutdownJoinsThread) {
  setup_plugin(0.05);

  // Shutdown should join the poll thread without hanging
  plugin_->shutdown();

  // Double shutdown should be safe
  plugin_->shutdown();
}

TEST_F(ParamBeaconPluginTest, ExceptionInPollNodeTriggersBackoff) {
  EXPECT_CALL(*mock_client_, wait_for_service(_)).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_client_, list_parameters(_, _))
      .WillRepeatedly(testing::Throw(std::runtime_error("node disappeared")));

  setup_plugin(0.05);

  IntrospectionInput input;
  App app;
  app.id = "crash_node";
  app.is_online = true;
  app.bound_fqn = "/crash_node";
  input.apps.push_back(app);
  plugin_->introspect(input);

  // Should not crash
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(plugin_->store().size(), 0u);
}
