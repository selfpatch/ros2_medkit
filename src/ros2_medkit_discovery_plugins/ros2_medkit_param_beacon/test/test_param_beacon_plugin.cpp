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
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_param_beacon/param_beacon_plugin.hpp"

using ros2_medkit_beacon::BeaconHint;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::GatewayPlugin;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_gateway::IntrospectionProvider;
using ros2_medkit_gateway::PLUGIN_API_VERSION;
using ros2_medkit_gateway::PluginContext;
using ros2_medkit_gateway::PluginEntityInfo;
using ros2_medkit_gateway::PluginRequest;
using ros2_medkit_gateway::PluginResponse;
using ros2_medkit_gateway::RosPluginContext;
using ros2_medkit_gateway::SovdEntityType;
using ros2_medkit_param_beacon::ParameterClientInterface;
using ros2_medkit_param_beacon::RealParameterClient;
using ::testing::_;
using ::testing::Return;

// Extern "C" plugin exports
extern "C" int plugin_api_version();
extern "C" GatewayPlugin * create_plugin();
extern "C" IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin);

// Stubs for PluginRequest/PluginResponse (implemented in gateway_core, not linked into tests)
namespace ros2_medkit_gateway {
PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t /*index*/) const {
  return {};
}
std::string PluginRequest::header(const std::string & /*name*/) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
}
PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json & /*data*/) {
}
void PluginResponse::send_error(int /*status*/, const std::string & /*error_code*/, const std::string & /*message*/,
                                const nlohmann::json & /*parameters*/) {
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

class MockPluginContext : public RosPluginContext {
 public:
  explicit MockPluginContext(rclcpp::Node * node) : node_(node) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }
  std::optional<PluginEntityInfo> get_entity(const std::string & /*entity_id*/) const override {
    return std::nullopt;
  }
  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*component_id*/) const override {
    return {};
  }
  nlohmann::json list_entity_faults(const std::string & /*entity_id*/) const override {
    return nlohmann::json::array();
  }
  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & /*req*/, PluginResponse & /*res*/,
                                                            const std::string & /*entity_id*/) const override {
    return std::nullopt;
  }
  void register_capability(SovdEntityType type, const std::string & name) override {
    registered_capabilities_.push_back({type, name});
  }
  void register_entity_capability(const std::string & /*entity_id*/, const std::string & /*name*/) override {
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType /*type*/) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  ros2_medkit_gateway::LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                                                   const std::string & /*collection*/) const override {
    return ros2_medkit_gateway::LockAccessResult{true, "", "", ""};
  }
  tl::expected<ros2_medkit_gateway::LockInfo, ros2_medkit_gateway::LockError>
  acquire_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
               const std::vector<std::string> & /*scopes*/, int /*expiration_seconds*/) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{"lock-disabled", "Not available", 503, std::nullopt});
  }
  tl::expected<void, ros2_medkit_gateway::LockError> release_lock(const std::string & /*entity_id*/,
                                                                  const std::string & /*client_id*/) override {
    return tl::make_unexpected(ros2_medkit_gateway::LockError{"lock-disabled", "Not available", 503, std::nullopt});
  }
  ros2_medkit_gateway::ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ros2_medkit_gateway::ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
  struct CapReg {
    SovdEntityType type;
    std::string name;
  };
  std::vector<CapReg> registered_capabilities_;

 private:
  rclcpp::Node * node_;
};

/// How a ParameterServiceStub answers get requests.
enum class GetAnswer { kNone, kValue, kNoValues };

/// A node with parameter services that answers only as told, on its own executor. Counts list and get requests.
class ParameterServiceStub {
 public:
  ParameterServiceStub(const std::string & name, bool answer_list, GetAnswer answer_get = GetAnswer::kNone) {
    node_ = std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().start_parameter_services(false));
    using rcl_interfaces::srv::GetParameters;
    using rcl_interfaces::srv::ListParameters;
    list_ = node_->create_service<ListParameters>(
        "~/list_parameters", [this, answer_list](const std::shared_ptr<rmw_request_id_t> & header,
                                                 const std::shared_ptr<ListParameters::Request> &) {
          ++list_requests_;
          if (answer_list) {
            ListParameters::Response response;
            response.result.names = {"ros2_medkit.discovery.entity_id"};
            list_->send_response(*header, response);
          }
        });
    get_ = node_->create_service<GetParameters>(
        "~/get_parameters", [this, name, answer_get](const std::shared_ptr<rmw_request_id_t> & header,
                                                     const std::shared_ptr<GetParameters::Request> &) {
          ++get_requests_;
          if (answer_get != GetAnswer::kNone) {
            GetParameters::Response response;
            if (answer_get == GetAnswer::kValue) {
              response.values.push_back(rclcpp::ParameterValue(name).to_value_msg());
            }
            get_->send_response(*header, response);
          }
        });
    types_ = silent<rcl_interfaces::srv::GetParameterTypes>("~/get_parameter_types");
    set_ = silent<rcl_interfaces::srv::SetParameters>("~/set_parameters");
    atomically_ = silent<rcl_interfaces::srv::SetParametersAtomically>("~/set_parameters_atomically");
    describe_ = silent<rcl_interfaces::srv::DescribeParameters>("~/describe_parameters");
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {
      executor_.spin();
    });
  }

  ~ParameterServiceStub() {
    executor_.cancel();
    spin_thread_.join();
    executor_.remove_node(node_);
  }

  ParameterServiceStub(const ParameterServiceStub &) = delete;
  ParameterServiceStub & operator=(const ParameterServiceStub &) = delete;
  ParameterServiceStub(ParameterServiceStub &&) = delete;
  ParameterServiceStub & operator=(ParameterServiceStub &&) = delete;

  std::string fqn() const {
    return node_->get_fully_qualified_name();
  }
  int list_requests() const {
    return list_requests_.load();
  }
  int get_requests() const {
    return get_requests_.load();
  }

 private:
  template <typename Service>
  std::shared_ptr<rclcpp::ServiceBase> silent(const std::string & name) {
    return node_->create_service<Service>(
        name, [](const std::shared_ptr<rmw_request_id_t> &, const std::shared_ptr<typename Service::Request> &) {});
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_;
  std::shared_ptr<rclcpp::ServiceBase> types_;
  std::shared_ptr<rclcpp::ServiceBase> set_;
  std::shared_ptr<rclcpp::ServiceBase> atomically_;
  std::shared_ptr<rclcpp::ServiceBase> describe_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::atomic<int> list_requests_{0};
  std::atomic<int> get_requests_{0};
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

TEST_F(ParamBeaconPluginTest, CapabilitiesRegistered) {
  setup_plugin();
  ASSERT_EQ(mock_ctx_->registered_capabilities_.size(), 2u);
  EXPECT_EQ(mock_ctx_->registered_capabilities_[0].name, "x-medkit-param-beacon");
  EXPECT_EQ(mock_ctx_->registered_capabilities_[1].name, "x-medkit-param-beacon");
}

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

  // Poll until backoff recovery completes instead of fixed sleep
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  decltype(plugin_->store().get("backoff_test")) stored;
  while (std::chrono::steady_clock::now() < deadline) {
    stored = plugin_->store().get("backoff_test");
    if (stored.has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(stored.has_value()) << "Backoff recovery did not complete within 5 seconds";
}

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

TEST_F(ParamBeaconPluginTest, PollCycleAfterShutdownIsNoop) {
  setup_plugin();

  // Let initial poll happen
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  plugin_->shutdown();

  // Store should not grow after shutdown (poll_cycle exits early)
  size_t store_size = plugin_->store().size();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  EXPECT_EQ(plugin_->store().size(), store_size);
}

// A get request that gets no answer in time throws, like a list request.
TEST_F(ParamBeaconPluginTest, RealClientThrowsWhenGetParametersGetsNoAnswer) {
  ParameterServiceStub stub("get_never_answers", true);
  RealParameterClient client(node_, stub.fqn(), std::chrono::duration<double>(0.2));
  ASSERT_TRUE(client.wait_for_service(std::chrono::seconds(10)));
  const auto names = client.list_parameters({"ros2_medkit.discovery"}, 0).names;
  ASSERT_EQ(names, (std::vector<std::string>{"ros2_medkit.discovery.entity_id"}));

  const auto started = std::chrono::steady_clock::now();
  EXPECT_THROW(client.get_parameters(names), std::runtime_error);
  EXPECT_GE(std::chrono::steady_clock::now() - started, std::chrono::milliseconds(200));
  EXPECT_EQ(stub.get_requests(), 1);
}

// A request that gets no answer in time leaves nothing pending in its client.
TEST_F(ParamBeaconPluginTest, RealClientLeavesNoRequestPendingAfterTimeouts) {
  ParameterServiceStub list_silent("pending_list_silent", false);
  ParameterServiceStub get_silent("pending_get_silent", true);
  RealParameterClient list_client(node_, list_silent.fqn(), std::chrono::duration<double>(0.005));
  RealParameterClient get_client(node_, get_silent.fqn(), std::chrono::duration<double>(0.005));
  ASSERT_TRUE(list_client.wait_for_service(std::chrono::seconds(10)));
  ASSERT_TRUE(get_client.wait_for_service(std::chrono::seconds(10)));

  constexpr int kCalls = 50;
  int timeouts = 0;
  for (int i = 0; i < kCalls; ++i) {
    try {
      list_client.list_parameters({"ros2_medkit.discovery"}, 0);
    } catch (const std::runtime_error &) {
      ++timeouts;
    }
    try {
      get_client.get_parameters({"ros2_medkit.discovery.entity_id"});
    } catch (const std::runtime_error &) {
      ++timeouts;
    }
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while ((list_silent.list_requests() < kCalls || get_silent.get_requests() < kCalls) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(list_silent.list_requests(), kCalls);
  ASSERT_EQ(get_silent.get_requests(), kCalls);
  EXPECT_EQ(timeouts, 2 * kCalls);
  EXPECT_EQ(list_client.prune_pending_requests(), 0U);
  EXPECT_EQ(get_client.prune_pending_requests(), 0U);
}

// A get answer without values is an answer: no exception, no parameters, nothing pending.
TEST_F(ParamBeaconPluginTest, RealClientReturnsNoParametersForAnAnswerWithoutValues) {
  ParameterServiceStub stub("get_answers_no_values", true, GetAnswer::kNoValues);
  RealParameterClient client(node_, stub.fqn(), std::chrono::duration<double>(10.0));
  ASSERT_TRUE(client.wait_for_service(std::chrono::seconds(10)));

  std::vector<rclcpp::Parameter> parameters{rclcpp::Parameter("placeholder", 1)};
  EXPECT_NO_THROW(parameters = client.get_parameters({"ros2_medkit.discovery.entity_id"}));
  EXPECT_TRUE(parameters.empty());
  EXPECT_EQ(stub.get_requests(), 1);
  EXPECT_EQ(client.prune_pending_requests(), 0U);
}

// A wait with no time left returns within 1 s, also when the service does not exist.
TEST_F(ParamBeaconPluginTest, RealClientWaitWithNoTimeLeftReturns) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(
      {
        // A wait that never returns ends the child with SIGALRM.
        alarm(10);
        RealParameterClient client(node_, "/param_beacon_absent_node", std::chrono::duration<double>(1.0));
        const auto start = std::chrono::steady_clock::now();
        const bool found = client.wait_for_service(std::chrono::duration<double>(0.0));
        const auto waited_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        // libtsan intercepts _exit, so a race reported in the child sets its exit code.
        if (found) {
          _exit(1);
        }
        if (waited_ms >= 1000) {
          std::fprintf(stderr, "the wait took %lld ms\n", static_cast<long long>(waited_ms));
          _exit(2);
        }
        _exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

// In a graph read, a node whose get or list requests time out is backed off; one that answers is
// polled every cycle.
TEST_F(ParamBeaconPluginTest, RuntimeTargetsWhoseParameterRequestsTimeOutAreBackedOff) {
  ParameterServiceStub healthy("beacon_healthy", true, GetAnswer::kValue);
  ParameterServiceStub get_silent("beacon_get_silent", true);
  ParameterServiceStub list_silent("beacon_list_silent", false);

  plugin_ = std::make_unique<ParameterBeaconPlugin>();
  nlohmann::json config;
  config["poll_interval_sec"] = 0.1;
  config["param_timeout_sec"] = 0.1;
  config["poll_budget_sec"] = 5.0;
  plugin_->configure(config);
  plugin_->set_context(*mock_ctx_);

  // Twelve cycles: a node that times out on every poll is asked on cycles 1, 3, 6 and 11.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (healthy.list_requests() < 12 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  plugin_->shutdown();
  const int cycles = healthy.list_requests();
  ASSERT_GE(cycles, 12) << "the plugin never polled the node that answers";
  EXPECT_GE(get_silent.get_requests(), 3);
  EXPECT_LE(get_silent.get_requests(), 5) << "a node whose get requests time out was asked on "
                                          << get_silent.get_requests() << " of " << cycles << " cycles";
  EXPECT_GE(list_silent.list_requests(), 3);
  EXPECT_LE(list_silent.list_requests(), 5) << "a node whose list requests time out was asked on "
                                            << list_silent.list_requests() << " of " << cycles << " cycles";
}

// NaN, -inf and values below the minimum clamp to the minimum, +inf to kMaxSeconds. Polling keeps
// its 0.1 s timeout.
TEST_F(ParamBeaconPluginTest, OutOfRangeDurationsAreClampedAndPollingKeepsItsTimeout) {
  ParameterServiceStub get_silent("beacon_nonfinite_get_silent", true);
  ParameterServiceStub list_silent("beacon_nonfinite_list_silent", false);

  plugin_ = std::make_unique<ParameterBeaconPlugin>();
  nlohmann::json config;
  config["poll_interval_sec"] = std::numeric_limits<double>::quiet_NaN();
  config["param_timeout_sec"] = -1.0;
  config["poll_budget_sec"] = std::numeric_limits<double>::infinity();
  config["beacon_ttl_sec"] = -std::numeric_limits<double>::infinity();
  config["beacon_expiry_sec"] = std::numeric_limits<double>::quiet_NaN();
  plugin_->configure(config);
  plugin_->set_context(*mock_ctx_);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (list_silent.list_requests() < 3 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  plugin_->shutdown();
  EXPECT_GE(list_silent.list_requests(), 3);
  EXPECT_GE(get_silent.get_requests(), 1);
}

// A get answer without values, as rclpy gives for a typed parameter with no value, causes no backoff
// and stores no hint.
TEST_F(ParamBeaconPluginTest, NodeAnsweringWithoutValuesIsPolledEveryCycle) {
  ParameterServiceStub healthy("no_values_healthy", true, GetAnswer::kValue);
  ParameterServiceStub no_values("no_values_answer", true, GetAnswer::kNoValues);

  plugin_ = std::make_unique<ParameterBeaconPlugin>();
  nlohmann::json config;
  config["poll_interval_sec"] = 0.1;
  config["param_timeout_sec"] = 5.0;
  config["poll_budget_sec"] = 20.0;
  plugin_->configure(config);
  plugin_->set_context(*mock_ctx_);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (healthy.list_requests() < 12 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  plugin_->shutdown();
  const int cycles = healthy.list_requests();
  ASSERT_GE(cycles, 12) << "the plugin never polled the node that answers";
  EXPECT_GE(no_values.get_requests(), cycles - 1)
      << "a node answering without values was asked on " << no_values.get_requests() << " of " << cycles << " cycles";
  EXPECT_TRUE(plugin_->store().get("no_values_healthy").has_value());
  EXPECT_EQ(plugin_->store().size(), 1U);
}

// Timed-out polls leave nothing pending, and each node keeps one client across cycles.
TEST_F(ParamBeaconPluginTest, TimedOutPollsLeaveNothingPendingAndKeepOneClientPerNode) {
  ParameterServiceStub get_silent("pending_poll_get_silent", true);
  ParameterServiceStub list_silent("pending_poll_list_silent", false);
  auto client_node = std::make_shared<rclcpp::Node>("_pending_poll_clients");
  std::mutex created_mutex;
  std::map<std::string, std::vector<std::shared_ptr<RealParameterClient>>> created;

  plugin_ = std::make_unique<ParameterBeaconPlugin>([&](const std::string & target) {
    auto client = std::make_shared<RealParameterClient>(client_node, target, std::chrono::duration<double>(0.05));
    std::lock_guard<std::mutex> lock(created_mutex);
    created[target].push_back(client);
    return client;
  });
  nlohmann::json config;
  config["poll_interval_sec"] = 0.05;
  config["param_timeout_sec"] = 1.0;
  config["poll_budget_sec"] = 5.0;
  plugin_->configure(config);
  plugin_->set_context(*mock_ctx_);

  // Four timeouts each: cycles 1, 3, 6 and 11 of the backoff.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while ((get_silent.get_requests() < 4 || list_silent.list_requests() < 4) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  plugin_->shutdown();
  ASSERT_GE(get_silent.get_requests(), 4);
  ASSERT_GE(list_silent.list_requests(), 4);

  std::lock_guard<std::mutex> lock(created_mutex);
  std::size_t pending = 0;
  for (const auto & [target, clients] : created) {
    for (const auto & client : clients) {
      pending += client->prune_pending_requests();
    }
  }
  EXPECT_EQ(pending, 0U);
  EXPECT_EQ(created[get_silent.fqn()].size(), 1U);
  EXPECT_EQ(created[list_silent.fqn()].size(), 1U);
}

// A graph read that finds no target drops every client.
TEST_F(ParamBeaconPluginTest, EmptyTargetListDropsEveryClient) {
  auto stub = std::make_unique<ParameterServiceStub>("dropped_when_gone", true, GetAnswer::kValue);
  const std::string request_topic = "rq" + stub->fqn() + "/list_parametersRequest";
  auto clients = [&]() {
    const auto infos = node_->get_publishers_info_by_topic(request_topic, true);
    return std::count_if(infos.begin(), infos.end(), [](const rclcpp::TopicEndpointInfo & info) {
      return info.node_name() == "_param_beacon_node";
    });
  };
  auto wait_clients = [&](std::ptrdiff_t expected) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (clients() != expected && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return clients();
  };

  plugin_ = std::make_unique<ParameterBeaconPlugin>();
  nlohmann::json config;
  config["poll_interval_sec"] = 0.1;
  config["param_timeout_sec"] = 5.0;
  plugin_->configure(config);
  plugin_->set_context(*mock_ctx_);

  ASSERT_EQ(wait_clients(1), 1) << "the plugin never created a client for " << stub->fqn();
  stub.reset();
  EXPECT_EQ(wait_clients(0), 0) << "the client stayed after the graph listed no target";
}

// A graph read skips the gateway's helper nodes.
TEST_F(ParamBeaconPluginTest, GatewayHelperNodesAreNotPolled) {
  ParameterServiceStub healthy("helper_skip_healthy", true, GetAnswer::kValue);
  ParameterServiceStub fault_clients("test_param_beacon_node_fault_clients", true, GetAnswer::kValue);
  ParameterServiceStub state_reader("test_param_beacon_node_lifecycle_state_reader", true, GetAnswer::kValue);

  plugin_ = std::make_unique<ParameterBeaconPlugin>();
  nlohmann::json config;
  config["poll_interval_sec"] = 0.1;
  config["param_timeout_sec"] = 5.0;
  plugin_->configure(config);
  plugin_->set_context(*mock_ctx_);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (healthy.list_requests() < 5 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  plugin_->shutdown();
  ASSERT_GE(healthy.list_requests(), 5);
  EXPECT_EQ(fault_clients.list_requests(), 0);
  EXPECT_EQ(state_reader.list_requests(), 0);
}

// The plugin's node joins the graph listener in set_context(). A first join after rclcpp shuts down
// fails half-way, and destroying the node then terminates the process.
TEST_F(ParamBeaconPluginTest, NodeDestroyedAfterShutdownDoesNotTerminate) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_EXIT(
      {
        auto plugin = std::make_unique<ParameterBeaconPlugin>();
        nlohmann::json config;
        config["poll_interval_sec"] = 1000.0;
        plugin->configure(config);
        plugin->set_context(*mock_ctx_);
        // Starts the graph listener, so rclcpp::shutdown() shuts it down.
        node_->get_node_graph_interface()->get_graph_event();
        rclcpp::shutdown();
        try {
          plugin->param_node()->get_node_graph_interface()->get_graph_event();
        } catch (const std::exception &) {
        }
        plugin.reset();
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}
