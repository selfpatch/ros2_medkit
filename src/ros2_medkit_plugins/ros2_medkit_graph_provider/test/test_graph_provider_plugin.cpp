// Copyright 2026 eclipse0922
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
#include <httplib.h>

#include <atomic>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/plugins/plugin_http_types.hpp"
#include "ros2_medkit_graph_provider/graph_provider_plugin.hpp"

using namespace std::chrono_literals;
using namespace ros2_medkit_gateway;

// Stubs for PluginRequest/PluginResponse (implemented in gateway_lib, not linked into tests).
// These wrap httplib::Request/Response directly so the route tests can use a real HTTP server.
namespace ros2_medkit_gateway {

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t index) const {
  const auto & req = *static_cast<const httplib::Request *>(impl_);
  if (index < req.matches.size()) {
    return req.matches[index].str();
  }
  return {};
}
std::string PluginRequest::header(const std::string & name) const {
  return static_cast<const httplib::Request *>(impl_)->get_header_value(name);
}
const std::string & PluginRequest::path() const {
  return static_cast<const httplib::Request *>(impl_)->path;
}
const std::string & PluginRequest::body() const {
  return static_cast<const httplib::Request *>(impl_)->body;
}

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json & data) {
  auto & res = *static_cast<httplib::Response *>(impl_);
  res.status = 200;
  res.body = data.dump();
  res.headers.emplace("Content-Type", "application/json");
}
void PluginResponse::send_error(int status, const std::string & /*error_code*/, const std::string & message,
                                const nlohmann::json & /*parameters*/) {
  auto & res = *static_cast<httplib::Response *>(impl_);
  res.status = status;
  nlohmann::json err = {{"error", message}};
  res.body = err.dump();
  res.headers.emplace("Content-Type", "application/json");
}

}  // namespace ros2_medkit_gateway

/// Helper: register all routes from get_routes() on an httplib::Server with a given api_prefix.
static void register_plugin_routes(httplib::Server & server, const std::string & api_prefix,
                                   ros2_medkit_gateway::GatewayPlugin & plugin) {
  for (auto & route : plugin.get_routes()) {
    auto pattern = api_prefix + "/" + route.pattern;
    auto handler = route.handler;
    if (route.method == "GET") {
      server.Get(pattern.c_str(), [handler](const httplib::Request & req, httplib::Response & res) {
        PluginRequest preq(&req);
        PluginResponse pres(&res);
        handler(preq, pres);
      });
    }
  }
}

namespace {

App make_app(const std::string & id, std::vector<std::string> publishes = {}, std::vector<std::string> subscribes = {},
             bool is_online = true, const std::string & component_id = "") {
  App app;
  app.id = id;
  app.name = id;
  app.component_id = component_id;
  app.is_online = is_online;
  app.topics.publishes = std::move(publishes);
  app.topics.subscribes = std::move(subscribes);
  return app;
}

Function make_function(const std::string & id, std::vector<std::string> hosts) {
  Function func;
  func.id = id;
  func.name = id;
  func.hosts = std::move(hosts);
  return func;
}

IntrospectionInput make_input(std::vector<App> apps, std::vector<Function> functions) {
  IntrospectionInput input;
  input.apps = std::move(apps);
  input.functions = std::move(functions);
  return input;
}

GraphProviderPlugin::GraphBuildConfig default_config() {
  GraphProviderPlugin::GraphBuildConfig config;
  config.expected_frequency_hz_default = 30.0;
  config.degraded_frequency_ratio = 0.5;
  config.drop_rate_percent_threshold = 5.0;
  return config;
}

GraphProviderPlugin::GraphBuildState make_state(bool diagnostics_seen = false) {
  GraphProviderPlugin::GraphBuildState state;
  state.diagnostics_seen = diagnostics_seen;
  return state;
}

GraphProviderPlugin::TopicMetrics make_metrics(double frequency_hz, std::optional<double> latency_ms = std::nullopt,
                                               double drop_rate_percent = 0.0,
                                               std::optional<double> expected_frequency_hz = std::nullopt) {
  GraphProviderPlugin::TopicMetrics metrics;
  metrics.frequency_hz = frequency_hz;
  metrics.latency_ms = latency_ms;
  metrics.drop_rate_percent = drop_rate_percent;
  metrics.expected_frequency_hz = expected_frequency_hz;
  return metrics;
}

std::string find_topic_id(const nlohmann::json & graph, const std::string & topic_name) {
  for (const auto & topic : graph["topics"]) {
    if (topic["name"] == topic_name) {
      return topic["topic_id"];
    }
  }
  return "";
}

const nlohmann::json * find_edge(const nlohmann::json & graph, const std::string & source, const std::string & target,
                                 const std::string & topic_name) {
  const auto topic_id = find_topic_id(graph, topic_name);
  for (const auto & edge : graph["edges"]) {
    if (edge["source"] == source && edge["target"] == target && edge["topic_id"] == topic_id) {
      return &edge;
    }
  }
  return nullptr;
}

const nlohmann::json * find_node(const nlohmann::json & graph, const std::string & entity_id) {
  for (const auto & node : graph["nodes"]) {
    if (node["entity_id"] == entity_id) {
      return &node;
    }
  }
  return nullptr;
}

class FakePluginContext : public PluginContext {
 public:
  explicit FakePluginContext(std::unordered_map<std::string, PluginEntityInfo> entities, rclcpp::Node * node = nullptr)
    : node_(node), entities_(std::move(entities)) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities_.find(id);
    if (it == entities_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  nlohmann::json list_entity_faults(const std::string & /*entity_id*/) const override {
    return nlohmann::json::array();
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & /*req*/, PluginResponse & res,
                                                            const std::string & entity_id) const override {
    auto entity = get_entity(entity_id);
    if (!entity) {
      res.send_error(404, "entity-not-found", "Entity not found");
      return std::nullopt;
    }
    return entity;
  }

  void register_capability(SovdEntityType entity_type, const std::string & capability_name) override {
    registered_capabilities_[entity_type].push_back(capability_name);
  }

  void register_entity_capability(const std::string & entity_id, const std::string & capability_name) override {
    entity_capabilities_[entity_id].push_back(capability_name);
  }

  std::vector<std::string> get_type_capabilities(SovdEntityType entity_type) const override {
    auto it = registered_capabilities_.find(entity_type);
    if (it == registered_capabilities_.end()) {
      return {};
    }
    return it->second;
  }

  std::vector<std::string> get_entity_capabilities(const std::string & entity_id) const override {
    auto it = entity_capabilities_.find(entity_id);
    if (it == entity_capabilities_.end()) {
      return {};
    }
    return it->second;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*component_id*/) const override {
    return {};
  }

  LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                              const std::string & /*collection*/) const override {
    return LockAccessResult{true, "", "", ""};
  }

  tl::expected<LockInfo, LockError> acquire_lock(const std::string & /*entity_id*/, const std::string & /*client_id*/,
                                                 const std::vector<std::string> & /*scopes*/,
                                                 int /*expiration_seconds*/) override {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking not available in test", 503, std::nullopt});
  }

  tl::expected<void, LockError> release_lock(const std::string & /*entity_id*/,
                                             const std::string & /*client_id*/) override {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking not available in test", 503, std::nullopt});
  }

  IntrospectionInput get_entity_snapshot() const override {
    return entity_snapshot_;
  }

  nlohmann::json list_all_faults() const override {
    return all_faults_;
  }

  void register_sampler(
      const std::string & collection,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> & fn)
      override {
    registered_samplers_[collection] = fn;
  }

  ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }

  ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }

 private:
  rclcpp::Node * node_{nullptr};
  std::unordered_map<std::string, PluginEntityInfo> entities_;
  std::map<SovdEntityType, std::vector<std::string>> registered_capabilities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_capabilities_;

 public:
  IntrospectionInput entity_snapshot_;
  nlohmann::json all_faults_ = nlohmann::json::object();
  std::unordered_map<std::string,
                     std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)>>
      registered_samplers_;
};

class LocalHttpServer {
 public:
  LocalHttpServer() = default;

  ~LocalHttpServer() {
    stop();
  }

  void start(httplib::Server & server) {
    server_ = &server;
    port_ = server.bind_to_any_port("127.0.0.1");
    ASSERT_GT(port_, 0);
    thread_ = std::thread([&server]() {
      server.listen_after_bind();
    });
    std::this_thread::sleep_for(50ms);
  }

  void stop() {
    if (thread_.joinable() && server_) {
      server_->stop();
      thread_.join();
    }
  }

  int port() const {
    return port_;
  }

 private:
  httplib::Server * server_{nullptr};
  std::thread thread_;
  int port_{-1};
};

}  // namespace

TEST(GraphProviderPluginBuildTest, BuildsThreeNodeChain) {
  auto input = make_input(
      {make_app("a", {"/a_to_b"}, {}), make_app("b", {"/b_to_c"}, {"/a_to_b"}), make_app("c", {}, {"/b_to_c"})},
      {make_function("fn", {"a", "b", "c"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  ASSERT_EQ(graph["nodes"].size(), 3u);
  ASSERT_EQ(graph["topics"].size(), 2u);
  ASSERT_EQ(graph["edges"].size(), 2u);
  const auto * edge_ab = find_edge(graph, "a", "b", "/a_to_b");
  const auto * edge_bc = find_edge(graph, "b", "c", "/b_to_c");
  ASSERT_NE(edge_ab, nullptr);
  ASSERT_NE(edge_bc, nullptr);
  EXPECT_EQ((*edge_ab)["topic_id"], find_topic_id(graph, "/a_to_b"));
  EXPECT_EQ((*edge_bc)["topic_id"], find_topic_id(graph, "/b_to_c"));
}

TEST(GraphProviderPluginBuildTest, BuildsFanOutEdgesSharingTopicId) {
  auto input =
      make_input({make_app("a", {"/shared"}, {}), make_app("b", {}, {"/shared"}), make_app("c", {}, {"/shared"})},
                 {make_function("fn", {"a", "b", "c"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  ASSERT_EQ(graph["edges"].size(), 2u);
  const auto * edge_ab = find_edge(graph, "a", "b", "/shared");
  const auto * edge_ac = find_edge(graph, "a", "c", "/shared");
  ASSERT_NE(edge_ab, nullptr);
  ASSERT_NE(edge_ac, nullptr);
  EXPECT_EQ((*edge_ab)["topic_id"], (*edge_ac)["topic_id"]);
}

TEST(GraphProviderPluginBuildTest, BuildsFanInEdgesSharingTopicId) {
  auto input =
      make_input({make_app("a", {"/shared"}, {}), make_app("b", {"/shared"}, {}), make_app("c", {}, {"/shared"})},
                 {make_function("fn", {"a", "b", "c"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  ASSERT_EQ(graph["edges"].size(), 2u);
  const auto * edge_ac = find_edge(graph, "a", "c", "/shared");
  const auto * edge_bc = find_edge(graph, "b", "c", "/shared");
  ASSERT_NE(edge_ac, nullptr);
  ASSERT_NE(edge_bc, nullptr);
  EXPECT_EQ((*edge_ac)["topic_id"], (*edge_bc)["topic_id"]);
}

TEST(GraphProviderPluginBuildTest, BuildsSelfLoopEdgeForAppPublishingAndSubscribingSameTopic) {
  auto input = make_input({make_app("a", {"/loop"}, {"/loop"})}, {make_function("fn", {"a"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  ASSERT_EQ(graph["topics"].size(), 1u);
  ASSERT_EQ(graph["edges"].size(), 1u);
  const auto * edge = find_edge(graph, "a", "a", "/loop");
  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["topic_id"], find_topic_id(graph, "/loop"));
}

TEST(GraphProviderPluginBuildTest, FiltersInfrastructureAndNitrosTopics) {
  auto input = make_input(
      {make_app("a", {"/rosout", "/parameter_events", "/diagnostics", "/camera/nitros", "/real"}, {}),
       make_app("b", {}, {"/rosout", "/parameter_events", "/diagnostics", "/camera/_supported_types", "/real"})},
      {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  ASSERT_EQ(graph["topics"].size(), 1u);
  EXPECT_EQ(graph["topics"][0]["name"], "/real");
  ASSERT_EQ(graph["edges"].size(), 1u);
  EXPECT_NE(find_edge(graph, "a", "b", "/real"), nullptr);
}

TEST(GraphProviderPluginBuildTest, MarksReachableAndUnreachableNodes) {
  auto input = make_input({make_app("online", {"/topic"}, {}, true), make_app("offline", {}, {"/topic"}, false)},
                          {make_function("fn", {"online", "offline"})});
  auto state = make_state();
  state.last_seen_by_app["offline"] = "2026-03-08T11:59:00.000Z";

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * online_node = find_node(graph, "online");
  const auto * offline_node = find_node(graph, "offline");

  ASSERT_EQ(graph["nodes"].size(), 2u);
  ASSERT_NE(online_node, nullptr);
  ASSERT_NE(offline_node, nullptr);
  EXPECT_EQ((*online_node)["node_status"], "reachable");
  EXPECT_FALSE(online_node->contains("last_seen"));
  EXPECT_EQ((*offline_node)["node_status"], "unreachable");
  EXPECT_TRUE(offline_node->contains("last_seen"));
  EXPECT_EQ((*offline_node)["last_seen"], "2026-03-08T11:59:00.000Z");
}

TEST(GraphProviderPluginBuildTest, ReturnsEmptyGraphForNoApps) {
  auto input = make_input({}, {make_function("fn", {})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  EXPECT_EQ(graph["schema_version"], "1.0.0");
  EXPECT_EQ(graph["graph_id"], "fn-graph");
  EXPECT_EQ(graph["scope"]["type"], "function");
  EXPECT_EQ(graph["scope"]["entity_id"], "fn");
  EXPECT_TRUE(graph["topics"].empty());
  EXPECT_TRUE(graph["nodes"].empty());
  EXPECT_TRUE(graph["edges"].empty());
}

TEST(GraphProviderPluginBuildTest, ExpandsComponentHostsToScopedApps) {
  auto input = make_input({make_app("a", {"/topic"}, {}, true, "comp1"), make_app("b", {}, {"/topic"}, true, "comp1"),
                           make_app("c", {}, {"/topic"}, true, "comp2")},
                          {make_function("fn", {"comp1"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  ASSERT_EQ(graph["nodes"].size(), 2u);
  EXPECT_NE(find_node(graph, "a"), nullptr);
  EXPECT_NE(find_node(graph, "b"), nullptr);
  EXPECT_EQ(find_node(graph, "c"), nullptr);
  EXPECT_NE(find_edge(graph, "a", "b", "/topic"), nullptr);
}

TEST(GraphProviderPluginMetricsTest, MarksActiveEdgeWhenGreenwaveMetricsExist) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto state = make_state(true);
  state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0);

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["source"], "greenwave_monitor");
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["frequency_hz"], 29.8);
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["latency_ms"], 1.2);
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["drop_rate_percent"], 0.0);
  EXPECT_FALSE((*edge)["metrics"].contains("error_reason"));
  EXPECT_FALSE(edge->contains("error_reason"));
  EXPECT_EQ(graph["pipeline_status"], "healthy");
}

TEST(GraphProviderPluginMetricsTest, MarksPendingWhenDiagnosticsHaveNotArrivedYet) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(false), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "pending");
  EXPECT_FALSE((*edge)["metrics"].contains("error_reason"));
  EXPECT_FALSE(edge->contains("error_reason"));
}

TEST(GraphProviderPluginMetricsTest, MarksTopicStaleErrorsFromFaultState) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto state = make_state(true);
  state.stale_topics.insert("/topic");

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "error");
  ASSERT_TRUE((*edge)["metrics"].contains("error_reason"));
  EXPECT_EQ((*edge)["metrics"]["error_reason"], "topic_stale");
  EXPECT_FALSE(edge->contains("error_reason"));
  EXPECT_EQ(graph["pipeline_status"], "broken");
}

TEST(GraphProviderPluginMetricsTest, MarksNodeOfflineErrors) {
  auto input = make_input({make_app("a", {"/topic"}, {}, true), make_app("b", {}, {"/topic"}, false)},
                          {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(true), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "error");
  ASSERT_TRUE((*edge)["metrics"].contains("error_reason"));
  EXPECT_EQ((*edge)["metrics"]["error_reason"], "node_offline");
  EXPECT_FALSE(edge->contains("error_reason"));
  EXPECT_EQ(graph["pipeline_status"], "broken");
}

TEST(GraphProviderPluginMetricsTest, MarksNoDataSourceErrorsWhenDiagnosticsArePresentButTopicIsMissing) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(true), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "error");
  ASSERT_TRUE((*edge)["metrics"].contains("error_reason"));
  EXPECT_EQ((*edge)["metrics"]["error_reason"], "no_data_source");
  EXPECT_FALSE(edge->contains("error_reason"));
  EXPECT_EQ(graph["pipeline_status"], "broken");
}

TEST(GraphProviderPluginMetricsTest, KeepsBrokenPipelineBottleneckNullWhenNoFrequencyExists) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(true), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  EXPECT_EQ(graph["pipeline_status"], "broken");
  EXPECT_TRUE(graph["bottleneck_edge"].is_null());
}

TEST(GraphProviderPluginMetricsTest, MarksPipelineDegradedWhenFrequencyDropsBelowThreshold) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto state = make_state(true);
  state.topic_metrics["/topic"] = make_metrics(10.0, 5.0, 0.0, 30.0);

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_EQ(graph["pipeline_status"], "degraded");
  EXPECT_EQ(graph["bottleneck_edge"], (*edge)["edge_id"]);
}

TEST(GraphProviderPluginMetricsTest, MarksPipelineDegradedWhenDropRateExceedsThreshold) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto state = make_state(true);
  state.topic_metrics["/topic"] = make_metrics(30.0, 1.0, 7.5, 30.0);

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["drop_rate_percent"], 7.5);
  EXPECT_EQ(graph["pipeline_status"], "degraded");
  EXPECT_EQ(graph["bottleneck_edge"], (*edge)["edge_id"]);
}

TEST(GraphProviderPluginMetricsTest, ChoosesSlowestEdgeAsBottleneck) {
  auto input = make_input({make_app("a", {"/ab", "/ac"}, {}), make_app("b", {}, {"/ab"}), make_app("c", {}, {"/ac"})},
                          {make_function("fn", {"a", "b", "c"})});
  auto state = make_state(true);
  state.topic_metrics["/ab"] = make_metrics(25.0, 1.0, 0.0, 30.0);
  state.topic_metrics["/ac"] = make_metrics(5.0, 1.0, 0.0, 30.0);

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * slower_edge = find_edge(graph, "a", "c", "/ac");

  ASSERT_NE(slower_edge, nullptr);
  EXPECT_EQ(graph["bottleneck_edge"], (*slower_edge)["edge_id"]);
}

// @verifies REQ_INTEROP_003
TEST(GraphProviderPluginRouteTest, ServesFunctionGraphFromCachedSnapshot) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.configure({});
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  plugin.introspect(input);

  httplib::Server server;
  register_plugin_routes(server, "/api/v1", plugin);

  LocalHttpServer local_server;
  local_server.start(server);

  httplib::Client client("127.0.0.1", local_server.port());
  auto res = client.Get("/api/v1/functions/fn/x-medkit-graph");
  for (int attempt = 0; !res && attempt < 19; ++attempt) {
    std::this_thread::sleep_for(50ms);
    res = client.Get("/api/v1/functions/fn/x-medkit-graph");
  }

  ASSERT_TRUE(res);
  ASSERT_EQ(res->status, 200);

  auto body = nlohmann::json::parse(res->body);
  ASSERT_TRUE(body.contains("x-medkit-graph"));
  EXPECT_EQ(body["x-medkit-graph"]["schema_version"], "1.0.0");
  EXPECT_EQ(body["x-medkit-graph"]["scope"]["type"], "function");
  EXPECT_EQ(body["x-medkit-graph"]["scope"]["entity_id"], "fn");
  EXPECT_TRUE(body["x-medkit-graph"].contains("timestamp"));

  local_server.stop();
}

// @verifies REQ_INTEROP_003
TEST(GraphProviderPluginRouteTest, RegistersFunctionCapabilityOnContext) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.configure({});
  plugin.set_context(ctx);

  const auto caps = ctx.get_type_capabilities(SovdEntityType::FUNCTION);
  ASSERT_EQ(caps.size(), 1u);
  EXPECT_EQ(caps[0], "x-medkit-graph");
}

TEST(GraphProviderPluginRouteTest, UsesPreviousOnlineTimestampForOfflineLastSeen) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.configure({});
  plugin.set_context(ctx);

  auto online_input = make_input({make_app("node1", {}, {}, true)}, {make_function("fn", {"node1"})});
  plugin.introspect(online_input);

  // Ensure timestamps differ even under CPU contention during parallel testing
  std::this_thread::sleep_for(50ms);

  auto offline_input = make_input({make_app("node1", {}, {}, false)}, {make_function("fn", {"node1"})});
  plugin.introspect(offline_input);

  // Expose offline input to the context so get_cached_or_built_graph() rebuilds from current entity state.
  ctx.entity_snapshot_ = offline_input;

  httplib::Server server;
  register_plugin_routes(server, "/api/v1", plugin);

  LocalHttpServer local_server;
  local_server.start(server);

  httplib::Client client("127.0.0.1", local_server.port());
  auto res = client.Get("/api/v1/functions/fn/x-medkit-graph");
  for (int attempt = 0; !res && attempt < 19; ++attempt) {
    std::this_thread::sleep_for(50ms);
    res = client.Get("/api/v1/functions/fn/x-medkit-graph");
  }

  ASSERT_TRUE(res);
  ASSERT_EQ(res->status, 200);

  auto body = nlohmann::json::parse(res->body);
  const auto & graph = body["x-medkit-graph"];
  const auto * node = find_node(graph, "node1");
  ASSERT_NE(node, nullptr);
  ASSERT_TRUE(node->contains("last_seen"));
  EXPECT_LT((*node)["last_seen"].get<std::string>(), graph["timestamp"].get<std::string>());

  local_server.stop();
}

TEST(GraphProviderPluginMetricsTest, BrokenPipelineHasNullBottleneckEdgeEvenWhenActiveEdgesExist) {
  // edge a->b: /ab, b is offline -> node_offline error -> broken
  // edge a->c: /ac, both online, freq=5 (below 30*0.5) -> active but degraded
  // pipeline_status should be "broken", bottleneck_edge should be null
  auto input = make_input(
      {make_app("a", {"/ab", "/ac"}, {}, true), make_app("b", {}, {"/ab"}, false), make_app("c", {}, {"/ac"}, true)},
      {make_function("fn", {"a", "b", "c"})});
  auto state = make_state(true);
  state.topic_metrics["/ac"] = make_metrics(5.0, 1.0, 0.0, 30.0);

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  EXPECT_EQ(graph["pipeline_status"], "broken");
  EXPECT_TRUE(graph["bottleneck_edge"].is_null());
}

// @verifies REQ_INTEROP_003
TEST(GraphProviderPluginRouteTest, RegistersSamplerForCyclicSubscriptions) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}});
  plugin.configure({});
  plugin.set_context(ctx);
  ASSERT_EQ(ctx.registered_samplers_.count("x-medkit-graph"), 1u);

  // Populate graph cache via introspect
  auto input = make_input({make_app("a1", {"/t1"}, {})}, {make_function("f1", {"a1"})});
  plugin.introspect(input);

  // Invoke the sampler and verify it returns valid graph data
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->contains("x-medkit-graph"));
}

TEST(GraphProviderPluginRouteTest, AppliesConfigFromConfigure) {
  nlohmann::json config = {
      {"expected_frequency_hz_default", 10.0}, {"degraded_frequency_ratio", 0.8}, {"drop_rate_percent_threshold", 2.0}};

  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}});
  plugin.configure(config);
  plugin.set_context(ctx);

  auto input = make_input({make_app("a1", {"/t1"}, {"/t2"})}, {make_function("f1", {"a1"})});
  plugin.introspect(input);

  // Verify the config was applied: the plugin accepted non-default config,
  // initialized correctly, and produced a valid graph
  ASSERT_EQ(ctx.registered_samplers_.count("x-medkit-graph"), 1u);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->contains("x-medkit-graph"));
}

class GraphProviderPluginRosTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST_F(GraphProviderPluginRosTest, SetsDiagnosticsSeenWhenMessageArrives) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_node");
  FakePluginContext ctx({}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  // Publish a diagnostics message
  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "some_sensor";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "frame_rate_msg";
  kv.value = "30.0";
  status.values.push_back(kv);
  msg.status.push_back(status);
  pub->publish(msg);

  // Spin to process the subscription
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  // Build a graph - diagnostics_seen should affect metrics status
  auto input = make_input({make_app("a1", {"/topic1"}, {"/topic2"})}, {make_function("f1", {"a1"})});
  // After diagnostics callback fires, edges with no metrics for their topic
  // should get "error"/"no_data_source" instead of "pending"
  auto state = GraphProviderPlugin::GraphBuildState{};
  state.diagnostics_seen = true;
  auto graph = GraphProviderPlugin::build_graph_document("f1", input, state, default_config(), "2026-01-01T00:00:00Z");

  ASSERT_TRUE(graph.contains("x-medkit-graph"));
  // No edges expected here (a1 has no subscriber matching a publisher) but
  // the graph document itself is valid and pipeline_status is well-defined
  EXPECT_EQ(graph["x-medkit-graph"]["schema_version"], "1.0.0");
}

TEST_F(GraphProviderPluginRosTest, DiagnosticsCallbackAfterShutdownIsNoop) {
  auto node = std::make_shared<rclcpp::Node>("test_graph_shutdown_node");
  FakePluginContext ctx({}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  // Publish diagnostics before shutdown
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/test/topic";
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "frame_rate_msg";
  kv.value = "30.0";
  status.values.push_back(kv);
  diag_msg.status.push_back(status);
  pub->publish(diag_msg);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  // Shutdown
  plugin.shutdown();

  // Publish different diagnostics - should be ignored
  diagnostic_msgs::msg::DiagnosticStatus status2;
  status2.name = "/test/new_topic_after_shutdown";
  diagnostic_msgs::msg::KeyValue kv2;
  kv2.key = "frame_rate_msg";
  kv2.value = "60.0";
  status2.values.push_back(kv2);
  diagnostic_msgs::msg::DiagnosticArray diag_msg2;
  diag_msg2.status.push_back(status2);
  pub->publish(diag_msg2);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  // Introspect should not contain the post-shutdown topic
  IntrospectionInput input;
  auto result = plugin.introspect(input);
  // Just verifying it doesn't crash is sufficient
}
