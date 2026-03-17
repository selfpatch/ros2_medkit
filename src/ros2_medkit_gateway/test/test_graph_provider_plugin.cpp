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
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/plugins/graph_provider_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/list_faults.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using namespace std::chrono_literals;
using namespace ros2_medkit_gateway;

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

diagnostic_msgs::msg::KeyValue make_key_value(const std::string & key, const std::string & value) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
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

  std::optional<PluginEntityInfo> validate_entity_for_route(const httplib::Request & /*req*/, httplib::Response & res,
                                                            const std::string & entity_id) const override {
    auto entity = get_entity(entity_id);
    if (!entity) {
      send_error(res, 404, "entity-not-found", "Entity not found");
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

 private:
  rclcpp::Node * node_{nullptr};
  std::unordered_map<std::string, PluginEntityInfo> entities_;
  std::map<SovdEntityType, std::vector<std::string>> registered_capabilities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_capabilities_;
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

/// RAII guard that joins a spin_some loop thread on all exit paths (including
/// early returns from ASSERT_* macros) so we never hit std::terminate from a
/// joinable std::thread destructor.
struct SpinGuard {
  std::atomic<bool> & flag;
  std::thread & thread;
  ~SpinGuard() {
    flag.store(true, std::memory_order_relaxed);
    if (thread.joinable()) {
      thread.join();
    }
  }
};

class GraphProviderPluginRosTest : public ::testing::Test {
 protected:
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

TEST(GraphProviderPluginRouteTest, ServesFunctionGraphFromCachedSnapshot) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  plugin.introspect(input);

  httplib::Server server;
  plugin.register_routes(server, "/api/v1");

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

TEST(GraphProviderPluginRouteTest, RegistersFunctionCapabilityOnContext) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});

  plugin.set_context(ctx);

  const auto caps = ctx.get_type_capabilities(SovdEntityType::FUNCTION);
  ASSERT_EQ(caps.size(), 1u);
  EXPECT_EQ(caps[0], "x-medkit-graph");
}

TEST(GraphProviderPluginRouteTest, UsesPreviousOnlineTimestampForOfflineLastSeen) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.set_context(ctx);

  auto online_input = make_input({make_app("node1", {}, {}, true)}, {make_function("fn", {"node1"})});
  plugin.introspect(online_input);

  std::this_thread::sleep_for(5ms);

  auto offline_input = make_input({make_app("node1", {}, {}, false)}, {make_function("fn", {"node1"})});
  plugin.introspect(offline_input);

  httplib::Server server;
  plugin.register_routes(server, "/api/v1");

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

TEST_F(GraphProviderPluginRosTest, PrefersMergedGatewayEntityCacheOverStalePluginSnapshot) {
  auto gateway_node = std::make_shared<GatewayNode>();
  const auto server_host = gateway_node->get_parameter("server.host").as_string();
  const auto server_port = static_cast<int>(gateway_node->get_parameter("server.port").as_int());

  {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5);
    httplib::Client health_client(server_host, server_port);
    while (std::chrono::steady_clock::now() - start < timeout) {
      if (auto health = health_client.Get("/api/v1/health")) {
        if (health->status == 200) {
          break;
        }
      }
      std::this_thread::sleep_for(50ms);
    }

    GraphProviderPlugin * plugin = nullptr;
    for (auto * provider : gateway_node->get_plugin_manager()->get_introspection_providers()) {
      plugin = dynamic_cast<GraphProviderPlugin *>(provider);
      if (plugin) {
        break;
      }
    }
    ASSERT_NE(plugin, nullptr);

    auto stale_input =
        make_input({make_app("a", {}, {}, false), make_app("b", {}, {}, false)}, {make_function("fn", {"a", "b"})});
    plugin->introspect(stale_input);

    auto fresh_input = make_input({make_app("a", {"/topic"}, {}, true), make_app("b", {}, {"/topic"}, true)},
                                  {make_function("fn", {"a", "b"})});

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node->get_thread_safe_cache());
    cache.update_apps(fresh_input.apps);
    cache.update_functions(fresh_input.functions);

    httplib::Client client(server_host, server_port);
    auto res = client.Get("/api/v1/functions/fn/x-medkit-graph");
    for (int attempt = 0; !res && attempt < 19; ++attempt) {
      std::this_thread::sleep_for(50ms);
      res = client.Get("/api/v1/functions/fn/x-medkit-graph");
    }

    ASSERT_TRUE(res);
    ASSERT_EQ(res->status, 200);

    auto body = nlohmann::json::parse(res->body);
    const auto & graph = body["x-medkit-graph"];
    const auto * edge = find_edge(graph, "a", "b", "/topic");
    const auto * node_a = find_node(graph, "a");
    const auto * node_b = find_node(graph, "b");

    ASSERT_NE(edge, nullptr);
    ASSERT_NE(node_a, nullptr);
    ASSERT_NE(node_b, nullptr);
    EXPECT_EQ((*node_a)["node_status"], "reachable");
    EXPECT_EQ((*node_b)["node_status"], "reachable");
    EXPECT_EQ(graph["edges"].size(), 1u);
  }

  gateway_node.reset();
}

TEST_F(GraphProviderPluginRosTest, IntrospectDoesNotQueryFaultManagerServiceOnGatewayThread) {
  auto service_node = std::make_shared<rclcpp::Node>("graph_faults_service_node");
  std::atomic<int> list_fault_calls{0};
  auto report_fault_service = service_node->create_service<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> /*request*/,
                                        std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> response) {
        response->accepted = true;
      });
  auto get_fault_service = service_node->create_service<ros2_medkit_msgs::srv::GetFault>(
      "/fault_manager/get_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> /*request*/,
                                     std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> response) {
        response->success = false;
        response->error_message = "not found";
      });
  auto list_faults_service = service_node->create_service<ros2_medkit_msgs::srv::ListFaults>(
      "/fault_manager/list_faults",
      [&list_fault_calls](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> /*request*/,
                          std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> response) {
        ++list_fault_calls;
        response->faults.clear();
      });
  auto clear_fault_service = service_node->create_service<ros2_medkit_msgs::srv::ClearFault>(
      "/fault_manager/clear_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> /*request*/,
                                       std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> response) {
        response->success = true;
        response->message = "cleared";
      });

  auto gateway_node = std::make_shared<GatewayNode>();

  // Only spin service_node - gateway_node must NOT be added to the executor.
  // GatewayNode creates wall timers (refresh_cache, cleanup, subscription_cleanup)
  // that fire on the executor thread. refresh_cache() calls ROS 2 graph APIs
  // (get_node_names_and_namespaces, get_topic_names_and_types) which can stall
  // on CI runners, causing executor.cancel() + join() to deadlock.
  // Service discovery (is_available/service_is_ready) works via DDS without
  // the node spinning in an executor.
  //
  // Use spin_some() in a loop instead of spin() so shutdown doesn't depend on
  // DDS guard conditions waking rmw_wait() - those are unreliable on loaded CI
  // runners and caused recurring buildfarm timeouts.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(service_node);
  std::atomic<bool> stop_spinning{false};
  std::thread spin_thread([&executor, &stop_spinning]() {
    while (!stop_spinning.load(std::memory_order_relaxed)) {
      executor.spin_some(100ms);
    }
  });
  SpinGuard spin_guard{stop_spinning, spin_thread};

  auto * plugin = [&]() -> GraphProviderPlugin * {
    for (auto * provider : gateway_node->get_plugin_manager()->get_introspection_providers()) {
      auto * graph_provider = dynamic_cast<GraphProviderPlugin *>(provider);
      if (graph_provider) {
        return graph_provider;
      }
    }
    return nullptr;
  }();
  ASSERT_NE(plugin, nullptr);

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && !gateway_node->get_fault_manager()->is_available()) {
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_TRUE(gateway_node->get_fault_manager()->is_available());

  list_fault_calls.store(0);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  plugin->introspect(input);

  EXPECT_EQ(list_fault_calls.load(), 0);

  // Explicit shutdown (SpinGuard is a safety net for early ASSERT returns)
  stop_spinning.store(true, std::memory_order_relaxed);
  spin_thread.join();
  executor.remove_node(service_node);
  gateway_node.reset();
  clear_fault_service.reset();
  list_faults_service.reset();
  get_fault_service.reset();
  report_fault_service.reset();
  service_node.reset();
}

TEST_F(GraphProviderPluginRosTest, HttpGraphRequestDoesNotQueryFaultManagerService) {
  auto service_node = std::make_shared<rclcpp::Node>("graph_route_faults_service_node");
  std::atomic<int> list_fault_calls{0};
  auto report_fault_service = service_node->create_service<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Request> /*request*/,
                                        std::shared_ptr<ros2_medkit_msgs::srv::ReportFault::Response> response) {
        response->accepted = true;
      });
  auto get_fault_service = service_node->create_service<ros2_medkit_msgs::srv::GetFault>(
      "/fault_manager/get_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Request> /*request*/,
                                     std::shared_ptr<ros2_medkit_msgs::srv::GetFault::Response> response) {
        response->success = false;
        response->error_message = "not found";
      });
  auto list_faults_service = service_node->create_service<ros2_medkit_msgs::srv::ListFaults>(
      "/fault_manager/list_faults",
      [&list_fault_calls](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Request> /*request*/,
                          std::shared_ptr<ros2_medkit_msgs::srv::ListFaults::Response> response) {
        ++list_fault_calls;
        response->faults.clear();
      });
  auto clear_fault_service = service_node->create_service<ros2_medkit_msgs::srv::ClearFault>(
      "/fault_manager/clear_fault", [](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> /*request*/,
                                       std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> response) {
        response->success = true;
        response->message = "cleared";
      });

  auto gateway_node = std::make_shared<GatewayNode>();

  // Only spin service_node - see IntrospectDoesNotQueryFaultManagerServiceOnGatewayThread
  // for rationale. gateway_node's wall timers can deadlock executor shutdown.
  // Use spin_some() loop to avoid guard condition unreliability on CI runners.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(service_node);
  std::atomic<bool> stop_spinning{false};
  std::thread spin_thread([&executor, &stop_spinning]() {
    while (!stop_spinning.load(std::memory_order_relaxed)) {
      executor.spin_some(100ms);
    }
  });
  SpinGuard spin_guard{stop_spinning, spin_thread};

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && !gateway_node->get_fault_manager()->is_available()) {
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_TRUE(gateway_node->get_fault_manager()->is_available());

  auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node->get_thread_safe_cache());
  cache.update_apps({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})});
  cache.update_functions({make_function("fn", {"a", "b"})});

  const auto server_host = gateway_node->get_parameter("server.host").as_string();
  const auto server_port = static_cast<int>(gateway_node->get_parameter("server.port").as_int());

  httplib::Client client(server_host, server_port);
  auto res = client.Get("/api/v1/functions/fn/x-medkit-graph");
  for (int attempt = 0; !res && attempt < 19; ++attempt) {
    std::this_thread::sleep_for(50ms);
    res = client.Get("/api/v1/functions/fn/x-medkit-graph");
  }

  ASSERT_TRUE(res);
  ASSERT_EQ(res->status, 200);
  EXPECT_EQ(list_fault_calls.load(), 0);

  // Explicit shutdown (SpinGuard is a safety net for early ASSERT returns)
  stop_spinning.store(true, std::memory_order_relaxed);
  spin_thread.join();
  executor.remove_node(service_node);
  gateway_node.reset();
  clear_fault_service.reset();
  list_faults_service.reset();
  get_fault_service.reset();
  report_fault_service.reset();
  service_node.reset();
}

TEST(GraphProviderPluginMetricsTest, BrokenPipelineHasNullBottleneckEdgeEvenWhenActiveEdgesExist) {
  // edge a→b: /ab, b is offline → node_offline error → broken
  // edge a→c: /ac, both online, freq=5 (below 30*0.5) → active but degraded
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

TEST_F(GraphProviderPluginRosTest, SetsDiagnosticsSeenWhenNonGreenwaveMessageArrives) {
  auto node = std::make_shared<rclcpp::Node>("graph_diag_seen_test_node");
  auto pub_node = std::make_shared<rclcpp::Node>("graph_diag_seen_pub");
  auto diagnostics_pub = pub_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(pub_node);

  {
    GraphProviderPlugin plugin;
    FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}}, node.get());
    plugin.set_context(ctx);

    auto input =
        make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});

    // Publish a diagnostic message with no greenwave keys (non-greenwave hardware diagnostics)
    diagnostic_msgs::msg::DiagnosticArray msg;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/some/hardware/driver";
    status.values = {make_key_value("temperature", "72.3"), make_key_value("voltage", "3.3")};
    msg.status.push_back(status);
    diagnostics_pub->publish(msg);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
    executor.spin_some();

    plugin.introspect(input);

    httplib::Server server;
    plugin.register_routes(server, "/api/v1");
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
    const auto * edge = find_edge(graph, "a", "b", "/topic");
    ASSERT_NE(edge, nullptr);
    // /diagnostics is alive (non-greenwave message received), but /topic has no greenwave entry
    // → must be "error"/"no_data_source", NOT "pending"
    EXPECT_EQ((*edge)["metrics"]["metrics_status"], "error");
    EXPECT_EQ((*edge)["metrics"]["error_reason"], "no_data_source");

    local_server.stop();
  }

  executor.remove_node(pub_node);
  executor.remove_node(node);
  pub_node.reset();
  node.reset();
}

TEST_F(GraphProviderPluginRosTest, AppliesPerFunctionConfigOverridesFromNodeParameters) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("graph_provider.function_overrides.fn.drop_rate_percent_threshold", 1.0);
  auto node = std::make_shared<rclcpp::Node>("graph_provider_test_node", options);
  auto publisher_node = std::make_shared<rclcpp::Node>("graph_provider_test_pub");
  auto diagnostics_pub = publisher_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(publisher_node);

  {
    GraphProviderPlugin plugin;
    FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}}, node.get());
    plugin.set_context(ctx);

    auto input =
        make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});

    diagnostic_msgs::msg::DiagnosticArray msg;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/topic";
    status.values = {make_key_value("frame_rate_msg", "30.0"), make_key_value("current_delay_from_realtime_ms", "1.0"),
                     make_key_value("expected_frequency", "30.0"), make_key_value("drop_rate_percent", "2.0")};
    msg.status.push_back(status);
    diagnostics_pub->publish(msg);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
    executor.spin_some();

    plugin.introspect(input);

    httplib::Server server;
    plugin.register_routes(server, "/api/v1");

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
    EXPECT_EQ(graph["pipeline_status"], "degraded");

    const auto * edge = find_edge(graph, "a", "b", "/topic");
    ASSERT_NE(edge, nullptr);
    EXPECT_DOUBLE_EQ((*edge)["metrics"]["drop_rate_percent"], 2.0);

    local_server.stop();
  }

  executor.remove_node(publisher_node);
  executor.remove_node(node);
  publisher_node.reset();
  node.reset();
}
