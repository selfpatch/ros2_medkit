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
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
#include "ros2_medkit_graph_provider/graph_provider_plugin.hpp"

using namespace std::chrono_literals;
using namespace ros2_medkit_gateway;

// Stubs for PluginRequest/PluginResponse (implemented in gateway_core, not linked into tests).
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

// Accessor for private state, used only by tests that need to observe an
// internal bound (last_seen_by_app_ size) rather than a JSON-visible field.
class GraphProviderPluginTestAccess {
 public:
  static size_t last_seen_by_app_count(const GraphProviderPlugin & plugin) {
    std::lock_guard<std::mutex> lock(plugin.status_mutex_);
    return plugin.last_seen_by_app_.size();
  }

  static GraphProviderPlugin::GraphBuildConfig resolve_config(const GraphProviderPlugin & plugin,
                                                              const std::string & function_id) {
    return plugin.resolve_config(function_id);
  }
};

}  // namespace ros2_medkit_gateway

/// Helper: register all routes from get_routes() on an httplib::Server with a given api_prefix.
static void register_plugin_routes(httplib::Server & server, const std::string & api_prefix,
                                   ros2_medkit_gateway::GatewayPlugin & plugin) {
  for (auto & route : plugin.get_routes()) {
    auto pattern = api_prefix + "/" + route.pattern;
    auto handler = route.handler;
    if (route.method == "GET") {
      server.Get(pattern, [handler](const httplib::Request & req, httplib::Response & res) {
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
  config.freshness_headroom_factor = 3.0;
  config.freshness_floor_sec = 5.0;
  config.stale_grace_sec = 2.0;
  return config;
}

// now_ns defaults to 0, matching TopicMetrics::last_update_ns's default, so a
// test that doesn't care about freshness (age 0 <= any positive window) gets
// "active" without having to thread timestamps through every call site.
GraphProviderPlugin::GraphBuildState make_state() {
  return GraphProviderPlugin::GraphBuildState{};
}

GraphProviderPlugin::TopicMetrics make_metrics(double frequency_hz, std::optional<double> latency_ms = std::nullopt,
                                               std::optional<double> drop_rate_percent = 0.0,
                                               std::optional<double> expected_frequency_hz = std::nullopt,
                                               int64_t last_update_ns = 0,
                                               std::optional<std::string> source = std::nullopt) {
  GraphProviderPlugin::TopicMetrics metrics;
  metrics.frequency_hz = frequency_hz;
  metrics.latency_ms = latency_ms;
  metrics.drop_rate_percent = drop_rate_percent;
  metrics.expected_frequency_hz = expected_frequency_hz;
  metrics.last_update_ns = last_update_ns;
  metrics.source = std::move(source);
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

class FakePluginContext : public RosPluginContext {
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

  // RAII cleanup: stop the listening thread on destruction. Owns a thread
  // and a raw httplib::Server pointer, so copying/moving is unsafe.
  LocalHttpServer(const LocalHttpServer &) = delete;
  LocalHttpServer & operator=(const LocalHttpServer &) = delete;
  LocalHttpServer(LocalHttpServer &&) = delete;
  LocalHttpServer & operator=(LocalHttpServer &&) = delete;

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

// The old filter matched "/nitros" and "_supported_types" as substrings
// anywhere in the topic name. That wrongly dropped real topics that merely
// contain those strings inside a longer segment name -
// "isaac_ros_nitros_bridge" is a real NVIDIA package, and any topic under a
// "/nitros_bridge" namespace contains "/nitros" as a substring;
// "/perception/nitros_input" has "nitros" embedded in a segment;
// "/a/_supported_types/b" contains "_supported_types" as a non-trailing
// segment. None of these are REP-2009 negotiation topics and must survive
// the filter.
TEST(GraphProviderPluginBuildTest, DoesNotFilterTopicsWhereNitrosOrSupportedTypesIsOnlyASubstring) {
  const std::vector<std::string> real_topics = {"/nitros_bridge/data", "/perception/nitros_input",
                                                "/a/_supported_types/b"};
  auto input =
      make_input({make_app("a", real_topics, {}), make_app("b", {}, real_topics)}, {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  ASSERT_EQ(graph["topics"].size(), real_topics.size());
  ASSERT_EQ(graph["edges"].size(), real_topics.size());
  for (const auto & topic : real_topics) {
    EXPECT_NE(find_edge(graph, "a", "b", topic), nullptr) << "expected edge for " << topic;
  }
}

// REP-2009 negotiation topics are formed by appending path segments to a
// base topic: "<base>/nitros" (negotiated-topics-info), "<base>/nitros/
// _supported_types" (supported-types), and "<base>/nitros/<format>"
// (negotiated data). All three must still be filtered regardless of how
// many segments precede "nitros".
TEST(GraphProviderPluginBuildTest, FiltersNitrosNegotiationTopicsAtAnySegmentDepth) {
  const std::vector<std::string> negotiation_topics = {"/nitros", "/a/nitros", "/a/b/nitros/c",
                                                       "/camera/nitros/_supported_types", "/camera/nitros/rgb8"};
  auto input = make_input({make_app("a", negotiation_topics, {}), make_app("b", {}, negotiation_topics)},
                          {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  EXPECT_TRUE(graph["topics"].empty());
  EXPECT_TRUE(graph["edges"].empty());
}

// The trailing-segment rule for "_supported_types" must not fire when the
// segment appears anywhere other than the end of the path.
TEST(GraphProviderPluginBuildTest, FiltersSupportedTypesOnlyAsTrailingSegment) {
  auto input = make_input({make_app("a", {"/a/b/_supported_types"}, {}), make_app("b", {}, {"/a/b/_supported_types"})},
                          {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  EXPECT_TRUE(graph["topics"].empty());
  EXPECT_TRUE(graph["edges"].empty());
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

  EXPECT_EQ(graph["schema_version"], kGraphSchemaVersion);
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

TEST(GraphProviderPluginMetricsTest, MarksActiveEdgeWhenMetricsExist) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto state = make_state();
  state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/0, "/resolved_monitor_node");

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["source"], "/resolved_monitor_node");
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["frequency_hz"], 29.8);
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["latency_ms"], 1.2);
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["drop_rate_percent"], 0.0);
  EXPECT_FALSE((*edge)["metrics"].contains("error_reason"));
  EXPECT_FALSE(edge->contains("error_reason"));
  EXPECT_EQ(graph["pipeline_status"], "healthy");
}

// Honest-failure requirement: a TopicMetrics entry whose source could not be
// resolved (e.g. the publisher GID had no match in get_publishers_info_by_topic)
// must never surface a fabricated or default value - the "source" key must be
// absent from the emitted JSON entirely, exactly like a pending edge omits it.
TEST(GraphProviderPluginMetricsTest, OmitsSourceKeyWhenTopicMetricsSourceIsUnresolved) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto state = make_state();
  state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0);  // source left unresolved (nullopt)

  auto doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, default_config(), "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_FALSE((*edge)["metrics"].contains("source"));
}

TEST(GraphProviderPluginMetricsTest, MarksPendingWhenNoMetricsHaveArrivedForThisTopic) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, make_state(), default_config(),
                                                       "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "pending");
  EXPECT_FALSE((*edge)["metrics"].contains("error_reason"));
  EXPECT_FALSE(edge->contains("error_reason"));
  EXPECT_FALSE((*edge)["metrics"].contains("source"));
}

TEST(GraphProviderPluginMetricsTest, FreshMetricIsActiveAgedMetricIsStaleAndBreaksPipeline) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto config = default_config();
  // This test is about the freshness WINDOW boundary, not the stale-grace
  // debounce (which has its own dedicated tests) - pin grace at 0 so age >
  // window alone decides, exactly like before the debounce existed.
  config.stale_grace_sec = 0.0;
  // expected_frequency_hz=30 -> interval ~0.033s; headroom*interval is tiny,
  // so the floor (5s) is the effective freshness window.
  const auto window_ns = static_cast<int64_t>(config.freshness_floor_sec * 1e9);

  auto fresh_state = make_state();
  fresh_state.now_ns = 10 * window_ns;
  fresh_state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/fresh_state.now_ns);

  auto fresh_doc =
      GraphProviderPlugin::build_graph_document("fn", input, fresh_state, config, "2026-03-08T12:00:00.000Z");
  const auto & fresh_graph = fresh_doc["x-medkit-graph"];
  const auto * fresh_edge = find_edge(fresh_graph, "a", "b", "/topic");
  ASSERT_NE(fresh_edge, nullptr);
  EXPECT_EQ((*fresh_edge)["metrics"]["metrics_status"], "active");
  EXPECT_FALSE((*fresh_edge)["metrics"].contains("error_reason"));
  EXPECT_EQ(fresh_graph["pipeline_status"], "healthy");

  auto stale_state = make_state();
  stale_state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/0);
  stale_state.now_ns = window_ns + static_cast<int64_t>(1e9);  // one window + 1s past the last update

  auto stale_doc =
      GraphProviderPlugin::build_graph_document("fn", input, stale_state, config, "2026-03-08T12:00:00.000Z");
  const auto & stale_graph = stale_doc["x-medkit-graph"];
  const auto * stale_edge = find_edge(stale_graph, "a", "b", "/topic");
  ASSERT_NE(stale_edge, nullptr);
  EXPECT_EQ((*stale_edge)["metrics"]["metrics_status"], "error");
  ASSERT_TRUE((*stale_edge)["metrics"].contains("error_reason"));
  EXPECT_EQ((*stale_edge)["metrics"]["error_reason"], "metrics_stale");
  EXPECT_EQ(stale_graph["pipeline_status"], "broken");
}

// Robustness fix: is_metrics_stale must never treat a NEGATIVE computed age
// (last_update_ns reading "in the future" relative to now_ns) as active. In
// production this cannot occur any more now that both stamps are
// steady_clock, but this guards the underlying bug directly and simulates
// exactly the failure mode a backward wall-clock step (e.g. an NTP
// correction on a Jetson booting before sync) used to cause when both
// stamps were system_clock: `(now_ns - last_update_ns)` goes negative, and a
// naive `age <= window` reads a large negative number as "fresh forever".
// Against the pre-fix `<=` comparison (no negative-age guard), this test
// fails: it would read "active"/"healthy" instead of "error"/"broken".
TEST(GraphProviderPluginMetricsTest, NegativeAgeFromBackwardClockStepIsTreatedAsStaleNotFresh) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto config = default_config();
  config.stale_grace_sec = 0.0;  // isolate the negative-age guard from the debounce

  auto state = make_state();
  state.now_ns = static_cast<int64_t>(1e9);  // t = 1s
  // last_update_ns is AHEAD of now_ns - simulates a sample stamped just
  // before a backward clock step, now reading as "in the future".
  state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/static_cast<int64_t>(5e9));

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, state, config, "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "error");
  ASSERT_TRUE((*edge)["metrics"].contains("error_reason"));
  EXPECT_EQ((*edge)["metrics"]["error_reason"], "metrics_stale");
  EXPECT_EQ(graph["pipeline_status"], "broken");
}

// Pins the `stale_grace_sec == 0.0` case exactly: is_metrics_stale must
// reduce to today's un-debounced `age > window` check, transitioning to
// stale the instant age crosses the window. This is what makes grace=0
// backward-compatible.
TEST(GraphProviderPluginMetricsTest, ZeroStaleGraceTransitionsToStaleExactlyAtWindowWithNoDelay) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto config = default_config();
  config.stale_grace_sec = 0.0;
  const auto window_ns = static_cast<int64_t>(config.freshness_floor_sec * 1e9);

  auto at_window_state = make_state();
  at_window_state.now_ns = window_ns;  // exactly at the boundary - still fresh
  at_window_state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/0);
  auto at_window_doc =
      GraphProviderPlugin::build_graph_document("fn", input, at_window_state, config, "2026-03-08T12:00:00.000Z");
  const auto * at_window_edge = find_edge(at_window_doc["x-medkit-graph"], "a", "b", "/topic");
  ASSERT_NE(at_window_edge, nullptr);
  EXPECT_EQ((*at_window_edge)["metrics"]["metrics_status"], "active");

  auto past_window_state = make_state();
  past_window_state.now_ns = window_ns + 1;  // one nanosecond past the boundary
  past_window_state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/0);
  auto past_window_doc =
      GraphProviderPlugin::build_graph_document("fn", input, past_window_state, config, "2026-03-08T12:00:00.000Z");
  const auto & past_window_graph = past_window_doc["x-medkit-graph"];
  const auto * past_window_edge = find_edge(past_window_graph, "a", "b", "/topic");
  ASSERT_NE(past_window_edge, nullptr);
  EXPECT_EQ((*past_window_edge)["metrics"]["metrics_status"], "error");
  EXPECT_EQ((*past_window_edge)["metrics"]["error_reason"], "metrics_stale");
  EXPECT_EQ(past_window_graph["pipeline_status"], "broken");
}

// Grace debounce: an edge whose age has just crossed outside the freshness
// window must stay "active" while the continuous-stale duration is still
// within stale_grace_sec, and only flip to "metrics_stale"/"broken" once
// grace has fully elapsed. is_metrics_stale is a pure function of
// (now_ns, last_update_ns, window_sec, grace_sec) - no separate onset/tick
// state, so these ages are expressed directly via last_update_ns/now_ns.
TEST(GraphProviderPluginMetricsTest, StaleGraceKeepsEdgeActiveUntilGraceElapsesThenBreaksPipeline) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto config = default_config();
  config.stale_grace_sec = 2.0;
  const auto window_ns = static_cast<int64_t>(config.freshness_floor_sec * 1e9);
  const auto grace_ns = static_cast<int64_t>(config.stale_grace_sec * 1e9);

  // last_update_ns = 0 (the window crossing); "now" is window + 1s - past
  // the window, but short of the 2s grace.
  auto within_grace_state = make_state();
  within_grace_state.now_ns = window_ns + static_cast<int64_t>(1e9);
  within_grace_state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/0);

  auto within_grace_doc =
      GraphProviderPlugin::build_graph_document("fn", input, within_grace_state, config, "2026-03-08T12:00:00.000Z");
  const auto & within_grace_graph = within_grace_doc["x-medkit-graph"];
  const auto * within_grace_edge = find_edge(within_grace_graph, "a", "b", "/topic");
  ASSERT_NE(within_grace_edge, nullptr);
  EXPECT_EQ((*within_grace_edge)["metrics"]["metrics_status"], "active");
  EXPECT_FALSE((*within_grace_edge)["metrics"].contains("error_reason"));
  EXPECT_EQ(within_grace_graph["pipeline_status"], "healthy");

  // Same last_update_ns; "now" is past window + grace - the debounce has
  // elapsed.
  auto past_grace_state = make_state();
  past_grace_state.now_ns = window_ns + grace_ns + static_cast<int64_t>(1e9);
  past_grace_state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/0);

  auto past_grace_doc =
      GraphProviderPlugin::build_graph_document("fn", input, past_grace_state, config, "2026-03-08T12:00:00.000Z");
  const auto & past_grace_graph = past_grace_doc["x-medkit-graph"];
  const auto * past_grace_edge = find_edge(past_grace_graph, "a", "b", "/topic");
  ASSERT_NE(past_grace_edge, nullptr);
  EXPECT_EQ((*past_grace_edge)["metrics"]["metrics_status"], "error");
  EXPECT_EQ((*past_grace_edge)["metrics"]["error_reason"], "metrics_stale");
  EXPECT_EQ(past_grace_graph["pipeline_status"], "broken");
}

// Regression guard for a Critical review finding: is_metrics_stale must use
// whichever freshness window the CALLER resolved for a given function, never
// a value computed against a different (e.g. global-default) window. Before
// this fix, the grace debounce's onset (TopicMetrics::stale_since_ns) was
// advanced on the plugin's introspect() cadence using ONLY the global
// default config (GraphProviderPlugin::advance_stale_since, now removed) -
// never the per-function config a function's own freshness overrides
// resolve to. A topic scoped into a function with a materially different
// freshness_floor_sec then got the WRONG stale-grace transition: at an age
// past that function's own window+grace but still within the global
// window, the old onset-tracking never even noticed the topic had gone
// stale (from the global window's perspective it was still fresh), so
// stale_since_ns was never set - and with it unset, the pre-fix
// `is_metrics_stale` unconditionally treated the onset as "now" and could
// never satisfy grace_sec > 0, permanently reading "active" no matter how
// long the real outage lasted. The stateless design has no such divergence:
// is_metrics_stale always evaluates the exact window_sec/grace_sec it is
// called with.
TEST(GraphProviderPluginMetricsTest, PerFunctionFreshnessWindowOverrideDrivesStaleGraceTransitionNotGlobalWindow) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});

  // Global default window is generous (20s floor); "fn" resolves to a much
  // narrower 1s floor. Both share the same 2s grace.
  auto global_config = default_config();
  global_config.freshness_floor_sec = 20.0;
  global_config.stale_grace_sec = 2.0;
  auto function_config = global_config;
  function_config.freshness_floor_sec = 1.0;

  auto state = make_state();
  // Age = 5s: past fn's own window+grace (1.0 + 2.0 = 3.0s), but
  // comfortably inside the GLOBAL window alone (20.0s), let alone its grace.
  state.now_ns = static_cast<int64_t>(5.0 * 1e9);
  state.topic_metrics["/topic"] = make_metrics(29.8, 1.2, 0.0, 30.0, /*last_update_ns=*/0);

  // Sanity: evaluated against the global config alone, this age is still
  // well inside the window itself - stays active regardless of grace.
  auto global_doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, global_config, "2026-03-08T12:00:00.000Z");
  const auto * global_edge = find_edge(global_doc["x-medkit-graph"], "a", "b", "/topic");
  ASSERT_NE(global_edge, nullptr);
  EXPECT_EQ((*global_edge)["metrics"]["metrics_status"], "active");

  // "fn"'s own resolved config (1s floor, same 2s grace) must decide this
  // edge's verdict: window+grace = 3s, age = 5s -> stale/broken, regardless
  // of what the global window alone would say about the same sample.
  auto fn_doc =
      GraphProviderPlugin::build_graph_document("fn", input, state, function_config, "2026-03-08T12:00:00.000Z");
  const auto & fn_graph = fn_doc["x-medkit-graph"];
  const auto * fn_edge = find_edge(fn_graph, "a", "b", "/topic");
  ASSERT_NE(fn_edge, nullptr);
  EXPECT_EQ((*fn_edge)["metrics"]["metrics_status"], "error");
  EXPECT_EQ((*fn_edge)["metrics"]["error_reason"], "metrics_stale");
  EXPECT_EQ(fn_graph["pipeline_status"], "broken");
}

// Config validation: a negative stale_grace_sec is silently invalid (unlike
// 0.0, which is a real, meaningful "no debounce" setting) - it must be
// rejected and fall back to the default, exactly like drop_rate_percent_threshold's
// validate_non_negative pattern.
TEST(GraphProviderPluginRouteTest, RejectsNegativeStaleGraceSecAndKeepsDefault) {
  nlohmann::json config = {{"stale_grace_sec", -1.0}};

  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.configure(config);
  plugin.set_context(ctx);

  const auto resolved = GraphProviderPluginTestAccess::resolve_config(plugin, "fn");
  EXPECT_DOUBLE_EQ(resolved.stale_grace_sec, 2.0);
}

// Config validation: a per-function override of stale_grace_sec must apply
// only to the overridden function, leaving every other function's resolved
// config at the (valid) global default - same nested-config delivery shape
// as the other function_overrides fields (see
// PerFunctionOverrideFromNestedConfigChangesPipelineStatus).
TEST(GraphProviderPluginRouteTest, PerFunctionOverrideOfStaleGraceSecApplies) {
  nlohmann::json config = {{"function_overrides", {{"fn", {{"stale_grace_sec", 4.0}}}}}};

  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}},
                         {"other", PluginEntityInfo{SovdEntityType::FUNCTION, "other", "", ""}}});
  plugin.configure(config);
  plugin.set_context(ctx);

  const auto overridden = GraphProviderPluginTestAccess::resolve_config(plugin, "fn");
  EXPECT_DOUBLE_EQ(overridden.stale_grace_sec, 4.0);

  const auto default_resolved = GraphProviderPluginTestAccess::resolve_config(plugin, "other");
  EXPECT_DOUBLE_EQ(default_resolved.stale_grace_sec, 2.0);
}

TEST(GraphProviderPluginMetricsTest, KeepsBrokenPipelineBottleneckNullWhenMetricsAreStale) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto config = default_config();
  // Pin grace at 0 - this test is about bottleneck-null-when-stale, not the
  // debounce itself.
  config.stale_grace_sec = 0.0;

  auto state = make_state();
  // A stale-but-numerically-good ratio must never surface as the bottleneck:
  // an error edge is excluded from ratio/bottleneck consideration entirely.
  state.topic_metrics["/topic"] = make_metrics(30.0, 1.0, 0.0, 30.0, /*last_update_ns=*/0);
  state.now_ns = static_cast<int64_t>((config.freshness_floor_sec + 1.0) * 1e9);

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, state, config, "2026-03-08T12:00:00.000Z");
  const auto & graph = doc["x-medkit-graph"];

  EXPECT_EQ(graph["pipeline_status"], "broken");
  EXPECT_TRUE(graph["bottleneck_edge"].is_null());
}

TEST(GraphProviderPluginMetricsTest, MarksPipelineDegradedWhenFrequencyDropsBelowThreshold) {
  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  auto state = make_state();
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
  auto state = make_state();
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
  auto state = make_state();
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
  EXPECT_EQ(body["x-medkit-graph"]["schema_version"], kGraphSchemaVersion);
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

  // introspect() now sources the app set for its last_seen_by_app_ side
  // effect from ctx_->get_entity_snapshot(), not from its own `input`
  // parameter - matching the real gateway, which always calls introspect()
  // with an empty IntrospectionInput. Set the snapshot on the context and
  // pass an empty input to introspect(), exactly as production does.
  auto online_input = make_input({make_app("node1", {}, {}, true)}, {make_function("fn", {"node1"})});
  ctx.entity_snapshot_ = online_input;
  plugin.introspect(IntrospectionInput{});

  // Ensure timestamps differ even under CPU contention during parallel testing
  std::this_thread::sleep_for(50ms);

  auto offline_input = make_input({make_app("node1", {}, {}, false)}, {make_function("fn", {"node1"})});
  ctx.entity_snapshot_ = offline_input;
  plugin.introspect(IntrospectionInput{});

  // ctx.entity_snapshot_ already reflects offline_input, which build_current_graph() rebuilds from below.

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

TEST(GraphProviderPluginMetricsTest, BrokenPipelineHasNullBottleneckEdgeEvenWhenDegradedEdgesExist) {
  // edge a->b: /ab, metrics aged past the freshness window -> "error"/"metrics_stale" -> broken
  // edge a->c: /ac, fresh, freq=5 (below 30*0.5) -> active but degraded, has a real ratio
  // pipeline_status must be "broken" overall, and bottleneck_edge must stay null: an
  // error edge is never eligible to be reported as the bottleneck, even though a
  // perfectly valid ratio exists elsewhere in the same graph.
  auto input = make_input(
      {make_app("a", {"/ab", "/ac"}, {}, true), make_app("b", {}, {"/ab"}, true), make_app("c", {}, {"/ac"}, true)},
      {make_function("fn", {"a", "b", "c"})});
  auto config = default_config();
  // Pin grace at 0 - this test is about bottleneck selection, not the
  // debounce itself.
  config.stale_grace_sec = 0.0;
  auto state = make_state();
  state.now_ns = static_cast<int64_t>((config.freshness_floor_sec + 1.0) * 1e9);
  // /ab: stamped at t=0, now far past the freshness window -> stale -> broken.
  state.topic_metrics["/ab"] = make_metrics(30.0, 1.0, 0.0, 30.0, /*last_update_ns=*/0);
  // /ac: stamped at "now" -> fresh -> active, but its ratio (5/30) is below
  // the degraded threshold.
  state.topic_metrics["/ac"] = make_metrics(5.0, 1.0, 0.0, 30.0, /*last_update_ns=*/state.now_ns);

  auto doc = GraphProviderPlugin::build_graph_document("fn", input, state, config, "2026-03-08T12:00:00.000Z");
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

// Defect C: last_seen_by_app_ must not retain an entry for an app that has
// left discovery entirely (e.g. a ROS 2 `_ros2cli_<pid>` name that will never
// be seen again) - only entries for apps still present (online OR offline)
// in the current app set are useful, since only offline apps ever have their
// last_seen read, and only while they are still known to the graph.
TEST(GraphProviderPluginRouteTest, LastSeenByAppPrunesEntriesForAppsNoLongerInDiscovery) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.configure({});
  plugin.set_context(ctx);

  // introspect() sources the app set for this side effect from
  // ctx_->get_entity_snapshot(), not from its own `input` parameter -
  // matching the real gateway, which always calls introspect() with an
  // empty IntrospectionInput. Set the snapshot on the context and pass an
  // empty input to introspect(), exactly as production does.
  auto input_three_apps =
      make_input({make_app("a", {}, {}, true), make_app("b", {}, {}, true), make_app("c", {}, {}, true)},
                 {make_function("fn", {"a", "b", "c"})});
  ctx.entity_snapshot_ = input_three_apps;
  plugin.introspect(IntrospectionInput{});
  EXPECT_EQ(GraphProviderPluginTestAccess::last_seen_by_app_count(plugin), 3u);

  // "b" and "c" leave discovery entirely on the next cycle (not merely go
  // offline) - their last_seen entries must be pruned, not retained forever.
  auto input_one_app = make_input({make_app("a", {}, {}, true)}, {make_function("fn", {"a"})});
  ctx.entity_snapshot_ = input_one_app;
  plugin.introspect(IntrospectionInput{});
  EXPECT_EQ(GraphProviderPluginTestAccess::last_seen_by_app_count(plugin), 1u);
}

// Finding 1 regression test: the real gateway (gateway_node.cpp's refresh
// loop) calls every plugin's introspect() with a default-constructed, EMPTY
// IntrospectionInput on every cycle - the backstop timer and the debounced
// graph-change poller both do this, in every discovery mode. Before the fix,
// build_state_snapshot()'s known_app_ids set was built straight from that
// empty `input`, so the prune erased last_seen_by_app_ in its entirety on
// every single cycle - including entries for apps that are genuinely
// present-but-offline in the real entity cache, destroying the offline-node
// last_seen feature the prune was supposed to preserve. This test drives
// introspect() exactly the way the gateway does (empty input, real state only
// reachable via ctx_->get_entity_snapshot()) and asserts the offline app's
// last_seen survives.
TEST(GraphProviderPluginRouteTest, IntrospectWithEmptyInputStillPreservesOfflineLastSeen) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.configure({});
  plugin.set_context(ctx);

  // Cycle 1: "node1" is online. The gateway would call introspect() with an
  // empty input here too, but the entity cache (ctx.entity_snapshot_) already
  // reflects the real, populated graph.
  auto online_input = make_input({make_app("node1", {}, {}, true)}, {make_function("fn", {"node1"})});
  ctx.entity_snapshot_ = online_input;
  plugin.introspect(IntrospectionInput{});
  EXPECT_EQ(GraphProviderPluginTestAccess::last_seen_by_app_count(plugin), 1u);

  std::this_thread::sleep_for(50ms);

  // Cycle 2: "node1" goes offline but is still present in the entity cache.
  // This is exactly what the gateway's refresh loop does: it calls
  // introspect() with an EMPTY IntrospectionInput, never the real one.
  auto offline_input = make_input({make_app("node1", {}, {}, false)}, {make_function("fn", {"node1"})});
  ctx.entity_snapshot_ = offline_input;
  plugin.introspect(IntrospectionInput{});

  // The offline-but-still-known app's last_seen entry must have survived the
  // prune (pre-fix, it would have been erased because the empty `input`
  // parameter produced an empty known_app_ids set every cycle).
  EXPECT_EQ(GraphProviderPluginTestAccess::last_seen_by_app_count(plugin), 1u);

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

// Defect D pin: introspect() no longer pre-builds a per-function graph cache
// (it had no reader - get_cached_or_built_graph's rebuild-first branch always
// wins whenever ctx_ is set, which is always true in production). Serving a
// function that was NEVER passed to introspect() must still work, proving
// nothing depends on introspect() pre-populating anything for this function.
TEST(GraphProviderPluginRouteTest, ServesGraphForFunctionNeverPassedToIntrospect) {
  GraphProviderPlugin plugin;
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}});
  plugin.configure({});
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  ctx.entity_snapshot_ = input;

  auto result = ctx.registered_samplers_["x-medkit-graph"]("fn", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  EXPECT_EQ(graph["edges"].size(), 1u);
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

// Regression test for defect 1: a /diagnostics message about a topic that is
// NOT part of this edge must not change the edge's status. Previously
// diagnostics_callback set a global diagnostics_seen_ flag on ANY message,
// which poisoned every edge with no matching metrics to
// "error"/"no_data_source" instead of leaving it "pending".
TEST_F(GraphProviderPluginRosTest, DiagnosticsForUnrelatedTopicDoesNotPoisonPendingEdge) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_no_poison_node");
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  // Function "fn" has an edge a->b over "/topic_c"; metrics never arrive for it.
  auto input =
      make_input({make_app("a", {"/topic_c"}, {}), make_app("b", {}, {"/topic_c"})}, {make_function("fn", {"a", "b"})});
  ctx.entity_snapshot_ = input;

  // Publish a diagnostics message about a DIFFERENT topic ("/topic_a").
  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/topic_a";
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "frame_rate_msg";
  kv.value = "30.0";
  status.values.push_back(kv);
  msg.status.push_back(status);
  pub->publish(msg);

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("fn", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic_c");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "pending");
  EXPECT_FALSE((*edge)["metrics"].contains("error_reason"));
  // A pending edge makes no claim about a producer: no diagnostics ever
  // arrived for its own topic, so "source" must be absent, not defaulted to
  // any literal.
  EXPECT_FALSE((*edge)["metrics"].contains("source"));
}

// This is the provenance regression test: metrics.source must be the REAL
// resolved node name that published the /diagnostics message (via publisher
// GID matching against get_publishers_info_by_topic), never a hardcoded
// literal and never DiagnosticStatus.hardware_id (a vendor label). The
// publishing node here is named distinctly from any literal the
// implementation might hardcode, and hardware_id is deliberately set to
// "nvidia" (a real vendor label greenwave uses) to prove that field is never
// read for provenance either.
TEST_F(GraphProviderPluginRosTest, DiagnosticsSourceIsRealPublisherNodeNameNotAHardcodedLiteral) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_real_source_node");
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  auto input = make_input({make_app("a1", {"/topic1"}, {}), make_app("a2", {}, {"/topic1"})},
                          {make_function("f1", {"a1", "a2"})});
  ctx.entity_snapshot_ = input;

  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/topic1";
  status.hardware_id = "nvidia";  // vendor label - must never be used as source
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "frame_rate_msg";
  kv.value = "30.0";
  status.values.push_back(kv);
  msg.status.push_back(status);
  pub->publish(msg);

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a1", "a2", "/topic1");

  ASSERT_NE(edge, nullptr);
  ASSERT_TRUE((*edge)["metrics"].contains("source"));
  EXPECT_EQ((*edge)["metrics"]["source"], std::string(node->get_fully_qualified_name()));
  EXPECT_NE((*edge)["metrics"]["source"], "greenwave_monitor");
  EXPECT_NE((*edge)["metrics"]["source"], "nvidia");
}

// Two distinct /diagnostics publishers, each reporting a different topic in
// its own message, must resolve to two different "source" values on their
// respective edges. Per-message GID resolution is what disambiguates which
// publisher sent which message - a "first publisher on the topic" shortcut
// would collapse both edges onto the same (wrong) source.
TEST_F(GraphProviderPluginRosTest, DiagnosticsSourceResolvesPerMessageNotFirstPublisherOnTopic) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_multi_monitor_sub_node");
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  auto input = make_input({make_app("a1", {"/topic_one", "/topic_two"}, {}), make_app("a2", {}, {"/topic_one"}),
                           make_app("a3", {}, {"/topic_two"})},
                          {make_function("f1", {"a1", "a2", "a3"})});
  ctx.entity_snapshot_ = input;

  auto monitor_one = std::make_shared<rclcpp::Node>("test_diag_monitor_one_node");
  auto monitor_two = std::make_shared<rclcpp::Node>("test_diag_monitor_two_node");
  auto pub_one = monitor_one->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  auto pub_two = monitor_two->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  diagnostic_msgs::msg::DiagnosticArray msg_one;
  diagnostic_msgs::msg::DiagnosticStatus status_one;
  status_one.name = "/topic_one";
  diagnostic_msgs::msg::KeyValue kv_one;
  kv_one.key = "frame_rate_msg";
  kv_one.value = "30.0";
  status_one.values.push_back(kv_one);
  msg_one.status.push_back(status_one);

  diagnostic_msgs::msg::DiagnosticArray msg_two;
  diagnostic_msgs::msg::DiagnosticStatus status_two;
  status_two.name = "/topic_two";
  diagnostic_msgs::msg::KeyValue kv_two;
  kv_two.key = "frame_rate_msg";
  kv_two.value = "15.0";
  status_two.values.push_back(kv_two);
  msg_two.status.push_back(status_two);

  pub_one->publish(msg_one);
  pub_two->publish(msg_two);

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge_one = find_edge(graph, "a1", "a2", "/topic_one");
  const auto * edge_two = find_edge(graph, "a1", "a3", "/topic_two");

  ASSERT_NE(edge_one, nullptr);
  ASSERT_NE(edge_two, nullptr);
  ASSERT_TRUE((*edge_one)["metrics"].contains("source"));
  ASSERT_TRUE((*edge_two)["metrics"].contains("source"));
  EXPECT_EQ((*edge_one)["metrics"]["source"], std::string(monitor_one->get_fully_qualified_name()));
  EXPECT_EQ((*edge_two)["metrics"]["source"], std::string(monitor_two->get_fully_qualified_name()));
  EXPECT_NE((*edge_one)["metrics"]["source"], (*edge_two)["metrics"]["source"]);
}

TEST_F(GraphProviderPluginRosTest, DiagnosticsMessageMakesMatchingEdgeActive) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_node");
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  auto input = make_input({make_app("a1", {"/topic1"}, {}), make_app("a2", {}, {"/topic1"})},
                          {make_function("f1", {"a1", "a2"})});
  ctx.entity_snapshot_ = input;

  // Publish a diagnostics message for the edge's own topic.
  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/topic1";
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

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a1", "a2", "/topic1");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["frequency_hz"], 30.0);
}

// Defect 3: a header-less message type reports frame_rate_msg: 0.000000 (the
// per-message-header rate, always 0 with no header) alongside the real rate
// in frame_rate_node. The parser must resolve to the frame_rate_node value,
// not report the topic as active at 0 Hz.
TEST_F(GraphProviderPluginRosTest, ResolvesFrequencyFromNodeRateWhenMessageRateIsZero) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_headerless_node");
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  auto input = make_input({make_app("a1", {"/headerless"}, {}), make_app("a2", {}, {"/headerless"})},
                          {make_function("f1", {"a1", "a2"})});
  ctx.entity_snapshot_ = input;

  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/headerless";
  diagnostic_msgs::msg::KeyValue kv_msg;
  kv_msg.key = "frame_rate_msg";
  kv_msg.value = "0.000000";
  diagnostic_msgs::msg::KeyValue kv_node;
  kv_node.key = "frame_rate_node";
  kv_node.value = "9.230788";
  status.values.push_back(kv_msg);
  status.values.push_back(kv_node);
  msg.status.push_back(status);
  pub->publish(msg);

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a1", "a2", "/headerless");

  ASSERT_NE(edge, nullptr);
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["frequency_hz"], 9.230788);
}

// Defect 4: a negative frame_rate_msg (a "-1 not measured yet" sentinel) must
// be rejected outright - it must not populate frequency_hz, and therefore
// must never win a bottleneck ratio comparison against a real measurement.
TEST_F(GraphProviderPluginRosTest, RejectsNegativeFrameRateAndExcludesItFromBottleneck) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_negative_rate_node");
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  // Two edges in the same function: "/sentinel" gets the negative sentinel,
  // "/good" gets a real, badly-degraded rate that WOULD lose a bottleneck
  // comparison to a spurious negative ratio.
  auto input = make_input(
      {make_app("a1", {"/sentinel", "/good"}, {}), make_app("a2", {}, {"/sentinel"}), make_app("a3", {}, {"/good"})},
      {make_function("f1", {"a1", "a2", "a3"})});
  ctx.entity_snapshot_ = input;

  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;

  diagnostic_msgs::msg::DiagnosticStatus sentinel_status;
  sentinel_status.name = "/sentinel";
  diagnostic_msgs::msg::KeyValue sentinel_kv;
  sentinel_kv.key = "frame_rate_msg";
  sentinel_kv.value = "-1.0";
  sentinel_status.values.push_back(sentinel_kv);
  msg.status.push_back(sentinel_status);

  diagnostic_msgs::msg::DiagnosticStatus good_status;
  good_status.name = "/good";
  diagnostic_msgs::msg::KeyValue good_kv;
  good_kv.key = "frame_rate_msg";
  good_kv.value = "1.0";  // ratio 1/30, badly degraded but a real, positive ratio
  good_status.values.push_back(good_kv);
  msg.status.push_back(good_status);

  pub->publish(msg);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto graph_result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(graph_result.has_value());
  const auto & graph = (*graph_result)["x-medkit-graph"];

  const auto * sentinel_edge = find_edge(graph, "a1", "a2", "/sentinel");
  ASSERT_NE(sentinel_edge, nullptr);
  EXPECT_TRUE((*sentinel_edge)["metrics"]["frequency_hz"].is_null());

  // The negative sentinel must never be selected as the bottleneck; the only
  // edge with a real ratio ("/good") must be.
  const auto * good_edge = find_edge(graph, "a1", "a3", "/good");
  ASSERT_NE(good_edge, nullptr);
  EXPECT_EQ(graph["bottleneck_edge"], (*good_edge)["edge_id"]);
}

// Defect 5: a diagnostics sample where one key fails to parse (e.g. a
// trailing-space value that std::stod's strict-length check rejects) must
// merge into the existing entry, not wipe previously-good fields.
TEST_F(GraphProviderPluginRosTest, MergesUnparseableSampleWithPreviousGoodMetrics) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_merge_node");
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a1", {"/flaky"}, {}), make_app("a2", {}, {"/flaky"})}, {make_function("f1", {"a1", "a2"})});
  ctx.entity_snapshot_ = input;

  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  // Good sample: frequency 29.9, latency 2.5.
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/flaky";
    diagnostic_msgs::msg::KeyValue freq_kv;
    freq_kv.key = "frame_rate_msg";
    freq_kv.value = "29.9";
    diagnostic_msgs::msg::KeyValue latency_kv;
    latency_kv.key = "current_delay_from_realtime_ms";
    latency_kv.value = "2.5";
    status.values.push_back(freq_kv);
    status.values.push_back(latency_kv);
    msg.status.push_back(status);
    pub->publish(msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node);
  }

  // Bad sample: frame_rate_msg has a trailing space, which std::stod's
  // strict parsed_chars-length check rejects outright.
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "/flaky";
    diagnostic_msgs::msg::KeyValue freq_kv;
    freq_kv.key = "frame_rate_msg";
    freq_kv.value = "29.9 ";  // trailing space -> unparseable
    status.values.push_back(freq_kv);
    msg.status.push_back(status);
    pub->publish(msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node);
  }

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a1", "a2", "/flaky");

  ASSERT_NE(edge, nullptr);
  // The good frequency and latency from the first sample must survive the
  // second, unparseable sample - not be wiped to null/pending.
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["frequency_hz"], 29.9);
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["latency_ms"], 2.5);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
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

// Defect A: the gateway delivers per-function overrides as NESTED objects
// (param_utils.hpp's insert_nested_param splits every dotted parameter on
// '.'), never as a flat "function_overrides.<fn>.<field>" key. Construct the
// config the way the gateway actually would, and assert the override changes
// an observable graph field (pipeline_status) - not just that parsing
// produced a map entry.
TEST_F(GraphProviderPluginRosTest, PerFunctionOverrideFromNestedConfigChangesPipelineStatus) {
  auto node = std::make_shared<rclcpp::Node>("test_nested_override_node");
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}}, node.get());

  nlohmann::json config = {{"function_overrides", {{"fn", {{"degraded_frequency_ratio", 0.95}}}}}};

  GraphProviderPlugin plugin;
  plugin.configure(config);
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  ctx.entity_snapshot_ = input;

  // Ratio 25/30 ~= 0.833: healthy under the default degraded_frequency_ratio
  // (0.5), degraded once the 0.95 override for "fn" actually applies.
  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/topic";
  diagnostic_msgs::msg::KeyValue freq_kv;
  freq_kv.key = "frame_rate_msg";
  freq_kv.value = "25.0";
  diagnostic_msgs::msg::KeyValue expected_kv;
  expected_kv.key = "expected_frequency";
  expected_kv.value = "30.0";
  status.values.push_back(freq_kv);
  status.values.push_back(expected_kv);
  msg.status.push_back(status);
  pub->publish(msg);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("fn", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_EQ(graph["pipeline_status"], "degraded");
  EXPECT_EQ(graph["bottleneck_edge"], (*edge)["edge_id"]);
}

// Defect B: expected_frequency_hz_default: 0 must be rejected (it makes
// resolve_expected_frequency return 0, the `expected_frequency > 0.0` guard
// in build_edge_json never passes, no edge is ever degraded, and
// bottleneck_edge is always null - silently). The valid default (30.0) must
// apply instead, so degradation detection keeps working.
TEST_F(GraphProviderPluginRosTest, RejectsNonPositiveExpectedFrequencyDefaultAndKeepsDegradationDetectionWorking) {
  auto node = std::make_shared<rclcpp::Node>("test_reject_zero_expected_frequency_node");
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}}, node.get());

  nlohmann::json config = {{"expected_frequency_hz_default", 0.0}};

  GraphProviderPlugin plugin;
  plugin.configure(config);
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  ctx.entity_snapshot_ = input;

  // No "expected_frequency" key in this sample, so resolve_expected_frequency
  // must fall through to config.expected_frequency_hz_default.
  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/topic";
  diagnostic_msgs::msg::KeyValue freq_kv;
  freq_kv.key = "frame_rate_msg";
  freq_kv.value = "5.0";
  status.values.push_back(freq_kv);
  msg.status.push_back(status);
  pub->publish(msg);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("fn", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ(graph["pipeline_status"], "degraded");
  EXPECT_FALSE(graph["bottleneck_edge"].is_null());
}

// Defect B: a negative degraded_frequency_ratio has the same silent effect as
// a zero expected_frequency_hz_default (the ratio comparison never trips), so
// it must be rejected too, falling back to the valid default (0.5).
TEST_F(GraphProviderPluginRosTest, RejectsNegativeDegradedFrequencyRatioAndKeepsDefault) {
  auto node = std::make_shared<rclcpp::Node>("test_reject_negative_ratio_node");
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}}, node.get());

  nlohmann::json config = {{"degraded_frequency_ratio", -1.0}};

  GraphProviderPlugin plugin;
  plugin.configure(config);
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  ctx.entity_snapshot_ = input;

  // Ratio 9/30 = 0.3: degraded under the valid default (0.5), but NOT
  // degraded if -1.0 were accepted literally (0.3 < -1.0 is false).
  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/topic";
  diagnostic_msgs::msg::KeyValue freq_kv;
  freq_kv.key = "frame_rate_msg";
  freq_kv.value = "9.0";
  diagnostic_msgs::msg::KeyValue expected_kv;
  expected_kv.key = "expected_frequency";
  expected_kv.value = "30.0";
  status.values.push_back(freq_kv);
  status.values.push_back(expected_kv);
  msg.status.push_back(status);
  pub->publish(msg);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("fn", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];

  EXPECT_EQ(graph["pipeline_status"], "degraded");
}

// Defect B regression: drop_rate_percent_threshold is the one field where
// zero is a VALID, meaningful value (it uses validate_non_negative, unlike
// every other field here which uses validate_positive and rejects zero).
// Without a test, an accidental switch to validate_positive would silently
// start rejecting a configured 0 and falling back to the 5.0 default -
// nothing would catch it. Configure the threshold to 0 and publish a small
// but positive drop rate (1.0%) that is healthy under the 5.0 default but
// must trip degraded under a live 0 threshold (1.0 > 0.0).
TEST_F(GraphProviderPluginRosTest, AcceptsZeroDropRateThresholdAndTripsOnAnyPositiveDropRate) {
  auto node = std::make_shared<rclcpp::Node>("test_zero_drop_rate_threshold_node");
  FakePluginContext ctx({{"fn", PluginEntityInfo{SovdEntityType::FUNCTION, "fn", "", ""}}}, node.get());

  nlohmann::json config = {{"drop_rate_percent_threshold", 0.0}};

  GraphProviderPlugin plugin;
  plugin.configure(config);
  plugin.set_context(ctx);

  auto input =
      make_input({make_app("a", {"/topic"}, {}), make_app("b", {}, {"/topic"})}, {make_function("fn", {"a", "b"})});
  ctx.entity_snapshot_ = input;

  // Frequency ratio 30/30 = 1.0: healthy on its own, so only the drop rate
  // can trip degraded here. drop_rate_percent 1.0 is healthy under the
  // default threshold (5.0) but degraded under a live 0 threshold.
  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/topic";
  diagnostic_msgs::msg::KeyValue freq_kv;
  freq_kv.key = "frame_rate_msg";
  freq_kv.value = "30.0";
  diagnostic_msgs::msg::KeyValue expected_kv;
  expected_kv.key = "expected_frequency";
  expected_kv.value = "30.0";
  diagnostic_msgs::msg::KeyValue drop_kv;
  drop_kv.key = "drop_rate_percent";
  drop_kv.value = "1.0";
  status.values.push_back(freq_kv);
  status.values.push_back(expected_kv);
  status.values.push_back(drop_kv);
  msg.status.push_back(status);
  pub->publish(msg);
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("fn", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a", "b", "/topic");

  ASSERT_NE(edge, nullptr);
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["drop_rate_percent"], 1.0);
  EXPECT_EQ(graph["pipeline_status"], "degraded");
}

// The same segment-anchored filter runs over DiagnosticStatus.name in
// diagnostics_callback. A monitored topic whose name merely contains
// "nitros" as part of a longer segment (e.g. "/nitros_bridge/data" under
// isaac_ros_nitros_bridge) must have its metrics ingested and reach its
// edge, not be silently discarded as if it were a NITROS negotiation topic.
TEST_F(GraphProviderPluginRosTest, DiagnosticsForTopicWithNitrosSubstringInSegmentIsIngested) {
  auto node = std::make_shared<rclcpp::Node>("test_diag_nitros_substring_node");
  FakePluginContext ctx({{"f1", PluginEntityInfo{SovdEntityType::FUNCTION, "f1", "", ""}}}, node.get());

  GraphProviderPlugin plugin;
  plugin.configure({});
  plugin.set_context(ctx);

  auto input = make_input({make_app("a1", {"/nitros_bridge/data"}, {}), make_app("a2", {}, {"/nitros_bridge/data"})},
                          {make_function("f1", {"a1", "a2"})});
  ctx.entity_snapshot_ = input;

  auto pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "/nitros_bridge/data";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "frame_rate_msg";
  kv.value = "30.0";
  status.values.push_back(kv);
  msg.status.push_back(status);
  pub->publish(msg);

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(50ms);
  rclcpp::spin_some(node);

  plugin.introspect(input);
  auto result = ctx.registered_samplers_["x-medkit-graph"]("f1", "");
  ASSERT_TRUE(result.has_value());
  const auto & graph = (*result)["x-medkit-graph"];
  const auto * edge = find_edge(graph, "a1", "a2", "/nitros_bridge/data");

  ASSERT_NE(edge, nullptr);
  EXPECT_EQ((*edge)["metrics"]["metrics_status"], "active");
  EXPECT_DOUBLE_EQ((*edge)["metrics"]["frequency_hz"], 30.0);
}
