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

#include "ros2_medkit_graph_provider/graph_provider_plugin.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <unordered_set>
#include <utility>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

namespace ros2_medkit_gateway {

namespace {

std::string format_timestamp_ns(int64_t ns) {
  auto seconds = ns / 1'000'000'000;
  auto nanos = ns % 1'000'000'000;
  std::time_t time = static_cast<std::time_t>(seconds);
  std::tm tm_buf{};
  std::tm * tm = gmtime_r(&time, &tm_buf);
  if (!tm) {
    return "1970-01-01T00:00:00.000Z";
  }
  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", tm);
  char result[80];
  std::snprintf(result, sizeof(result), "%s.%03dZ", buf, static_cast<int>(nanos / 1'000'000));
  return result;
}

constexpr size_t kMaxCachedTopicMetrics = 512;

// True if any '/'-delimited path segment of `topic_name` is exactly `segment`.
// Anchors matching to whole segments instead of a substring search, so a
// real topic like "/nitros_bridge/data" (under the isaac_ros_nitros_bridge
// package) does not falsely match "nitros" the way a plain substring find
// would.
bool has_path_segment(const std::string & topic_name, const std::string & segment) {
  size_t start = 0;
  while (start <= topic_name.size()) {
    const size_t end = topic_name.find('/', start);
    const std::string current =
        (end == std::string::npos) ? topic_name.substr(start) : topic_name.substr(start, end - start);
    if (current == segment) {
      return true;
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return false;
}

// True if the final '/'-delimited path segment of `topic_name` is exactly
// `segment`.
bool has_trailing_path_segment(const std::string & topic_name, const std::string & segment) {
  const auto last_slash = topic_name.find_last_of('/');
  const std::string trailing = (last_slash == std::string::npos) ? topic_name : topic_name.substr(last_slash + 1);
  return trailing == segment;
}

bool is_filtered_topic_name(const std::string & topic_name) {
  if (topic_name == "/parameter_events" || topic_name == "/rosout" || topic_name == "/diagnostics") {
    return true;
  }
  // REP-2009 NITROS negotiation side topics: "<base>/nitros" (negotiated-
  // topics-info) and "<base>/nitros/<format>" (negotiated data) both have a
  // path segment exactly "nitros"; "<base>/nitros/_supported_types" (the
  // supported-types topic) also has a trailing segment exactly
  // "_supported_types". Matching whole segments (not substrings) keeps real
  // topics that merely contain "nitros" or "_supported_types" inside a
  // longer segment name out of the filter.
  if (has_path_segment(topic_name, "nitros")) {
    return true;
  }
  if (has_trailing_path_segment(topic_name, "_supported_types")) {
    return true;
  }
  return false;
}

// Flatten a (possibly nested) JSON object back to dotted keys. The gateway
// splits every dotted plugin-config parameter on '.' and builds a nested
// object (param_utils.hpp's insert_nested_param), so
// plugins.graph_provider.function_overrides.<function_id>.<field> arrives as
// {"function_overrides": {"<function_id>": {"<field>": value}}}, never as a
// flat "function_overrides.<function_id>.<field>" key. Walk it back to the
// full dotted key so the existing "split on the last dot" parsing keeps
// working unchanged, the same way the commercial graph_watchdog package's
// ParamDriftConfig::flatten_expect() un-nests a dotted `expect` config key.
void flatten_json_object(const nlohmann::json & obj, const std::string & prefix,
                         std::map<std::string, nlohmann::json> & out) {
  for (const auto & item : obj.items()) {
    const std::string key = prefix.empty() ? item.key() : prefix + "." + item.key();
    if (item.value().is_object()) {
      flatten_json_object(item.value(), key, out);
    } else {
      out.emplace(key, item.value());
    }
  }
}

struct EdgeBuildResult {
  nlohmann::json json;
  std::optional<double> ratio;
  bool is_error{false};
  bool is_degraded{false};
};

const Function * find_function(const std::vector<Function> & functions, const std::string & function_id) {
  auto it = std::find_if(functions.begin(), functions.end(), [&](const Function & function) {
    return function.id == function_id;
  });
  if (it == functions.end()) {
    return nullptr;
  }
  return &(*it);
}

std::vector<const App *> resolve_scoped_apps(const std::string & function_id, const IntrospectionInput & input) {
  std::vector<const App *> scoped_apps;
  const auto * function = find_function(input.functions, function_id);
  if (!function) {
    return scoped_apps;
  }

  std::unordered_map<std::string, const App *> apps_by_id;
  apps_by_id.reserve(input.apps.size());
  for (const auto & app : input.apps) {
    apps_by_id.emplace(app.id, &app);
  }

  std::unordered_set<std::string> seen_app_ids;
  for (const auto & host : function->hosts) {
    auto app_it = apps_by_id.find(host);
    if (app_it != apps_by_id.end()) {
      if (seen_app_ids.insert(app_it->second->id).second) {
        scoped_apps.push_back(app_it->second);
      }
      continue;
    }

    for (const auto & app : input.apps) {
      if (app.component_id == host && seen_app_ids.insert(app.id).second) {
        scoped_apps.push_back(&app);
      }
    }
  }

  std::sort(scoped_apps.begin(), scoped_apps.end(), [](const App * lhs, const App * rhs) {
    return lhs->id < rhs->id;
  });
  return scoped_apps;
}

std::vector<std::string> filtered_topics(const std::vector<std::string> & topics) {
  std::set<std::string> unique_topics;
  for (const auto & topic : topics) {
    if (!is_filtered_topic_name(topic)) {
      unique_topics.insert(topic);
    }
  }
  return {unique_topics.begin(), unique_topics.end()};
}

std::unordered_set<std::string> filtered_topic_set(const std::vector<std::string> & topics) {
  auto sorted = filtered_topics(topics);
  return {sorted.begin(), sorted.end()};
}

std::set<std::string> collect_unique_topics(const std::vector<const App *> & apps) {
  std::set<std::string> topics;
  for (const auto * app : apps) {
    for (const auto & topic : filtered_topics(app->topics.publishes)) {
      topics.insert(topic);
    }
    for (const auto & topic : filtered_topics(app->topics.subscribes)) {
      topics.insert(topic);
    }
  }
  return topics;
}

double resolve_expected_frequency(const GraphProviderPlugin::TopicMetrics * metrics,
                                  const GraphProviderPlugin::GraphBuildConfig & config) {
  if (metrics && metrics->expected_frequency_hz.has_value() && *metrics->expected_frequency_hz > 0.0) {
    return *metrics->expected_frequency_hz;
  }
  return config.expected_frequency_hz_default;
}

// Freshness window derived from the resolved expected frequency: a topic
// expected at f Hz should not be reported stale before roughly
// headroom_factor message intervals have passed, floored at a few seconds so
// slow producers (e.g. the 1 Hz greenwave_monitor reference) don't flap
// between samples.
double resolve_freshness_window_sec(const GraphProviderPlugin::TopicMetrics * metrics,
                                    const GraphProviderPlugin::GraphBuildConfig & config) {
  const double expected_frequency = resolve_expected_frequency(metrics, config);
  if (expected_frequency <= 0.0) {
    return config.freshness_floor_sec;
  }
  const double expected_interval_sec = 1.0 / expected_frequency;
  return std::max(config.freshness_floor_sec, expected_interval_sec * config.freshness_headroom_factor);
}

bool is_metrics_fresh(const GraphProviderPlugin::TopicMetrics & metrics, int64_t now_ns, double window_sec) {
  const auto window_ns = static_cast<int64_t>(window_sec * 1e9);
  return (now_ns - metrics.last_update_ns) <= window_ns;
}

EdgeBuildResult build_edge_json(const std::string & edge_id, const App & source, const App & target,
                                const std::string & topic_id, const GraphProviderPlugin::TopicMetrics * metrics,
                                const GraphProviderPlugin::GraphBuildState & state,
                                const GraphProviderPlugin::GraphBuildConfig & config) {
  EdgeBuildResult result;
  auto metrics_json = nlohmann::json{
      {"source", "greenwave_monitor"}, {"frequency_hz", nullptr},     {"latency_ms", nullptr},
      {"drop_rate_percent", 0.0},      {"metrics_status", "pending"},
  };

  if (metrics) {
    if (metrics->frequency_hz.has_value()) {
      metrics_json["frequency_hz"] = *metrics->frequency_hz;
    }
    if (metrics->latency_ms.has_value()) {
      metrics_json["latency_ms"] = *metrics->latency_ms;
    }
    metrics_json["drop_rate_percent"] = metrics->drop_rate_percent.value_or(0.0);
  }

  result.json = {{"edge_id", edge_id},   {"source", source.id},         {"target", target.id},
                 {"topic_id", topic_id}, {"transport_type", "unknown"}, {"metrics", metrics_json}};

  // Note: an offline app (source or target) always has an empty topic list
  // (see App::topics population sites), so it can never contribute an edge
  // here in the first place - there is no "node offline" case to special-case.

  if (!metrics) {
    result.json["metrics"]["metrics_status"] = "pending";
    return result;
  }

  const double freshness_window_sec = resolve_freshness_window_sec(metrics, config);
  if (!is_metrics_fresh(*metrics, state.now_ns, freshness_window_sec)) {
    result.json["metrics"]["metrics_status"] = "error";
    result.json["metrics"]["error_reason"] = "metrics_stale";
    result.is_error = true;
    return result;
  }

  // Observed and current. Metrics that carry latency/drop-rate but no
  // frequency (e.g. a rejected negative frame_rate_msg sample) are still
  // "active" - freshness, not field completeness, drives status.
  result.json["metrics"]["metrics_status"] = "active";
  if (metrics->frequency_hz.has_value()) {
    const double expected_frequency = resolve_expected_frequency(metrics, config);
    const double frequency = *metrics->frequency_hz;
    if (expected_frequency > 0.0) {
      result.ratio = frequency / expected_frequency;
      if (*result.ratio < config.degraded_frequency_ratio) {
        result.is_degraded = true;
      }
    }
  }
  if (metrics->drop_rate_percent.value_or(0.0) > config.drop_rate_percent_threshold) {
    result.is_degraded = true;
  }
  return result;
}

nlohmann::json build_graph_document_for_apps(const std::string & function_id,
                                             const std::vector<const App *> & scoped_apps,
                                             const GraphProviderPlugin::GraphBuildState & state,
                                             const GraphProviderPlugin::GraphBuildConfig & config,
                                             const std::string & timestamp) {
  nlohmann::json graph = {{"schema_version", kGraphSchemaVersion},
                          {"graph_id", function_id + "-graph"},
                          {"timestamp", timestamp},
                          {"scope", {{"type", "function"}, {"entity_id", function_id}}},
                          {"pipeline_status", "healthy"},
                          {"bottleneck_edge", nullptr},
                          {"topics", nlohmann::json::array()},
                          {"nodes", nlohmann::json::array()},
                          {"edges", nlohmann::json::array()}};

  const auto topic_names = collect_unique_topics(scoped_apps);

  std::unordered_map<std::string, std::string> topic_ids;
  size_t topic_index = 0;
  for (const auto & topic_name : topic_names) {
    const auto topic_id = "topic-" + std::to_string(++topic_index);
    topic_ids.emplace(topic_name, topic_id);
    graph["topics"].push_back({{"topic_id", topic_id}, {"name", topic_name}});
  }

  for (const auto * app : scoped_apps) {
    nlohmann::json node = {{"entity_id", app->id}, {"node_status", app->is_online ? "reachable" : "unreachable"}};
    if (!app->is_online) {
      const auto last_seen_it = state.last_seen_by_app.find(app->id);
      if (last_seen_it != state.last_seen_by_app.end()) {
        node["last_seen"] = last_seen_it->second;
      }
    }
    graph["nodes"].push_back(std::move(node));
  }

  bool has_errors = false;
  bool has_degraded = false;
  std::optional<std::pair<std::string, double>> bottleneck;
  std::string pipeline_status = "healthy";
  size_t edge_index = 0;

  std::unordered_map<std::string, std::unordered_set<std::string>> subscribes_by_app;
  subscribes_by_app.reserve(scoped_apps.size());
  for (const auto * app : scoped_apps) {
    subscribes_by_app.emplace(app->id, filtered_topic_set(app->topics.subscribes));
  }

  for (const auto * publisher : scoped_apps) {
    for (const auto & topic_name : filtered_topics(publisher->topics.publishes)) {
      auto topic_it = topic_ids.find(topic_name);
      if (topic_it == topic_ids.end()) {
        continue;
      }

      for (const auto * subscriber : scoped_apps) {
        const auto sub_it = subscribes_by_app.find(subscriber->id);
        if (sub_it == subscribes_by_app.end() || sub_it->second.count(topic_name) == 0) {
          continue;
        }

        const auto edge_id = "edge-" + std::to_string(++edge_index);
        const auto metrics_it = state.topic_metrics.find(topic_name);
        const GraphProviderPlugin::TopicMetrics * metrics =
            metrics_it == state.topic_metrics.end() ? nullptr : &metrics_it->second;
        auto edge = build_edge_json(edge_id, *publisher, *subscriber, topic_it->second, metrics, state, config);

        has_errors = has_errors || edge.is_error;
        has_degraded = has_degraded || edge.is_degraded;
        if (edge.ratio.has_value()) {
          if (!bottleneck.has_value() || edge.ratio.value() < bottleneck->second) {
            bottleneck = std::make_pair(edge_id, edge.ratio.value());
          }
        }
        graph["edges"].push_back(std::move(edge.json));
      }
    }
  }

  if (has_errors) {
    pipeline_status = "broken";
  } else if (has_degraded) {
    pipeline_status = "degraded";
  }
  graph["pipeline_status"] = pipeline_status;

  if (pipeline_status == "degraded" && bottleneck.has_value()) {
    graph["bottleneck_edge"] = bottleneck->first;
  }

  return nlohmann::json{{"x-medkit-graph", std::move(graph)}};
}

}  // namespace

GraphProviderPlugin::~GraphProviderPlugin() noexcept {
  // On Lyrical (originally observed on Rolling), destroying rclcpp resources
  // (Subscription, Node) after rclcpp::shutdown() has invalidated the context
  // can throw graph_listener::NodeNotFoundError. An exception escaping a
  // destructor calls std::terminate(), so swallow it here.
  try {
    shutdown();
  } catch (...) {
  }
}

void GraphProviderPlugin::shutdown() {
  if (shutdown_requested_.exchange(true)) {
    return;
  }
  // ~rclcpp::Subscription can throw on Lyrical (and Rolling) when the rclcpp
  // context was torn down before us; swallow so plugin_manager shutdown and
  // the plugin destructor calling back into us do not abort the process.
  try {
    diagnostics_sub_.reset();
  } catch (...) {
  }
}

std::string GraphProviderPlugin::name() const {
  return "graph-provider";
}

void GraphProviderPlugin::configure(const nlohmann::json & config) {
  plugin_config_ = config;
}

void GraphProviderPlugin::set_context(PluginContext & context) {
  ctx_ = as_ros_plugin_context(context);
  ctx_->register_capability(SovdEntityType::FUNCTION, "x-medkit-graph");

  load_parameters();
  subscribe_to_diagnostics();

  ctx_->register_sampler("x-medkit-graph",
                         [this](const std::string & entity_id,
                                const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
                           auto payload = build_current_graph(entity_id);
                           if (!payload.has_value()) {
                             return tl::make_unexpected(std::string("Graph snapshot not available: ") + entity_id);
                           }
                           return *payload;
                         });
  log_info("Registered x-medkit-graph cyclic subscription sampler");
}

std::vector<GatewayPlugin::PluginRoute> GraphProviderPlugin::get_routes() {
  std::vector<GatewayPlugin::PluginRoute> routes;
  routes.push_back(
      {"GET", R"(functions/([^/]+)/x-medkit-graph)", [this](const PluginRequest & req, PluginResponse & res) {
         if (!ctx_) {
           res.send_error(503, ERR_SERVICE_UNAVAILABLE, "Graph provider context not initialized");
           return;
         }

         const auto function_id = req.path_param(1);
         auto entity = ctx_->validate_entity_for_route(req, res, function_id);
         if (!entity) {
           return;
         }

         // Check lock access for vendor extension collection
         auto client_id = req.header("X-Client-Id");
         auto lock_result = ctx_->check_lock(function_id, client_id, "x-medkit-graph");
         if (!lock_result.allowed) {
           nlohmann::json params = {{"entity_id", function_id}, {"collection", "x-medkit-graph"}};
           if (!lock_result.denied_by_lock_id.empty()) {
             params["lock_id"] = lock_result.denied_by_lock_id;
           }
           if (lock_result.denied_code == "lock-required") {
             res.send_error(409, ERR_INVALID_REQUEST, lock_result.denied_reason, params);
           } else {
             res.send_error(409, ERR_LOCK_BROKEN, lock_result.denied_reason, params);
           }
           return;
         }

         auto payload = build_current_graph(function_id);
         if (!payload.has_value()) {
           res.send_error(503, ERR_SERVICE_UNAVAILABLE, "Graph snapshot not available", {{"function_id", function_id}});
           return;
         }

         res.send_json(*payload);
       }});
  return routes;
}

IntrospectionResult GraphProviderPlugin::introspect(const IntrospectionInput & input) {
  // Building a per-function graph document here used to feed graph_cache_,
  // but nothing ever read it: get_cached_or_built_graph's (now
  // build_current_graph's) rebuild-from-live-snapshot branch runs whenever
  // ctx_ is set, which is unconditionally true in production, so the cache
  // read was dead code and this was O(functions x apps x topics) of pure
  // waste on the discovery thread every cycle. Only run build_state_snapshot
  // for its last_seen_by_app_ side effect (marks online apps seen now, prunes
  // apps no longer in the graph at all) - the graph itself is always built
  // fresh, on demand, in build_current_graph.
  //
  // The gateway's refresh loop (backstop timer + graph-change poller) always
  // calls every plugin's introspect() with a default-constructed, EMPTY
  // IntrospectionInput (gateway_node.cpp: `IntrospectionInput input;` right
  // before `provider->introspect(input)`), in every discovery mode. Trusting
  // that empty `input` for the prune below would erase last_seen_by_app_ in
  // its entirety on every cycle, including apps that are genuinely
  // present-but-offline, destroying the very feature the prune exists to
  // preserve. Pull the real, current app set from the live entity cache
  // instead - the same source build_graph_from_entity_cache already uses on
  // the on-request path - so the prune sees the true app set regardless of
  // what the caller happened to pass in. This only fetches the flat entity
  // lists (areas/components/apps/functions) from the cache under a shared
  // lock, not a per-function graph document build, so it does not reintroduce
  // the waste removed above.
  const auto timestamp = current_timestamp();
  const auto snapshot_input = ctx_ ? ctx_->get_entity_snapshot() : input;
  build_state_snapshot(snapshot_input, timestamp);
  return {};
}

nlohmann::json GraphProviderPlugin::build_graph_document(const std::string & function_id,
                                                         const IntrospectionInput & input,
                                                         const GraphBuildState & state, const GraphBuildConfig & config,
                                                         const std::string & timestamp) {
  const auto scoped_apps = resolve_scoped_apps(function_id, input);
  return build_graph_document_for_apps(function_id, scoped_apps, state, config, timestamp);
}

void GraphProviderPlugin::subscribe_to_diagnostics() {
  if (!ctx_ || !ctx_->node() || diagnostics_sub_) {
    return;
  }

  diagnostics_sub_ = ctx_->node()->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10), [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg) {
        diagnostics_callback(msg);
      });

  log_info("Subscribed to /diagnostics for x-medkit-graph metrics");
}

void GraphProviderPlugin::diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg) {
  if (shutdown_requested_.load()) {
    return;
  }
  std::unordered_map<std::string, TopicMetrics> updates;
  for (const auto & status : msg->status) {
    if (is_filtered_topic_name(status.name)) {
      continue;
    }
    auto metrics = parse_topic_metrics(status);
    if (metrics.has_value()) {
      updates[status.name] = *metrics;
    }
  }

  const auto now_ns = current_time_ns();

  std::lock_guard<std::mutex> lock(metrics_mutex_);
  for (auto & [topic_name, incoming] : updates) {
    auto it = topic_metrics_.find(topic_name);
    if (it == topic_metrics_.end()) {
      topic_metrics_order_.push_back(topic_name);
      incoming.last_update_ns = now_ns;
      topic_metrics_.emplace(topic_name, incoming);
      continue;
    }

    // Merge field-by-field into the existing entry instead of replacing it
    // wholesale: a sample where only one key failed to parse must not wipe
    // the other, previously-good fields (e.g. a trailing-space frame_rate_msg
    // must not erase a good latency_ms from an earlier sample).
    TopicMetrics & existing = it->second;
    if (incoming.frequency_hz.has_value()) {
      existing.frequency_hz = incoming.frequency_hz;
    }
    if (incoming.latency_ms.has_value()) {
      existing.latency_ms = incoming.latency_ms;
    }
    if (incoming.drop_rate_percent.has_value()) {
      existing.drop_rate_percent = incoming.drop_rate_percent;
    }
    if (incoming.expected_frequency_hz.has_value()) {
      existing.expected_frequency_hz = incoming.expected_frequency_hz;
    }
    // The message itself arrived now regardless of which fields parsed, so
    // the freshness stamp always advances.
    existing.last_update_ns = now_ns;
  }
  while (topic_metrics_order_.size() > kMaxCachedTopicMetrics) {
    const auto evicted_topic = topic_metrics_order_.front();
    topic_metrics_order_.pop_front();
    topic_metrics_.erase(evicted_topic);
  }
}

std::optional<GraphProviderPlugin::TopicMetrics>
GraphProviderPlugin::parse_topic_metrics(const diagnostic_msgs::msg::DiagnosticStatus & status) {
  TopicMetrics metrics;
  bool has_relevant_key = false;
  std::optional<double> frame_rate_msg;
  std::optional<double> frame_rate_node;

  for (const auto & kv : status.values) {
    if (kv.key == "frame_rate_msg") {
      frame_rate_msg = parse_double(kv.value);
      has_relevant_key = true;
    } else if (kv.key == "frame_rate_node") {
      frame_rate_node = parse_double(kv.value);
      has_relevant_key = true;
    } else if (kv.key == "current_delay_from_realtime_ms") {
      auto value = parse_double(kv.value);
      if (value.has_value() && std::isfinite(*value)) {
        metrics.latency_ms = *value;
      }
      has_relevant_key = true;
    } else if (kv.key == "expected_frequency") {
      auto value = parse_double(kv.value);
      if (value.has_value() && std::isfinite(*value) && *value > 0.0) {
        metrics.expected_frequency_hz = *value;
      }
      has_relevant_key = true;
    } else if (kv.key == "drop_rate_percent" || kv.key == "drop_rate") {
      auto value = parse_double(kv.value);
      if (value.has_value() && std::isfinite(*value) && *value >= 0.0) {
        metrics.drop_rate_percent = *value;
      }
      has_relevant_key = true;
    } else if (kv.key == "total_dropped_frames") {
      has_relevant_key = true;
    }
  }

  // Header-less message types report frame_rate_msg: 0 with the real rate in
  // frame_rate_node (the reference producer, greenwave_monitor, does this for
  // any message type with no header field). Fall back to frame_rate_node only
  // when frame_rate_msg is present but exactly zero. Negative values (e.g. a
  // "-1 not measured yet" sentinel) are rejected outright - they must never
  // win a bottleneck ratio comparison.
  if (frame_rate_msg.has_value() && std::isfinite(*frame_rate_msg) && *frame_rate_msg > 0.0) {
    metrics.frequency_hz = *frame_rate_msg;
  } else if (frame_rate_msg.has_value() && std::isfinite(*frame_rate_msg) && *frame_rate_msg >= 0.0 &&
             frame_rate_node.has_value() && std::isfinite(*frame_rate_node) && *frame_rate_node > 0.0) {
    // Reached only when frame_rate_msg is not > 0.0, so combined with the
    // >= 0.0 check above this branch is exactly the "present but zero" case
    // (avoids an exact floating-point == comparison).
    metrics.frequency_hz = *frame_rate_node;
  }

  if (!has_relevant_key) {
    return std::nullopt;
  }
  return metrics;
}

std::optional<double> GraphProviderPlugin::parse_double(const std::string & value) {
  try {
    size_t parsed_chars = 0;
    auto parsed = std::stod(value, &parsed_chars);
    if (parsed_chars != value.size()) {
      return std::nullopt;
    }
    return parsed;
  } catch (...) {
    return std::nullopt;
  }
}

std::string GraphProviderPlugin::current_timestamp() {
  return format_timestamp_ns(current_time_ns());
}

int64_t GraphProviderPlugin::current_time_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

GraphProviderPlugin::GraphBuildConfig GraphProviderPlugin::resolve_config(const std::string & function_id) const {
  std::lock_guard<std::mutex> lock(config_mutex_);
  auto resolved = config_.defaults;
  auto it = config_.by_function.find(function_id);
  if (it != config_.by_function.end()) {
    resolved = it->second;
  }
  return resolved;
}

std::optional<nlohmann::json> GraphProviderPlugin::build_current_graph(const std::string & function_id) {
  // In the real gateway, the merged entity cache (read via
  // ctx_->get_entity_snapshot() inside build_graph_from_entity_cache) is the
  // source of truth, so this always rebuilds from it rather than serving a
  // possibly-stale pre-built document. There used to be a graph_cache_
  // fallback for when ctx_ was unset, but ctx_ is always set by the time
  // either caller (the HTTP route and the cyclic-subscription sampler) can
  // reach this method - both are only reachable after set_context() has run -
  // so that fallback was unreachable dead code and has been removed.
  if (!ctx_) {
    return std::nullopt;
  }
  return build_graph_from_entity_cache(function_id);
}

std::optional<nlohmann::json> GraphProviderPlugin::build_graph_from_entity_cache(const std::string & function_id) {
  if (!ctx_) {
    return std::nullopt;
  }

  auto input = ctx_->get_entity_snapshot();
  const auto timestamp = current_timestamp();
  auto state = build_state_snapshot(input, timestamp);
  const auto scoped_apps = resolve_scoped_apps(function_id, input);
  return build_graph_document_for_apps(function_id, scoped_apps, state, resolve_config(function_id), timestamp);
}

GraphProviderPlugin::GraphBuildState GraphProviderPlugin::build_state_snapshot(const IntrospectionInput & input,
                                                                               const std::string & timestamp) {
  GraphBuildState state;
  state.now_ns = current_time_ns();
  {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    state.topic_metrics = topic_metrics_;
  }

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    std::unordered_set<std::string> known_app_ids;
    known_app_ids.reserve(input.apps.size());
    for (const auto & app : input.apps) {
      known_app_ids.insert(app.id);
      if (app.is_online) {
        last_seen_by_app_[app.id] = timestamp;
      }
    }
    // Bound the map: an entry is only useful while its app is still known to
    // the graph (only offline apps' entries are ever read, to fill last_seen
    // on an unreachable node). Without this, ROS 2's continuously-minted
    // unique node names (_ros2cli_<pid> from every CLI call, respawned nodes)
    // make this map grow without bound, and the whole thing is deep-copied
    // below on every HTTP request, sampler tick, and discovery cycle. An app
    // that is merely offline (still present in input.apps) is untouched here.
    for (auto it = last_seen_by_app_.begin(); it != last_seen_by_app_.end();) {
      if (known_app_ids.count(it->first) == 0) {
        it = last_seen_by_app_.erase(it);
      } else {
        ++it;
      }
    }
    state.last_seen_by_app = last_seen_by_app_;
  }

  return state;
}

void GraphProviderPlugin::load_parameters() {
  auto get_config_double = [this](const std::string & key, double default_val) -> double {
    if (plugin_config_.contains(key) && plugin_config_[key].is_number()) {
      return plugin_config_[key].get<double>();
    }
    return default_val;
  };

  // A non-positive expected_frequency_hz_default/degraded_frequency_ratio/
  // freshness_headroom_factor/freshness_floor_sec silently disables
  // degradation and staleness detection (resolve_expected_frequency and
  // resolve_freshness_window_sec both divide by / gate on these), and a
  // negative drop_rate_percent_threshold has the same silent effect (zero is
  // a valid threshold there). Reject a bad value, fall back to `fallback`
  // (always one of this function's own hardcoded, always-valid defaults), and
  // warn naming the offending key and value.
  auto validate_positive = [this](const std::string & key, double value, double fallback) -> double {
    if (value > 0.0) {
      return value;
    }
    log_warn("Ignoring invalid graph provider config '" + key + "'=" + std::to_string(value) +
             " (want a positive number); keeping " + std::to_string(fallback));
    return fallback;
  };
  auto validate_non_negative = [this](const std::string & key, double value, double fallback) -> double {
    if (value >= 0.0) {
      return value;
    }
    log_warn("Ignoring invalid graph provider config '" + key + "'=" + std::to_string(value) +
             " (want a non-negative number); keeping " + std::to_string(fallback));
    return fallback;
  };

  ConfigOverrides new_config;
  new_config.defaults.expected_frequency_hz_default = validate_positive(
      "expected_frequency_hz_default", get_config_double("expected_frequency_hz_default", 30.0), 30.0);
  new_config.defaults.degraded_frequency_ratio =
      validate_positive("degraded_frequency_ratio", get_config_double("degraded_frequency_ratio", 0.5), 0.5);
  new_config.defaults.drop_rate_percent_threshold =
      validate_non_negative("drop_rate_percent_threshold", get_config_double("drop_rate_percent_threshold", 5.0), 5.0);
  new_config.defaults.freshness_headroom_factor =
      validate_positive("freshness_headroom_factor", get_config_double("freshness_headroom_factor", 3.0), 3.0);
  new_config.defaults.freshness_floor_sec =
      validate_positive("freshness_floor_sec", get_config_double("freshness_floor_sec", 5.0), 5.0);

  // Per-function overrides. The gateway delivers plugin config as nested
  // objects, never as a flat "function_overrides.<function_id>.<field>" key
  // (see param_utils.hpp's insert_nested_param): a dotted parameter
  // plugins.graph_provider.function_overrides.<function_id>.<field> arrives
  // as {"function_overrides": {"<function_id>": {"<field>": value}}}.
  // Flatten it back to a dotted key exactly the way
  // ros2_medkit_graph_watchdog's ParamDriftConfig::flatten_expect() un-nests
  // its `expect` config key, then split on the LAST dot: every field name
  // below is a fixed, dot-free identifier, so this is unambiguous even if the
  // function id itself contains a dot (matching how insert_nested_param would
  // have split such an id into further nesting levels on the way in).
  std::map<std::string, nlohmann::json> flat_overrides;
  if (plugin_config_.contains("function_overrides") && plugin_config_["function_overrides"].is_object()) {
    flatten_json_object(plugin_config_["function_overrides"], "", flat_overrides);
  }

  for (const auto & [key, value] : flat_overrides) {
    if (!value.is_number()) {
      continue;
    }

    const auto dot = key.rfind('.');
    if (dot == std::string::npos) {
      continue;
    }

    const auto function_id = key.substr(0, dot);
    const auto field = key.substr(dot + 1);
    const auto log_key = "function_overrides." + key;

    auto [config_it, inserted] = new_config.by_function.emplace(function_id, new_config.defaults);
    auto & function_config = config_it->second;
    (void)inserted;

    const double numeric_value = value.get<double>();
    // An override must not be able to smuggle in a value the defaults above
    // would reject (e.g. a zero expected_frequency_hz); on rejection it falls
    // back to whatever this function's config already resolved to (the
    // validated default, or an earlier valid override for the same field).
    if (field == "expected_frequency_hz") {
      function_config.expected_frequency_hz_default =
          validate_positive(log_key, numeric_value, function_config.expected_frequency_hz_default);
    } else if (field == "degraded_frequency_ratio") {
      function_config.degraded_frequency_ratio =
          validate_positive(log_key, numeric_value, function_config.degraded_frequency_ratio);
    } else if (field == "drop_rate_percent_threshold") {
      function_config.drop_rate_percent_threshold =
          validate_non_negative(log_key, numeric_value, function_config.drop_rate_percent_threshold);
    } else if (field == "freshness_headroom_factor") {
      function_config.freshness_headroom_factor =
          validate_positive(log_key, numeric_value, function_config.freshness_headroom_factor);
    } else if (field == "freshness_floor_sec") {
      function_config.freshness_floor_sec =
          validate_positive(log_key, numeric_value, function_config.freshness_floor_sec);
    }
  }

  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = std::move(new_config);
}

}  // namespace ros2_medkit_gateway
