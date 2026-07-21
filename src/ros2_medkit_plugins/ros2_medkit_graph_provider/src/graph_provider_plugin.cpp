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
#include <rclcpp/message_info.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
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

// Fully-qualified node name (namespace + name), the same form `ros2 node
// list` displays: "/talker" for a node in the root namespace, "/ns/talker"
// otherwise. Chosen over the bare node_name() so two publishers with the
// same short name in different namespaces are not conflated in metrics.source.
std::string endpoint_fqn(const rclcpp::TopicEndpointInfo & endpoint) {
  const auto & ns = endpoint.node_namespace();
  const auto & name = endpoint.node_name();
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  if (ns.back() == '/') {
    return ns + name;
  }
  return ns + "/" + name;
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

// Debounced freshness verdict used by build_edge_json: an edge is only
// reported "metrics_stale" once its age has been outside the freshness
// window for more than `grace_sec` - a single late DDS/executor-jitter
// sample must not flip pipeline_status to "broken" and back on the very
// next on-time sample. Stale iff `age > window_sec + grace_sec`: a pure
// function of (now_ns, last_update_ns, window_sec, grace_sec) with no
// separate onset/tick state to maintain, so it always uses whichever
// window_sec the caller resolved (build_edge_json always resolves the real,
// per-function config - see GraphProviderPlugin::resolve_config) - never a
// value computed against a different (e.g. global-default) window.
//
// `grace_sec == 0.0` reduces this to exactly `age > window_sec`, preserving
// today's un-debounced behavior bit-for-bit (including its boundary: age
// exactly equal to the window is still active, not stale).
//
// A negative age (last_update_ns reading "in the future" relative to
// now_ns) can only come from a backward clock step - both stamps are
// steady_clock in production so this cannot happen there, but it must never
// read as a healthy, active edge, so it is treated as stale here regardless
// of window/grace.
bool is_metrics_stale(const GraphProviderPlugin::TopicMetrics & metrics, int64_t now_ns, double window_sec,
                      double grace_sec) {
  const auto age_ns = now_ns - metrics.last_update_ns;
  if (age_ns < 0) {
    return true;
  }
  const auto threshold_ns = static_cast<int64_t>((window_sec + grace_sec) * 1e9);
  return age_ns > threshold_ns;
}

EdgeBuildResult build_edge_json(const std::string & edge_id, const App & source, const App & target,
                                const std::string & topic_id, const GraphProviderPlugin::TopicMetrics * metrics,
                                std::optional<int> publisher_count, const GraphProviderPlugin::GraphBuildState & state,
                                const GraphProviderPlugin::GraphBuildConfig & config) {
  EdgeBuildResult result;
  auto metrics_json = nlohmann::json{
      {"frequency_hz", nullptr},
      {"latency_ms", nullptr},
      {"drop_rate_percent", 0.0},
      {"metrics_status", "pending"},
  };

  // publisher_count is a live ROS-graph fact about the DATA topic itself
  // (get_publishers_info_by_topic), not something carried in TopicMetrics, so
  // it applies to every edge on this topic whether or not /diagnostics has
  // ever reported anything for it - hence this runs before (and regardless
  // of) the `metrics` null-check below. Unresolved (nullopt: the query was
  // never run, came back empty, or ctx_/node() was unavailable) omits the
  // key entirely, exactly like TopicMetrics::source - never a fabricated 0.
  // Both policies always emit publisher_count when resolved, even at 1.
  const bool rate_ambiguous = publisher_count.has_value() && *publisher_count > 1;
  const bool suppress_rate =
      rate_ambiguous && config.multi_publisher_rate == GraphProviderPlugin::MultiPublisherRatePolicy::kSuppress;
  if (publisher_count.has_value()) {
    metrics_json["publisher_count"] = *publisher_count;
    if (rate_ambiguous) {
      metrics_json["rate_ambiguous"] = true;
    }
  }

  if (metrics) {
    // Under kSuppress with more than one live publisher, the measured rate is
    // untrustworthy (it may be inflated by a leftover/duplicate publisher) -
    // omit it rather than let a genuinely broken pipeline read as healthy.
    // kAnnotate (the default) always shows the measured rate unchanged.
    if (metrics->frequency_hz.has_value() && !suppress_rate) {
      metrics_json["frequency_hz"] = *metrics->frequency_hz;
    }
    if (metrics->latency_ms.has_value()) {
      metrics_json["latency_ms"] = *metrics->latency_ms;
    }
    metrics_json["drop_rate_percent"] = metrics->drop_rate_percent.value_or(0.0);
    // Honest failure handling: metrics.source (the resolved /diagnostics
    // publisher node name) is only emitted when it was actually resolved.
    // No GID match (publisher left the graph between publishing and the
    // graph query, or a race) leaves TopicMetrics::source empty, and that
    // must never be papered over with a fabricated name or a vendor literal
    // - the key is simply omitted, the same way a pending edge omits it.
    if (metrics->source.has_value()) {
      metrics_json["source"] = *metrics->source;
    }
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
  if (is_metrics_stale(*metrics, state.now_ns, freshness_window_sec, config.stale_grace_sec)) {
    result.json["metrics"]["metrics_status"] = "error";
    result.json["metrics"]["error_reason"] = "metrics_stale";
    result.is_error = true;
    return result;
  }

  // Observed and current. Metrics that carry latency/drop-rate but no
  // frequency (e.g. a rejected negative frame_rate_msg sample) are still
  // "active" - freshness, not field completeness, drives status.
  result.json["metrics"]["metrics_status"] = "active";
  // suppress_rate excludes the (untrustworthy, possibly-inflated) rate from
  // the degraded ratio / bottleneck calculation too - a missing frequency
  // must never itself drive a degraded verdict for this edge (drop_rate below
  // is unaffected and can still trip degraded on its own).
  if (metrics->frequency_hz.has_value() && !suppress_rate) {
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

      // Resolved once per unique topic (see resolve_data_topic_publisher_counts)
      // and reused below for every edge on this topic - a single map lookup
      // here, not a re-query, even when several subscribers fan out from the
      // same topic.
      std::optional<int> publisher_count;
      const auto publisher_count_it = state.topic_publisher_counts.find(topic_name);
      if (publisher_count_it != state.topic_publisher_counts.end()) {
        publisher_count = publisher_count_it->second;
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
        auto edge = build_edge_json(edge_id, *publisher, *subscriber, topic_it->second, metrics, publisher_count, state,
                                    config);

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
  // Defense-in-depth, matching the guard diagnostics_callback already has at
  // its entry: not exploitable today (the gateway only tears plugins down
  // after the executor has joined, so introspect() cannot race a live
  // shutdown in production), but introspect() otherwise relies entirely on
  // that external, non-local ordering invariant. Cheap to check, so check it.
  if (shutdown_requested_.load()) {
    return {};
  }

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
      "/diagnostics", rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg, const rclcpp::MessageInfo & msg_info) {
        diagnostics_callback(msg, msg_info);
      });

  log_info("Subscribed to /diagnostics for x-medkit-graph metrics");
}

void GraphProviderPlugin::diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg,
                                               const rclcpp::MessageInfo & msg_info) {
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

  if (updates.empty()) {
    // Nothing to stamp, so skip the graph query below entirely - it is a
    // live introspection call and every status in this message was already
    // filtered out or unparseable.
    return;
  }

  // A DiagnosticArray carries exactly one publisher for every status it
  // contains, so the source is resolved once per message here and applied
  // to every topic this message updates below - never once per status, and
  // never "the first publisher on the topic" (there can be more than one
  // /diagnostics publisher; only the GID match tells us which one sent THIS
  // message).
  const auto source = resolve_publisher_source(msg_info);

  // Monotonic clock: this stamp is later diffed against GraphBuildState::now_ns
  // (also steady_clock) to compute age - see current_steady_ns().
  const auto now_ns = current_steady_ns();

  std::lock_guard<std::mutex> lock(metrics_mutex_);
  for (auto & [topic_name, incoming] : updates) {
    incoming.source = source;
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
    // must not erase a good latency_ms from an earlier sample). `source` is
    // the one deliberate exception to this merge-not-replace rule - see its
    // assignment below.
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
    // Latest-wins, unconditionally - including clearing on unresolved. Every
    // field above merges (a sample that omits a key must not wipe a
    // previously-good value) because each one accumulates the best-known
    // reading. `source` means something different: "who sent the message
    // this entry was LAST updated from" (see the doc comment on
    // TopicMetrics::source and docs/api/rest.rst's metrics.source field
    // note). A later message whose GID does not resolve still advances
    // last_update_ns (the message really did arrive), so leaving the old
    // `source` in place would attribute a fresh sample to a stale, no-longer-
    // confirmed publisher - exactly the honesty violation this field exists
    // to prevent. Assigning incoming.source unconditionally (nullopt clears
    // it) keeps source and last_update_ns in lockstep: both always describe
    // the same, most recent message.
    existing.source = incoming.source;
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

std::optional<std::string> GraphProviderPlugin::resolve_publisher_source(const rclcpp::MessageInfo & msg_info) const {
  if (!ctx_ || !ctx_->node()) {
    return std::nullopt;
  }

  // A fresh graph query, run once per message (see diagnostics_callback) -
  // never cached across messages, since a cached mapping would misattribute
  // a later message once a publisher restarts (new GID, same or new node).
  const auto & msg_gid = msg_info.get_rmw_message_info().publisher_gid;
  const auto endpoints = ctx_->node()->get_publishers_info_by_topic("/diagnostics");
  for (const auto & endpoint : endpoints) {
    const auto & endpoint_gid = endpoint.endpoint_gid();
    if (std::equal(msg_gid.data, msg_gid.data + RMW_GID_STORAGE_SIZE, endpoint_gid.begin())) {
      return endpoint_fqn(endpoint);
    }
  }
  // No match: the publisher may have left the graph between publishing and
  // this query, or this is a race. Leave the source unresolved rather than
  // guessing - the caller (diagnostics_callback) must not substitute a
  // fabricated name or a vendor literal for an empty result.
  return std::nullopt;
}

std::unordered_map<std::string, int>
GraphProviderPlugin::resolve_data_topic_publisher_counts(const std::vector<const App *> & scoped_apps) const {
  std::unordered_map<std::string, int> counts;
  if (!ctx_ || !ctx_->node()) {
    return counts;
  }

  // collect_unique_topics already dedupes across every app in this function's
  // scope (both publishes and subscribes), so this queries the real ROS
  // graph exactly once per unique DATA topic per graph build - never once per
  // edge. build_graph_document_for_apps looks the result up by topic name and
  // reuses it for every edge sharing that topic (fan-out/fan-in), so a topic
  // shared by N edges costs one query here, not N.
  for (const auto & topic_name : collect_unique_topics(scoped_apps)) {
    // A fresh query per build (never cached across builds): a stale count
    // would miss a publisher that joined or left since the last build - the
    // exact failure mode this defense exists to catch.
    const auto endpoints = ctx_->node()->get_publishers_info_by_topic(topic_name);
    // Empty is left OUT of the map (never recorded as a count of 0): a data
    // topic reachable from a modeled app should always have at least one
    // live publisher, so an empty result here means the query raced the
    // publisher leaving the graph, or it hasn't been discovered yet -
    // unresolved, not "zero publishers". See
    // GraphBuildState::topic_publisher_counts.
    if (!endpoints.empty()) {
      counts[topic_name] = static_cast<int>(endpoints.size());
    }
  }
  return counts;
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
  // Wall clock (system_clock) deliberately: this feeds only the
  // human-readable `timestamp` document field, never an age comparison - see
  // current_time_ns()'s and current_steady_ns()'s doc comments.
  return format_timestamp_ns(current_time_ns());
}

int64_t GraphProviderPlugin::current_time_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

int64_t GraphProviderPlugin::current_steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
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
  // Real ROS-graph query, scoped to exactly this function's unique data
  // topics - see resolve_data_topic_publisher_counts() for the once-per-
  // topic (not once-per-edge) cost.
  state.topic_publisher_counts = resolve_data_topic_publisher_counts(scoped_apps);
  return build_graph_document_for_apps(function_id, scoped_apps, state, resolve_config(function_id), timestamp);
}

GraphProviderPlugin::GraphBuildState GraphProviderPlugin::build_state_snapshot(const IntrospectionInput & input,
                                                                               const std::string & timestamp) {
  GraphBuildState state;
  // Monotonic clock: diffed against TopicMetrics::last_update_ns (also
  // steady_clock) for freshness/grace age comparisons - see
  // current_steady_ns()'s doc comment.
  state.now_ns = current_steady_ns();
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
  // A key that is absent is silently treated as "not configured" - every
  // call site below passes its own hardcoded default_val as fallback, and
  // absence is normal (most global keys are never set). A key PRESENT with a
  // non-number JSON value is a different, genuine config mistake (a typo'd
  // type, not a missing key) and now warns, naming the key, before falling
  // back to default_val - the same distinction the per-function override
  // loop below draws between a bad field NAME (unrecognized, warned via
  // warn_unrecognized_function_override_field) and a bad field VALUE on a
  // recognized field (wrong type, warned via
  // warn_non_numeric_function_override_field/warn_non_string_function_override_field).
  auto get_config_double = [this](const std::string & key, double default_val) -> double {
    if (!plugin_config_.contains(key)) {
      return default_val;
    }
    if (plugin_config_[key].is_number()) {
      return plugin_config_[key].get<double>();
    }
    log_warn("Ignoring non-numeric graph provider config '" + key + "': expected a number");
    return default_val;
  };
  // A key that is absent is silently treated as "not configured" here -
  // exactly like get_config_double above. A key PRESENT with a non-string
  // JSON value is likewise a genuine config mistake (not absence) and now
  // warns, naming the key, before falling back to nullopt (so the caller's
  // own default applies) - the same wrong-type-vs-absent distinction
  // get_config_double draws above. Only a STRING value outside
  // {"annotate", "suppress"} is the OTHER kind of config mistake and gets a
  // separate warning, via validate_multi_publisher_rate below.
  auto get_config_string = [this](const std::string & key) -> std::optional<std::string> {
    if (!plugin_config_.contains(key)) {
      return std::nullopt;
    }
    if (plugin_config_[key].is_string()) {
      return plugin_config_[key].get<std::string>();
    }
    log_warn("Ignoring non-string graph provider config '" + key + "': expected a string");
    return std::nullopt;
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
  // Same warn-and-default shape as validate_positive/validate_non_negative
  // above, for a string enum instead of a numeric range.
  auto validate_multi_publisher_rate = [this](const std::string & key, const std::string & value,
                                              MultiPublisherRatePolicy fallback) -> MultiPublisherRatePolicy {
    if (value == "annotate") {
      return MultiPublisherRatePolicy::kAnnotate;
    }
    if (value == "suppress") {
      return MultiPublisherRatePolicy::kSuppress;
    }
    log_warn("Ignoring invalid graph provider config '" + key + "'='" + value +
             "' (want one of \"annotate\", \"suppress\"); keeping '" +
             (fallback == MultiPublisherRatePolicy::kSuppress ? "suppress" : "annotate") + "'");
    return fallback;
  };
  // A typo'd per-function override FIELD NAME (e.g. "expected_frequency"
  // instead of "expected_frequency_hz") used to be silently dropped by the
  // per-function override loop below - unlike a recognized field with a bad
  // VALUE (which warns via the three validators above), a bad field NAME gave
  // an operator zero signal that their override never applied. Defined here,
  // alongside the other warn-and-default helpers, rather than built inline in
  // the loop, so the string concatenation happens in a function called once
  // per (rare) unrecognized field, not textually inside the per-override loop
  // body itself.
  auto warn_unrecognized_function_override_field = [this](const std::string & field, const std::string & function_id,
                                                          const std::string & log_key) {
    log_warn("Ignoring unrecognized graph provider function override field '" + field + "' for function '" +
             function_id + "' (" + log_key + ")");
  };
  // Sibling of warn_unrecognized_function_override_field above: fires only
  // for a KNOWN numeric field given the wrong JSON type (e.g.
  // expected_frequency_hz: "foo"). The per-override loop below checks
  // whether `field` is recognized BEFORE this is_number() gate runs, so an
  // UNKNOWN field with a non-numeric value is reported as unrecognized
  // instead (via warn_unrecognized_function_override_field above) and never
  // reaches this helper - the field NAME is the more useful diagnosis there,
  // regardless of the value's type. Built here, not inline in the loop, for
  // the same reason as the sibling helper: the string concatenation runs
  // once per (rare) rejected value, not textually inside the per-override
  // loop body.
  auto warn_non_numeric_function_override_field = [this](const std::string & field, const std::string & function_id,
                                                         const std::string & log_key) {
    log_warn("Ignoring non-numeric graph provider function override field '" + field + "' for function '" +
             function_id + "' (" + log_key + "): expected a number");
  };
  // Sibling of warn_non_numeric_function_override_field above, for the one
  // override field that goes the other way: multi_publisher_rate is
  // string-valued, so a non-string value (e.g. multi_publisher_rate: 2.0)
  // used to be dropped silently by the is_string() gate in the loop below.
  // Same rationale as the two helpers above for keeping the string
  // concatenation out of the per-override loop body.
  auto warn_non_string_function_override_field = [this](const std::string & field, const std::string & function_id,
                                                        const std::string & log_key) {
    log_warn("Ignoring non-string graph provider function override field '" + field + "' for function '" + function_id +
             "' (" + log_key + "): expected a string");
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
  // Zero is a valid, meaningful grace (point-in-time, today's un-debounced
  // behavior) so this uses validate_non_negative like drop_rate_percent_threshold,
  // not validate_positive.
  new_config.defaults.stale_grace_sec =
      validate_non_negative("stale_grace_sec", get_config_double("stale_grace_sec", 2.0), 2.0);
  if (auto raw = get_config_string("multi_publisher_rate")) {
    new_config.defaults.multi_publisher_rate =
        validate_multi_publisher_rate("multi_publisher_rate", *raw, new_config.defaults.multi_publisher_rate);
  }

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

  // The recognized per-function override field set: the six numeric fields
  // plus the one string field, multi_publisher_rate. Checked FIRST in the
  // loop below, before any value-type gate, so an unrecognized field NAME is
  // always reported as unrecognized regardless of its value's JSON type -
  // the more useful diagnosis when the field itself is a typo (e.g.
  // "expected_frequency" instead of "expected_frequency_hz"), rather than a
  // misleading "expected a number"/"expected a string" about a field that
  // was never going to apply anyway. Only a RECOGNIZED field reaches its
  // type-specific gate (is_string() for multi_publisher_rate, is_number()
  // for the rest) below.
  static const std::unordered_set<std::string> kNumericFunctionOverrideFields = {
      "expected_frequency_hz",     "degraded_frequency_ratio", "drop_rate_percent_threshold",
      "freshness_headroom_factor", "freshness_floor_sec",      "stale_grace_sec"};

  for (const auto & [key, value] : flat_overrides) {
    const auto dot = key.rfind('.');
    if (dot == std::string::npos) {
      continue;
    }

    const auto function_id = key.substr(0, dot);
    const auto field = key.substr(dot + 1);
    const auto log_key = "function_overrides." + key;

    const bool is_multi_publisher_rate_field = (field == "multi_publisher_rate");
    const bool is_numeric_field = kNumericFunctionOverrideFields.count(field) > 0;
    if (!is_multi_publisher_rate_field && !is_numeric_field) {
      warn_unrecognized_function_override_field(field, function_id, log_key);
      continue;
    }

    // multi_publisher_rate is the one string-valued override field; every
    // other recognized field is numeric. Branch on it before the is_number()
    // gate below so a legitimate string value neither gets rejected there
    // nor misreported by warn_non_numeric_function_override_field as
    // "expected a number".
    if (is_multi_publisher_rate_field) {
      if (!value.is_string()) {
        warn_non_string_function_override_field(field, function_id, log_key);
        continue;
      }
      auto [config_it, inserted] = new_config.by_function.emplace(function_id, new_config.defaults);
      (void)inserted;
      config_it->second.multi_publisher_rate =
          validate_multi_publisher_rate(log_key, value.get<std::string>(), config_it->second.multi_publisher_rate);
      continue;
    }

    if (!value.is_number()) {
      warn_non_numeric_function_override_field(field, function_id, log_key);
      continue;
    }

    auto [config_it, inserted] = new_config.by_function.emplace(function_id, new_config.defaults);
    auto & function_config = config_it->second;
    (void)inserted;

    const double numeric_value = value.get<double>();
    // An override must not be able to smuggle in a value the defaults above
    // would reject (e.g. a zero expected_frequency_hz); on rejection it falls
    // back to whatever this function's config already resolved to (the
    // validated default, or an earlier valid override for the same field).
    // `is_numeric_field` above already guarantees `field` is exactly one of
    // the six keys handled below, so this if/else-if chain needs no trailing
    // else.
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
    } else if (field == "stale_grace_sec") {
      function_config.stale_grace_sec = validate_non_negative(log_key, numeric_value, function_config.stale_grace_sec);
    }
  }

  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = std::move(new_config);
}

}  // namespace ros2_medkit_gateway
