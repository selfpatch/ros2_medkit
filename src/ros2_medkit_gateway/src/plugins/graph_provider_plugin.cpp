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

#include "ros2_medkit_gateway/plugins/graph_provider_plugin.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ros2_medkit_msgs/msg/fault.hpp>
#include <set>
#include <utility>

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"

namespace ros2_medkit_gateway {

namespace {

constexpr size_t kMaxCachedTopicMetrics = 512;

bool is_filtered_topic_name(const std::string & topic_name) {
  if (topic_name == "/parameter_events" || topic_name == "/rosout" || topic_name == "/diagnostics") {
    return true;
  }
  if (topic_name.find("/nitros") != std::string::npos) {
    return true;
  }
  if (topic_name.find("_supported_types") != std::string::npos) {
    return true;
  }
  return false;
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

EdgeBuildResult build_edge_json(const std::string & edge_id, const App & source, const App & target,
                                const std::string & topic_name, const std::string & topic_id,
                                const GraphProviderPlugin::TopicMetrics * metrics,
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
    metrics_json["drop_rate_percent"] = metrics->drop_rate_percent;
  }

  result.json = {{"edge_id", edge_id},   {"source", source.id},         {"target", target.id},
                 {"topic_id", topic_id}, {"transport_type", "unknown"}, {"metrics", metrics_json}};

  const bool node_offline = !source.is_online || !target.is_online;
  const bool topic_stale = state.stale_topics.count(topic_name) > 0;
  const bool has_frequency = metrics && metrics->frequency_hz.has_value();

  if (node_offline) {
    result.json["metrics"]["metrics_status"] = "error";
    result.json["metrics"]["error_reason"] = "node_offline";
    result.is_error = true;
    return result;
  }

  if (topic_stale) {
    result.json["metrics"]["metrics_status"] = "error";
    result.json["metrics"]["error_reason"] = "topic_stale";
    result.is_error = true;
    return result;
  }

  if (has_frequency) {
    const double expected_frequency = resolve_expected_frequency(metrics, config);
    const double frequency = *metrics->frequency_hz;
    result.json["metrics"]["metrics_status"] = "active";

    if (expected_frequency > 0.0) {
      result.ratio = frequency / expected_frequency;
      if (*result.ratio < config.degraded_frequency_ratio) {
        result.is_degraded = true;
      }
    }
    if (metrics->drop_rate_percent > config.drop_rate_percent_threshold) {
      result.is_degraded = true;
    }
    return result;
  }

  if (!state.diagnostics_seen) {
    result.json["metrics"]["metrics_status"] = "pending";
    return result;
  }

  result.json["metrics"]["metrics_status"] = "error";
  result.json["metrics"]["error_reason"] = "no_data_source";
  result.is_error = true;
  return result;
}

nlohmann::json build_graph_document_for_apps(const std::string & function_id,
                                             const std::vector<const App *> & scoped_apps,
                                             const GraphProviderPlugin::GraphBuildState & state,
                                             const GraphProviderPlugin::GraphBuildConfig & config,
                                             const std::string & timestamp) {
  nlohmann::json graph = {{"schema_version", "1.0.0"},
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
        auto edge =
            build_edge_json(edge_id, *publisher, *subscriber, topic_name, topic_it->second, metrics, state, config);

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

std::string GraphProviderPlugin::name() const {
  return "graph-provider";
}

void GraphProviderPlugin::configure(const nlohmann::json & /*config*/) {
}

void GraphProviderPlugin::set_context(PluginContext & context) {
  ctx_ = &context;
  ctx_->register_capability(SovdEntityType::FUNCTION, "x-medkit-graph");

  load_parameters();
  subscribe_to_diagnostics();

  ctx_->register_sampler("x-medkit-graph",
                         [this](const std::string & entity_id,
                                const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
                           auto payload = get_cached_or_built_graph(entity_id);
                           if (!payload.has_value()) {
                             return tl::make_unexpected(std::string("Graph snapshot not available: ") + entity_id);
                           }
                           return *payload;
                         });
  log_info("Registered x-medkit-graph cyclic subscription sampler");
}

void GraphProviderPlugin::register_routes(httplib::Server & server, const std::string & api_prefix) {
  server.Get(api_prefix + R"(/functions/([^/]+)/x-medkit-graph)",
             [this](const httplib::Request & req, httplib::Response & res) {
               if (!ctx_) {
                 PluginContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Graph provider context not initialized");
                 return;
               }

               const auto function_id = req.matches[1].str();
               auto entity = ctx_->validate_entity_for_route(req, res, function_id);
               if (!entity) {
                 return;
               }

               auto payload = get_cached_or_built_graph(function_id);
               if (!payload.has_value()) {
                 PluginContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Graph snapshot not available",
                                           {{"function_id", function_id}});
                 return;
               }

               PluginContext::send_json(res, *payload);
             });
}

IntrospectionResult GraphProviderPlugin::introspect(const IntrospectionInput & input) {
  std::unordered_map<std::string, nlohmann::json> new_cache;
  const auto timestamp = current_timestamp();
  auto state = build_state_snapshot("", input, timestamp, false);

  for (const auto & function : input.functions) {
    new_cache.emplace(function.id,
                      build_graph_document(function.id, input, state, resolve_config(function.id), timestamp));
  }

  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    graph_cache_.swap(new_cache);
  }

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

  std::lock_guard<std::mutex> lock(metrics_mutex_);
  diagnostics_seen_ = true;
  for (auto & [topic_name, metrics] : updates) {
    if (topic_metrics_.find(topic_name) == topic_metrics_.end()) {
      topic_metrics_order_.push_back(topic_name);
    }
    topic_metrics_[topic_name] = metrics;
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

  for (const auto & kv : status.values) {
    if (kv.key == "frame_rate_msg") {
      auto value = parse_double(kv.value);
      if (value.has_value() && std::isfinite(*value)) {
        metrics.frequency_hz = *value;
      }
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

std::string GraphProviderPlugin::generate_fault_code(const std::string & diagnostic_name) {
  std::string result;
  result.reserve(diagnostic_name.size());

  bool last_was_separator = true;
  for (char c : diagnostic_name) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      result += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      last_was_separator = false;
    } else if (!last_was_separator) {
      result += '_';
      last_was_separator = true;
    }
  }

  if (!result.empty() && result.back() == '_') {
    result.pop_back();
  }
  return result;
}

std::string GraphProviderPlugin::current_timestamp() {
  const auto now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  return format_timestamp_ns(now_ns);
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

std::optional<nlohmann::json> GraphProviderPlugin::get_cached_or_built_graph(const std::string & function_id) {
  // In the real gateway, the merged entity cache is the source of truth. The
  // plugin-side cache is populated during the merge pipeline before runtime
  // linking finishes, so rebuilding here avoids serving stale node/topic state.
  if (ctx_) {
    auto rebuilt = build_graph_from_entity_cache(function_id);
    if (rebuilt.has_value()) {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      graph_cache_[function_id] = *rebuilt;
      return rebuilt;
    }
  }

  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = graph_cache_.find(function_id);
    if (it != graph_cache_.end()) {
      return it->second;
    }
  }

  return std::nullopt;
}

std::optional<nlohmann::json> GraphProviderPlugin::build_graph_from_entity_cache(const std::string & function_id) {
  if (!ctx_) {
    return std::nullopt;
  }

  auto input = ctx_->get_entity_snapshot();
  const auto timestamp = current_timestamp();
  auto state = build_state_snapshot(function_id, input, timestamp, false);
  const auto scoped_apps = resolve_scoped_apps(function_id, input);
  return build_graph_document_for_apps(function_id, scoped_apps, state, resolve_config(function_id), timestamp);
}

std::unordered_set<std::string> GraphProviderPlugin::collect_stale_topics(const std::string & function_id,
                                                                          const IntrospectionInput & input) const {
  std::unordered_set<std::string> stale_topics;
  if (!ctx_) {
    return stale_topics;
  }

  std::unordered_map<std::string, std::string> fault_code_to_topic;
  for (const auto * app : resolve_scoped_apps(function_id, input)) {
    for (const auto & topic_name : filtered_topics(app->topics.publishes)) {
      fault_code_to_topic.emplace(generate_fault_code(topic_name), topic_name);
    }
    for (const auto & topic_name : filtered_topics(app->topics.subscribes)) {
      fault_code_to_topic.emplace(generate_fault_code(topic_name), topic_name);
    }
  }

  auto fault_data = ctx_->list_all_faults();
  if (!fault_data.contains("faults") || !fault_data["faults"].is_array()) {
    return stale_topics;
  }

  for (const auto & fault : fault_data["faults"]) {
    if (!fault.contains("fault_code") || !fault.contains("severity") || !fault.contains("status")) {
      continue;
    }
    const auto code_it = fault_code_to_topic.find(fault["fault_code"].get<std::string>());
    if (code_it == fault_code_to_topic.end()) {
      continue;
    }
    if (fault["status"].get<std::string>() == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED &&
        fault["severity"].get<uint8_t>() == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
      stale_topics.insert(code_it->second);
    }
  }

  return stale_topics;
}

GraphProviderPlugin::GraphBuildState GraphProviderPlugin::build_state_snapshot(const std::string & function_id,
                                                                               const IntrospectionInput & input,
                                                                               const std::string & timestamp,
                                                                               bool include_stale_topics) {
  GraphBuildState state;
  {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    state.topic_metrics = topic_metrics_;
    state.diagnostics_seen = diagnostics_seen_;
  }
  if (include_stale_topics) {
    state.stale_topics = collect_stale_topics(function_id, input);
  }

  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    for (const auto & app : input.apps) {
      if (app.is_online) {
        last_seen_by_app_[app.id] = timestamp;
      }
    }
    state.last_seen_by_app = last_seen_by_app_;
  }

  return state;
}

void GraphProviderPlugin::load_parameters() {
  if (!ctx_ || !ctx_->node()) {
    return;
  }

  auto * node = ctx_->node();

  const auto declare_if_needed = [node](const std::string & name, double default_value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter(name, default_value);
    }
  };

  declare_if_needed("graph_provider.expected_frequency_hz_default", 30.0);
  declare_if_needed("graph_provider.degraded_frequency_ratio", 0.5);
  declare_if_needed("graph_provider.drop_rate_percent_threshold", 5.0);

  ConfigOverrides new_config;
  new_config.defaults.expected_frequency_hz_default =
      node->get_parameter("graph_provider.expected_frequency_hz_default").as_double();
  new_config.defaults.degraded_frequency_ratio =
      node->get_parameter("graph_provider.degraded_frequency_ratio").as_double();
  new_config.defaults.drop_rate_percent_threshold =
      node->get_parameter("graph_provider.drop_rate_percent_threshold").as_double();

  const auto overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  const std::string prefix = "graph_provider.function_overrides.";
  for (const auto & [name, value] : overrides) {
    if (name.rfind(prefix, 0) != 0) {
      continue;
    }

    const auto remainder = name.substr(prefix.size());
    const auto split = remainder.rfind('.');
    if (split == std::string::npos) {
      continue;
    }

    const auto function_id = remainder.substr(0, split);
    const auto field = remainder.substr(split + 1);

    auto [config_it, inserted] = new_config.by_function.emplace(function_id, new_config.defaults);
    auto & function_config = config_it->second;
    (void)inserted;

    if (value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
        value.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
      continue;
    }

    const double numeric_value = value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                                     ? value.get<double>()
                                     : static_cast<double>(value.get<int64_t>());

    if (field == "expected_frequency_hz") {
      function_config.expected_frequency_hz_default = numeric_value;
    } else if (field == "degraded_frequency_ratio") {
      function_config.degraded_frequency_ratio = numeric_value;
    } else if (field == "drop_rate_percent_threshold") {
      function_config.drop_rate_percent_threshold = numeric_value;
    }
  }

  std::lock_guard<std::mutex> lock(config_mutex_);
  config_ = std::move(new_config);
}

}  // namespace ros2_medkit_gateway
