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

#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

namespace ros2_medkit_gateway {

/// Schema version stamped on the `x-medkit-graph` document's `schema_version`
/// field. Bump the major component whenever a change to the document's
/// shape or semantics (field removal, value-set change, altered meaning of
/// an existing field) would break an external consumer parsing it.
inline constexpr const char * kGraphSchemaVersion = "2.0.0";

class PluginContext;
class RosPluginContext;

class GraphProviderPluginTestAccess;  // Forward declaration for test friend

class GraphProviderPlugin : public GatewayPlugin, public IntrospectionProvider {
  friend class GraphProviderPluginTestAccess;

 public:
  struct TopicMetrics {
    std::optional<double> frequency_hz;
    std::optional<double> latency_ms;
    // Optional (not defaulted to 0.0) so a diagnostics sample that omits this
    // key can be merged into an existing entry without clobbering a
    // previously-observed drop rate.
    std::optional<double> drop_rate_percent;
    std::optional<double> expected_frequency_hz;
    // Epoch nanoseconds (plugin clock) of the last update merged into this
    // entry. Defaults to 0 so hand-built test fixtures with a matching
    // default-constructed GraphBuildState::now_ns (also 0) read as fresh.
    int64_t last_update_ns{0};
  };

  struct GraphBuildState {
    std::unordered_map<std::string, TopicMetrics> topic_metrics;
    std::unordered_map<std::string, std::string> last_seen_by_app;
    // Epoch nanoseconds (plugin clock) used as "now" for freshness
    // comparisons against TopicMetrics::last_update_ns. Explicit field
    // (rather than reading the wall clock inside the pure build functions)
    // so build_graph_document stays a deterministic pure function - the
    // production path stamps it in build_state_snapshot(), tests set it
    // directly.
    int64_t now_ns{0};
  };

  struct GraphBuildConfig {
    double expected_frequency_hz_default{30.0};
    double degraded_frequency_ratio{0.5};
    double drop_rate_percent_threshold{5.0};
    // Freshness window = max(freshness_floor_sec, freshness_headroom_factor / expected_frequency_hz).
    double freshness_headroom_factor{3.0};
    double freshness_floor_sec{5.0};
  };

  GraphProviderPlugin() = default;
  ~GraphProviderPlugin() noexcept override;

  std::string name() const override;
  void configure(const nlohmann::json & config) override;
  void set_context(PluginContext & context) override;
  std::vector<PluginRoute> get_routes() override;
  void shutdown() override;
  IntrospectionResult introspect(const IntrospectionInput & input) override;

  static nlohmann::json build_graph_document(const std::string & function_id, const IntrospectionInput & input,
                                             const GraphBuildState & state, const GraphBuildConfig & config,
                                             const std::string & timestamp);

 private:
  struct ConfigOverrides {
    std::unordered_map<std::string, GraphBuildConfig> by_function;
    GraphBuildConfig defaults;
  };

  void subscribe_to_diagnostics();
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg);
  static std::optional<TopicMetrics> parse_topic_metrics(const diagnostic_msgs::msg::DiagnosticStatus & status);
  static std::optional<double> parse_double(const std::string & value);
  static std::string current_timestamp();
  static int64_t current_time_ns();
  GraphBuildConfig resolve_config(const std::string & function_id) const;
  std::optional<nlohmann::json> build_current_graph(const std::string & function_id);
  std::optional<nlohmann::json> build_graph_from_entity_cache(const std::string & function_id);
  GraphBuildState build_state_snapshot(const IntrospectionInput & input, const std::string & timestamp);
  void load_parameters();

  RosPluginContext * ctx_{nullptr};
  nlohmann::json plugin_config_;

  // Each mutex protects an independent state bucket; no code path acquires more than one.
  mutable std::mutex metrics_mutex_;
  std::unordered_map<std::string, TopicMetrics> topic_metrics_;
  std::deque<std::string> topic_metrics_order_;

  mutable std::mutex status_mutex_;
  std::unordered_map<std::string, std::string> last_seen_by_app_;

  mutable std::mutex config_mutex_;
  ConfigOverrides config_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  std::atomic<bool> shutdown_requested_{false};
};

}  // namespace ros2_medkit_gateway
