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

#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace ros2_medkit_gateway {

class PluginContext;

class GraphProviderPlugin : public GatewayPlugin, public IntrospectionProvider {
 public:
  struct TopicMetrics {
    std::optional<double> frequency_hz;
    std::optional<double> latency_ms;
    double drop_rate_percent{0.0};
    std::optional<double> expected_frequency_hz;
  };

  struct GraphBuildState {
    std::unordered_map<std::string, TopicMetrics> topic_metrics;
    std::unordered_set<std::string> stale_topics;
    std::unordered_map<std::string, std::string> last_seen_by_app;
    bool diagnostics_seen{false};
  };

  struct GraphBuildConfig {
    double expected_frequency_hz_default{30.0};
    double degraded_frequency_ratio{0.5};
    double drop_rate_percent_threshold{5.0};
  };

  GraphProviderPlugin() = default;
  ~GraphProviderPlugin() override = default;

  std::string name() const override;
  void configure(const nlohmann::json & config) override;
  void set_context(PluginContext & context) override;
  void register_routes(httplib::Server & server, const std::string & api_prefix) override;
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
  static bool is_filtered_topic(const std::string & topic_name);
  static std::string generate_fault_code(const std::string & diagnostic_name);
  static std::string current_timestamp();
  GraphBuildConfig resolve_config(const std::string & function_id) const;
  std::optional<nlohmann::json> get_cached_or_built_graph(const std::string & function_id);
  std::optional<nlohmann::json> build_graph_from_entity_cache(const std::string & function_id);
  std::unordered_set<std::string> collect_stale_topics(const IntrospectionInput & input) const;
  GraphBuildState build_state_snapshot(const IntrospectionInput & input, const std::string & timestamp);
  void load_parameters();

  PluginContext * ctx_{nullptr};

  mutable std::mutex cache_mutex_;
  std::unordered_map<std::string, nlohmann::json> graph_cache_;

  mutable std::mutex metrics_mutex_;
  std::unordered_map<std::string, TopicMetrics> topic_metrics_;
  bool diagnostics_seen_{false};

  mutable std::mutex status_mutex_;
  std::unordered_map<std::string, std::string> last_seen_by_app_;

  mutable std::mutex config_mutex_;
  ConfigOverrides config_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
};

}  // namespace ros2_medkit_gateway
