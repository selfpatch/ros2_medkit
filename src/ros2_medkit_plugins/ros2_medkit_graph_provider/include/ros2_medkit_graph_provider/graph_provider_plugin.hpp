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
#include <rclcpp/message_info.hpp>
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
  // Policy for how an edge's `metrics` react when more than one live
  // publisher is found on its DATA topic (see
  // resolve_data_topic_publisher_counts()). `frame_rate_msg` (the value the
  // rest of this file reads into TopicMetrics::frequency_hz) is a
  // SUBSCRIBER-SIDE arrival rate summed across every publisher on the topic
  // name, so a genuinely slow producer plus a leftover/duplicate publisher on
  // the same topic can sum to a healthy-looking rate while the real pipeline
  // is broken - the plugin cannot tell the difference from frame_rate_msg
  // alone.
  enum class MultiPublisherRatePolicy {
    // Default, safe: keep emitting frequency_hz (and let it drive the
    // degraded ratio) exactly as before this policy existed, but also emit
    // publisher_count and rate_ambiguous so the operator sees the ambiguity
    // and can judge for themselves.
    kAnnotate,
    // Conservative against a false "healthy": omit frequency_hz (and exclude
    // it from the degraded ratio / bottleneck calculation) whenever
    // publisher_count > 1, so an inflated arrival rate can never mask a
    // broken pipeline as healthy. publisher_count and rate_ambiguous are
    // still emitted; metrics_status is untouched - suppression is only about
    // the untrustworthy rate number, never about freshness.
    kSuppress,
  };

  struct TopicMetrics {
    std::optional<double> frequency_hz;
    std::optional<double> latency_ms;
    // Optional (not defaulted to 0.0) so a diagnostics sample that omits this
    // key can be merged into an existing entry without clobbering a
    // previously-observed drop rate.
    std::optional<double> drop_rate_percent;
    std::optional<double> expected_frequency_hz;
    // Resolved node name that published the /diagnostics message this entry
    // was last updated from (via publisher GID matching, see
    // resolve_publisher_source()). Empty when no GID match was found for a
    // sample (publisher left the graph between publishing and the graph
    // query, or a race) - never a fabricated name or a vendor literal.
    // Unlike every other field in this struct, `source` is LATEST-WINS, not
    // merge-not-replace: it tracks who sent the MOST RECENT sample, so an
    // unresolved current message must clear a stale earlier attribution
    // rather than preserve it. See the merge in diagnostics_callback().
    std::optional<std::string> source;
    // Monotonic-clock (steady_clock) nanoseconds of the last update merged
    // into this entry. Defaults to 0 so hand-built test fixtures with a
    // matching default-constructed GraphBuildState::now_ns (also 0) read as
    // fresh. Deliberately NOT system_clock: age is computed as
    // `now_ns - last_update_ns`, and a backward wall-clock step (e.g. an NTP
    // correction on a Jetson booting before sync) would make that
    // subtraction go negative and read as "fresh forever" against a
    // non-monotonic clock. See GraphProviderPlugin::current_steady_ns().
    int64_t last_update_ns{0};
  };

  struct GraphBuildState {
    std::unordered_map<std::string, TopicMetrics> topic_metrics;
    std::unordered_map<std::string, std::string> last_seen_by_app;
    // Monotonic-clock (steady_clock) nanoseconds used as "now" for freshness
    // comparisons against TopicMetrics::last_update_ns.
    // Explicit field (rather than reading the clock inside the pure build
    // functions) so build_graph_document stays a deterministic pure function
    // - the production path stamps it in build_state_snapshot() from
    // current_steady_ns(), tests set it directly. Deliberately NOT
    // system_clock - see TopicMetrics::last_update_ns.
    int64_t now_ns{0};
    // Live ROS-graph publisher count per DATA topic (from
    // get_publishers_info_by_topic - NOT /diagnostics), resolved once per
    // unique topic per graph build and reused across every edge sharing that
    // topic - see resolve_data_topic_publisher_counts(). A topic absent from
    // this map is UNRESOLVED (the query was never run - e.g. a hand-built
    // test fixture that only exercises the pure build_graph_document path -
    // or it returned empty: the publisher left the graph between being
    // modeled and this query, or a race). Absent is deliberately never
    // recorded as a count of 0: build_edge_json omits `publisher_count` from
    // the JSON in that case, exactly like TopicMetrics::source omits
    // `source` when unresolved - never a fabricated value.
    std::unordered_map<std::string, int> topic_publisher_counts;
  };

  struct GraphBuildConfig {
    double expected_frequency_hz_default{30.0};
    double degraded_frequency_ratio{0.5};
    double drop_rate_percent_threshold{5.0};
    // Freshness window = max(freshness_floor_sec, freshness_headroom_factor / expected_frequency_hz).
    double freshness_headroom_factor{3.0};
    double freshness_floor_sec{5.0};
    // Minimum continuous duration (seconds) a topic must remain outside its
    // freshness window before an edge is reported "metrics_stale" - absorbs
    // a single late DDS/executor-jitter sample without flapping
    // pipeline_status to "broken" and back on the very next on-time sample.
    // 0.0 = point-in-time, today's un-debounced behavior (age > window alone
    // decides). Purely a threshold added to the window - see
    // is_metrics_stale() in graph_provider_plugin.cpp: stale iff
    // `age > window + grace`, a stateless function of (now_ns,
    // last_update_ns, window, grace) with no separate onset/tick state to
    // maintain.
    double stale_grace_sec{2.0};
    // See MultiPublisherRatePolicy. Defaults to the safe, non-breaking
    // choice: annotate but never hide a measured rate.
    MultiPublisherRatePolicy multi_publisher_rate{MultiPublisherRatePolicy::kAnnotate};
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
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg,
                            const rclcpp::MessageInfo & msg_info);
  // Resolves the node that published this /diagnostics message by matching
  // msg_info's publisher GID against the endpoints returned by
  // get_publishers_info_by_topic("/diagnostics") - a fresh graph query, run
  // once per message (never cached: a stale mapping would misattribute after
  // a publisher restarts). Returns nullopt on no match (publisher left the
  // graph between publishing and this query, or a race) - callers must never
  // substitute a fabricated name or a vendor literal for an empty result.
  std::optional<std::string> resolve_publisher_source(const rclcpp::MessageInfo & msg_info) const;
  // Resolves the live publisher count for every unique DATA topic reachable
  // from `scoped_apps`, via get_publishers_info_by_topic - once per unique
  // topic name (not once per edge; a topic shared by a fan-out/fan-in group
  // of edges is queried exactly once and the result is reused for all of
  // them). A topic whose query returns empty is left OUT of the returned map
  // rather than recorded as 0 - see GraphBuildState::topic_publisher_counts.
  std::unordered_map<std::string, int>
  resolve_data_topic_publisher_counts(const std::vector<const App *> & scoped_apps) const;
  static std::optional<TopicMetrics> parse_topic_metrics(const diagnostic_msgs::msg::DiagnosticStatus & status);
  static std::optional<double> parse_double(const std::string & value);
  static std::string current_timestamp();
  // Wall-clock (system_clock) nanoseconds - used ONLY for the human-readable
  // `timestamp` document field (via current_timestamp()/format_timestamp_ns).
  // Never use this for age/freshness arithmetic - see current_steady_ns().
  static int64_t current_time_ns();
  // Monotonic-clock (steady_clock) nanoseconds - used for every freshness/age
  // comparison (TopicMetrics::last_update_ns, GraphBuildState::now_ns) so a
  // backward wall-clock step (NTP correction) can never make a dead topic
  // read as fresh.
  static int64_t current_steady_ns();
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
