// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/alarm_state_machine.hpp"
#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

/// Snapshot of all polled values at a point in time
struct PollSnapshot {
  std::chrono::system_clock::time_point timestamp;
  std::unordered_map<std::string, OpcuaValue> values;  // node_id_str -> value
  std::unordered_map<std::string, bool> alarms;        // fault_code -> active
  bool connected{false};
  uint64_t poll_count{0};
  uint64_t error_count{0};
};

/// Callback when an alarm state changes (for fault reporting)
using AlarmChangeCallback =
    std::function<void(const std::string & fault_code, const AlarmConfig & config, bool active)>;

/// Callback when a native OPC-UA AlarmCondition lifecycle transitions to a
/// new SOVD status (issue #386). Fires at most once per logical transition;
/// suppressed events do not invoke the callback.
struct AlarmEventDelivery {
  std::string fault_code;
  std::string entity_id;
  SovdAlarmStatus next_status;
  AlarmAction action;
  uint16_t severity{0};      // raw OPC-UA Severity 1-1000
  std::string message;       // event Message field (or AlarmEventConfig override)
  std::string condition_id;  // string form of OPC-UA ConditionId
};
using EventAlarmCallback = std::function<void(const AlarmEventDelivery & delivery)>;

/// Looked-up runtime state for a unique OPC-UA Condition instance. The
/// poller keeps one entry per distinct ConditionId observed; the entry
/// outlives ack/confirm round-trips so the PR3 ``acknowledge_fault`` SOVD
/// operation can resolve a fault_code to the live ConditionId NodeId and
/// the latest ``EventId`` ByteString (required for spec-compliant Ack).
struct ConditionRuntime {
  opcua::NodeId condition_id;
  opcua::ByteString latest_event_id;
  std::string entity_id;
  std::string fault_code;
  SovdAlarmStatus last_status{SovdAlarmStatus::Suppressed};
};

/// Callback after each poll cycle (for publishing values to ROS 2 topics)
using PollCallback = std::function<void(const PollSnapshot & snapshot)>;

/// Configuration for the poller
struct PollerConfig {
  bool prefer_subscriptions{false};  // poll mode by default (subscriptions need event loop)
  double subscription_interval_ms{500.0};
  std::chrono::milliseconds poll_interval{1000};
  std::chrono::milliseconds reconnect_interval{5000};
};

/// Manages OPC-UA data collection via subscriptions (preferred) or polling
class OpcuaPoller {
 public:
  OpcuaPoller(OpcuaClient & client, const NodeMap & node_map);
  ~OpcuaPoller();

  OpcuaPoller(const OpcuaPoller &) = delete;
  OpcuaPoller & operator=(const OpcuaPoller &) = delete;

  /// Start the poller thread
  void start(const PollerConfig & config);

  /// Stop the poller thread
  void stop();

  /// Get current snapshot (thread-safe copy)
  PollSnapshot snapshot() const;

  /// Get value for a specific node (thread-safe)
  std::optional<OpcuaValue> get_value(const std::string & node_id_str) const;

  /// Set callback for alarm state changes
  void set_alarm_callback(AlarmChangeCallback callback);

  /// Set callback for native AlarmCondition event lifecycle transitions
  /// (issue #386). Must be called before ``start()``.
  void set_event_alarm_callback(EventAlarmCallback callback);

  /// Set callback fired after each poll cycle (for value bridging)
  void set_poll_callback(PollCallback callback);

  /// Check if using subscriptions (vs polling)
  bool using_subscriptions() const {
    return using_subscriptions_.load();
  }

  /// Look up a live condition by ``(entity_id, fault_code)``. Used by the
  /// SOVD ``acknowledge_fault`` / ``confirm_fault`` operation handlers to
  /// resolve which OPC-UA ConditionId should receive the Method call. The
  /// returned snapshot is a copy, so the caller can release any locks
  /// before performing the OPC-UA round-trip.
  ///
  /// Returns ``std::nullopt`` if no condition with that fault_code is
  /// currently active for the entity.
  std::optional<ConditionRuntime> lookup_condition(const std::string & entity_id, const std::string & fault_code) const;

 private:
  void poll_loop();
  void do_poll();
  void setup_subscriptions();
  void evaluate_alarms();
  void on_data_change(const std::string & node_id, const OpcuaValue & value);

  // Issue #386 helpers.
  void setup_event_subscriptions();
  void on_event(const AlarmEventConfig & cfg, const std::vector<opcua::Variant> & values,
                const opcua::NodeId & source_node, const opcua::NodeId & event_type);
  void condition_refresh();

  OpcuaClient & client_;
  const NodeMap & node_map_;
  PollerConfig config_;

  std::thread poll_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> using_subscriptions_{false};

  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  mutable std::mutex snapshot_mutex_;
  PollSnapshot snapshot_;

  mutable std::mutex alarm_mutex_;
  AlarmChangeCallback alarm_callback_;
  std::unordered_map<std::string, bool> alarm_states_;  // fault_code -> last known state

  // Issue #386: event-mode AlarmCondition state.
  EventAlarmCallback event_alarm_callback_;
  std::mutex event_alarm_callback_mutex_;

  uint32_t event_subscription_id_{0};
  std::vector<uint32_t> event_monitored_item_ids_;

  mutable std::shared_mutex conditions_mutex_;
  std::unordered_map<std::string, ConditionRuntime> conditions_;  // ConditionId stringForm -> runtime

  // ConditionRefresh bracketing state. open62541 sends the buffered
  // historical condition burst between RefreshStartEvent and
  // RefreshEndEvent; we apply each event during the burst as normal but
  // use this flag in tests / diagnostics. Production note: per Part 9
  // §5.5.7 the spec also requires the client to ignore ConditionRefresh
  // notifications carrying ``Retain=false`` for non-current branches; the
  // state machine already drops branches via BranchId, and the trampoline
  // cannot distinguish refresh-burst events from live events without
  // tracking the EventType, which we do explicitly below.
  std::atomic<bool> condition_refresh_in_progress_{false};

  // Thread safety: must be set via set_poll_callback() before start().
  // Not modified after start(), so safe to read from the poll thread without a mutex.
  PollCallback poll_callback_;
};

}  // namespace ros2_medkit_gateway
