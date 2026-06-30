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
#include <set>
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

/// Callback when a fault-detection signal transitions (for fault reporting).
/// Fires once per raise/clear edge with the hosting entity and the shared
/// ``FaultSignal`` (fault code, severity, message, active) produced by the
/// shared evaluator. Threshold, status-bit and enum modes all use this path.
using AlarmChangeCallback =
    std::function<void(const std::string & entity_id, const ros2_medkit::fault_detection::FaultSignal & signal)>;

/// Callback when a native OPC-UA AlarmCondition lifecycle transitions to a
/// new SOVD status (issue #386). Fires at most once per logical transition;
/// suppressed events do not invoke the callback.
struct AlarmEventDelivery {
  std::string fault_code;
  std::string entity_id;
  SovdAlarmStatus next_status;
  AlarmAction action;
  uint16_t severity{0};           // raw OPC-UA Severity 1-1000
  std::string severity_override;  // resolved severity bucket override (issue #389), empty = derive from severity
  std::string message;            // event Message field (or resolved override)
  std::string condition_id;       // string form of OPC-UA ConditionId
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
  /// String form of the alarm source NodeId that owns this condition. Used by
  /// the read-based reconcile to skip clearing conditions whose source scan
  /// failed (issue #479).
  std::string source_id;
  SovdAlarmStatus last_status{SovdAlarmStatus::Suppressed};
};

/// Callback after each poll cycle (for publishing values to ROS 2 topics)
using PollCallback = std::function<void(const PollSnapshot & snapshot)>;

/// Strategy for replaying already-active OPC-UA conditions on (re)subscribe
/// (issue #389). Some servers (e.g. Siemens S7-1500) reject the standard
/// ConditionRefresh method with BadNodeIdUnknown, so the active alarm set is
/// otherwise lost across a reconnect / gateway restart.
///   - Method: call ConditionRefresh only (Part 9 §5.5.7 standard path).
///   - Read:   skip ConditionRefresh; browse the alarm sources and read each
///             condition's current state, then reconcile the fault set.
///   - Auto:   try ConditionRefresh first; on rejection fall back to Read.
///   - Off:    no replay (only live transitions surface).
enum class ConditionReplayStrategy { Method, Read, Auto, Off };

/// Configuration for the poller
struct PollerConfig {
  bool prefer_subscriptions{false};  // poll mode by default (subscriptions need event loop)
  double subscription_interval_ms{500.0};
  std::chrono::milliseconds poll_interval{1000};
  std::chrono::milliseconds reconnect_interval{5000};
  /// Active-condition replay strategy on (re)subscribe (issue #389).
  /// Default Auto: ConditionRefresh with a read-based fallback so hardened
  /// servers that reject the method still recover their active alarms.
  ConditionReplayStrategy condition_replay_strategy{ConditionReplayStrategy::Auto};
  /// Optional warn-level log sink for operator-visible failures inside the
  /// poll thread. Set by the plugin owning the poller to its log_warn
  /// helper so events like ``ConditionRefresh failed`` reach the ROS 2 log
  /// instead of stderr only. Empty by default - the poller falls back to
  /// stderr in that case.
  std::function<void(const std::string &)> log_warn;
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

  /// Parse a replay-strategy name ("method", "read", "auto", "off").
  /// Case-insensitive; unknown input falls back to Auto.
  static ConditionReplayStrategy parse_replay_strategy(const std::string & name);

  /// Decide whether a tracked condition should be cleared after a read-based
  /// replay scan (issue #479). A condition is cleared only when it was active,
  /// was NOT observed this scan (``seen``), and its owning source scan
  /// succeeded. If the source is in ``failed_sources`` the condition is left
  /// untouched - its absence from ``seen`` means "not scanned", not "gone".
  /// Pure and static so the false-clear guard is unit-testable without a server.
  static bool should_clear_condition(SovdAlarmStatus last_status, const std::string & condition_id,
                                     const std::string & source_id, const std::set<std::string> & seen,
                                     const std::set<std::string> & failed_sources);

 private:
  void poll_loop();
  void do_poll();
  void setup_subscriptions();
  void evaluate_alarms();
  void on_data_change(const std::string & node_id, const OpcuaValue & value);

  // Issue #386 helpers.
  void setup_event_subscriptions();
  void on_event(const AlarmEventConfig & cfg, const std::vector<opcua::Variant> & values,
                const opcua::NodeId & source_node, const opcua::NodeId & event_type,
                const opcua::NodeId & condition_id);

  /// Run the configured active-condition replay (issue #389). Dispatches to
  /// ConditionRefresh and/or the read-based fallback per
  /// ``condition_replay_strategy``.
  void replay_active_conditions();
  /// Call OPC-UA ConditionRefresh. Returns true when the server accepted it.
  bool try_condition_refresh();
  /// Read-based fallback: browse each alarm source, read current condition
  /// state, drive the state machine, and reconcile the fault set.
  void read_fallback_replay();
  /// Clear faults for conditions that were active before the replay but are
  /// no longer present/active in the read scan (``seen`` = condition ids
  /// observed this scan). Conditions whose owning source is in
  /// ``failed_sources`` (its browse failed this scan) are left untouched so a
  /// transient disconnect cannot falsely clear live alarms (issue #479).
  void reconcile_after_read(const std::set<std::string> & seen, const std::set<std::string> & failed_sources);

  /// Apply one condition state observation (from a live event or a read scan)
  /// to the tracked condition map + state machine, dispatching the resulting
  /// fault action. ``event_id`` is the live EventId for ack/confirm (null for
  /// read-based observations).
  void apply_condition_state(const AlarmEventConfig & cfg, const opcua::NodeId & condition_id,
                             const AlarmEventInput & input, uint16_t severity, const std::string & message,
                             const opcua::ByteString * event_id);

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
  // Shared raise/clear edge detector across all detection modes, keyed by
  // fault_code. Replaces the previous threshold-only last-state map.
  ros2_medkit::fault_detection::FaultTransitionTracker alarm_tracker_;

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

  // Throttle the warn-level log emitted from condition_refresh() failures.
  // Reset to false on each fresh subscribe in setup_event_subscriptions()
  // so a transient server error (BadMethodInvalid on cold-start, recovers
  // later) gets one log per connect, not one per re-subscribe attempt.
  bool condition_refresh_warned_{false};

  // Thread safety: must be set via set_poll_callback() before start().
  // Not modified after start(), so safe to read from the poll thread without a mutex.
  PollCallback poll_callback_;
};

}  // namespace ros2_medkit_gateway
