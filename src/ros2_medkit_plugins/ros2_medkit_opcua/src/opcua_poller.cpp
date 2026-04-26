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

#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <utility>

#include <open62541/types.h>

namespace ros2_medkit_gateway {

namespace {

// EventType NodeIds for ConditionRefresh bracketing per OPC-UA Part 9 §5.5.7
// (RefreshStartEventType i=2787, RefreshEndEventType i=2788).
constexpr uint32_t kRefreshStartEventTypeId = 2787;
constexpr uint32_t kRefreshEndEventTypeId = 2788;

// ShelvingState.CurrentState.Id NodeId for "Unshelved" state per §5.8.16.
// Anything other than this (TimedShelved=2930, OneShotShelved=2932) means
// the alarm is operator-suppressed.
constexpr uint32_t kShelvedStateUnshelved = 2929;

// EventFilter select-clause indices used by the AlarmCondition trampoline.
// Order MUST match build_alarm_event_select_specs() below; the trampoline
// reads positionally because OPC-UA does not return field names with the
// notification. ``ConditionId`` is delivered to the EventCallback as a
// separate parameter (auto-prepended by add_event_monitored_item per
// Part 9 §5.5.2.13), so it is not in the user_values vector.
constexpr size_t kFieldEventId = 0;
constexpr size_t kFieldSeverity = 1;
constexpr size_t kFieldMessage = 2;
constexpr size_t kFieldEnabledState = 3;
constexpr size_t kFieldActiveState = 4;
constexpr size_t kFieldAckedState = 5;
constexpr size_t kFieldConfirmedState = 6;
constexpr size_t kFieldShelvingState = 7;
constexpr size_t kFieldBranchId = 8;
constexpr size_t kAlarmFieldCount = 9;

// Standard NodeIds for the types that *directly* define each field (open62541
// servers reject SAOs whose BrowsePath is inherited rather than direct).
constexpr uint32_t kBaseEventType = 2041;
constexpr uint32_t kConditionType = 2782;
constexpr uint32_t kAcknowledgeableConditionType = 2881;
constexpr uint32_t kAlarmConditionType = 2915;

std::vector<OpcuaClient::EventFieldSpec> build_alarm_event_select_specs() {
  // Each clause carries the type that *directly* defines its first browse
  // segment - inheritance traversal is not honored by the open62541 server
  // validator (verified against 1.4.6 with FULL ns0).
  return {
      {opcua::NodeId(0, kBaseEventType), {{0, "EventId"}}, UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kBaseEventType), {{0, "Severity"}}, UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kBaseEventType), {{0, "Message"}}, UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kConditionType), {{0, "EnabledState"}, {0, "Id"}}, UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kAlarmConditionType), {{0, "ActiveState"}, {0, "Id"}}, UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kAcknowledgeableConditionType), {{0, "AckedState"}, {0, "Id"}}, UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kAcknowledgeableConditionType), {{0, "ConfirmedState"}, {0, "Id"}}, UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kAlarmConditionType),
       {{0, "ShelvingState"}, {0, "CurrentState"}, {0, "Id"}},
       UA_ATTRIBUTEID_VALUE},
      {opcua::NodeId(0, kConditionType), {{0, "BranchId"}}, UA_ATTRIBUTEID_VALUE},
  };
}

// Extract a scalar of type T from a Variant, returning the default if absent.
template <class T>
T variant_or(const opcua::Variant & v, T fallback) {
  if (v.isType<T>()) {
    return v.getScalarCopy<T>();
  }
  return fallback;
}

// Decode a LocalizedText event field to plain UTF-8 (Message uses LT).
std::string variant_to_localized_text(const opcua::Variant & v) {
  if (v.isType<opcua::LocalizedText>()) {
    auto lt = v.getScalarCopy<opcua::LocalizedText>();
    return std::string(lt.getText());
  }
  if (v.isType<opcua::String>()) {
    return std::string(v.getScalarCopy<opcua::String>());
  }
  return "";
}

}  // namespace

OpcuaPoller::OpcuaPoller(OpcuaClient & client, const NodeMap & node_map) : client_(client), node_map_(node_map) {
}

OpcuaPoller::~OpcuaPoller() {
  stop();
}

void OpcuaPoller::start(const PollerConfig & config) {
  if (running_.load()) {
    return;
  }

  config_ = config;
  running_ = true;

  // Try subscription mode first
  if (config_.prefer_subscriptions) {
    setup_subscriptions();
  }

  // Issue #386: subscribe to native AlarmConditionType events. Independent
  // of data-change subscriptions; runs whenever event_alarms are configured.
  if (!node_map_.event_alarms().empty()) {
    setup_event_subscriptions();
  }

  // Start poll/reconnect thread regardless (handles reconnection and poll fallback)
  poll_thread_ = std::thread(&OpcuaPoller::poll_loop, this);
}

void OpcuaPoller::stop() {
  running_ = false;
  stop_cv_.notify_all();
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
  client_.remove_subscriptions();
}

PollSnapshot OpcuaPoller::snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

std::optional<OpcuaValue> OpcuaPoller::get_value(const std::string & node_id_str) const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  auto it = snapshot_.values.find(node_id_str);
  if (it != snapshot_.values.end()) {
    return it->second;
  }
  return std::nullopt;
}

void OpcuaPoller::set_alarm_callback(AlarmChangeCallback callback) {
  std::lock_guard<std::mutex> lock(alarm_mutex_);
  alarm_callback_ = std::move(callback);
}

void OpcuaPoller::set_event_alarm_callback(EventAlarmCallback callback) {
  if (running_.load()) {
    throw std::logic_error("set_event_alarm_callback must be called before start()");
  }
  std::lock_guard<std::mutex> lock(event_alarm_callback_mutex_);
  event_alarm_callback_ = std::move(callback);
}

std::optional<ConditionRuntime> OpcuaPoller::lookup_condition(const std::string & entity_id,
                                                              const std::string & fault_code) const {
  std::shared_lock lock(conditions_mutex_);
  for (const auto & [cid_str, runtime] : conditions_) {
    if (runtime.entity_id == entity_id && runtime.fault_code == fault_code) {
      return runtime;
    }
  }
  return std::nullopt;
}

void OpcuaPoller::set_poll_callback(PollCallback callback) {
  if (running_.load()) {
    throw std::logic_error("set_poll_callback must be called before start()");
  }
  poll_callback_ = std::move(callback);
}

void OpcuaPoller::setup_subscriptions() {
  auto sub_id = client_.create_subscription(config_.subscription_interval_ms,
                                            [this](const std::string & nid, const OpcuaValue & val) {
                                              on_data_change(nid, val);
                                            });

  if (sub_id == 0) {
    using_subscriptions_ = false;
    return;
  }

  bool all_ok = true;
  for (const auto & entry : node_map_.entries()) {
    if (!client_.add_monitored_item(sub_id, entry.node_id)) {
      all_ok = false;
      break;
    }
  }

  if (all_ok) {
    using_subscriptions_ = true;
  } else {
    client_.remove_subscriptions();
    using_subscriptions_ = false;
  }
}

void OpcuaPoller::setup_event_subscriptions() {
  // Issue #386: one dedicated subscription for AlarmCondition events; uses
  // a default no-op data callback because we wire MIs of EVENTNOTIFIER
  // attribute, not data-change MIs. The EventCallback bound below is what
  // actually receives notifications. Caller (start() and the poll_loop
  // reconnect arm) is responsible for zeroing event_subscription_id_ before
  // calling - both currently do.
  event_subscription_id_ = client_.create_subscription(config_.subscription_interval_ms,
                                                       [](const std::string &, const OpcuaValue &) { /* no-op */ });
  if (event_subscription_id_ == 0) {
    return;
  }

  const auto select_specs = build_alarm_event_select_specs();
  event_monitored_item_ids_.clear();

  for (const auto & cfg : node_map_.event_alarms()) {
    // Capture cfg BY VALUE: even though range-for ``const auto & cfg`` binds
    // to a vector element that outlives the loop, an `&cfg`-by-reference
    // capture would chain through a local reference variable whose name
    // goes out of scope after each iteration. Defensible per current C++
    // semantics (the captured reference resolves to the underlying vector
    // element), but value capture is unambiguous and matches Copilot's
    // review feedback. AlarmEventConfig is a small struct of strings, so
    // copying is cheap.
    auto callback = [this, cfg](const std::vector<opcua::Variant> & values, const opcua::NodeId & source_node,
                                const opcua::NodeId & event_type, const opcua::NodeId & condition_id) {
      on_event(cfg, values, source_node, event_type, condition_id);
    };
    uint32_t mi_id =
        client_.add_event_monitored_item(event_subscription_id_, cfg.source_node_id, select_specs, std::move(callback));
    if (mi_id != 0) {
      event_monitored_item_ids_.push_back(mi_id);
    }
  }

  // Fire ConditionRefresh so the server pushes any conditions that fired
  // before our subscription started (Part 9 §5.5.7 mandates the bracketing
  // RefreshStartEvent / RefreshEndEvent which we treat as ordinary events
  // tagged by EventType).
  if (!event_monitored_item_ids_.empty()) {
    condition_refresh();
  }
}

void OpcuaPoller::condition_refresh() {
  // Server object NodeId (i=2253) hosts the ConditionRefresh method
  // (i=3875 - per Part 9 §5.5.7). We use the standard NodeId; servers that
  // diverge from the spec (rare) will return BadMethodInvalid which is
  // logged but does not abort subscribing.
  static constexpr uint32_t kServerObjectId = 2253;
  static constexpr uint32_t kConditionRefreshMethodId = 3875;
  std::vector<opcua::Variant> args;
  args.push_back(opcua::Variant::fromScalar(static_cast<uint32_t>(event_subscription_id_)));
  auto result =
      client_.call_method(opcua::NodeId(0, kServerObjectId), opcua::NodeId(0, kConditionRefreshMethodId), args);
  if (!result.has_value()) {
    // Not fatal but operator-visible: when ConditionRefresh is rejected by
    // the server (BadMethodInvalid in open62541 v1.4.x, BadNotImplemented
    // on Siemens S7-1500, etc.) the gateway will not re-receive any active
    // conditions on reconnect; only live transitions surface in /faults.
    // Worth a single warn per connect so the operator knows their
    // alarm-replay-on-reconnect contract is broken with this PLC.
    if (!condition_refresh_warned_) {
      const std::string msg = "OPC-UA ConditionRefresh rejected (" + result.error().message +
                              "); active conditions will NOT be replayed on reconnect with this server. "
                              "Live transitions still flow. See issue #389.";
      if (config_.log_warn) {
        config_.log_warn(msg);
      } else {
        std::cerr << "[opcua_poller WARN] " << msg << std::endl;
      }
      condition_refresh_warned_ = true;
    }
  } else {
    // Reset the throttle: a successful refresh means the server is
    // cooperating again, so the next failure (e.g., after a restart of a
    // server with a different config) earns a fresh warn.
    condition_refresh_warned_ = false;
  }
}

void OpcuaPoller::on_event(const AlarmEventConfig & cfg, const std::vector<opcua::Variant> & values,
                           const opcua::NodeId & /*source_node*/, const opcua::NodeId & event_type,
                           const opcua::NodeId & condition_id) {
  std::cerr << "[opcua_poller] on_event fault=" << cfg.fault_code << " event_type=" << event_type.toString()
            << " condition=" << condition_id.toString() << " values=" << values.size() << std::endl;
  // Detect ConditionRefresh bracketing per Part 9 §5.5.7. The flag is for
  // diagnostics only; the state machine itself does not need to know
  // because RefreshStart / RefreshEnd notifications carry no condition
  // payload, so positional-empty values trip the early-return below.
  if (event_type.getNamespaceIndex() == 0 && event_type.getIdentifierType() == opcua::NodeIdType::Numeric) {
    auto numeric = event_type.getIdentifierAs<uint32_t>();
    if (numeric == kRefreshStartEventTypeId) {
      condition_refresh_in_progress_.store(true, std::memory_order_release);
      return;
    }
    if (numeric == kRefreshEndEventTypeId) {
      condition_refresh_in_progress_.store(false, std::memory_order_release);
      return;
    }
  }

  if (values.size() < kAlarmFieldCount) {
    // Server returned fewer fields than our filter requested - typical when
    // BadAttributeIdInvalid was returned for one or more select clauses.
    // We need at minimum the state fields to make a decision; bail to
    // avoid garbage transitions.
    return;
  }

  AlarmEventInput input;
  input.enabled_state = variant_or<bool>(values[kFieldEnabledState], true);
  input.active_state = variant_or<bool>(values[kFieldActiveState], false);
  input.acked_state = variant_or<bool>(values[kFieldAckedState], false);
  input.confirmed_state = variant_or<bool>(values[kFieldConfirmedState], false);

  // ShelvingState.CurrentState.Id is itself a NodeId (one of i=2929/2930/
  // 2932). Anything other than Unshelved => alarm is operator-suppressed.
  if (values[kFieldShelvingState].isType<opcua::NodeId>()) {
    auto shelv_state = values[kFieldShelvingState].getScalarCopy<opcua::NodeId>();
    // Treat as shelved only when ShelvingState/CurrentState/Id explicitly
    // points at TimedShelved (i=2930) or OneShotShelved (i=2932). A null /
    // unset / unknown Id is interpreted as Unshelved - some servers do not
    // initialize the Id property when the optional ShelvingState field is
    // attached, and we should not treat that as suppression.
    bool is_known_shelved =
        (shelv_state.getNamespaceIndex() == 0 && shelv_state.getIdentifierType() == opcua::NodeIdType::Numeric) &&
        (shelv_state.getIdentifierAs<uint32_t>() == 2930u || shelv_state.getIdentifierAs<uint32_t>() == 2932u);
    input.shelved = is_known_shelved;
  } else {
    input.shelved = false;
  }

  // BranchId is a NodeId; null branch maps to identifier i=0 in namespace
  // 0. Either an empty NodeId (server omitted) or the canonical null
  // counts as "live state" for the bridge.
  if (values[kFieldBranchId].isType<opcua::NodeId>()) {
    auto branch = values[kFieldBranchId].getScalarCopy<opcua::NodeId>();
    bool is_null = branch.getNamespaceIndex() == 0 && branch.getIdentifierType() == opcua::NodeIdType::Numeric &&
                   branch.getIdentifierAs<uint32_t>() == 0u;
    input.branch_id_present = !is_null;
  } else {
    input.branch_id_present = false;
  }

  // ``condition_id`` is supplied by add_event_monitored_item via the
  // auto-prepended SAO with empty BrowsePath + AttributeId=NodeId
  // (Part 9 §5.5.2.13). Key the runtime map on its stringForm so distinct
  // condition instances within the same event source remain separate.
  std::string condition_id_str = condition_id.toString();

  ConditionRuntime runtime_snapshot;
  SovdAlarmStatus prev_status;
  {
    std::unique_lock lock(conditions_mutex_);
    auto it = conditions_.find(condition_id_str);
    if (it == conditions_.end()) {
      ConditionRuntime fresh;
      fresh.condition_id = condition_id;
      fresh.entity_id = cfg.entity_id;
      fresh.fault_code = cfg.fault_code;
      fresh.last_status = SovdAlarmStatus::Suppressed;
      auto inserted = conditions_.emplace(condition_id_str, std::move(fresh));
      it = inserted.first;
    }
    prev_status = it->second.last_status;

    // Track the latest EventId for spec-compliant Acknowledge calls.
    if (values[kFieldEventId].isType<opcua::ByteString>()) {
      it->second.latest_event_id = values[kFieldEventId].getScalarCopy<opcua::ByteString>();
      std::cerr << "[opcua_poller] captured EventId len=" << it->second.latest_event_id.length() << " hex=";
      const auto * bytes = it->second.latest_event_id.data();
      for (size_t i = 0; i < std::min<size_t>(it->second.latest_event_id.length(), 16); ++i) {
        char buf[3];
        std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned>(bytes[i]) & 0xffu);
        std::cerr << buf;
      }
      std::cerr << std::endl;
    } else {
      std::cerr << "[opcua_poller] EventId field not a ByteString" << std::endl;
    }

    auto outcome = AlarmStateMachine::compute(prev_status, input);
    std::cerr << "[opcua_poller] state machine: enabled=" << input.enabled_state << " active=" << input.active_state
              << " acked=" << input.acked_state << " confirmed=" << input.confirmed_state
              << " shelved=" << input.shelved << " branch=" << input.branch_id_present
              << " prev=" << static_cast<int>(prev_status) << " action=" << static_cast<int>(outcome.action)
              << std::endl;
    it->second.last_status = outcome.next_status;
    runtime_snapshot = it->second;

    if (outcome.action == AlarmAction::NoOp) {
      // No downstream notification - drop while still holding the lock to
      // avoid a callback round-trip for redundant events.
      return;
    }

    AlarmEventDelivery delivery;
    delivery.fault_code = cfg.fault_code;
    delivery.entity_id = cfg.entity_id;
    delivery.next_status = outcome.next_status;
    delivery.action = outcome.action;
    delivery.severity = variant_or<uint16_t>(values[kFieldSeverity], 0);
    if (!cfg.message_override.empty()) {
      delivery.message = cfg.message_override;
    } else {
      delivery.message = variant_to_localized_text(values[kFieldMessage]);
    }
    delivery.condition_id = condition_id_str;

    // Release lock BEFORE invoking user callback (which may take its own
    // locks or block on a ROS service - we must not hold conditions_mutex_
    // across that).
    lock.unlock();

    EventAlarmCallback cb_copy;
    {
      std::lock_guard cb_lock(event_alarm_callback_mutex_);
      cb_copy = event_alarm_callback_;
    }
    std::cerr << "[opcua_poller] dispatching action=" << static_cast<int>(delivery.action)
              << " cb_set=" << (cb_copy ? 1 : 0) << std::endl;
    if (cb_copy) {
      cb_copy(delivery);
    }
  }
}

void OpcuaPoller::on_data_change(const std::string & node_id, const OpcuaValue & value) {
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.values[node_id] = value;
    snapshot_.timestamp = std::chrono::system_clock::now();
    snapshot_.connected = true;
    snapshot_.poll_count++;
  }
  evaluate_alarms();
}

void OpcuaPoller::poll_loop() {
  auto reconnect_wait = config_.reconnect_interval;
  constexpr auto max_reconnect_wait = std::chrono::milliseconds(60000);

  while (running_.load()) {
    // Handle reconnection
    if (!client_.is_connected()) {
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.connected = false;
      }

      // Attempt reconnect with original config (preserves timeout, etc.)
      if (client_.connect(client_.current_config())) {
        reconnect_wait = config_.reconnect_interval;  // reset on success
        if (config_.prefer_subscriptions) {
          setup_subscriptions();
        }
        // Issue #386: re-subscribe to AlarmCondition events after reconnect
        // and re-fire ConditionRefresh so we recover any conditions that
        // changed state while we were offline. The OpcuaClient's
        // generation counter has already advanced (incremented in
        // disconnect()/maybe_mark_disconnected), so any stale event
        // callbacks captured from the previous subscription are filtered
        // out by the trampoline before re-subscription registers fresh
        // contexts.
        event_subscription_id_ = 0;
        event_monitored_item_ids_.clear();
        if (!node_map_.event_alarms().empty()) {
          setup_event_subscriptions();
        }
      } else {
        // Exponential backoff capped at 60s. condition_variable so stop() wakes immediately.
        {
          std::unique_lock<std::mutex> lock(stop_mutex_);
          stop_cv_.wait_for(lock, reconnect_wait, [this] {
            return !running_.load();
          });
        }
        reconnect_wait = std::min(reconnect_wait * 2, max_reconnect_wait);
        continue;
      }
    }

    // Poll if not using subscriptions, or as a health check
    if (!using_subscriptions_.load() || !client_.is_connected()) {
      do_poll();
    }

    // Issue #386: dispatch incoming AlarmCondition notifications. open62541's
    // client only delivers subscription callbacks during a runIterate (or
    // any sync API call). When the YAML has only event_alarms (no scalar
    // ``nodes:``) do_poll() does nothing, so we explicitly pump iterate
    // here to keep events flowing. Cheap when there are no pending
    // notifications; bounded by the timeout.
    if (event_subscription_id_ != 0) {
      client_.run_iterate(50);
    }

    // Fire poll callback for value bridging.
    // Called every cycle regardless of transport mode (poll or subscription)
    // so that ROS 2 publishers always receive updates.
    if (poll_callback_) {
      poll_callback_(snapshot());
    }

    // Sleep for poll interval. condition_variable so stop() wakes immediately.
    {
      std::unique_lock<std::mutex> lock(stop_mutex_);
      stop_cv_.wait_for(lock, config_.poll_interval, [this] {
        return !running_.load();
      });
    }
  }
}

void OpcuaPoller::do_poll() {
  std::vector<opcua::NodeId> node_ids;
  node_ids.reserve(node_map_.entries().size());
  for (const auto & entry : node_map_.entries()) {
    node_ids.push_back(entry.node_id);
  }

  auto results = client_.read_values(node_ids);

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.timestamp = std::chrono::system_clock::now();
    snapshot_.connected = client_.is_connected();
    snapshot_.poll_count++;

    for (const auto & r : results) {
      if (r.good) {
        snapshot_.values[r.node_id] = r.value;
      } else {
        snapshot_.error_count++;
      }
    }
  }

  evaluate_alarms();
}

void OpcuaPoller::evaluate_alarms() {
  auto alarm_entries = node_map_.alarm_entries();
  if (alarm_entries.empty()) {
    return;
  }

  // Collect state changes while holding snapshot mutex
  struct AlarmChange {
    std::string fault_code;
    AlarmConfig config;
    bool active;
  };
  std::vector<AlarmChange> changes;

  {
    std::lock_guard<std::mutex> snap_lock(snapshot_mutex_);

    for (const auto * entry : alarm_entries) {
      auto it = snapshot_.values.find(entry->node_id_str);
      if (it == snapshot_.values.end()) {
        continue;
      }

      const auto & alarm = *entry->alarm;
      bool active = false;

      std::visit(
          [&active, &alarm](auto && val) {
            using T = std::decay_t<decltype(val)>;
            if constexpr (std::is_same_v<T, bool>) {
              active = val;
            } else if constexpr (std::is_arithmetic_v<T>) {
              double dval = static_cast<double>(val);
              if (alarm.above_threshold) {
                active = dval > alarm.threshold;
              } else {
                active = dval < alarm.threshold;
              }
            }
          },
          it->second);

      auto & prev_state = alarm_states_[alarm.fault_code];
      snapshot_.alarms[alarm.fault_code] = active;

      if (active != prev_state) {
        prev_state = active;
        changes.push_back({alarm.fault_code, alarm, active});
      }
    }
  }  // snapshot_mutex_ released

  // Fire callbacks outside of locks
  if (!changes.empty()) {
    std::lock_guard<std::mutex> alarm_lock(alarm_mutex_);
    if (alarm_callback_) {
      for (const auto & c : changes) {
        alarm_callback_(c.fault_code, c.config, c.active);
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
