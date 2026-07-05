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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <open62541/types.h>
#include <rclcpp/logging.hpp>
#include <rcutils/logging.h>

namespace ros2_medkit_gateway {

namespace {
inline rclcpp::Logger opcua_poller_logger() {
  static auto logger = rclcpp::get_logger("opcua.poller");
  return logger;
}
}  // namespace

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
constexpr size_t kFieldConditionName = 9;  // issue #389 (multi-alarm identity)
constexpr size_t kAlarmFieldCount = 9;     // count of fixed alarm-state fields (indices 0-8)
// ConditionName is always appended at index kFieldConditionName; configured
// associated values follow at index kFieldConditionName + 1 onward.
constexpr size_t kFirstAssociatedValueField = kFieldConditionName + 1;

// Standard NodeIds for the types that *directly* define each field (open62541
// servers reject SAOs whose BrowsePath is inherited rather than direct).
constexpr uint32_t kBaseEventType = 2041;
constexpr uint32_t kConditionType = 2782;
constexpr uint32_t kAcknowledgeableConditionType = 2881;
constexpr uint32_t kAlarmConditionType = 2915;

std::vector<OpcuaClient::EventFieldSpec> build_alarm_event_select_specs(const AlarmEventConfig & cfg) {
  // Each clause carries the type that *directly* defines its first browse
  // segment - inheritance traversal is not honored by the open62541 server
  // validator (verified against 1.4.6 with FULL ns0).
  std::vector<OpcuaClient::EventFieldSpec> specs = {
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
      // Issue #389: ConditionName, used to route distinct conditions from one
      // source to distinct faults.
      {opcua::NodeId(0, kConditionType), {{0, "ConditionName"}}, UA_ATTRIBUTEID_VALUE},
  };
  // Issue #389: configured associated values (e.g. Siemens SD_1..SD_n) as
  // BaseEventType properties.
  for (const auto & av : cfg.associated_values) {
    specs.push_back({opcua::NodeId(0, kBaseEventType), {{av.namespace_index, av.name}}, UA_ATTRIBUTEID_VALUE});
  }
  return specs;
}

// Render an arbitrary scalar event field value to a short string for inclusion
// in a fault description (associated values can be of any builtin type).
std::string variant_to_display(const opcua::Variant & v) {
  if (v.isEmpty()) {
    return "";
  }
  if (v.isType<opcua::LocalizedText>()) {
    return std::string(v.getScalarCopy<opcua::LocalizedText>().getText());
  }
  if (v.isType<opcua::String>()) {
    return std::string(v.getScalarCopy<opcua::String>());
  }
  if (v.isType<bool>()) {
    return v.getScalarCopy<bool>() ? "true" : "false";
  }
  if (v.isType<int16_t>()) {
    return std::to_string(v.getScalarCopy<int16_t>());
  }
  if (v.isType<uint16_t>()) {
    return std::to_string(v.getScalarCopy<uint16_t>());
  }
  if (v.isType<int32_t>()) {
    return std::to_string(v.getScalarCopy<int32_t>());
  }
  if (v.isType<uint32_t>()) {
    return std::to_string(v.getScalarCopy<uint32_t>());
  }
  if (v.isType<int64_t>()) {
    return std::to_string(v.getScalarCopy<int64_t>());
  }
  if (v.isType<float>()) {
    return std::to_string(v.getScalarCopy<float>());
  }
  if (v.isType<double>()) {
    return std::to_string(v.getScalarCopy<double>());
  }
  return "";
}

std::string variant_to_string_scalar(const opcua::Variant & v) {
  if (v.isType<opcua::String>()) {
    return std::string(v.getScalarCopy<opcua::String>());
  }
  if (v.isType<opcua::LocalizedText>()) {
    return std::string(v.getScalarCopy<opcua::LocalizedText>().getText());
  }
  return "";
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

  event_monitored_item_ids_.clear();

  for (const auto & cfg : node_map_.event_alarms()) {
    // Per-source select specs so each source can carry its own associated
    // values (issue #389) in addition to the fixed alarm-state fields.
    const auto select_specs = build_alarm_event_select_specs(cfg);
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

  // Replay conditions that were already active before this (re)subscribe so a
  // reconnect / restart does not drop the live fault set. Strategy-driven:
  // ConditionRefresh (Part 9 §5.5.7) and/or a read-based fallback (issue
  // #389) for servers that reject the method.
  if (!event_monitored_item_ids_.empty()) {
    replay_active_conditions();
  }
}

ConditionReplayStrategy OpcuaPoller::parse_replay_strategy(const std::string & name) {
  std::string n = name;
  std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (n == "method") {
    return ConditionReplayStrategy::Method;
  }
  if (n == "read" || n == "read_fallback" || n == "readfallback") {
    return ConditionReplayStrategy::Read;
  }
  if (n == "off" || n == "none" || n == "disabled") {
    return ConditionReplayStrategy::Off;
  }
  return ConditionReplayStrategy::Auto;
}

void OpcuaPoller::warn_operator(const std::string & msg) {
  if (config_.log_warn) {
    config_.log_warn(msg);
  } else {
    RCLCPP_WARN(opcua_poller_logger(), "%s", msg.c_str());
  }
}

void OpcuaPoller::replay_active_conditions() {
  switch (config_.condition_replay_strategy) {
    case ConditionReplayStrategy::Off:
      return;
    case ConditionReplayStrategy::Method:
      if (!try_condition_refresh() && !condition_refresh_warned_) {
        warn_operator(
            "OPC-UA ConditionRefresh rejected and replay strategy is 'method'; active conditions "
            "will NOT be replayed on reconnect with this server. Live transitions still flow. See issue #389.");
        condition_refresh_warned_ = true;
      } else if (condition_refresh_warned_) {
        condition_refresh_warned_ = false;
      }
      return;
    case ConditionReplayStrategy::Read:
      read_fallback_replay();
      return;
    case ConditionReplayStrategy::Auto:
      if (try_condition_refresh()) {
        condition_refresh_warned_ = false;
        return;
      }
      if (!condition_refresh_warned_) {
        warn_operator(
            "OPC-UA ConditionRefresh rejected; using read-based active-condition replay fallback (issue #389).");
        condition_refresh_warned_ = true;
      }
      read_fallback_replay();
      return;
  }
}

bool OpcuaPoller::try_condition_refresh() {
  // Per Part 9 §5.5.7 ConditionRefresh is a Method of the ConditionType, so the
  // Call's ObjectId MUST be the well-known ConditionType node (i=2782), NOT the
  // Server object (i=2253). Calling it on the Server object is the root cause of
  // BadNodeIdUnknown / BadMethodInvalid rejections on conformant servers
  // (including Siemens S7-1500, which documents ConditionRefresh as supported -
  // only ConditionRefresh2 is unsupported). The method itself is i=3875
  // (ConditionType.ConditionRefresh) and takes the SubscriptionId argument.
  static constexpr uint32_t kConditionTypeId = 2782;
  static constexpr uint32_t kConditionRefreshMethodId = 3875;
  std::vector<opcua::Variant> args;
  args.push_back(opcua::Variant::fromScalar(static_cast<uint32_t>(event_subscription_id_)));
  auto result =
      client_.call_method(opcua::NodeId(0, kConditionTypeId), opcua::NodeId(0, kConditionRefreshMethodId), args);
  return result.has_value();
}

OpcuaPoller::ReadReplayDisposition
OpcuaPoller::classify_read_snapshot(const OpcuaClient::ConditionStateSnapshot & snap) {
  // Transient read/browse failure: present but unreliable. Keep it (so reconcile
  // does not clear) but do not feed an untrustworthy state into the machine.
  if (snap.state_read_failed) {
    return ReadReplayDisposition::KeepOnly;
  }
  // EnabledState=false: a Disabled condition is never active and is never
  // replayed by ConditionRefresh (Part 9 §5.7.2). Exclude it entirely so a
  // stale tracked fault can reconcile away.
  if (!snap.enabled_state) {
    return ReadReplayDisposition::Skip;
  }
  // A reliably-read ActiveState=true condition is always interesting and must
  // never be Skipped on the Retain check below (issue #478). Retain is an
  // optional node that can read false by default when it is unreadable this
  // scan; Skipping a live, reliably-active condition on that basis would drop
  // it from ``seen`` and let reconcile clear a still-active fault on a modeled
  // source. ConditionRefresh always replays an active condition, so mirror it.
  if (snap.active_state) {
    return ReadReplayDisposition::Feed;
  }
  // Retain mirrors ConditionRefresh's interesting-state filter (Part 9
  // §5.5.2): only Retain==true conditions are replayed. An inactive condition
  // that has gone quiet (Retain==false) is no longer of interest.
  if (!snap.retain) {
    return ReadReplayDisposition::Skip;
  }
  return ReadReplayDisposition::Feed;
}

void OpcuaPoller::read_fallback_replay() {
  // Browse each configured alarm source, read its conditions' current state,
  // and drive the same state machine as live events. Conditions that are no
  // longer active/interesting are reconciled (cleared) afterwards - but only
  // for sources whose scan succeeded AND that positively expose Condition
  // instance nodes, so neither a transient disconnect nor an EventNotifier-only
  // server (S7-1500) can falsely clear live alarms (issue #478).
  //
  // This fallback assumes each AlarmCondition instance is browseable from its
  // alarm source (hierarchical child or via HasCondition). Validated against
  // the open62541 reference server; NOT yet validated against a real Siemens
  // S7-1500, whose condition-instance address-space layout must be confirmed
  // before this path can be relied on there (use ConditionRefresh on Siemens).
  std::set<std::string> seen;
  std::set<std::string> failed_sources;
  for (const auto & cfg : node_map_.event_alarms()) {
    bool scan_ok = false;
    auto conditions = client_.read_source_conditions(cfg.source_node_id, &scan_ok);
    if (!scan_ok) {
      // Browse failed (disconnect / Bad status). Record the source so reconcile
      // does not clear its still-tracked conditions on an empty/partial scan.
      failed_sources.insert(cfg.source_node_id_str);
      continue;
    }
    const bool has_reliable_condition =
        std::any_of(conditions.begin(), conditions.end(), [](const OpcuaClient::ConditionStateSnapshot & s) {
          return !s.state_read_failed;
        });
    if (has_reliable_condition) {
      // The source positively exposes at least one RELIABLY-read Condition
      // instance node -> read-fallback is viable here and reconcile may clear
      // stale faults for it.
      read_modeled_sources_.insert(cfg.source_node_id_str);
      read_unsupported_warned_sources_.erase(cfg.source_node_id_str);
    } else if (read_modeled_sources_.find(cfg.source_node_id_str) == read_modeled_sources_.end()) {
      // Either zero Condition instance nodes, or every node this scan was a
      // transient (unreliable) read - and this source has never yielded a
      // reliably-read condition. Treat an all-transient scan exactly like an
      // empty/unsupported scan: an EventNotifier-only server (e.g. S7-1500)
      // yields zero condition nodes, and a lone transient browse error on a
      // non-condition child must NOT flip the source into 'modeled' (which
      // would let reconcile wipe the live event-fault set, whose ConditionIds
      // are never in ``seen``). Do NOT clear its tracked faults; warn once.
      if (read_unsupported_warned_sources_.insert(cfg.source_node_id_str).second) {
        warn_operator(
            "OPC-UA read-based active-condition replay found no Condition instance nodes under source '" +
            cfg.source_node_id_str +
            "'. This server appears to expose alarms via EventNotifier only (e.g. Siemens S7-1500), so the "
            "read fallback cannot recover the active set; tracked faults are preserved (not cleared) and live "
            "transitions still flow. Prefer ConditionRefresh ('method' or 'auto'). See issue #478.");
      }
    }
    for (const auto & snap : conditions) {
      const std::string cid = snap.condition_id.toString();
      const ReadReplayDisposition disp = classify_read_snapshot(snap);
      if (disp == ReadReplayDisposition::Skip) {
        // Not interesting (Disabled or Retain=false): leave out of ``seen`` so
        // any stale tracked fault reconciles away.
        continue;
      }
      // Mark every interesting/observed condition as seen, even if it matches
      // no mapping below. Otherwise reconcile_after_read would treat a live (but
      // unmapped) condition as "cleared while offline" and wrongly clear it.
      seen.insert(cid);
      if (disp == ReadReplayDisposition::KeepOnly) {
        // Present but unreliable (transient read failure): kept, not fed.
        continue;
      }

      AlarmEventInput input;
      input.enabled_state = snap.enabled_state;
      input.active_state = snap.active_state;
      input.acked_state = snap.acked_state;
      input.confirmed_state = snap.confirmed_state;
      // Read-based replay never observes a historical branch (we read the
      // current branch state directly) and shelving is folded into the live
      // event path; treat read snapshots as live, non-shelved state.
      input.shelved = false;
      input.branch_id_present = false;

      // Resolve identity to a specific fault. EventType is not available from
      // a state read, so event_type-only mappings will not match here; routing
      // by condition_name / source_node / message still works.
      ResolvedAlarm resolved =
          NodeMap::resolve_alarm(cfg, snap.condition_name, cfg.source_node_id_str, /*event_type=*/"", snap.message);
      if (!resolved.matched) {
        continue;
      }
      AlarmEventConfig eff = cfg;
      eff.fault_code = resolved.fault_code;
      eff.severity_override = resolved.severity_override;
      eff.message_override = resolved.message_override;

      apply_condition_state(eff, snap.condition_id, input, snap.severity, snap.message, /*event_id=*/nullptr);
    }
  }
  reconcile_after_read(seen, failed_sources, read_modeled_sources_);
}

bool OpcuaPoller::should_clear_condition(SovdAlarmStatus last_status, const std::string & condition_id,
                                         const std::string & source_id, const std::set<std::string> & seen,
                                         const std::set<std::string> & failed_sources,
                                         const std::set<std::string> & modeled_sources) {
  const bool was_active = (last_status == SovdAlarmStatus::Confirmed || last_status == SovdAlarmStatus::Healed);
  if (!was_active) {
    return false;
  }
  // Never clear a condition whose source scan failed this cycle: its absence
  // from ``seen`` means "not scanned", not "no longer active".
  if (failed_sources.find(source_id) != failed_sources.end()) {
    return false;
  }
  // Safety gate (issue #478): only clear when the source is positively known to
  // model Condition instances as address-space nodes. An EventNotifier-only
  // server (S7-1500) yields zero condition nodes, so an empty read scan must
  // never wipe its tracked faults - this is the single most important guard.
  if (modeled_sources.find(source_id) == modeled_sources.end()) {
    return false;
  }
  return seen.find(condition_id) == seen.end();
}

bool OpcuaPoller::should_clear_after_refresh(SovdAlarmStatus last_status, const std::string & condition_id,
                                             const std::set<std::string> & seen) {
  const bool was_active = (last_status == SovdAlarmStatus::Confirmed || last_status == SovdAlarmStatus::Healed);
  if (!was_active) {
    return false;
  }
  return seen.find(condition_id) == seen.end();
}

void OpcuaPoller::reconcile_after_read(const std::set<std::string> & seen, const std::set<std::string> & failed_sources,
                                       const std::set<std::string> & modeled_sources) {
  std::vector<AlarmEventDelivery> clears;
  {
    std::unique_lock lock(conditions_mutex_);
    for (auto & [cid, runtime] : conditions_) {
      if (should_clear_condition(runtime.last_status, cid, runtime.source_id, seen, failed_sources, modeled_sources)) {
        // Tracked as active but absent from a successful read scan -> the alarm
        // cleared while we were offline. Clear the SOVD fault.
        runtime.last_status = SovdAlarmStatus::Cleared;
        AlarmEventDelivery d;
        d.fault_code = runtime.fault_code;
        d.entity_id = runtime.entity_id;
        d.next_status = SovdAlarmStatus::Cleared;
        d.action = AlarmAction::ClearFault;
        d.condition_id = cid;
        clears.push_back(std::move(d));
      }
    }
  }
  dispatch_condition_clears(clears);
}

void OpcuaPoller::reconcile_after_refresh(const std::set<std::string> & seen) {
  std::vector<AlarmEventDelivery> clears;
  {
    std::unique_lock lock(conditions_mutex_);
    for (auto & [cid, runtime] : conditions_) {
      if (!should_clear_after_refresh(runtime.last_status, cid, seen)) {
        continue;
      }
      // Tracked as active but the completed ConditionRefresh burst did not
      // replay it -> the alarm cleared while we were offline. Clear the fault.
      runtime.last_status = SovdAlarmStatus::Cleared;
      AlarmEventDelivery d;
      d.fault_code = runtime.fault_code;
      d.entity_id = runtime.entity_id;
      d.next_status = SovdAlarmStatus::Cleared;
      d.action = AlarmAction::ClearFault;
      d.condition_id = cid;
      clears.push_back(std::move(d));
    }
  }
  dispatch_condition_clears(clears);
}

void OpcuaPoller::dispatch_condition_clears(const std::vector<AlarmEventDelivery> & clears) {
  if (clears.empty()) {
    return;
  }
  EventAlarmCallback cb_copy;
  {
    std::lock_guard cb_lock(event_alarm_callback_mutex_);
    cb_copy = event_alarm_callback_;
  }
  if (cb_copy) {
    for (const auto & d : clears) {
      cb_copy(d);
    }
  }
}

void OpcuaPoller::on_event(const AlarmEventConfig & cfg, const std::vector<opcua::Variant> & values,
                           const opcua::NodeId & source_node, const opcua::NodeId & event_type,
                           const opcua::NodeId & condition_id) {
  RCLCPP_DEBUG_STREAM(opcua_poller_logger(), "on_event source_fault=" << cfg.fault_code
                                                                      << " event_type=" << event_type.toString()
                                                                      << " condition=" << condition_id.toString()
                                                                      << " values=" << values.size());
  // Detect ConditionRefresh bracketing per Part 9 §5.5.7. RefreshStart /
  // RefreshEnd carry no condition payload. Between them the server replays its
  // active (Retain=true) condition set; we accumulate every replayed
  // ConditionId in ``refresh_seen_`` so RefreshEnd can reconcile away tracked
  // conditions that were NOT replayed - they cleared while we were offline and
  // would otherwise stay latched Confirmed (issue #480).
  if (event_type.getNamespaceIndex() == 0 && event_type.getIdentifierType() == opcua::NodeIdType::Numeric) {
    auto numeric = event_type.getIdentifierAs<uint32_t>();
    if (numeric == kRefreshStartEventTypeId) {
      refresh_seen_.clear();
      condition_refresh_in_progress_.store(true, std::memory_order_release);
      return;
    }
    if (numeric == kRefreshEndEventTypeId) {
      condition_refresh_in_progress_.store(false, std::memory_order_release);
      reconcile_after_refresh(refresh_seen_);
      refresh_seen_.clear();
      return;
    }
  }

  // A condition (re)reported inside a refresh burst is still active; remember it
  // so RefreshEnd does not clear it. Recorded before the field-count bail below
  // so even a partial-field replay counts as "seen".
  if (condition_refresh_in_progress_.load(std::memory_order_acquire)) {
    refresh_seen_.insert(condition_id.toString());
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

  uint16_t severity = variant_or<uint16_t>(values[kFieldSeverity], 0);
  std::string message = variant_to_localized_text(values[kFieldMessage]);

  // Issue #389: resolve this event's identity (ConditionName / SourceNode /
  // EventType) to a specific fault via the config's mappings. This lets one
  // OPC-UA source emit many distinct conditions that surface as distinct
  // SOVD faults with their own text.
  std::string condition_name;
  if (values.size() > kFieldConditionName) {
    condition_name = variant_to_string_scalar(values[kFieldConditionName]);
  }
  ResolvedAlarm resolved =
      NodeMap::resolve_alarm(cfg, condition_name, source_node.toString(), event_type.toString(), message);
  if (!resolved.matched) {
    // No mapping and no source-level fault_code applies to this condition.
    RCLCPP_DEBUG_STREAM(opcua_poller_logger(),
                        "on_event: no fault mapping for condition_name='" << condition_name << "' - ignoring");
    return;
  }

  // Append configured associated values (e.g. SD_1..SD_n) to the description.
  for (size_t i = 0; i < cfg.associated_values.size(); ++i) {
    const size_t idx = kFirstAssociatedValueField + i;
    if (idx >= values.size()) {
      break;
    }
    const std::string rendered = variant_to_display(values[idx]);
    if (rendered.empty()) {
      continue;
    }
    if (!message.empty()) {
      message += "; ";
    }
    message += cfg.associated_values[i].label + "=" + rendered;
  }

  // Build the effective config for this specific event (resolved fault_code +
  // overrides) so apply_condition_state tracks the right fault_code / entity.
  AlarmEventConfig eff = cfg;
  eff.fault_code = resolved.fault_code;
  eff.severity_override = resolved.severity_override;
  eff.message_override = resolved.message_override;

  // Capture the live EventId for spec-compliant Acknowledge / Confirm.
  opcua::ByteString event_id;
  bool have_event_id = false;
  if (values[kFieldEventId].isType<opcua::ByteString>()) {
    event_id = values[kFieldEventId].getScalarCopy<opcua::ByteString>();
    have_event_id = true;
  }

  apply_condition_state(eff, condition_id, input, severity, message, have_event_id ? &event_id : nullptr);
}

void OpcuaPoller::apply_condition_state(const AlarmEventConfig & cfg, const opcua::NodeId & condition_id,
                                        const AlarmEventInput & input, uint16_t severity, const std::string & message,
                                        const opcua::ByteString * event_id) {
  // Key the runtime map on the ConditionId string form so distinct condition
  // instances within the same event source remain separate (Part 9
  // §5.5.2.13). Shared by the live event path and the read-based replay.
  const std::string condition_id_str = condition_id.toString();

  AlarmEventDelivery delivery;
  bool dispatch = false;
  {
    std::unique_lock lock(conditions_mutex_);
    auto it = conditions_.find(condition_id_str);
    if (it == conditions_.end()) {
      ConditionRuntime fresh;
      fresh.condition_id = condition_id;
      fresh.entity_id = cfg.entity_id;
      fresh.fault_code = cfg.fault_code;
      fresh.source_id = cfg.source_node_id_str;
      fresh.last_status = SovdAlarmStatus::Suppressed;
      it = conditions_.emplace(condition_id_str, std::move(fresh)).first;
    }
    const SovdAlarmStatus prev_status = it->second.last_status;

    if (event_id != nullptr) {
      it->second.latest_event_id = *event_id;
    }

    auto outcome = AlarmStateMachine::compute(prev_status, input, config_.require_confirm_for_clear);
    RCLCPP_DEBUG_STREAM(opcua_poller_logger(),
                        "state machine: enabled="
                            << input.enabled_state << " active=" << input.active_state << " acked=" << input.acked_state
                            << " confirmed=" << input.confirmed_state << " shelved=" << input.shelved
                            << " branch=" << input.branch_id_present << " prev=" << static_cast<int>(prev_status)
                            << " action=" << static_cast<int>(outcome.action));
    it->second.last_status = outcome.next_status;

    if (outcome.action == AlarmAction::NoOp) {
      // Redundant observation (same as last status) - drop while still holding
      // the lock to avoid a needless callback round-trip.
      return;
    }

    delivery.fault_code = cfg.fault_code;
    delivery.entity_id = cfg.entity_id;
    delivery.next_status = outcome.next_status;
    delivery.action = outcome.action;
    delivery.severity = severity;
    delivery.severity_override = cfg.severity_override;
    delivery.message = cfg.message_override.empty() ? message : cfg.message_override;
    delivery.condition_id = condition_id_str;
    dispatch = true;
  }  // conditions_mutex_ released before invoking the user callback

  if (!dispatch) {
    return;
  }
  EventAlarmCallback cb_copy;
  {
    std::lock_guard cb_lock(event_alarm_callback_mutex_);
    cb_copy = event_alarm_callback_;
  }
  if (cb_copy) {
    cb_copy(delivery);
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

bool OpcuaPoller::comms_lost_should_raise(bool enabled, bool already_raised,
                                          std::chrono::steady_clock::time_point down_since,
                                          std::chrono::steady_clock::time_point now,
                                          std::chrono::milliseconds debounce) {
  if (!enabled || already_raised) {
    return false;
  }
  return (now - down_since) >= debounce;
}

void OpcuaPoller::emit_comms_lost(bool active) {
  ros2_medkit::fault_detection::FaultSignal signal;
  signal.fault_code = "PLC_COMMS_LOST";
  signal.severity = config_.comms_lost_severity;
  signal.message = active ? ("OPC-UA connection lost to " + client_.endpoint_url())
                          : ("OPC-UA connection restored to " + client_.endpoint_url());
  signal.active = active;
  std::lock_guard<std::mutex> alarm_lock(alarm_mutex_);
  if (alarm_callback_) {
    alarm_callback_(node_map_.component_id(), signal);
  }
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

      // Issue #496: remember when the connection first went down so the
      // comms-lost fault only fires after a continuous debounce window.
      if (!comms_down_since_) {
        comms_down_since_ = std::chrono::steady_clock::now();
      }

      // Attempt reconnect with original config (preserves timeout, etc.)
      if (client_.connect(client_.current_config())) {
        reconnect_wait = config_.reconnect_interval;  // reset on success
        // Issue #496: connection restored - clear the comms-lost fault if it
        // was raised, then reset the debounce timer.
        if (comms_lost_raised_) {
          emit_comms_lost(/*active=*/false);
          comms_lost_raised_ = false;
        }
        comms_down_since_.reset();
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
        // Issue #496: still down - raise the component-scoped comms-lost fault
        // once the debounce window has elapsed (idempotent via the raised flag).
        if (comms_lost_should_raise(config_.comms_lost_fault_enabled, comms_lost_raised_, *comms_down_since_,
                                    std::chrono::steady_clock::now(), config_.comms_lost_debounce)) {
          emit_comms_lost(/*active=*/true);
          comms_lost_raised_ = true;
        }
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
  namespace fd = ros2_medkit::fault_detection;

  auto detection_entries = node_map_.detection_entries();
  if (detection_entries.empty()) {
    return;
  }

  // Translate a polled OPC-UA value into the shared, protocol-agnostic value
  // type understood by the fault-detection evaluator.
  auto to_fd_value = [](const OpcuaValue & v) -> fd::Value {
    return std::visit(
        [](auto && val) -> fd::Value {
          using T = std::decay_t<decltype(val)>;
          if constexpr (std::is_same_v<T, bool>) {
            return fd::Value{val};
          } else if constexpr (std::is_same_v<T, std::string>) {
            return fd::Value{val};
          } else if constexpr (std::is_floating_point_v<T>) {
            return fd::Value{static_cast<double>(val)};
          } else {
            return fd::Value{static_cast<std::int64_t>(val)};
          }
        },
        v);
  };

  // Collect raise/clear edges while holding the snapshot mutex.
  struct FaultChange {
    std::string entity_id;
    fd::FaultSignal signal;
  };
  std::vector<FaultChange> changes;

  {
    std::lock_guard<std::mutex> snap_lock(snapshot_mutex_);

    for (const auto * entry : detection_entries) {
      auto it = snapshot_.values.find(entry->node_id_str);
      if (it == snapshot_.values.end()) {
        continue;
      }

      auto signals = fd::evaluate(to_fd_value(it->second), *entry->detection);
      for (const auto & s : signals) {
        snapshot_.alarms[s.fault_code] = s.active;
      }

      // Edge-detect raise/clear; only transitions reach the callback.
      for (auto & edge : alarm_tracker_.apply(signals)) {
        changes.push_back({entry->entity_id, std::move(edge)});
      }
    }
  }  // snapshot_mutex_ released

  // Fire callbacks outside of locks
  if (!changes.empty()) {
    std::lock_guard<std::mutex> alarm_lock(alarm_mutex_);
    if (alarm_callback_) {
      for (const auto & c : changes) {
        alarm_callback_(c.entity_id, c.signal);
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
