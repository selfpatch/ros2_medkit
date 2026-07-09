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

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <open62541pp/open62541pp.hpp>

#include "ros2_medkit_fault_detection/fault_detection.hpp"

namespace ros2_medkit_gateway {

/// Threshold-mode alarm configuration for a node that maps to a SOVD fault.
/// Retained for backward compatibility with the original ``alarm:`` node-map
/// block; at load time it is also lowered to a shared
/// ``fault_detection::ThresholdRule`` (see ``NodeMapEntry::detection``) so the
/// runtime evaluator is the same one used by status-bit and enum modes.
struct AlarmConfig {
  std::string fault_code;
  std::string severity;  // "ERROR", "WARNING", "INFO"
  std::string message;
  double threshold{0.0};
  bool above_threshold{true};  // true = alarm when value > threshold
};

/// Reference to one extra OPC-UA event field to select and surface as an
/// associated value (issue #389). Used for vendor associated values such as
/// Siemens Program_Alarm ``SD_1``..``SD_n``. ``label`` is what appears in the
/// fault description; ``name``/``namespace_index`` address the event field via
/// a SimpleAttributeOperand on BaseEventType.
struct AssociatedValueRef {
  uint16_t namespace_index{0};
  std::string name;
  std::string label;  // defaults to ``name`` when unset
};

/// One condition-identity mapping inside an ``event_alarms`` source (issue
/// #389). A single OPC-UA source (e.g. the Server object as a catch-all, or a
/// real owning Object) can emit many distinct conditions; each mapping routes
/// a subset to its own SOVD fault. Match fields are AND-combined; an empty
/// match field is a wildcard. ``match_condition_name`` / ``match_source_node``
/// / ``match_event_type`` are equality matches; ``match_message`` is a
/// case-sensitive substring match on the event Message. The first mapping whose
/// non-empty match fields all match the observed event wins (declaration order
/// = precedence).
struct AlarmMapping {
  std::string match_condition_name;  // ConditionType.ConditionName (empty = any)
  std::string match_source_node;     // event SourceNode id string (empty = any)
  std::string match_event_type;      // event EventType id string (empty = any)
  std::string match_message;         // Message substring (empty = any); case-sensitive contains

  std::string fault_code;
  std::string severity_override;
  std::string message_override;
};

/// Configuration for a native OPC-UA AlarmConditionType event subscription
/// (issue #386). The plugin subscribes to events emitted from
/// ``alarm_source`` and bridges them through ``AlarmStateMachine`` into
/// SOVD faults. Mutually exclusive with the threshold-based ``AlarmConfig``
/// on a single ``NodeMapEntry``.
struct AlarmEventConfig {
  /// OPC-UA NodeId of the source node emitting the AlarmConditionType
  /// events (typically the parent Object that owns the condition, e.g.
  /// "ns=2;s=Tank.Pressure" or the Server object for system-wide alarms).
  std::string source_node_id_str;
  opcua::NodeId source_node_id;

  /// SOVD entity that should host the resulting fault.
  std::string entity_id;

  /// Source-level / fallback SOVD fault code (e.g. ``PLC_OVERPRESSURE``).
  /// Used when no ``mappings`` entry matches an observed event. May be empty
  /// when every alarm is routed through ``mappings``.
  std::string fault_code;

  /// Optional severity override. When empty, ``AlarmStateMachine`` derives
  /// the SOVD severity bucket from the event's ``Severity`` (1-1000) per
  /// the convention documented in design/index.rst.
  std::string severity_override;

  /// Optional friendly message override; falls back to the event's
  /// ``Message`` field when empty.
  std::string message_override;

  /// Issue #389: per-condition-identity mappings (multi-alarm). Resolved in
  /// declaration order; first match wins, falling back to the source-level
  /// ``fault_code`` above.
  std::vector<AlarmMapping> mappings;

  /// Issue #389: extra event fields to append to the fault description.
  std::vector<AssociatedValueRef> associated_values;
};

/// Result of resolving an observed event against an ``AlarmEventConfig``
/// (issue #389). ``matched`` is false when neither a mapping nor the
/// source-level fault_code applies (the event should be ignored).
struct ResolvedAlarm {
  std::string fault_code;
  std::string severity_override;
  std::string message_override;
  bool matched{false};
};

/// OPC-UA Severity (1-1000) -> SOVD severity bucket thresholds for
/// ``auto_alarms`` (zero-config native A&C). Mirrors the fixed bands
/// ``OpcuaPlugin::map_severity`` uses for explicit ``event_alarms``
/// (>=801 CRITICAL, >=501 ERROR, >=201 WARNING, else INFO) but is
/// per-deployment overridable via ``auto_alarms.severity_bands`` since a PLC
/// vendor's own severity convention need not match ours.
struct AutoAlarmsSeverityBands {
  uint16_t critical_min{801};
  uint16_t error_min{501};
  uint16_t warning_min{201};
};

/// Zero-config native OPC-UA Alarms & Conditions (completes the #509
/// discovery -> #510 auto_browse -> native faults story with no per-alarm
/// ``event_alarms`` mapping). When enabled, the plugin subscribes to
/// ``source_node_id`` (default the Server object, ``i=2253``) for
/// AlarmConditionType events and derives a fault for every condition it does
/// not already know from an explicit ``event_alarms`` mapping (issue #389
/// mappings always take precedence - see ``NodeMap::resolve_alarm``).
struct AutoAlarmsConfig {
  bool enabled{false};

  /// EventNotifier source to subscribe. Default is the Server object
  /// (``i=2253``, "system-wide" catch-all): the natural zero-config choice
  /// since Program_Alarm / ProDiag style conditions on Siemens, Beckhoff and
  /// CodeSys controllers surface there without the operator having to find
  /// the owning Object first.
  std::string source_node_id_str = "i=2253";
  opcua::NodeId source_node_id;

  /// SOVD entity that hosts an auto-derived fault when its SourceNode is not
  /// a known node-map entry. Defaults to the node map's ``component_id`` at
  /// load time (see ``NodeMap::load``). When the event's SourceNode matches
  /// an existing ``nodes:`` entry, the fault is hosted on THAT entry's
  /// entity instead (a more specific home than this fallback).
  std::string entity_id;

  /// When true (default), an auto-derived alarm clears as soon as
  /// ActiveState becomes false, bypassing BOTH halves of the
  /// Acknowledge/Confirm gate that otherwise applies to native alarms
  /// (``AlarmStateMachine``'s clear rule is ``acked && (confirmed ||
  /// !require_confirm_for_clear)`` - dropping only the Confirm half is not
  /// enough, since a zero-config alarm has no SOVD ``acknowledge_fault``
  /// operation registered to ever set Acked either). Zero-config is meant to
  /// "just work" without an operator workflow, and several PLCs (e.g.
  /// Siemens S7-1500) do not implement the optional Confirm transition at
  /// all - without this, such an alarm would latch forever. Set false to
  /// inherit the poller's normal ``require_confirm_for_clear`` gating
  /// (Acknowledge still required) for auto-derived alarms too.
  bool auto_clear{true};

  AutoAlarmsSeverityBands severity_bands;

  /// Case-sensitive substring filters against ConditionName / SourceName /
  /// Message. ``exclude`` is checked first: any match drops the event
  /// entirely (used to filter Siemens Server-object system messages such as
  /// "CPU not in RUN" that are not real alarms). ``include`` is then checked
  /// only if non-empty: the event is dropped unless at least one pattern
  /// matches. Both empty (the default) admits every event.
  std::vector<std::string> include_patterns;
  std::vector<std::string> exclude_patterns;
};

/// Mapping entry: OPC-UA NodeId -> SOVD entity data point
struct NodeMapEntry {
  std::string node_id_str;          // OPC-UA node ID string (e.g., "ns=1;s=TankLevel")
  opcua::NodeId node_id;            // Parsed NodeId
  std::string entity_id;            // SOVD entity this belongs to (e.g., "tank_process")
  std::string data_name;            // Data point name (e.g., "tank_level")
  std::string display_name;         // Human-readable name
  std::string unit;                 // Unit of measurement (e.g., "mm", "C", "bar")
  std::string data_type;            // "float", "int", "bool", "string"
  bool writable{false};             // Can be written via x-plc-operations
  std::optional<double> min_value;  // Optional range validation for writes
  std::optional<double> max_value;
  std::optional<AlarmConfig> alarm;  // Optional threshold alarm -> fault mapping (back-compat)
  std::string ros2_topic;            // ROS 2 topic for value bridging (auto: /plc/{entity}/{name})

  // Shared fault-detection rule for this point: threshold, status-word bit
  // decode, or fault-code enum. Built at load time from the ``alarm:``,
  // ``status_bits:`` or ``fault_enum:`` node-map block. The poller evaluates
  // this through the shared ``fault_detection::evaluate`` regardless of mode.
  std::optional<ros2_medkit::fault_detection::DetectionRule> detection;

  bool has_range() const {
    return min_value.has_value() && max_value.has_value();
  }
};

/// SOVD entity definition derived from the node map
struct PlcEntityDef {
  std::string id;
  std::string name;
  std::string component_id;
  bool is_app{true};
  std::vector<std::string> data_names;
  std::vector<std::string> writable_names;
  bool has_faults{false};
};

/// Manages the OPC-UA NodeId to SOVD entity mapping, loaded from YAML
class NodeMap {
 public:
  NodeMap() = default;

  /// Load node map from YAML file
  /// @return true if loaded successfully
  bool load(const std::string & yaml_path);

  /// Get all entries
  const std::vector<NodeMapEntry> & entries() const {
    return entries_;
  }

  /// Get entries for a specific SOVD entity
  std::vector<const NodeMapEntry *> entries_for_entity(const std::string & entity_id) const;

  /// Get writable entries for a specific SOVD entity
  std::vector<const NodeMapEntry *> writable_entries_for_entity(const std::string & entity_id) const;

  /// Find entry by data_name within an entity
  const NodeMapEntry * find_by_data_name(const std::string & entity_id, const std::string & data_name) const;

  /// Find entry by OPC-UA node ID string
  const NodeMapEntry * find_by_node_id(const std::string & node_id_str) const;

  /// Get all entries that have threshold-based alarm configuration.
  /// Test / back-compat only - the live fault-evaluation path is
  /// detection_entries(); nothing reads this at runtime.
  std::vector<const NodeMapEntry *> alarm_entries() const;

  /// Get all entries carrying a shared fault-detection rule (threshold,
  /// status-word bit decode, or fault-code enum). This is the set the poller
  /// evaluates each cycle.
  std::vector<const NodeMapEntry *> detection_entries() const;

  /// Get all native OPC-UA AlarmConditionType event-mode entries (issue #386).
  const std::vector<AlarmEventConfig> & event_alarms() const {
    return event_alarms_;
  }

  /// Find an event-mode alarm by ``(entity_id, fault_code)`` (used by the
  /// SOVD ``acknowledge_fault`` / ``confirm_fault`` operations). Matches the
  /// source-level fault_code or any of the entry's mapping fault_codes.
  const AlarmEventConfig * find_event_alarm(const std::string & entity_id, const std::string & fault_code) const;

  /// Resolve an observed event against a config's mappings (issue #389).
  /// First matching mapping wins; falls back to the source-level fault_code.
  /// Pure / static so the precedence rules are unit-testable without a server.
  static ResolvedAlarm resolve_alarm(const AlarmEventConfig & cfg, const std::string & condition_name,
                                     const std::string & source_node, const std::string & event_type,
                                     const std::string & message);

  /// Get the zero-config native A&C configuration. ``enabled`` is false
  /// unless the node map YAML declares a truthy ``auto_alarms:``.
  const AutoAlarmsConfig & auto_alarms() const {
    return auto_alarms_;
  }

  /// Derive a stable SOVD fault_code for an auto-derived alarm with NO
  /// per-alarm mapping. Tiered, in order:
  ///   1. slug(ConditionName)  - the OPC-UA condition identity, when present.
  ///   2. slug(SourceName)     - the human-readable event source, when
  ///      ConditionName is empty.
  ///   3. a hash of SourceNode + EventType + Message, when neither name is
  ///      available.
  /// Tier 3 folds in ``message`` deliberately: a real Siemens S7-1500
  /// multiplexes every Program_Alarm of one FB through a single SourceNode
  /// (e.g. ``i=1845``) with no ConditionName/SourceName, distinguished only
  /// by Message text ("pa" vs "pa2"). Without the message in the hash, every
  /// alarm on that FB would collapse onto one fault_code. Pure / static so
  /// it is unit-testable without a server.
  static std::string derive_auto_fault_code(const std::string & condition_name, const std::string & source_name,
                                            const std::string & source_node_str, const std::string & event_type_str,
                                            const std::string & message);

  /// Apply ``auto_alarms.include``/``exclude`` substring filters to an
  /// observed event's identity fields. ``exclude`` wins first (drops the
  /// event on any match); a non-empty ``include`` then requires at least one
  /// match. Pure / static so it is unit-testable without a server.
  static bool auto_alarm_passes_filters(const std::string & condition_name, const std::string & source_name,
                                        const std::string & message, const std::vector<std::string> & include_patterns,
                                        const std::vector<std::string> & exclude_patterns);

  /// Map a raw OPC-UA event Severity (1-1000) to a SOVD severity bucket using
  /// ``bands`` (``auto_alarms.severity_bands``, overridable per deployment).
  /// Pure / static so it is unit-testable without a server.
  static std::string map_auto_severity(uint16_t severity, const AutoAlarmsSeverityBands & bands);

  /// Get derived SOVD entity definitions
  const std::vector<PlcEntityDef> & entity_defs() const {
    return entity_defs_;
  }

  /// Get area ID configured for PLC systems
  const std::string & area_id() const {
    return area_id_;
  }
  const std::string & area_name() const {
    return area_name_;
  }

  /// Get component ID for the PLC runtime
  const std::string & component_id() const {
    return component_id_;
  }
  const std::string & component_name() const {
    return component_name_;
  }

  /// Whether auto-browse mode is enabled
  bool auto_browse() const {
    return auto_browse_;
  }

  /// Parse an OPC-UA node ID string into an `opcua::NodeId`.
  ///
  /// Supports the full OPC 10000-6 section 5.3.1.10 format:
  ///
  ///     [ns=<ns-index>;]i=<uint32>     numeric
  ///     [ns=<ns-index>;]s=<string>     string (Siemens, Beckhoff, ...)
  ///     [ns=<ns-index>;]g=<guid>       GUID   (Microsoft-style UUID)
  ///     [ns=<ns-index>;]b=<base64>     opaque ByteString
  ///
  /// When the `ns=` prefix is omitted the namespace defaults to 0.
  /// Returns a default-constructed `opcua::NodeId` on parse failure; the
  /// plugin treats such an entry as unresolvable at runtime.
  ///
  /// Declared public so it can be unit-tested directly.
  static opcua::NodeId parse_node_id(const std::string & str);

 private:
  /// Build entity_defs_ from entries_
  void build_entity_defs();

  std::vector<NodeMapEntry> entries_;
  std::vector<PlcEntityDef> entity_defs_;
  std::unordered_map<std::string, std::vector<size_t>> entity_index_;  // entity_id -> entry indices
  std::unordered_map<std::string, size_t> node_id_index_;              // node_id_str -> entry index

  // Issue #386: native OPC-UA AlarmConditionType subscriptions, loaded from
  // top-level ``event_alarms:`` in the YAML. Stored separately from
  // ``entries_`` because event-mode alarms do not have a scalar node to
  // poll; their entity definitions are merged into ``entity_defs_`` via
  // build_entity_defs() so SOVD discovery is unaffected.
  std::vector<AlarmEventConfig> event_alarms_;

  // Zero-config native A&C (issue #(auto-alarms)), loaded from top-level
  // ``auto_alarms:``. Unlike ``event_alarms_`` this carries no per-condition
  // mappings; the poller derives a fault_code/entity/severity per observed
  // event at runtime (see NodeMap::derive_auto_fault_code / map_auto_severity).
  AutoAlarmsConfig auto_alarms_;

  std::string area_id_ = "plc_systems";
  std::string area_name_ = "PLC Systems";
  std::string component_id_ = "openplc_runtime";
  std::string component_name_ = "OpenPLC Runtime";
  bool auto_browse_{false};
};

}  // namespace ros2_medkit_gateway
