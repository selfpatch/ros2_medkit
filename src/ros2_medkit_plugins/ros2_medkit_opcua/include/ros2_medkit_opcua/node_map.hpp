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

namespace ros2_medkit_gateway {

/// Alarm configuration for a node that maps to a SOVD fault
struct AlarmConfig {
  std::string fault_code;
  std::string severity;  // "ERROR", "WARNING", "INFO"
  std::string message;
  double threshold{0.0};
  bool above_threshold{true};  // true = alarm when value > threshold
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

  /// SOVD fault code (e.g. ``PLC_OVERPRESSURE``).
  std::string fault_code;

  /// Optional severity override. When empty, ``AlarmStateMachine`` derives
  /// the SOVD severity bucket from the event's ``Severity`` (1-1000) per
  /// the convention documented in design/index.rst.
  std::string severity_override;

  /// Optional friendly message override; falls back to the event's
  /// ``Message`` field when empty.
  std::string message_override;
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
  std::optional<AlarmConfig> alarm;  // Optional alarm -> fault mapping
  std::string ros2_topic;            // ROS 2 topic for value bridging (auto: /plc/{entity}/{name})

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

  /// Get all entries that have threshold-based alarm configuration
  std::vector<const NodeMapEntry *> alarm_entries() const;

  /// Get all native OPC-UA AlarmConditionType event-mode entries (issue #386).
  const std::vector<AlarmEventConfig> & event_alarms() const {
    return event_alarms_;
  }

  /// Find an event-mode alarm by ``(entity_id, fault_code)`` (used by the
  /// SOVD ``acknowledge_fault`` / ``confirm_fault`` operations).
  const AlarmEventConfig * find_event_alarm(const std::string & entity_id, const std::string & fault_code) const;

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

  std::string area_id_ = "plc_systems";
  std::string area_name_ = "PLC Systems";
  std::string component_id_ = "openplc_runtime";
  std::string component_name_ = "OpenPLC Runtime";
  bool auto_browse_{false};
};

}  // namespace ros2_medkit_gateway
