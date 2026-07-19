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

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"

namespace ros2_medkit_gateway {

/// Read-only access to an OPC-UA address space, as needed by ``AutoBrowser``.
/// Abstracts away the live client so the recursive walk (hierarchy mapping,
/// type inference, depth/node caps, namespace filtering, entity/data-name
/// derivation) is unit-testable with an injected fake - no server, no
/// network. ``OpcuaClientBrowseSource`` below is the real implementation.
class AutoBrowseSource {
 public:
  virtual ~AutoBrowseSource() = default;

  /// Browse the forward-hierarchical Object/Variable children of ``parent``.
  /// Empty on failure or when the node has no such children.
  virtual std::vector<OpcuaClient::BrowseChild> browse_children(const opcua::NodeId & parent) = 0;

  /// Resolve a Variable node's DataType to an OPC-UA builtin scalar type name
  /// ("Boolean", "Int32", "Float", ...), or "" when the type is not a
  /// supported scalar (structured/enum/vendor type) or the read failed.
  virtual std::string read_type_name(const opcua::NodeId & variable_node) = 0;

  /// Best-effort read of a Variable's current value, used only as an
  /// optional reachability check (see ``AutoBrowseConfig::read_initial_values``).
  /// ``std::nullopt`` means the read failed (Bad status / not connected).
  virtual std::optional<OpcuaValue> read_initial_value(const opcua::NodeId & variable_node) = 0;

  /// True/false when the server's AccessLevel/UserAccessLevel for this Variable
  /// resolved (CurrentWrite bit -> writable), ``std::nullopt`` when the
  /// attribute read failed. Drives ``NodeMapEntry::writable`` under
  /// ``AutoBrowseConfig::infer_writable`` so a config-less walk exposes a write
  /// surface exactly where the server permits it.
  virtual std::optional<bool> read_writable(const opcua::NodeId & variable_node) = 0;
};

/// ``AutoBrowseSource`` backed by a live ``OpcuaClient`` session.
class OpcuaClientBrowseSource : public AutoBrowseSource {
 public:
  explicit OpcuaClientBrowseSource(OpcuaClient & client) : client_(client) {
  }

  std::vector<OpcuaClient::BrowseChild> browse_children(const opcua::NodeId & parent) override {
    return client_.browse_detailed(parent);
  }

  std::string read_type_name(const opcua::NodeId & variable_node) override {
    return client_.read_variable_type_name(variable_node);
  }

  std::optional<OpcuaValue> read_initial_value(const opcua::NodeId & variable_node) override {
    auto result = client_.read_value(variable_node);
    if (!result.good) {
      return std::nullopt;
    }
    return result.value;
  }

  std::optional<bool> read_writable(const opcua::NodeId & variable_node) override {
    const auto info = client_.read_access_level(variable_node);
    if (!info.ok) {
      return std::nullopt;
    }
    return info.writable;
  }

 private:
  OpcuaClient & client_;
};

/// Result of one ``AutoBrowser::browse()`` call.
struct AutoBrowseResult {
  /// NodeMapEntry per discovered, type-supported Variable. entity_id /
  /// data_name / display_name / ros2_topic are already fully derived; the
  /// caller only needs to merge these into a NodeMap
  /// (``NodeMap::merge_auto_browsed_entries``).
  std::vector<NodeMapEntry> entries;

  /// Total children considered across the walk (the configured root(s)
  /// themselves are not counted, only their descendants), whether or not a
  /// child passed the namespace filter or ended up mapped.
  std::size_t nodes_visited{0};

  /// True when ``AutoBrowseConfig::max_nodes`` was reached before the walk
  /// finished - the tree may be incomplete.
  bool node_cap_hit{false};

  /// True when ``AutoBrowseConfig::max_depth`` was reached on at least one
  /// branch - that branch's descendants were not explored.
  bool depth_cap_hit{false};
};

/// Recursive OPC-UA address-space walker: given a connected session (via
/// ``AutoBrowseSource``) and an ``AutoBrowseConfig``, builds the set of
/// NodeMapEntry that ``NodeMap::merge_auto_browsed_entries`` folds into the
/// SOVD entity tree. Read-only - never writes to the server.
///
/// Mapping rule: an Object node becomes a SOVD entity (App) only when it has
/// at least one directly-owned Variable child with a supported scalar type;
/// pure organizational Objects (folders with only child Objects) are walked
/// through but never become an entity of their own. Hierarchy is preserved in
/// the generated entity_id/display name as a path (e.g. "plc_1_db_test" /
/// "PLC_1 / DB_Test") rather than as a separate parent-entity field, matching
/// the existing flat Area -> Component -> App SOVD model this plugin already
/// produces from a hand-written node map.
class AutoBrowser {
 public:
  AutoBrowser(AutoBrowseSource & source, AutoBrowseConfig config);

  /// Run the walk from every configured root and return the merged result.
  AutoBrowseResult browse();

  // -- Pure helpers, exposed for direct unit testing --

  /// True when ``ns`` is eligible to become an entity/data point (and to be
  /// recursed into) under ``cfg``'s namespace_allow/namespace_deny.
  static bool namespace_allowed(uint16_t ns, const AutoBrowseConfig & cfg);

  /// Map an OPC-UA builtin type name (as returned by
  /// ``AutoBrowseSource::read_type_name``) to a medkit data_type
  /// ("bool"/"int"/"float"/"string"), or "" when unsupported.
  static std::string map_opcua_type(const std::string & opcua_type_name);

  /// Sanitize one path segment (BrowseName or DisplayName) into a valid SOVD
  /// id / ROS 2 topic segment: lowercase ascii alnum + '_'/'-' only, never
  /// starts with a digit, "_unnamed" when empty.
  static std::string sanitize_segment(const std::string & s);

  /// Join sanitized path segments into an entity_id ("plc_1_db_test").
  /// Empty ``segments`` (a root node with variables directly on it) maps to
  /// "root".
  static std::string join_entity_id(const std::vector<std::string> & segments);

  /// Join raw (unsanitized) DisplayName path segments into a human-readable
  /// entity name ("PLC_1 / DB_Test"). Empty ``segments`` maps to "Root".
  static std::string join_display_name(const std::vector<std::string> & segments);

 private:
  void visit_object(const opcua::NodeId & node, const std::vector<std::string> & path_ids,
                    const std::vector<std::string> & path_display, int depth, AutoBrowseResult & result,
                    std::unordered_set<std::string> & seen_entity_ids, std::unordered_set<std::string> & visited_nodes);

  AutoBrowseSource & source_;
  AutoBrowseConfig config_;
};

}  // namespace ros2_medkit_gateway
