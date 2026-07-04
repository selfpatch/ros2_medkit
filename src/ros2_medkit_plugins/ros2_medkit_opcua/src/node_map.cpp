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

#include "ros2_medkit_opcua/node_map.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace ros2_medkit_gateway {

namespace {

// Parse "XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX" into a 16-byte array.
// Returns std::nullopt if the input is not a valid GUID.
std::optional<std::array<uint8_t, 16>> parse_guid_hex(const std::string & s) {
  // Strip braces if present ("{...}" is the Microsoft registry form).
  std::string clean = s;
  if (!clean.empty() && clean.front() == '{' && clean.back() == '}') {
    clean = clean.substr(1, clean.size() - 2);
  }
  // Expected pattern: 8-4-4-4-12 hex digits with 4 dashes = 36 chars total.
  if (clean.size() != 36) {
    return std::nullopt;
  }
  constexpr std::array<size_t, 4> dash_pos{8, 13, 18, 23};
  for (auto p : dash_pos) {
    if (clean[p] != '-') {
      return std::nullopt;
    }
  }
  // Collapse to 32 hex chars by stripping dashes.
  std::string hex;
  hex.reserve(32);
  for (char c : clean) {
    if (c != '-') {
      hex += c;
    }
  }
  std::array<uint8_t, 16> bytes{};
  for (size_t i = 0; i < 16; ++i) {
    const char hi = hex[i * 2];
    const char lo = hex[i * 2 + 1];
    if (!std::isxdigit(static_cast<unsigned char>(hi)) || !std::isxdigit(static_cast<unsigned char>(lo))) {
      return std::nullopt;
    }
    try {
      bytes[i] = static_cast<uint8_t>(std::stoul(hex.substr(i * 2, 2), nullptr, 16));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return bytes;
}

}  // namespace

opcua::NodeId NodeMap::parse_node_id(const std::string & str) {
  // OPC-UA Node ID string format is defined in OPC 10000-6 section 5.3.1.10:
  //
  //     [ns=<ns-index>;]<type>=<identifier>
  //
  // where <type> is one of:
  //
  //     i=<uint32>             numeric (most common for compact servers)
  //     s=<string>             string   (Siemens DB addresses, Beckhoff tags)
  //     g=<guid>               GUID     (rare, MS-style uuid)
  //     b=<base64 bytestring>  opaque   (binary identifiers, legacy)
  //
  // If the `ns=` prefix is missing the namespace defaults to 0 (standard
  // namespace). The identifier portion extends to the end of the string, so
  // string identifiers may contain semicolons and other characters as long as
  // they are not the identifier type prefix of a second node ID.
  uint16_t ns = 0;
  size_t type_pos = std::string::npos;

  // Detect optional namespace prefix.
  if (str.rfind("ns=", 0) == 0) {
    const auto sep = str.find(';');
    if (sep == std::string::npos) {
      return {};  // malformed - no separator after namespace
    }
    try {
      ns = static_cast<uint16_t>(std::stoul(str.substr(3, sep - 3)));
    } catch (const std::exception &) {
      return {};
    }
    type_pos = sep + 1;
  } else {
    type_pos = 0;
  }

  if (type_pos + 1 >= str.size() || str[type_pos + 1] != '=') {
    return {};  // identifier type prefix missing
  }
  const char type_char = str[type_pos];
  const std::string identifier = str.substr(type_pos + 2);

  switch (type_char) {
    case 'i': {
      try {
        return {ns, static_cast<uint32_t>(std::stoul(identifier))};
      } catch (const std::exception &) {
        return {};
      }
    }
    case 's': {
      // String identifier is taken verbatim. OPC-UA explicitly allows any
      // Unicode characters in string node IDs, including semicolons, quotes,
      // dots etc. (e.g. Siemens: `"Tank_DB"."level"`, Beckhoff:
      // `MAIN.Tank.level`).
      return {ns, identifier};
    }
    case 'g': {
      const auto parsed = parse_guid_hex(identifier);
      if (!parsed) {
        return {};
      }
      return {ns, opcua::Guid{*parsed}};
    }
    case 'b': {
      try {
        return {ns, opcua::ByteString::fromBase64(identifier)};
      } catch (const std::exception &) {
        return {};
      }
    }
    default:
      return {};
  }
}

bool NodeMap::load(const std::string & yaml_path) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    // Top-level metadata
    if (root["area_id"]) {
      area_id_ = root["area_id"].as<std::string>();
    }
    if (root["area_name"]) {
      area_name_ = root["area_name"].as<std::string>();
    }
    if (root["component_id"]) {
      component_id_ = root["component_id"].as<std::string>();
    }
    if (root["component_name"]) {
      component_name_ = root["component_name"].as<std::string>();
    }
    if (root["auto_browse"]) {
      auto_browse_ = root["auto_browse"].as<bool>();
    }

    // Node entries. The ``nodes:`` section is optional: a config may declare
    // only ``event_alarms:`` (native AlarmCondition subscriptions with no
    // scalar data points to poll). The final emptiness check below rejects a
    // file that declares neither.
    entries_.clear();
    entity_index_.clear();
    node_id_index_.clear();

    auto nodes = root["nodes"];
    // A ``nodes:`` key that is present but neither null nor a sequence (e.g. a
    // scalar or a map) is a config error, not an absent section. Fail loudly
    // instead of silently dropping every node mapping. An explicitly empty
    // ``nodes:`` (null) stays valid and is handled by the emptiness check below.
    if (nodes && !nodes.IsNull() && !nodes.IsSequence()) {
      RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                   "'nodes:' must be a sequence of node entries - refusing to load");
      return false;
    }
    const bool has_nodes = nodes && nodes.IsSequence();

    if (has_nodes && nodes.size() > 10000) {
      RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                   "Node map has %zu entries (max 10000) - refusing to load to prevent resource exhaustion",
                   nodes.size());
      return false;
    }

    const auto logger = rclcpp::get_logger("opcua.node_map");

    // Defensive scalar decode: when a numeric/boolean field is *present* but
    // fails to convert, warn (naming node_id) and fall back to the default
    // rather than silently substituting a semantics-changing value (e.g.
    // ``threshold: "100 bar"`` -> 0.0 inverts a below-threshold alarm). yaml-cpp
    // ``as<T>(fallback)`` cannot tell "absent" from "present but bad". (#481)
    auto parse_double = [logger](const YAML::Node & node, double def, const char * field,
                                 const std::string & node_id) -> double {
      if (!node) {
        return def;
      }
      try {
        return node.as<double>();
      } catch (const YAML::Exception &) {
        RCLCPP_WARN(logger, "node_id=%s: non-numeric %s ignored - using default", node_id.c_str(), field);
        return def;
      }
    };
    auto parse_bool = [logger](const YAML::Node & node, bool def, const char * field,
                               const std::string & node_id) -> bool {
      if (!node) {
        return def;
      }
      try {
        return node.as<bool>();
      } catch (const YAML::Exception &) {
        RCLCPP_WARN(logger, "node_id=%s: non-boolean %s ignored - using default", node_id.c_str(), field);
        return def;
      }
    };
    auto parse_int64 = [logger](const YAML::Node & node, std::int64_t def, const char * field,
                                const std::string & node_id) -> std::int64_t {
      if (!node) {
        return def;
      }
      try {
        return node.as<std::int64_t>();
      } catch (const YAML::Exception &) {
        RCLCPP_WARN(logger, "node_id=%s: non-integer %s ignored - using default", node_id.c_str(), field);
        return def;
      }
    };
    // Defensive required-string read: a missing or wrong-typed (sequence / map /
    // null) string field warns naming node_id + field and returns nullopt so
    // the caller skips just this node / bit / code. An unguarded
    // ``.as<std::string>()`` throws YAML::TypedBadConversion, which the outer
    // catch turns into a whole-file abort that discards every other valid
    // node. (#481)
    auto parse_string = [logger](const YAML::Node & node, const char * field,
                                 const std::string & node_id) -> std::optional<std::string> {
      if (!node) {
        RCLCPP_WARN(logger, "node_id=%s: missing %s - skipping", node_id.c_str(), field);
        return std::nullopt;
      }
      try {
        return node.as<std::string>();
      } catch (const YAML::Exception &) {
        RCLCPP_WARN(logger, "node_id=%s: non-string %s - skipping", node_id.c_str(), field);
        return std::nullopt;
      }
    };
    // Severity must be one of the documented SOVD buckets; warn + default to
    // ERROR on a typo (e.g. ``SEVERE`` / wrong case) so it is not misrouted. (#481)
    auto validate_severity = [logger](const std::string & sev, const std::string & node_id) -> std::string {
      if (sev != "INFO" && sev != "WARNING" && sev != "ERROR" && sev != "CRITICAL") {
        RCLCPP_WARN(logger, "node_id=%s: unknown severity '%s' - defaulting to ERROR", node_id.c_str(), sev.c_str());
        return "ERROR";
      }
      return sev;
    };
    // Deterministic, unique-per-point catch-all code for an unmapped enum value.
    auto derive_unknown_code = [](const std::string & entity, const std::string & data) -> std::string {
      std::string s = entity + "_" + data + "_UNMAPPED";
      for (auto & c : s) {
        c = (std::isalnum(static_cast<unsigned char>(c)) != 0)
                ? static_cast<char>(std::toupper(static_cast<unsigned char>(c)))
                : '_';
      }
      return s;
    };

    for (size_t i = 0; has_nodes && i < nodes.size(); ++i) {
      const auto & n = nodes[i];

      // Validate required fields
      if (!n["node_id"] || !n["entity_id"] || !n["data_name"]) {
        RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"),
                    "Entry %zu missing required field (node_id/entity_id/data_name) - skipping", i);
        continue;
      }

      NodeMapEntry entry;

      // Required string fields use guarded reads: a present-but-wrong-typed
      // value (a sequence / map where a scalar is expected) warns and skips
      // just this entry rather than aborting the whole file via the outer
      // catch. (#481)
      {
        auto node_id_str = parse_string(n["node_id"], "node_id", "entry " + std::to_string(i));
        if (!node_id_str) {
          continue;
        }
        entry.node_id_str = *node_id_str;
      }
      entry.node_id = parse_node_id(entry.node_id_str);
      if (entry.node_id_str.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"), "Entry %zu has empty node_id - skipping", i);
        continue;
      }
      {
        auto entity_id = parse_string(n["entity_id"], "entity_id", entry.node_id_str);
        auto data_name = parse_string(n["data_name"], "data_name", entry.node_id_str);
        if (!entity_id || !data_name) {
          continue;
        }
        entry.entity_id = *entity_id;
        entry.data_name = *data_name;
      }
      entry.display_name = n["display_name"].as<std::string>(entry.data_name);
      entry.unit = n["unit"].as<std::string>("");
      entry.data_type = n["data_type"].as<std::string>("float");

      // Validate data_type is one of the known types
      if (entry.data_type != "bool" && entry.data_type != "int" && entry.data_type != "float" &&
          entry.data_type != "string") {
        RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"),
                    "Entry %zu (%s) has unknown data_type '%s' - defaulting to 'float'", i, entry.node_id_str.c_str(),
                    entry.data_type.c_str());
        entry.data_type = "float";
      }

      // Detect duplicate node_id (first wins)
      if (node_id_index_.count(entry.node_id_str) > 0) {
        RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"), "Duplicate node_id '%s' at entry %zu - skipping (first wins)",
                    entry.node_id_str.c_str(), i);
        continue;
      }

      entry.writable = n["writable"].as<bool>(false);
      if (n["min_value"]) {
        entry.min_value = n["min_value"].as<double>();
      }
      if (n["max_value"]) {
        entry.max_value = n["max_value"].as<double>();
      }

      // ROS 2 topic for value bridging (auto-generate if not specified).
      // ROS 2 topic segments: must match [a-zA-Z_][a-zA-Z0-9_]*, no empty segments.
      auto sanitize_topic_segment = [](const std::string & s) -> std::string {
        if (s.empty()) {
          return "_unnamed";
        }
        std::string out = s;
        for (auto & c : out) {
          if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            c = '_';
          }
        }
        // ROS 2 topic segments cannot start with a digit
        if (std::isdigit(static_cast<unsigned char>(out[0]))) {
          out.insert(out.begin(), '_');
        }
        return out;
      };
      auto is_valid_ros2_topic = [](const std::string & t) -> bool {
        if (t.empty() || t[0] != '/') {
          return false;
        }
        for (char c : t) {
          if (c == '/') {
            continue;
          }
          if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            return false;
          }
        }
        return t.find("//") == std::string::npos;
      };
      if (n["ros2_topic"]) {
        entry.ros2_topic = n["ros2_topic"].as<std::string>();
        if (!is_valid_ros2_topic(entry.ros2_topic)) {
          RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"), "Invalid custom ros2_topic '%s' for %s - auto-generating",
                      entry.ros2_topic.c_str(), entry.node_id_str.c_str());
          entry.ros2_topic.clear();
        }
      }
      if (entry.ros2_topic.empty()) {
        entry.ros2_topic =
            "/plc/" + sanitize_topic_segment(entry.entity_id) + "/" + sanitize_topic_segment(entry.data_name);
      }

      // Fault-detection block. Three mutually exclusive modes per point, all
      // lowered onto the shared ``fault_detection`` evaluator:
      //   alarm:        numeric threshold (above/below) - original behaviour
      //   status_bits:  decode named bits of an integer status word
      //   fault_enum:   map a fault-code register value to a fault + text
      namespace fd = ros2_medkit::fault_detection;
      const int detection_modes = (n["alarm"] ? 1 : 0) + (n["status_bits"] ? 1 : 0) + (n["fault_enum"] ? 1 : 0);
      if (detection_modes > 1) {
        RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                     "Entry node_id=%s declares more than one of alarm/status_bits/fault_enum - "
                     "these detection modes are mutually exclusive",
                     entry.node_id_str.c_str());
        return false;
      }

      if (n["alarm"]) {
        auto fault_code = parse_string(n["alarm"]["fault_code"], "alarm.fault_code", entry.node_id_str);
        if (!fault_code) {
          // No detection for this point; the data point itself still loads.
          RCLCPP_WARN(logger, "alarm on node_id=%s has no usable fault_code - no fault detection for this point",
                      entry.node_id_str.c_str());
        } else {
          AlarmConfig alarm;
          alarm.fault_code = *fault_code;
          alarm.severity = validate_severity(n["alarm"]["severity"].as<std::string>("ERROR"), entry.node_id_str);
          alarm.message = n["alarm"]["message"].as<std::string>(alarm.fault_code);
          alarm.threshold = parse_double(n["alarm"]["threshold"], 0.0, "alarm.threshold", entry.node_id_str);
          alarm.above_threshold =
              parse_bool(n["alarm"]["above_threshold"], true, "alarm.above_threshold", entry.node_id_str);

          fd::ThresholdRule rule;
          rule.fault = {alarm.fault_code, alarm.severity, alarm.message};
          rule.threshold = alarm.threshold;
          rule.above = alarm.above_threshold;
          entry.detection = std::move(rule);
          entry.alarm = std::move(alarm);
        }
      } else if (n["status_bits"]) {
        fd::StatusWordRule rule;
        // Optional source register width: mask off sign-extended high bits so a
        // signed status word read with its sign bit set does not fire spurious
        // faults above the real register width. (#481)
        if (n["status_word_width"]) {
          const auto w = parse_int64(n["status_word_width"], 0, "status_word_width", entry.node_id_str);
          if (w >= 1 && w <= 64) {
            rule.width = static_cast<unsigned>(w);
          } else {
            RCLCPP_WARN(logger, "status_word_width=%lld on node_id=%s out of range (1..64) - ignoring",
                        static_cast<long long>(w), entry.node_id_str.c_str());
          }
        }
        const auto & bits_node = n["status_bits"];
        if (bits_node.IsSequence()) {
          for (const auto & b : bits_node) {
            if (!b["bit"] || !b["fault_code"]) {
              RCLCPP_WARN(logger, "status_bits entry on node_id=%s missing bit/fault_code - skipping",
                          entry.node_id_str.c_str());
              continue;
            }
            fd::BitRule br;
            // A wrong-typed bit value skips just this bit (consistent with the
            // missing-field path above), never aborts the whole file. (#481)
            try {
              br.bit = b["bit"].as<unsigned>();
            } catch (const YAML::Exception &) {
              RCLCPP_WARN(logger, "status_bits entry on node_id=%s has non-integer bit - skipping",
                          entry.node_id_str.c_str());
              continue;
            }
            if (br.bit >= 64) {
              // The status word is decoded as a 64-bit register, so bits at or
              // above 64 can never be set and the rule would be dead config.
              RCLCPP_WARN(logger,
                          "status_bits entry on node_id=%s has bit=%u >= 64 (max status-word width); "
                          "this fault can never fire - skipping",
                          entry.node_id_str.c_str(), br.bit);
              continue;
            }
            if (rule.width > 0 && br.bit >= rule.width) {
              RCLCPP_WARN(logger,
                          "status_bits entry on node_id=%s has bit=%u >= status_word_width=%u; "
                          "this fault can never fire - skipping",
                          entry.node_id_str.c_str(), br.bit, rule.width);
              continue;
            }
            auto bit_fault_code = parse_string(b["fault_code"], "status_bits.fault_code", entry.node_id_str);
            if (!bit_fault_code) {
              continue;  // wrong-typed fault_code: skip just this bit
            }
            br.fault = {*bit_fault_code, validate_severity(b["severity"].as<std::string>("ERROR"), entry.node_id_str),
                        b["message"].as<std::string>(*bit_fault_code)};
            rule.bits.push_back(std::move(br));
          }
        }
        if (!rule.bits.empty()) {
          entry.detection = std::move(rule);
        } else {
          RCLCPP_WARN(logger,
                      "status_bits on node_id=%s declared but produced no usable rules - "
                      "no faults will be detected for this point",
                      entry.node_id_str.c_str());
        }
      } else if (n["fault_enum"]) {
        fd::EnumMapRule rule;
        rule.ok_value = parse_int64(n["fault_enum"]["ok_value"], 0, "fault_enum.ok_value", entry.node_id_str);
        const auto codes = n["fault_enum"]["codes"];
        if (codes && codes.IsSequence()) {
          for (const auto & c : codes) {
            if (!c["code"] || !c["fault_code"]) {
              RCLCPP_WARN(logger, "fault_enum entry on node_id=%s missing code/fault_code - skipping",
                          entry.node_id_str.c_str());
              continue;
            }
            fd::EnumRule er;
            // A wrong-typed code value skips just this entry, never aborts. (#481)
            try {
              er.code = c["code"].as<std::int64_t>();
            } catch (const YAML::Exception &) {
              RCLCPP_WARN(logger, "fault_enum entry on node_id=%s has non-integer code - skipping",
                          entry.node_id_str.c_str());
              continue;
            }
            auto code_fault_code = parse_string(c["fault_code"], "fault_enum.fault_code", entry.node_id_str);
            if (!code_fault_code) {
              continue;  // wrong-typed fault_code: skip just this enum code
            }
            er.fault = {*code_fault_code, validate_severity(c["severity"].as<std::string>("ERROR"), entry.node_id_str),
                        c["message"].as<std::string>(*code_fault_code)};
            rule.codes.push_back(std::move(er));
          }
        }
        if (!rule.codes.empty()) {
          // Catch-all: a non-ok value matching no configured code stays visible
          // rather than reading as healthy. Code is YAML-overridable, else
          // derived deterministically from the point. (#481)
          if (n["fault_enum"]["unknown_fault_code"]) {
            auto override_code =
                parse_string(n["fault_enum"]["unknown_fault_code"], "fault_enum.unknown_fault_code", entry.node_id_str);
            rule.unknown_fault.fault_code =
                override_code ? *override_code : derive_unknown_code(entry.entity_id, entry.data_name);
          } else {
            rule.unknown_fault.fault_code = derive_unknown_code(entry.entity_id, entry.data_name);
          }
          rule.unknown_fault.severity =
              validate_severity(n["fault_enum"]["unknown_severity"].as<std::string>("ERROR"), entry.node_id_str);
          rule.unknown_fault.message = n["fault_enum"]["unknown_message"].as<std::string>("");
          entry.detection = std::move(rule);
        } else {
          RCLCPP_WARN(logger,
                      "fault_enum on node_id=%s declared but produced no usable rules - "
                      "no faults will be detected for this point",
                      entry.node_id_str.c_str());
        }
      }

      size_t idx = entries_.size();
      node_id_index_[entry.node_id_str] = idx;
      entity_index_[entry.entity_id].push_back(idx);
      entries_.push_back(std::move(entry));
    }

    // Issue #386: native AlarmConditionType event subscriptions. Loaded from
    // top-level ``event_alarms:`` (sibling of ``nodes:``). Each entry must
    // declare its own entity_id; the entity will be merged into entity_defs_
    // alongside any threshold-mode entries pointing at the same id.
    event_alarms_.clear();
    auto event_alarms_node = root["event_alarms"];
    if (event_alarms_node && event_alarms_node.IsSequence()) {
      if (event_alarms_node.size() > 10000) {
        RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                     "event_alarms has %zu entries (max 10000) - refusing to load", event_alarms_node.size());
        return false;
      }
      // Resolve the configured severity for an event_alarms entry / mapping.
      // ``severity`` is accepted as an alias for ``severity_override`` because
      // the threshold ``alarm:`` block uses ``severity:``; an operator who
      // reuses that key here must not have it silently dropped. When both are
      // present ``severity_override`` wins. An explicit value is validated like
      // the threshold path; with neither key the result stays empty so the live
      // event Severity band mapping applies (see OpcuaPlugin::map_severity).
      auto read_severity_override = [&](const YAML::Node & node, const std::string & context) -> std::string {
        // yaml-cpp reports a present-but-null key (``severity_override:`` with
        // no value / ``~``) as defined/truthy; gate on a real value so a null
        // override neither shadows a valid ``severity:`` alias nor emits a
        // misleading "non-string severity_override" warning.
        auto has_value = [](const YAML::Node & n) {
          return n.IsDefined() && !n.IsNull();
        };
        const YAML::Node override_node = node["severity_override"];
        const bool use_override = has_value(override_node);
        const YAML::Node sev_node = use_override ? override_node : node["severity"];
        if (!has_value(sev_node)) {
          return "";
        }
        const char * field = use_override ? "severity_override" : "severity";
        auto sev = parse_string(sev_node, field, context);
        if (!sev || sev->empty()) {
          return "";
        }
        return validate_severity(*sev, context);
      };
      // Warn on keys the event_alarms loader does not recognise so a future
      // typo (e.g. ``severty:``) is surfaced instead of silently dropped.
      auto warn_unknown_keys = [logger](const YAML::Node & node, const std::string & context,
                                        std::initializer_list<const char *> known) {
        if (!node.IsMap()) {
          return;
        }
        for (const auto & kv : node) {
          std::string key;
          try {
            key = kv.first.as<std::string>();
          } catch (const YAML::Exception &) {
            continue;
          }
          const bool recognised = std::any_of(known.begin(), known.end(), [&key](const char * k) {
            return key == k;
          });
          if (!recognised) {
            RCLCPP_WARN(logger, "%s: unknown key '%s' - ignored", context.c_str(), key.c_str());
          }
        }
      };
      for (size_t i = 0; i < event_alarms_node.size(); ++i) {
        const auto & a = event_alarms_node[i];
        if (!a["alarm_source"] || !a["entity_id"]) {
          RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"),
                      "event_alarms[%zu] missing alarm_source/entity_id - skipping", i);
          continue;
        }
        // Required strings routed through the same guarded read as the nodes
        // block: a present-but-wrong-typed value warns and skips just this
        // event_alarms entry rather than throwing into the outer catch, which
        // would discard every valid node and disable the plugin. (#481)
        const std::string alarm_label = "event_alarms[" + std::to_string(i) + "]";
        auto alarm_source = parse_string(a["alarm_source"], "alarm_source", alarm_label);
        auto alarm_entity_id = parse_string(a["entity_id"], "entity_id", alarm_label);
        if (!alarm_source || !alarm_entity_id) {
          continue;
        }
        // Source-level fault_code is optional when ``mappings`` route every
        // alarm (issue #389); when present it must still be a usable string.
        std::string alarm_fault_code;
        if (a["fault_code"]) {
          auto parsed_fault_code = parse_string(a["fault_code"], "fault_code", alarm_label);
          if (!parsed_fault_code) {
            continue;
          }
          alarm_fault_code = *parsed_fault_code;
        }
        warn_unknown_keys(a, alarm_label,
                          {"alarm_source", "entity_id", "fault_code", "severity_override", "severity", "message",
                           "mappings", "associated_values"});
        AlarmEventConfig cfg;
        cfg.source_node_id_str = *alarm_source;
        cfg.source_node_id = parse_node_id(cfg.source_node_id_str);
        cfg.entity_id = *alarm_entity_id;
        cfg.fault_code = alarm_fault_code;
        cfg.severity_override = read_severity_override(a, alarm_label);
        cfg.message_override = a["message"].as<std::string>("");

        // Issue #389: per-condition-identity mappings (multi-alarm).
        if (a["mappings"] && a["mappings"].IsSequence()) {
          for (const auto & m : a["mappings"]) {
            if (!m["fault_code"]) {
              RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"),
                          "event_alarms[%zu] mapping missing fault_code - skipping", i);
              continue;
            }
            auto mapping_fault_code = parse_string(m["fault_code"], "mappings.fault_code", alarm_label);
            if (!mapping_fault_code) {
              continue;
            }
            const std::string mapping_label = alarm_label + " mapping";
            warn_unknown_keys(m, mapping_label,
                              {"condition_name", "source_node", "event_type", "fault_code", "severity_override",
                               "severity", "message"});
            AlarmMapping mapping;
            mapping.match_condition_name = m["condition_name"].as<std::string>("");
            mapping.match_source_node = m["source_node"].as<std::string>("");
            mapping.match_event_type = m["event_type"].as<std::string>("");
            mapping.fault_code = *mapping_fault_code;
            mapping.severity_override = read_severity_override(m, mapping_label);
            mapping.message_override = m["message"].as<std::string>("");
            cfg.mappings.push_back(std::move(mapping));
          }
        }

        // Issue #389: extra associated-value event fields (e.g. SD_1..SD_n).
        if (a["associated_values"] && a["associated_values"].IsSequence()) {
          for (const auto & v : a["associated_values"]) {
            AssociatedValueRef ref;
            if (v.IsScalar()) {
              ref.name = v.as<std::string>();
            } else {
              ref.namespace_index = v["namespace_index"].as<uint16_t>(0);
              ref.name = v["name"].as<std::string>("");
              ref.label = v["label"].as<std::string>("");
            }
            if (ref.name.empty()) {
              continue;
            }
            if (ref.label.empty()) {
              ref.label = ref.name;
            }
            cfg.associated_values.push_back(std::move(ref));
          }
        }

        if (cfg.fault_code.empty() && cfg.mappings.empty()) {
          RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"),
                      "event_alarms[%zu] has neither fault_code nor mappings - skipping", i);
          continue;
        }
        event_alarms_.push_back(std::move(cfg));
      }
    }

    // Schema validation under ``nodes:``: ``alarm_source`` belongs in the
    // top-level ``event_alarms:`` section, never under ``nodes:``. Silently
    // ignoring a misplaced ``alarm_source`` (the previous behavior unless
    // also paired with ``alarm.threshold``) lets a configuration typo land
    // a "subscribed alarm that never fires", which is impossible to
    // diagnose from runtime logs. Reject the whole file with an actionable
    // error pointing at the right place. (Copilot review on PR #387.)
    for (const auto & node : (nodes ? nodes : YAML::Node{})) {
      if (node["alarm_source"]) {
        RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                     "Entry node_id=%s uses ``alarm_source`` under ``nodes:``, which is invalid; "
                     "move this configuration to top-level ``event_alarms:`` (see README §event_alarms)",
                     node["node_id"] ? node["node_id"].as<std::string>().c_str() : "<unknown>");
        return false;
      }
    }

    // Global fault-code uniqueness validation. fault_manager keys and clears
    // faults by ``fault_code`` ALONE (clear_fault(fault_code) /
    // get_fault(fault_code)); the poller shares one ``FaultTransitionTracker``
    // that is likewise keyed by code alone. So a fault_code is a global
    // identifier and must be unique across EVERY source that can emit it,
    // regardless of entity_id:
    //
    //   * every polled detection fault (threshold + each status_bits bit +
    //     each fault_enum code + the enum catch-all), and
    //   * every native ``event_alarms`` subscription (issue #386).
    //
    // Two sources sharing a code - even a polled code on entity_a and an
    // event_alarms code on entity_b - would collide at fault_manager: one
    // source's clear wipes the other's fault, or the two flap raise/clear
    // every cycle. The (entity_id, fault_code) pair the earlier check keyed on
    // is NOT sufficient, because the fault manager never sees entity_id in its
    // key. Reject the whole file at load with an actionable error so the intent
    // - one code, one source - is enforced before anything runs.
    {
      namespace fd = ros2_medkit::fault_detection;
      struct EmittedFault {
        std::string entity_id;
        const char * pipeline;  // "polled detection" or "event_alarms"
      };
      std::unordered_map<std::string, EmittedFault> seen;  // fault_code -> first source
      auto check_unique = [&](const std::string & code, const std::string & entity_id, const char * pipeline) -> bool {
        auto [it, inserted] = seen.emplace(code, EmittedFault{entity_id, pipeline});
        if (!inserted) {
          RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                       "fault_code '%s' is emitted by more than one source (%s on entity '%s' and %s on "
                       "entity '%s'); fault codes must be globally unique across all detection entries and "
                       "event_alarms because the shared fault manager is keyed by code alone (a collision "
                       "clears the other source's fault or flaps raise/clear) - rename one of them",
                       code.c_str(), it->second.pipeline, it->second.entity_id.c_str(), pipeline, entity_id.c_str());
          return false;
        }
        return true;
      };

      for (const auto & entry : entries_) {
        if (!entry.detection.has_value()) {
          continue;
        }
        bool ok = true;
        std::visit(
            [&](const auto & rule) {
              using T = std::decay_t<decltype(rule)>;
              if constexpr (std::is_same_v<T, fd::ThresholdRule>) {
                ok = ok && check_unique(rule.fault.fault_code, entry.entity_id, "polled detection");
              } else if constexpr (std::is_same_v<T, fd::StatusWordRule>) {
                for (const auto & b : rule.bits) {
                  ok = ok && check_unique(b.fault.fault_code, entry.entity_id, "polled detection");
                }
              } else if constexpr (std::is_same_v<T, fd::EnumMapRule>) {
                for (const auto & e : rule.codes) {
                  ok = ok && check_unique(e.fault.fault_code, entry.entity_id, "polled detection");
                }
                if (!rule.unknown_fault.fault_code.empty()) {
                  ok = ok && check_unique(rule.unknown_fault.fault_code, entry.entity_id, "polled detection");
                }
              }
            },
            *entry.detection);
        if (!ok) {
          return false;
        }
      }

      for (const auto & cfg : event_alarms_) {
        // The source-level fault_code may be empty when every alarm is routed
        // through ``mappings`` (issue #389); only real codes enter the set.
        if (!cfg.fault_code.empty() && !check_unique(cfg.fault_code, cfg.entity_id, "event_alarms")) {
          return false;
        }
        for (const auto & m : cfg.mappings) {
          if (!check_unique(m.fault_code, cfg.entity_id, "event_alarms")) {
            return false;
          }
        }
      }
    }

    // A config that declares neither a 'nodes:' section nor any event alarms
    // carries no mappings and is almost always a mistake (wrong file / typo).
    // A present-but-empty 'nodes:' section is still a valid config.
    if (!has_nodes && event_alarms_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                   "Node map has neither 'nodes:' nor 'event_alarms:' entries - nothing to map");
      entity_defs_.clear();
      return false;
    }

    build_entity_defs();
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"), "NodeMap::load failed: %s", e.what());
    return false;
  }
}

const AlarmEventConfig * NodeMap::find_event_alarm(const std::string & entity_id,
                                                   const std::string & fault_code) const {
  for (const auto & cfg : event_alarms_) {
    if (cfg.entity_id != entity_id) {
      continue;
    }
    if (cfg.fault_code == fault_code) {
      return &cfg;
    }
    for (const auto & m : cfg.mappings) {
      if (m.fault_code == fault_code) {
        return &cfg;
      }
    }
  }
  return nullptr;
}

ResolvedAlarm NodeMap::resolve_alarm(const AlarmEventConfig & cfg, const std::string & condition_name,
                                     const std::string & source_node, const std::string & event_type) {
  ResolvedAlarm out;
  // First mapping whose non-empty match fields all equal the observed event
  // wins (declaration order = precedence).
  for (const auto & m : cfg.mappings) {
    if (!m.match_condition_name.empty() && m.match_condition_name != condition_name) {
      continue;
    }
    if (!m.match_source_node.empty() && m.match_source_node != source_node) {
      continue;
    }
    if (!m.match_event_type.empty() && m.match_event_type != event_type) {
      continue;
    }
    out.fault_code = m.fault_code;
    // A mapping-level override wins; otherwise inherit the source-level one.
    out.severity_override = m.severity_override.empty() ? cfg.severity_override : m.severity_override;
    out.message_override = m.message_override.empty() ? cfg.message_override : m.message_override;
    out.matched = true;
    return out;
  }
  // No mapping matched: fall back to the source-level fault_code (catch-all).
  if (!cfg.fault_code.empty()) {
    out.fault_code = cfg.fault_code;
    out.severity_override = cfg.severity_override;
    out.message_override = cfg.message_override;
    out.matched = true;
  }
  return out;
}

std::vector<const NodeMapEntry *> NodeMap::entries_for_entity(const std::string & entity_id) const {
  std::vector<const NodeMapEntry *> result;
  auto it = entity_index_.find(entity_id);
  if (it != entity_index_.end()) {
    for (auto idx : it->second) {
      result.push_back(&entries_[idx]);
    }
  }
  return result;
}

std::vector<const NodeMapEntry *> NodeMap::writable_entries_for_entity(const std::string & entity_id) const {
  std::vector<const NodeMapEntry *> result;
  auto it = entity_index_.find(entity_id);
  if (it != entity_index_.end()) {
    for (auto idx : it->second) {
      if (entries_[idx].writable) {
        result.push_back(&entries_[idx]);
      }
    }
  }
  return result;
}

const NodeMapEntry * NodeMap::find_by_data_name(const std::string & entity_id, const std::string & data_name) const {
  auto it = entity_index_.find(entity_id);
  if (it != entity_index_.end()) {
    for (auto idx : it->second) {
      if (entries_[idx].data_name == data_name) {
        return &entries_[idx];
      }
    }
  }
  return nullptr;
}

const NodeMapEntry * NodeMap::find_by_node_id(const std::string & node_id_str) const {
  auto it = node_id_index_.find(node_id_str);
  if (it != node_id_index_.end()) {
    return &entries_[it->second];
  }
  return nullptr;
}

// Test / back-compat only: the live fault-evaluation path is
// detection_entries() (see opcua_poller.cpp). This returns just the
// threshold-mode subset carrying the legacy ``alarm`` block and is not consulted
// at runtime; do not wire new callers to it.
std::vector<const NodeMapEntry *> NodeMap::alarm_entries() const {
  std::vector<const NodeMapEntry *> result;
  for (const auto & entry : entries_) {
    if (entry.alarm.has_value()) {
      result.push_back(&entry);
    }
  }
  return result;
}

std::vector<const NodeMapEntry *> NodeMap::detection_entries() const {
  std::vector<const NodeMapEntry *> result;
  for (const auto & entry : entries_) {
    if (entry.detection.has_value()) {
      result.push_back(&entry);
    }
  }
  return result;
}

void NodeMap::build_entity_defs() {
  entity_defs_.clear();

  // Group entries by entity_id
  std::unordered_map<std::string, PlcEntityDef> defs;

  for (const auto & entry : entries_) {
    auto & def = defs[entry.entity_id];
    if (def.id.empty()) {
      def.id = entry.entity_id;
      def.component_id = component_id_;
    }
    def.data_names.push_back(entry.data_name);
    if (entry.writable) {
      def.writable_names.push_back(entry.data_name);
    }
    if (entry.detection.has_value()) {
      def.has_faults = true;
    }
  }

  // Issue #386: event-mode entities show up as fault-bearing too, even when
  // they have no scalar data points of their own. Without this, the SOVD
  // discovery layer would not surface an entity that exists purely to host
  // alarm events (e.g. a system-wide ``ServerDiagnostics`` source).
  for (const auto & cfg : event_alarms_) {
    auto & def = defs[cfg.entity_id];
    if (def.id.empty()) {
      def.id = cfg.entity_id;
      def.component_id = component_id_;
    }
    def.has_faults = true;
  }

  // Build human-readable names from IDs
  for (auto & [id, def] : defs) {
    // Convert snake_case to Title Case
    std::string name;
    bool capitalize = true;
    for (char c : id) {
      if (c == '_') {
        name += ' ';
        capitalize = true;
      } else if (capitalize) {
        name += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        capitalize = false;
      } else {
        name += c;
      }
    }
    def.name = name;
    entity_defs_.push_back(std::move(def));
  }

  // Sort entity_defs_ by id so discovery output is deterministic across
  // runs and platforms. ``defs`` above is an unordered_map whose
  // iteration order is implementation-defined, which previously produced
  // noisy diffs in downstream tools consuming the IntrospectionResult.
  std::sort(entity_defs_.begin(), entity_defs_.end(), [](const PlcEntityDef & a, const PlcEntityDef & b) {
    return a.id < b.id;
  });
}

}  // namespace ros2_medkit_gateway
