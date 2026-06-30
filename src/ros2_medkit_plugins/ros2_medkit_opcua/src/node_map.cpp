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
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
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

    // Node entries
    auto nodes = root["nodes"];
    if (!nodes || !nodes.IsSequence()) {
      // Clear any stale state from previous load
      entries_.clear();
      entity_index_.clear();
      node_id_index_.clear();
      entity_defs_.clear();
      return false;
    }

    entries_.clear();
    entity_index_.clear();
    node_id_index_.clear();

    if (nodes.size() > 10000) {
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

    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto & n = nodes[i];

      // Validate required fields
      if (!n["node_id"] || !n["entity_id"] || !n["data_name"]) {
        RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"),
                    "Entry %zu missing required field (node_id/entity_id/data_name) - skipping", i);
        continue;
      }

      NodeMapEntry entry;

      entry.node_id_str = n["node_id"].as<std::string>();
      entry.node_id = parse_node_id(entry.node_id_str);
      if (entry.node_id_str.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"), "Entry %zu has empty node_id - skipping", i);
        continue;
      }
      entry.entity_id = n["entity_id"].as<std::string>();
      entry.data_name = n["data_name"].as<std::string>();
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
        AlarmConfig alarm;
        alarm.fault_code = n["alarm"]["fault_code"].as<std::string>();
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
            br.fault = {b["fault_code"].as<std::string>(),
                        validate_severity(b["severity"].as<std::string>("ERROR"), entry.node_id_str),
                        b["message"].as<std::string>(b["fault_code"].as<std::string>())};
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
            er.fault = {c["fault_code"].as<std::string>(),
                        validate_severity(c["severity"].as<std::string>("ERROR"), entry.node_id_str),
                        c["message"].as<std::string>(c["fault_code"].as<std::string>())};
            rule.codes.push_back(std::move(er));
          }
        }
        if (!rule.codes.empty()) {
          // Catch-all: a non-ok value matching no configured code stays visible
          // rather than reading as healthy. Code is YAML-overridable, else
          // derived deterministically from the point. (#481)
          rule.unknown_fault.fault_code = n["fault_enum"]["unknown_fault_code"]
                                              ? n["fault_enum"]["unknown_fault_code"].as<std::string>()
                                              : derive_unknown_code(entry.entity_id, entry.data_name);
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
      for (size_t i = 0; i < event_alarms_node.size(); ++i) {
        const auto & a = event_alarms_node[i];
        if (!a["alarm_source"] || !a["entity_id"] || !a["fault_code"]) {
          RCLCPP_WARN(rclcpp::get_logger("opcua.node_map"),
                      "event_alarms[%zu] missing alarm_source/entity_id/fault_code - skipping", i);
          continue;
        }
        AlarmEventConfig cfg;
        cfg.source_node_id_str = a["alarm_source"].as<std::string>();
        cfg.source_node_id = parse_node_id(cfg.source_node_id_str);
        cfg.entity_id = a["entity_id"].as<std::string>();
        cfg.fault_code = a["fault_code"].as<std::string>();
        cfg.severity_override = a["severity_override"].as<std::string>("");
        cfg.message_override = a["message"].as<std::string>("");
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

    // Fault-code collision validation. Two distinct concerns are checked here
    // against the full set of fault codes the polled detection entries can
    // emit (threshold + every status_bits bit + every fault_enum code), not
    // just ``nodes[*].alarm`` as the original check did.
    //
    //   1. Global uniqueness of every emitted detection fault_code. The
    //      poller shares one ``FaultTransitionTracker`` keyed by fault_code
    //      alone, and clears are global by code, but evaluation runs
    //      per-entry. Two detection entries emitting the same fault_code
    //      (even on different entities) would therefore flap raise+clear
    //      every poll cycle. Reject duplicates so the intent - one code, one
    //      source - is documented and enforced at load.
    //
    //   2. Cross-pipeline collision with native ``event_alarms`` on the same
    //      ``(entity_id, fault_code)`` (bburda review on PR #387). The
    //      threshold/bit/enum polling path and the AlarmCondition
    //      subscription path produce different semantics (debounced vs
    //      state-machine driven), so fault_manager receiving both a polled
    //      report and a state-machine report for one SOVD fault flaps and is
    //      impossible to debug at runtime.
    {
      namespace fd = ros2_medkit::fault_detection;
      // (entity_id, fault_code) for every fault a detection entry can emit.
      std::vector<std::pair<std::string, std::string>> detection_faults;
      for (const auto & entry : entries_) {
        if (!entry.detection.has_value()) {
          continue;
        }
        std::visit(
            [&](const auto & rule) {
              using T = std::decay_t<decltype(rule)>;
              if constexpr (std::is_same_v<T, fd::ThresholdRule>) {
                detection_faults.emplace_back(entry.entity_id, rule.fault.fault_code);
              } else if constexpr (std::is_same_v<T, fd::StatusWordRule>) {
                for (const auto & b : rule.bits) {
                  detection_faults.emplace_back(entry.entity_id, b.fault.fault_code);
                }
              } else if constexpr (std::is_same_v<T, fd::EnumMapRule>) {
                for (const auto & e : rule.codes) {
                  detection_faults.emplace_back(entry.entity_id, e.fault.fault_code);
                }
                if (!rule.unknown_fault.fault_code.empty()) {
                  detection_faults.emplace_back(entry.entity_id, rule.unknown_fault.fault_code);
                }
              }
            },
            *entry.detection);
      }

      // 1. Global uniqueness by fault_code across all detection entries.
      std::set<std::string> seen_codes;
      for (const auto & fault : detection_faults) {
        if (!seen_codes.insert(fault.second).second) {
          RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                       "fault_code '%s' is emitted by more than one detection entry "
                       "(threshold/status_bits/fault_enum); fault codes must be globally unique because "
                       "the shared fault tracker is keyed by code alone (a collision flaps raise/clear "
                       "every poll cycle) - rename one of them",
                       fault.second.c_str());
          return false;
        }
      }

      // 2. Cross-pipeline collision with event_alarms by (entity_id, fault_code).
      std::set<std::pair<std::string, std::string>> event_keys;
      for (const auto & cfg : event_alarms_) {
        event_keys.emplace(cfg.entity_id, cfg.fault_code);
      }
      for (const auto & [entity_id, code] : detection_faults) {
        if (event_keys.count({entity_id, code}) > 0) {
          RCLCPP_ERROR(rclcpp::get_logger("opcua.node_map"),
                       "Duplicate (entity_id=%s, fault_code=%s) declared by both a polled detection entry "
                       "(threshold/status_bits/fault_enum) and event_alarms[*] (subscription mode) - "
                       "mutually exclusive",
                       entity_id.c_str(), code.c_str());
          return false;
        }
      }
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
    if (cfg.entity_id == entity_id && cfg.fault_code == fault_code) {
      return &cfg;
    }
  }
  return nullptr;
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
