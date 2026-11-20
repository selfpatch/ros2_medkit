// Copyright 2026 bburda
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

#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway {

/// Convert an rclcpp::Parameter to a nlohmann::json value. Arrays/lists are
/// preserved as JSON arrays so plugin configs see typed leaf collections.
inline nlohmann::json parameter_to_json(const rclcpp::Parameter & param) {
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return param.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return param.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return param.as_string();
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return param.as_byte_array();
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return param.as_bool_array();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return param.as_integer_array();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return param.as_double_array();
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return param.as_string_array();
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      return nullptr;
  }
}

/// Insert a value into a nested JSON object, splitting a dotted key into a
/// hierarchy of nested objects. The dotted key must already have the
/// "plugins.<name>." prefix stripped.
///
///   "enabled"                      -> {"enabled": value}
///   "native_alarms.enabled"        -> {"native_alarms": {"enabled": value}}
///   "native_alarms.severity_bands.warning"
///                                  -> {"native_alarms": {"severity_bands": {"warning": value}}}
///
/// The final segment is a leaf and keeps the value verbatim (scalars, and
/// arrays from parameter_to_json, are stored as-is). Every non-final segment
/// is an intermediate object.
///
/// A key that is used both as a leaf and as an intermediate (e.g. a plugin sets
/// both `discovery: true` and `discovery.enabled: true`) is resolved in favour
/// of the nested object regardless of processing order, and a warning is logged.
/// This never throws on a well-formed dotted key.
inline void insert_nested_param(nlohmann::json & root, const std::string & dotted_key, nlohmann::json value,
                                const rclcpp::Logger & logger) {
  if (dotted_key.empty()) {
    return;
  }

  // Split on '.'.
  std::vector<std::string> segments;
  size_t start = 0;
  while (true) {
    size_t dot = dotted_key.find('.', start);
    if (dot == std::string::npos) {
      segments.push_back(dotted_key.substr(start));
      break;
    }
    segments.push_back(dotted_key.substr(start, dot - start));
    start = dot + 1;
  }

  nlohmann::json * cur = &root;
  for (size_t i = 0; i + 1 < segments.size(); ++i) {
    const std::string & seg = segments[i];
    auto it = cur->find(seg);
    if (it == cur->end()) {
      (*cur)[seg] = nlohmann::json::object();
    } else if (!it->is_object()) {
      // A scalar leaf already sits where we now need a nested group. Prefer the
      // nested object so the deeper keys are not lost; warn about the discarded
      // scalar.
      RCLCPP_WARN(logger,
                  "Plugin config key '%s': '%s' is used both as a value and as a nested group; "
                  "keeping the nested group and discarding the scalar",
                  dotted_key.c_str(), seg.c_str());
      (*cur)[seg] = nlohmann::json::object();
    }
    cur = &(*cur)[seg];
  }

  const std::string & leaf = segments.back();
  auto leaf_it = cur->find(leaf);
  if (leaf_it != cur->end() && leaf_it->is_object() && !value.is_object()) {
    // A nested group already exists here; do not clobber it with a scalar.
    RCLCPP_WARN(logger, "Plugin config key '%s': '%s' already holds nested keys; ignoring the scalar override",
                dotted_key.c_str(), leaf.c_str());
    return;
  }
  (*cur)[leaf] = std::move(value);
}

/// Extract per-plugin config from the node's parameter overrides into a NESTED
/// JSON object.
///
/// Scans for keys matching "plugins.<name>.<key>[.<subkey>...]" (excluding
/// ".path") and reconstructs the dotted remainder into a hierarchy of nested
/// objects, e.g. plugins.opcua.native_alarms.enabled -> {"native_alarms":
/// {"enabled": true}}. Plugins read their config as nested objects
/// (config.contains("native_alarms"), config["native_alarms"]["enabled"], ...),
/// so a flat object keyed by the dotted string would never match. Single-level
/// keys stay flat.
///
/// The overrides hold the params files, `-p` arguments and
/// NodeOptions::parameter_overrides. The keys are never declared as parameters,
/// so the parameter services and /parameter_events do not serve them.
inline nlohmann::json extract_plugin_config(rclcpp::Node * node, const std::string & plugin_name) {
  auto config = nlohmann::json::object();
  const std::string prefix = "plugins." + plugin_name + ".";
  const std::string path_key = prefix + "path";

  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : overrides) {
    if (name.rfind(prefix, 0) == 0 && name != path_key) {
      insert_nested_param(config, name.substr(prefix.size()), parameter_to_json(rclcpp::Parameter(name, value)),
                          node->get_logger());
    }
  }
  return config;
}

}  // namespace ros2_medkit_gateway
