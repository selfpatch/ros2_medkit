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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rcl/arguments.h>
#include <rcl_yaml_param_parser/parser.h>
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

/// Declare plugin config parameters from the global --params-file YAML.
///
/// Parameters from --params-file go into the ROS 2 global rcl context,
/// NOT into NodeOptions::parameter_overrides(). We must discover and
/// declare them explicitly so list_parameters()/get_parameter() can find them.
inline void declare_plugin_params_from_yaml(rclcpp::Node * node, const std::string & prefix,
                                            const std::string & path_key = "") {
  auto rcl_ctx = node->get_node_base_interface()->get_context()->get_rcl_context();
  rcl_params_t * global_params = nullptr;
  auto ret = rcl_arguments_get_param_overrides(&rcl_ctx->global_arguments, &global_params);
  if (ret != RCL_RET_OK || global_params == nullptr) {
    return;
  }
  auto cleanup = [](rcl_params_t * p) {
    if (p) {
      rcl_yaml_node_struct_fini(p);
    }
  };
  std::unique_ptr<rcl_params_t, decltype(cleanup)> guard(global_params, cleanup);

  std::string node_name = node->get_name();
  std::string node_fqn = node->get_fully_qualified_name();
  for (size_t n = 0; n < global_params->num_nodes; ++n) {
    std::string yaml_node = global_params->node_names[n];
    if (yaml_node != node_name && yaml_node != node_fqn && yaml_node != "/**") {
      continue;
    }
    auto * node_p = &global_params->params[n];
    for (size_t p = 0; p < node_p->num_params; ++p) {
      std::string pname = node_p->parameter_names[p];
      if (pname.rfind(prefix, 0) == 0 && pname != path_key && !node->has_parameter(pname)) {
        auto & val = node_p->parameter_values[p];
        try {
          if (val.string_value != nullptr) {
            node->declare_parameter(pname, std::string(val.string_value));
          } else if (val.bool_value != nullptr) {
            node->declare_parameter(pname, *val.bool_value);
          } else if (val.integer_value != nullptr) {
            node->declare_parameter(pname, static_cast<int64_t>(*val.integer_value));
          } else if (val.double_value != nullptr) {
            node->declare_parameter(pname, *val.double_value);
          } else if (val.string_array_value != nullptr) {
            // Preserve index alignment: map null elements to empty strings rather than
            // dropping them, so plugin configs see the same length the YAML declared.
            std::vector<std::string> arr;
            arr.reserve(val.string_array_value->size);
            for (size_t i = 0; i < val.string_array_value->size; ++i) {
              const char * elem = val.string_array_value->data[i];
              arr.emplace_back(elem != nullptr ? elem : "");
            }
            node->declare_parameter(pname, arr);
          } else if (val.bool_array_value != nullptr) {
            // Explicit loop (rather than range ctor) avoids UB when size==0 and
            // values==nullptr - nullptr+0 pointer arithmetic is undefined in C++.
            std::vector<bool> arr;
            arr.reserve(val.bool_array_value->size);
            for (size_t i = 0; i < val.bool_array_value->size; ++i) {
              arr.push_back(val.bool_array_value->values[i]);
            }
            node->declare_parameter(pname, arr);
          } else if (val.integer_array_value != nullptr) {
            std::vector<int64_t> arr;
            arr.reserve(val.integer_array_value->size);
            for (size_t i = 0; i < val.integer_array_value->size; ++i) {
              arr.push_back(val.integer_array_value->values[i]);
            }
            node->declare_parameter(pname, arr);
          } else if (val.double_array_value != nullptr) {
            std::vector<double> arr;
            arr.reserve(val.double_array_value->size);
            for (size_t i = 0; i < val.double_array_value->size; ++i) {
              arr.push_back(val.double_array_value->values[i]);
            }
            node->declare_parameter(pname, arr);
          } else {
            RCLCPP_WARN(node->get_logger(), "Skipping param '%s': unsupported type", pname.c_str());
          }
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
          // Expected when a base class / earlier pass already declared the
          // same parameter - ignore silently to keep idempotency.
        } catch (const std::exception & e) {
          // Real errors (invalid type/value, RCL errors, bad YAML typos) must
          // surface to the operator, not vanish into DEBUG.
          RCLCPP_WARN(node->get_logger(), "Could not declare param '%s': %s", pname.c_str(), e.what());
        }
      }
    }
  }
}

/// Extract per-plugin config from ROS 2 parameters into a NESTED JSON object.
///
/// Scans for keys matching "plugins.<name>.<key>[.<subkey>...]" (excluding
/// ".path") and reconstructs the dotted remainder into a hierarchy of nested
/// objects, e.g. plugins.opcua.native_alarms.enabled -> {"native_alarms":
/// {"enabled": true}}. Plugins read their config as nested objects
/// (config.contains("native_alarms"), config["native_alarms"]["enabled"], ...),
/// so a flat object keyed by the dotted string would never match. Single-level
/// keys stay flat, unchanged.
///
/// Checks two sources:
///   1. NodeOptions::parameter_overrides (set programmatically, e.g. in unit tests)
///   2. Global YAML overrides from --params-file (declared on-demand, production path)
inline nlohmann::json extract_plugin_config(rclcpp::Node * node, const std::string & plugin_name) {
  auto config = nlohmann::json::object();
  std::string prefix = "plugins." + plugin_name + ".";
  std::string path_key = prefix + "path";

  // Source 1: NodeOptions parameter_overrides (programmatic, used in tests)
  for (const auto & param : node->get_node_options().parameter_overrides()) {
    const auto & name = param.get_name();
    if (name.rfind(prefix, 0) == 0 && name != path_key) {
      insert_nested_param(config, name.substr(prefix.size()), parameter_to_json(param), node->get_logger());
    }
  }
  if (!config.empty()) {
    return config;
  }

  // Source 2: global YAML overrides from --params-file
  declare_plugin_params_from_yaml(node, prefix, path_key);

  // list_parameters uses "." as hierarchy separator - prefix must NOT have trailing dot.
  std::string list_prefix = "plugins." + plugin_name;
  auto result = node->list_parameters({list_prefix}, 10);
  for (const auto & name : result.names) {
    if (name != path_key) {
      insert_nested_param(config, name.substr(prefix.size()), parameter_to_json(node->get_parameter(name)),
                          node->get_logger());
    }
  }
  return config;
}

}  // namespace ros2_medkit_gateway
