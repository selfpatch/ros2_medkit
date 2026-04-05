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

#include <rcl/arguments.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway {

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
          } else {
            RCLCPP_WARN(node->get_logger(), "Skipping param '%s': unsupported type (array?)", pname.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_DEBUG(node->get_logger(), "Could not declare param '%s': %s", pname.c_str(), e.what());
        }
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
