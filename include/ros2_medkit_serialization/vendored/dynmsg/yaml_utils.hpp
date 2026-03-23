// Copyright 2021 Christophe Bedard
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

#ifndef ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__YAML_UTILS_HPP_
#define ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__YAML_UTILS_HPP_

#include <yaml-cpp/yaml.h>

#include <string>

namespace dynmsg
{

/// Convert YAML node to string.
/**
 * Flow style is one-line, JSON-like.
 * Non-flow style is block style, which is the normal multi-line style.
 *
 * \param yaml the YAML node
 * \param double_quoted whether to double quote all keys and values or not
 * \param flow_style whether to use flow-style or not
 * \return the YAML node as a string
 */
std::string yaml_to_string(
  const YAML::Node & yaml,
  const bool double_quoted = false,
  const bool flow_style = false);

}  // namespace dynmsg

#endif  // ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__YAML_UTILS_HPP_
