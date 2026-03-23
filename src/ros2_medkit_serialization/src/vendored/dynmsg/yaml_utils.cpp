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

#include <yaml-cpp/yaml.h>

#include <string>

#include "ros2_medkit_serialization/vendored/dynmsg/yaml_utils.hpp"

namespace dynmsg
{

std::string yaml_to_string(
  const YAML::Node & yaml,
  const bool double_quoted,
  const bool flow_style)
{
  YAML::Emitter emitter;
  if (double_quoted) {
    emitter << YAML::DoubleQuoted;
  }
  if (flow_style) {
    emitter << YAML::Flow;
  }
  emitter << yaml;
  return emitter.c_str();
}

}  // namespace dynmsg
