// Copyright 2025 mfaferek93
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

#include <yaml-cpp/yaml.h>  // NOLINT(build/include_order)

#include <nlohmann/json.hpp>
#include <string>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

class OutputParser {
 public:
  OutputParser() = default;

  /**
   * @brief Parse YAML string to JSON
   */
  json parse_yaml(const std::string & yaml_str);

 private:
  json yaml_to_json(const YAML::Node & node);
};

}  // namespace ros2_medkit_gateway
