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

#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

struct Area {
  std::string id;
  std::string namespace_path;
  std::string type = "Area";

  json to_json() const {
    return {{"id", id}, {"namespace", namespace_path}, {"type", type}};
  }
};

struct Component {
  std::string id;
  std::string namespace_path;
  std::string fqn;
  std::string type = "Component";
  std::string area;

  json to_json() const {
    return {{"id", id}, {"namespace", namespace_path}, {"fqn", fqn}, {"type", type}, {"area", area}};
  }
};

struct EntityCache {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::chrono::system_clock::time_point last_update;
};

}  // namespace ros2_medkit_gateway
