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

/// Information about a ROS2 service discovered in the system
struct ServiceInfo {
  std::string name;       // Service name (e.g., "calibrate")
  std::string full_path;  // Full service path (e.g., "/powertrain/engine/calibrate")
  std::string type;       // Service type (e.g., "std_srvs/srv/Trigger")

  json to_json() const {
    return {{"name", name}, {"path", full_path}, {"type", type}, {"kind", "service"}};
  }
};

/// Information about a ROS2 action discovered in the system
struct ActionInfo {
  std::string name;       // Action name (e.g., "navigate_to_pose")
  std::string full_path;  // Full action path (e.g., "/navigation/navigate_to_pose")
  std::string type;       // Action type (e.g., "nav2_msgs/action/NavigateToPose")

  json to_json() const {
    return {{"name", name}, {"path", full_path}, {"type", type}, {"kind", "action"}};
  }
};

struct Component {
  std::string id;
  std::string namespace_path;
  std::string fqn;
  std::string type = "Component";
  std::string area;
  std::vector<ServiceInfo> services;
  std::vector<ActionInfo> actions;

  json to_json() const {
    json j = {{"id", id}, {"namespace", namespace_path}, {"fqn", fqn}, {"type", type}, {"area", area}};

    // Add operations array combining services and actions
    json operations = json::array();
    for (const auto & svc : services) {
      operations.push_back(svc.to_json());
    }
    for (const auto & act : actions) {
      operations.push_back(act.to_json());
    }
    if (!operations.empty()) {
      j["operations"] = operations;
    }

    return j;
  }
};

struct EntityCache {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::chrono::system_clock::time_point last_update;
};

}  // namespace ros2_medkit_gateway
