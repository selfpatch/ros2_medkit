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

#include "ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp"

namespace ros2_medkit_gateway {

std::string lifecycle_status_from_state(const std::optional<std::string> & state_label) {
  return (state_label.has_value() && *state_label == "active") ? "ready" : "notReady";
}

std::optional<std::string> find_lifecycle_get_state_path(const std::vector<ServiceInfo> & services) {
  std::optional<std::string> get_state_path;
  bool has_change_state = false;
  for (const auto & svc : services) {
    if (svc.type == "lifecycle_msgs/srv/GetState") {
      get_state_path = svc.full_path;
    } else if (svc.type == "lifecycle_msgs/srv/ChangeState") {
      has_change_state = true;
    }
  }
  if (get_state_path.has_value() && has_change_state) {
    return get_state_path;
  }
  return std::nullopt;
}

}  // namespace ros2_medkit_gateway
