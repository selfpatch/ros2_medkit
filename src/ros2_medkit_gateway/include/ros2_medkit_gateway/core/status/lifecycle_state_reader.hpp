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

#include <optional>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"

namespace ros2_medkit_gateway {

/// Reads the current lifecycle state of a managed (rclcpp_lifecycle) node.
/// Implemented in gateway_ros2 over lifecycle_msgs/srv/GetState; stubbed in tests.
class LifecycleStateReader {
 public:
  virtual ~LifecycleStateReader() = default;

  /// Return the lifecycle state label (e.g. "active", "inactive", "unconfigured")
  /// for the node whose GetState service is at get_state_service_path, or nullopt
  /// if the service is unreachable or the call times out.
  virtual std::optional<std::string> get_state(const std::string & get_state_service_path) = 0;
};

/// Map a lifecycle state label to the SOVD status. "active" -> "ready"; any other
/// state, or a missing/unreachable read (nullopt), -> "notReady".
std::string lifecycle_status_from_state(const std::optional<std::string> & state_label);

/// If services expose both lifecycle_msgs/srv/GetState and lifecycle_msgs/srv/ChangeState
/// (a managed lifecycle node), return the GetState service full_path. Otherwise nullopt.
std::optional<std::string> find_lifecycle_get_state_path(const std::vector<ServiceInfo> & services);

}  // namespace ros2_medkit_gateway
