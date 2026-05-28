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

#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"

#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway {

void log_peer_drop_warning(const char * dto_name, const char * reason) {
  RCLCPP_WARN(rclcpp::get_logger("aggregation"), "fan_out_collection: peer item failed to parse as %s, dropping: %s",
              dto_name, reason);
}

}  // namespace ros2_medkit_gateway
