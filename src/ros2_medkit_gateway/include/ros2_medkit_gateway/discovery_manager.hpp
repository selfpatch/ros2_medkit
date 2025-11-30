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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/models.hpp"

namespace ros2_medkit_gateway {

class DiscoveryManager {
 public:
  explicit DiscoveryManager(rclcpp::Node * node);

  std::vector<Area> discover_areas();
  std::vector<Component> discover_components();

 private:
  std::string extract_area_from_namespace(const std::string & ns);

  rclcpp::Node * node_;
};

}  // namespace ros2_medkit_gateway
