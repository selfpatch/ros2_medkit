// Copyright 2026 sewon
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

namespace ros2_medkit_gateway {

std::string normalize_fault_manager_namespace(const std::string & namespace_value);
std::string get_fault_manager_namespace(rclcpp::Node * node);
std::string build_fault_manager_base_path(const std::string & namespace_value);
std::string build_fault_manager_base_path(rclcpp::Node * node);
std::string build_fault_manager_events_topic(const std::string & namespace_value);
std::string build_fault_manager_events_topic(rclcpp::Node * node);

}  // namespace ros2_medkit_gateway
