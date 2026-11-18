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

#include "ros2_medkit_gateway/ros2/ros2_type_schema_provider.hpp"

#include <exception>
#include <rclcpp/logging.hpp>

namespace ros2_medkit_gateway::ros2 {

namespace {
rclcpp::Logger logger() {
  return rclcpp::get_logger("type_schema_provider");
}
}  // namespace

Ros2TypeSchemaProvider::Ros2TypeSchemaProvider() : serializer_(), introspection_("") {
}

std::optional<nlohmann::json> Ros2TypeSchemaProvider::message_schema(const std::string & type_name) {
  try {
    return serializer_.get_schema(type_name);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger(), "No schema for '%s': %s", type_name.c_str(), e.what());
  } catch (...) {
    RCLCPP_DEBUG(logger(), "No schema for '%s'", type_name.c_str());
  }
  return std::nullopt;
}

std::optional<nlohmann::json> Ros2TypeSchemaProvider::type_info(const std::string & type_name) {
  try {
    auto info = introspection_.get_type_info(type_name);
    return nlohmann::json{{"schema", info.schema}, {"default_value", info.default_value}};
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger(), "Could not get type info for '%s': %s", type_name.c_str(), e.what());
  }
  return std::nullopt;
}

std::shared_ptr<const nlohmann::json> Ros2TypeSchemaProvider::service_type_info(const std::string & type_name) {
  try {
    return introspection_.get_service_type_info(type_name);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger(), "Could not get type info for service '%s': %s", type_name.c_str(), e.what());
  }
  return nullptr;
}

std::shared_ptr<const nlohmann::json> Ros2TypeSchemaProvider::action_type_info(const std::string & type_name) {
  try {
    return introspection_.get_action_type_info(type_name);
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(logger(), "Could not get type info for action '%s': %s", type_name.c_str(), e.what());
  }
  return nullptr;
}

}  // namespace ros2_medkit_gateway::ros2
