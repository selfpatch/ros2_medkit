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

#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "ros2_medkit_gateway/core/types/type_schema_provider.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/type_introspection.hpp"

namespace ros2_medkit_gateway::ros2 {

/// TypeSchemaProvider backed by rosidl type support. Owns the one
/// TypeIntrospection and JsonSerializer of the gateway.
class Ros2TypeSchemaProvider : public TypeSchemaProvider {
 public:
  Ros2TypeSchemaProvider();
  ~Ros2TypeSchemaProvider() override = default;

  Ros2TypeSchemaProvider(const Ros2TypeSchemaProvider &) = delete;
  Ros2TypeSchemaProvider & operator=(const Ros2TypeSchemaProvider &) = delete;
  Ros2TypeSchemaProvider(Ros2TypeSchemaProvider &&) = delete;
  Ros2TypeSchemaProvider & operator=(Ros2TypeSchemaProvider &&) = delete;

  std::optional<nlohmann::json> message_schema(const std::string & type_name) override;
  std::optional<nlohmann::json> type_info(const std::string & type_name) override;
  std::shared_ptr<const nlohmann::json> service_type_info(const std::string & type_name) override;
  std::shared_ptr<const nlohmann::json> action_type_info(const std::string & type_name) override;

  /// The shared introspection instance, for ROS-side consumers.
  ros2_medkit_serialization::TypeIntrospection * introspection() {
    return &introspection_;
  }

 private:
  ros2_medkit_serialization::JsonSerializer serializer_;
  ros2_medkit_serialization::TypeIntrospection introspection_;
};

}  // namespace ros2_medkit_gateway::ros2
