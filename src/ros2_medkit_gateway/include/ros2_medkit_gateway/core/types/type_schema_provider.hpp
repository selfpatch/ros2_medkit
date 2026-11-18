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

namespace ros2_medkit_gateway {

/// Message, service and action schemas by type name ("pkg/msg/Name",
/// "pkg/srv/Name", "pkg/action/Name"). An unknown type yields an empty result.
/// Implementations are thread-safe.
class TypeSchemaProvider {
 public:
  TypeSchemaProvider() = default;
  virtual ~TypeSchemaProvider() = default;
  TypeSchemaProvider(const TypeSchemaProvider &) = delete;
  TypeSchemaProvider & operator=(const TypeSchemaProvider &) = delete;
  TypeSchemaProvider(TypeSchemaProvider &&) = delete;
  TypeSchemaProvider & operator=(TypeSchemaProvider &&) = delete;

  /// JSON Schema of a message type.
  virtual std::optional<nlohmann::json> message_schema(const std::string & type_name) = 0;
  /// {"schema", "default_value"} of a message type.
  virtual std::optional<nlohmann::json> type_info(const std::string & type_name) = 0;
  /// Request and response schemas of a service type. Shared, not copied.
  virtual std::shared_ptr<const nlohmann::json> service_type_info(const std::string & type_name) = 0;
  /// Goal, result and feedback schemas of an action type. Shared, not copied.
  virtual std::shared_ptr<const nlohmann::json> action_type_info(const std::string & type_name) = 0;
};

}  // namespace ros2_medkit_gateway
