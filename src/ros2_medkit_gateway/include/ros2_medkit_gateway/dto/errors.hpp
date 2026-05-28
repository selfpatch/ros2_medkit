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

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// GenericError mirrors SchemaBuilder::generic_error():
//   required:   error_code, message
//   optional:   parameters (free-form JSON object - schema says {"type":"object"})
struct GenericError {
  std::string error_code;
  std::string message;
  std::optional<nlohmann::json> parameters;
};

template <>
inline constexpr auto dto_fields<GenericError> =
    std::make_tuple(field("error_code", &GenericError::error_code), field("message", &GenericError::message),
                    field("parameters", &GenericError::parameters));
template <>
inline constexpr std::string_view dto_name<GenericError> = "GenericError";

}  // namespace dto
}  // namespace ros2_medkit_gateway
