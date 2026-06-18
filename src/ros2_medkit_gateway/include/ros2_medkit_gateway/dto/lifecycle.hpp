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

#include <string>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"

namespace ros2_medkit_gateway {
namespace dto {

/// Response body for GET /{entity}/status. `status` is "ready"|"notReady";
/// each transition field is the URI for that action, present only when the
/// entity supports it.
struct LifecycleStatusResponse {
  std::string status;
  std::optional<std::string> start;
  std::optional<std::string> restart;
  std::optional<std::string> force_restart;  // wire key "force-restart"
  std::optional<std::string> shutdown;
  std::optional<std::string> force_shutdown;  // wire key "force-shutdown"
};

template <>
inline constexpr auto dto_fields<LifecycleStatusResponse> =
    std::make_tuple(field_enum("status", &LifecycleStatusResponse::status, kLifecycleStatusValues),
                    field("start", &LifecycleStatusResponse::start),
                    field("restart", &LifecycleStatusResponse::restart),
                    field("force-restart", &LifecycleStatusResponse::force_restart),
                    field("shutdown", &LifecycleStatusResponse::shutdown),
                    field("force-shutdown", &LifecycleStatusResponse::force_shutdown));

template <>
inline constexpr std::string_view dto_name<LifecycleStatusResponse> = "LifecycleStatusResponse";

}  // namespace dto
}  // namespace ros2_medkit_gateway
