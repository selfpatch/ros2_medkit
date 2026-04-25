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
#include <string>

namespace ros2_medkit_gateway {

/**
 * @brief Transport-neutral error descriptor used by the SOVD provider interfaces.
 *
 * Returned as the error branch of `tl::expected<T, ErrorInfo>` by provider
 * methods (Data, Operation, Fault, DataStream, Log). The handler layer
 * translates this into a SOVD-compliant GenericError response.
 *
 * @par Contract
 * - `http_status` must be in the SOVD error range (400-599). The handler layer
 *   clamps out-of-range values and logs a warning.
 * - `code` is the SOVD error code constant (see error_codes.hpp) or a
 *   vendor-specific `x-medkit-*` code.
 * - `params` is an optional JSON object with structured diagnostic parameters.
 *   Must be serializable and must not contain sensitive information.
 *
 * @par Thread safety
 * Plain value type; safe to copy and move across threads without synchronization.
 */
struct ErrorInfo {
  std::string code;      ///< SOVD error code (e.g. ERR_RESOURCE_NOT_FOUND, ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE)
  std::string message;   ///< Human-readable message (truncated to 512 chars by handlers)
  int http_status{500};  ///< HTTP status in SOVD range 400-599
  nlohmann::json params = nlohmann::json::object();  ///< Optional structured parameters
};

}  // namespace ros2_medkit_gateway
