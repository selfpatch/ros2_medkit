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

#include "ros2_medkit_gateway/core/http/parameter_error_classification.hpp"

#include "ros2_medkit_gateway/core/http/error_codes.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Map a structured `ParameterErrorCode` to an HTTP status + SOVD error code.
ParameterErrorClassification classify_error_code(ParameterErrorCode error_code) {
  ParameterErrorClassification result;

  switch (error_code) {
    case ParameterErrorCode::NOT_FOUND:
    case ParameterErrorCode::NO_DEFAULTS_CACHED:
      result.status_code = 404;
      result.error_code = ERR_RESOURCE_NOT_FOUND;
      break;
    case ParameterErrorCode::READ_ONLY:
      result.status_code = 403;
      result.error_code = ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY;
      break;
    case ParameterErrorCode::SERVICE_UNAVAILABLE:
    case ParameterErrorCode::TIMEOUT:
      result.status_code = 503;
      result.error_code = ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
      break;
    case ParameterErrorCode::TYPE_MISMATCH:
    case ParameterErrorCode::INVALID_VALUE:
      result.status_code = 400;
      result.error_code = ERR_INVALID_PARAMETER;
      break;
    case ParameterErrorCode::NONE:
      // A failed ParameterResult that still carries NONE cannot occur on the
      // shipped transport (every failure path sets a structured code), so a
      // NONE failure is a gateway-side defect: surface it as 500 internal-error.
      // The removed legacy string-matching fallback was unreachable, and would
      // have misrouted messages it did not recognise (e.g. "did not respond",
      // "not currently set") to 400 invalid-request had it ever run.
      result.status_code = 500;
      result.error_code = ERR_INTERNAL_ERROR;
      break;
    case ParameterErrorCode::SHUT_DOWN:
    case ParameterErrorCode::INTERNAL_ERROR:
    default:
      result.status_code = 500;
      result.error_code = ERR_INTERNAL_ERROR;
      break;
  }

  return result;
}

}  // namespace

ParameterErrorClassification classify_parameter_error(const ParameterResult & result) {
  return classify_error_code(result.error_code);
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
