// Copyright 2025 bburda
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

namespace ros2_medkit_gateway {

/**
 * @brief SOVD Standard Error Codes
 *
 * These error codes follow the SOVD GenericError schema defined in
 * sovd-openapi-spec/commons/errors.yaml. They should be used in all
 * error responses to ensure compliance with the SOVD specification.
 */

/// Invalid request format or missing required parameters
constexpr const char * ERR_INVALID_REQUEST = "invalid-request";

/// Entity (component, app, area, function) not found
constexpr const char * ERR_ENTITY_NOT_FOUND = "entity-not-found";

/// Resource (operation, configuration, data, fault) not found
constexpr const char * ERR_RESOURCE_NOT_FOUND = "resource-not-found";

/// Operation not found on entity
constexpr const char * ERR_OPERATION_NOT_FOUND = "operation-not-found";

/// Invalid parameter value or type
constexpr const char * ERR_INVALID_PARAMETER = "invalid-parameter";

/// Service temporarily unavailable
constexpr const char * ERR_SERVICE_UNAVAILABLE = "service-unavailable";

/// Internal server error
constexpr const char * ERR_INTERNAL_ERROR = "internal-error";

/// Collection not supported on entity type
constexpr const char * ERR_COLLECTION_NOT_SUPPORTED = "collection-not-supported";

/// Authentication required
constexpr const char * ERR_UNAUTHORIZED = "unauthorized";

/// Access denied / insufficient permissions
constexpr const char * ERR_FORBIDDEN = "forbidden";

/// Generic vendor-specific error (used with vendor_code field)
constexpr const char * ERR_VENDOR_ERROR = "vendor-error";

/**
 * @brief ros2_medkit Vendor Error Codes (x-medkit-*)
 *
 * These are ros2_medkit-specific error codes that provide detailed
 * information about ROS 2-specific failures. When used, the error
 * response should include:
 * - error_code: "vendor-error"
 * - vendor_code: one of these x-medkit-* codes
 */

/// ROS 2 service call timed out or service unavailable
constexpr const char * ERR_X_MEDKIT_ROS2_SERVICE_UNAVAILABLE = "x-medkit-ros2-service-unavailable";

/// ROS 2 action goal was rejected
constexpr const char * ERR_X_MEDKIT_ROS2_ACTION_REJECTED = "x-medkit-ros2-action-rejected";

/// ROS 2 parameter is read-only and cannot be modified
constexpr const char * ERR_X_MEDKIT_ROS2_PARAMETER_READ_ONLY = "x-medkit-ros2-parameter-read-only";

/// Dynamic type introspection failed for ROS 2 message
constexpr const char * ERR_X_MEDKIT_ROS2_TYPE_INTROSPECTION_FAILED = "x-medkit-ros2-type-introspection-failed";

/// ROS 2 node not available or not responding
constexpr const char * ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE = "x-medkit-ros2-node-unavailable";

/// ROS 2 topic not available
constexpr const char * ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE = "x-medkit-ros2-topic-unavailable";

/// ROS 2 action server not available
constexpr const char * ERR_X_MEDKIT_ROS2_ACTION_UNAVAILABLE = "x-medkit-ros2-action-unavailable";

/**
 * @brief Check if an error code is a vendor-specific code
 * @param error_code Error code to check
 * @return true if code starts with "x-medkit-"
 */
inline bool is_vendor_error_code(const std::string & error_code) {
  return error_code.rfind("x-medkit-", 0) == 0;
}

}  // namespace ros2_medkit_gateway
