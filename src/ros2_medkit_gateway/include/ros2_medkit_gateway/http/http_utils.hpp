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

#include <httplib.h>

#include <string>

#include "ros2_medkit_gateway/models/entity_types.hpp"

namespace ros2_medkit_gateway {

/// API version prefix for all endpoints
constexpr const char * API_BASE_PATH = "/api/v1";

/**
 * @brief Build versioned endpoint path
 * @param endpoint The endpoint path (e.g., "/health")
 * @return Full API path (e.g., "/api/v1/health")
 */
inline std::string api_path(const std::string & endpoint) {
  return std::string(API_BASE_PATH) + endpoint;
}

/**
 * @brief Extract expected entity type from request path
 *
 * Parses the URL path to determine which entity type the route expects.
 * Used for semantic validation - ensuring /components/{id} only accepts
 * component IDs, not app IDs.
 *
 * @param path Request path (e.g., "/api/v1/components/my_component/data")
 * @return Expected entity type, or UNKNOWN if path doesn't match entity routes
 */
inline SovdEntityType extract_entity_type_from_path(const std::string & path) {
  // Path format: /api/v1/{entity_type}/{id}/...
  // Check for each entity type prefix after API base path
  const std::string base = std::string(API_BASE_PATH) + "/";

  if (path.find(base + "components/") == 0 || path.find(base + "components") == 0) {
    return SovdEntityType::COMPONENT;
  }
  if (path.find(base + "apps/") == 0 || path.find(base + "apps") == 0) {
    return SovdEntityType::APP;
  }
  if (path.find(base + "areas/") == 0 || path.find(base + "areas") == 0) {
    return SovdEntityType::AREA;
  }
  if (path.find(base + "functions/") == 0 || path.find(base + "functions") == 0) {
    return SovdEntityType::FUNCTION;
  }

  return SovdEntityType::UNKNOWN;
}

/**
 * @brief Fault status filter flags for fault listing endpoints
 */
struct FaultStatusFilter {
  bool include_pending = true;
  bool include_confirmed = true;
  bool include_cleared = false;
  bool is_valid = true;
};

/**
 * @brief Parse fault status query parameter from request
 * @param req HTTP request
 * @return Filter flags and validity. If status param is invalid, is_valid=false.
 */
inline FaultStatusFilter parse_fault_status_param(const httplib::Request & req) {
  FaultStatusFilter filter;

  if (req.has_param("status")) {
    std::string status = req.get_param_value("status");
    // Reset defaults when explicit status filter is provided
    filter.include_pending = false;
    filter.include_confirmed = false;
    filter.include_cleared = false;

    if (status == "pending") {
      filter.include_pending = true;
    } else if (status == "confirmed") {
      filter.include_confirmed = true;
    } else if (status == "cleared") {
      filter.include_cleared = true;
    } else if (status == "all") {
      filter.include_pending = true;
      filter.include_confirmed = true;
      filter.include_cleared = true;
    } else {
      filter.is_valid = false;
    }
  }

  return filter;
}

}  // namespace ros2_medkit_gateway
