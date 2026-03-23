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

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

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

  // Require a segment boundary after the collection name:
  // matches "/api/v1/components" or "/api/v1/components/...", but not "/api/v1/componentship".
  auto matches_collection = [&path, &base](const std::string & collection) -> bool {
    const std::string prefix = base + collection;
    if (path.compare(0, prefix.size(), prefix) != 0) {
      return false;
    }
    if (path.size() == prefix.size()) {
      // Exact match: "/api/v1/{collection}"
      return true;
    }
    // Require a '/' segment separator after the collection name
    return path[prefix.size()] == '/';
  };

  if (matches_collection("components")) {
    return SovdEntityType::COMPONENT;
  }
  if (matches_collection("apps")) {
    return SovdEntityType::APP;
  }
  if (matches_collection("areas")) {
    return SovdEntityType::AREA;
  }
  if (matches_collection("functions")) {
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
  bool include_healed = false;
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
    filter.include_healed = false;

    if (status == "pending") {
      filter.include_pending = true;
    } else if (status == "confirmed") {
      filter.include_confirmed = true;
    } else if (status == "cleared") {
      filter.include_cleared = true;
      filter.include_healed = true;  // SOVD maps both CLEARED and HEALED to "cleared"
    } else if (status == "healed") {
      filter.include_healed = true;
    } else if (status == "all") {
      filter.include_pending = true;
      filter.include_confirmed = true;
      filter.include_cleared = true;
      filter.include_healed = true;
    } else {
      filter.is_valid = false;
    }
  }

  return filter;
}

/**
 * @brief Convert nanoseconds since epoch to ISO 8601 string with milliseconds.
 *
 * Shared utility for formatting timestamps in SOVD-compliant responses.
 * Used by fault_handlers and bulkdata_handlers.
 *
 * @param ns Nanoseconds since epoch
 * @return ISO 8601 formatted string (e.g., "2025-01-15T10:30:00.123Z"),
 *         or "1970-01-01T00:00:00.000Z" on conversion failure
 */
inline std::string format_timestamp_ns(int64_t ns) {
  auto seconds = ns / 1'000'000'000;
  auto nanos = ns % 1'000'000'000;

  std::time_t time = static_cast<std::time_t>(seconds);
  std::tm tm_buf{};
  std::tm * tm = gmtime_r(&time, &tm_buf);
  if (!tm) {
    return "1970-01-01T00:00:00.000Z";  // fallback for invalid timestamps
  }

  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", tm);

  // Add milliseconds
  char result[80];
  std::snprintf(result, sizeof(result), "%s.%03dZ", buf, static_cast<int>(nanos / 1'000'000));

  return result;
}

/**
 * @brief Stream file contents to HTTP response using chunked transfer.
 *
 * Generic utility for streaming any file to an HTTP response. The file path
 * must point to an existing regular file (resolve rosbag directories etc.
 * before calling this function).
 *
 * @param res HTTP response to write to
 * @param file_path Path to an existing regular file
 * @param content_type MIME type for Content-Type header
 * @return true if successful, false if file could not be read
 */
inline bool stream_file_to_response(httplib::Response & res, const std::string & file_path,
                                    const std::string & content_type) {
  std::error_code ec;
  auto file_size = std::filesystem::file_size(file_path, ec);
  if (ec) {
    return false;
  }

  static constexpr size_t kChunkSize = 64 * 1024;  // 64 KB chunks

  res.set_content_provider(static_cast<size_t>(file_size), content_type,
                           [file_path](size_t offset, size_t length, httplib::DataSink & sink) -> bool {
                             std::ifstream file(file_path, std::ios::binary);
                             if (!file.is_open()) {
                               return false;
                             }

                             file.seekg(static_cast<std::streamoff>(offset));
                             if (!file.good()) {
                               return false;
                             }

                             size_t remaining = length;
                             std::vector<char> buf(std::min(remaining, kChunkSize));

                             while (remaining > 0 && file.good()) {
                               size_t to_read = std::min(remaining, kChunkSize);
                               file.read(buf.data(), static_cast<std::streamsize>(to_read));
                               auto bytes_read = static_cast<size_t>(file.gcount());
                               if (bytes_read == 0) {
                                 break;
                               }
                               sink.write(buf.data(), bytes_read);
                               remaining -= bytes_read;
                             }

                             return remaining == 0;
                           });
  return true;
}

}  // namespace ros2_medkit_gateway
