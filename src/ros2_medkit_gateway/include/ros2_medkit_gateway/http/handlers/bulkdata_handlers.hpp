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

#include <httplib.h>

#include <string>

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief HTTP handlers for SOVD bulk-data endpoints.
 *
 * Provides handlers for listing bulk-data categories, descriptors,
 * and downloading bulk-data files (rosbags) for any entity type.
 *
 * Supports SOVD entity paths:
 * - /apps/{id}/bulk-data[/{category}[/{id}]]
 * - /components/{id}/bulk-data[/{category}[/{id}]]
 * - /areas/{id}/bulk-data[/{category}[/{id}]]
 * - /functions/{id}/bulk-data[/{category}[/{id}]]
 * - Nested entities (subareas, subcomponents)
 */
class BulkDataHandlers {
 public:
  /**
   * @brief Construct BulkDataHandlers.
   * @param ctx Handler context for sending responses and accessing FaultManager
   */
  explicit BulkDataHandlers(HandlerContext & ctx);

  /**
   * @brief GET {entity-path}/bulk-data - List bulk-data categories.
   *
   * Returns available bulk-data categories for an entity.
   * Currently only "rosbags" category is supported.
   *
   * @param req HTTP request
   * @param res HTTP response
   */
  void handle_list_categories(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief GET {entity-path}/bulk-data/{category} - List bulk-data descriptors.
   *
   * Returns BulkDataDescriptor array for the specified category.
   * For "rosbags" category, returns descriptors for all rosbags
   * associated with faults from this entity.
   *
   * @param req HTTP request
   * @param res HTTP response
   */
  void handle_list_descriptors(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief GET {entity-path}/bulk-data/{category}/{id} - Download bulk-data file.
   *
   * Downloads the bulk-data file (rosbag) identified by the ID.
   * Validates that the rosbag belongs to the specified entity.
   *
   * @param req HTTP request
   * @param res HTTP response
   */
  void handle_download(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Get MIME type for rosbag format.
   * @param format Storage format ("mcap", "sqlite3", "db3")
   * @return MIME type string
   */
  static std::string get_rosbag_mimetype(const std::string & format);

 private:
  HandlerContext & ctx_;

  /**
   * @brief Stream file contents to HTTP response.
   * @param res HTTP response to write to
   * @param file_path Path to file to stream (can be file or rosbag directory)
   * @param content_type MIME type for Content-Type header
   * @return true if successful, false if file could not be read
   */
  bool stream_file_to_response(httplib::Response & res, const std::string & file_path,
                               const std::string & content_type);

  /**
   * @brief Resolve rosbag file path from storage path.
   *
   * Rosbag2 creates a directory containing the actual db3/mcap file.
   * This function resolves the directory to the actual file path.
   *
   * @param path Path to rosbag (can be file or directory)
   * @return Resolved file path, or empty string if not found
   */
  static std::string resolve_rosbag_file_path(const std::string & path);
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
