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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Unified HTTP handlers for data operations across all SOVD entity types
 *
 * This class provides entity-agnostic handlers for /data endpoints.
 * Works for: /areas/{id}/data, /components/{id}/data, /apps/{id}/data, /functions/{id}/data
 *
 * Key design decisions:
 * - Single handler implementation shared across all entity types
 * - Entity type resolved from URL regex match or inferred from entity ID
 * - Data aggregated from entity hierarchy (e.g., component includes apps' topics)
 */
class DataHandlers {
 public:
  explicit DataHandlers(HandlerContext & ctx)
    : ctx_(ctx) {
  }

  /**
   * @brief List all data items (topics) for an entity
   *
   * GET /{entities}/{id}/data
   *
   * URL matches:
   * - matches[1]: entity_id
   *
   * Returns SOVD ValueMetadata items with x-medkit ROS2-specific extensions.
   */
  void handle_list_data(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Get a single data item (topic value) for an entity
   *
   * GET /{entities}/{id}/data/{topic}
   *
   * URL matches:
   * - matches[1]: entity_id
   * - matches[2]: topic_id (may be URL-encoded with slashes)
   *
   * Returns SOVD ReadValue with current topic data and type_info schema.
   */
  void handle_get_data_item(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Write data to a topic (publish)
   *
   * PUT /{entities}/{id}/data/{topic}
   *
   * URL matches:
   * - matches[1]: entity_id
   * - matches[2]: topic_id (may be URL-encoded with slashes)
   *
   * Request body: { "type": "pkg/msg/Type", "data": {...} }
   * Returns SOVD WriteValue with echoed data.
   */
  void handle_put_data_item(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief List data categories (not implemented for ROS 2)
   *
   * GET /{entities}/{id}/data-categories
   *
   * Returns 501 Not Implemented.
   */
  void handle_data_categories(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief List data groups (not implemented for ROS 2)
   *
   * GET /{entities}/{id}/data-groups
   *
   * Returns 501 Not Implemented.
   */
  void handle_data_groups(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
