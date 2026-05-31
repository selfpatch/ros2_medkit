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

#include "ros2_medkit_gateway/dto/data.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

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
 *
 * PR-403 commit 28: all 5 data routes migrate to the typed RouteRegistry API.
 * The list endpoint uses the typed `fan_out_collection<DataItem>` from commit 7
 * for peer aggregation (per-item wire shape now enforced by
 * `JsonReader<DataItem>`, closing the issue #338 gap on this endpoint). Read
 * responses return `DataValue` whose payload is an opaque object (live ROS
 * message JSON), and write responses return a bare DataValue-shaped object so
 * the wire bytes match the legacy `send_json` path. The 501 stubs
 * (data-categories / data-groups) keep their byte-identical SOVD error shape.
 */
class DataHandlers {
 public:
  explicit DataHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief List all data items (topics) for an entity.
   *
   * GET /{entities}/{id}/data
   *
   * Returns the opaque `DataListResult` envelope. For runtime entities the
   * handler builds a typed `Collection<DataItem, DataListXMedkit>` (SOVD
   * ValueMetadata items with x-medkit ROS2 extensions plus typed fan-out via
   * `fan_out_collection<DataItem>`) and serializes it into the envelope, so the
   * wire shape is unchanged. For plugin entities the provider's free-form item
   * shape (which may carry vendor per-item fields the gateway cannot describe at
   * compile time) is passed through verbatim, instead of being re-parsed into
   * `Collection<DataItem>` (which would silently drop those fields).
   */
  http::Result<dto::DataListResult> list_data(const http::TypedRequest & req);

  /**
   * @brief Get a single data item (topic value) for an entity.
   *
   * GET /{entities}/{id}/data/{topic}
   *
   * Returns a `DataValue` whose payload is the SOVD ReadValue object with the
   * current topic data + type_info schema (opaque envelope: shape depends on
   * the underlying ROS message and the plugin backend).
   */
  http::Result<dto::DataValue> get_data_item(const http::TypedRequest & req);

  /**
   * @brief Write data to a topic (publish).
   *
   * PUT /{entities}/{id}/data/{topic}
   *
   * Request body for the ROS path: `DataWriteRequest` ({ type: "pkg/msg/Type",
   * data: {...} }), parsed manually inside the handler. Plugin-owned entities
   * see the raw JSON body verbatim (lenient parse) - plugins like UDS expect a
   * hex-encoded bare string, not the `{type, data}` shape. Body-less typed
   * registration is used so the framework does not enforce a single schema on
   * both paths; the OpenAPI request-body schema is attached separately in the
   * route registration.
   *
   * Returns a `DataValue` whose payload is the SOVD WriteValue echo with the
   * x-medkit publish status (opaque envelope: per-publish metadata varies by
   * backend).
   */
  http::Result<dto::DataValue> put_data_item(const http::TypedRequest & req);

  /**
   * @brief List data categories (not implemented for ROS 2).
   *
   * GET /{entities}/{id}/data-categories - returns 501 Not Implemented.
   */
  http::Result<dto::DataValue> data_categories(const http::TypedRequest & req);

  /**
   * @brief List data groups (not implemented for ROS 2).
   *
   * GET /{entities}/{id}/data-groups - returns 501 Not Implemented.
   */
  http::Result<dto::DataValue> data_groups(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
