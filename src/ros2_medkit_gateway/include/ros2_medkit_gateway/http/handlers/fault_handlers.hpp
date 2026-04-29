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

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for fault management REST API endpoints.
 *
 * Provides handlers for:
 * - GET /faults - List all faults (extension)
 * - DELETE /faults - Clear all faults globally (extension)
 * - GET /{entity-path}/faults - List faults for entity
 * - GET /{entity-path}/faults/{code} - Get specific fault
 * - DELETE /{entity-path}/faults/{code} - Clear fault
 * - DELETE /{entity-path}/faults - Clear all faults for entity
 *
 * Note: Snapshot data is inline in fault responses (environment_data).
 * Rosbag downloads use the bulk-data endpoint pattern.
 */
class FaultHandlers {
 public:
  /**
   * @brief Construct fault handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit FaultHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /faults - list all faults.
   */
  void handle_list_all_faults(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/faults - list faults for component.
   */
  void handle_list_faults(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/faults/{fault_code} - get specific fault.
   */
  void handle_get_fault(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /components/{component_id}/faults/{fault_code} - clear fault.
   */
  void handle_clear_fault(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /components/{component_id}/faults - clear all faults for entity.
   */
  void handle_clear_all_faults(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /faults - clear all faults globally (extension, not SOVD).
   *
   * Accepts optional ?status= query parameter to filter which faults to clear.
   */
  void handle_clear_all_faults_global(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Build SOVD-compliant fault response from already-converted JSON.
   *
   * Consumes the JSON shape returned by FaultManager::get_fault_with_env -
   * `fault_json` is the per-fault flat representation (the FaultManager has
   * already translated severity_label, status string, timestamps, etc.) and
   * `env_data_json` carries `extended_data_records` plus a `snapshots` array
   * with intermediate-shape entries (freeze_frame: type/name/data/topic/
   * message_type/captured_at_ns; rosbag: type/name/fault_code/size_bytes/
   * duration_sec/format).
   *
   * Builds the final SOVD response containing:
   * - "item" with fault details and SOVD status object
   * - "environment_data" with extended_data_records and snapshots; for
   *   freeze_frame snapshots the handler parses "data", extracts the primary
   *   value and packs the full payload into x-medkit; for rosbag snapshots
   *   the handler appends `bulk_data_uri = entity_path + "/bulk-data/rosbags/" + fault_code`.
   * - "x-medkit" extensions with occurrence_count, severity_label, etc.
   *
   * @param fault_json Per-fault JSON (as produced by the transport adapter).
   * @param env_data_json Environment-data JSON (as produced by the transport).
   * @param entity_path Entity path used to construct rosbag bulk_data_uri.
   * @return SOVD-compliant JSON response
   */
  static nlohmann::json build_sovd_fault_response(const nlohmann::json & fault_json,
                                                  const nlohmann::json & env_data_json,
                                                  const std::string & entity_path);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
