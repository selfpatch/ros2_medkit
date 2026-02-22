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
#include "ros2_medkit_gateway/updates/update_manager.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief HTTP handlers for software update endpoints (/updates).
 *
 * All endpoints are server-level (no entity path). Without a loaded
 * backend plugin, all endpoints return 501 Not Implemented.
 */
class UpdateHandlers {
 public:
  UpdateHandlers(HandlerContext & ctx, UpdateManager * update_manager);

  void handle_list_updates(const httplib::Request & req, httplib::Response & res);
  void handle_get_update(const httplib::Request & req, httplib::Response & res);
  void handle_register_update(const httplib::Request & req, httplib::Response & res);
  void handle_delete_update(const httplib::Request & req, httplib::Response & res);
  void handle_prepare(const httplib::Request & req, httplib::Response & res);
  void handle_execute(const httplib::Request & req, httplib::Response & res);
  void handle_automated(const httplib::Request & req, httplib::Response & res);
  void handle_get_status(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
  UpdateManager * update_mgr_;

  /// Check backend loaded, send 501 if not. Returns true if OK.
  bool check_backend(httplib::Response & res);

  /// Convert UpdateStatusInfo to JSON
  static nlohmann::json status_to_json(const UpdateStatusInfo & status);
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
