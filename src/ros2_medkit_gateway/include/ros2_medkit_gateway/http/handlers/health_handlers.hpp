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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Health and system info endpoint handlers
 *
 * Handles:
 * - GET /health - Health check
 * - GET / - Root endpoint with capabilities
 * - GET /version-info - Version information
 */
class HealthHandlers {
 public:
  explicit HealthHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /// GET /health - Health check endpoint
  void handle_health(const httplib::Request & req, httplib::Response & res);

  /// GET / - Root endpoint with server capabilities
  void handle_root(const httplib::Request & req, httplib::Response & res);

  /// GET /version-info - Version information
  void handle_version_info(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
