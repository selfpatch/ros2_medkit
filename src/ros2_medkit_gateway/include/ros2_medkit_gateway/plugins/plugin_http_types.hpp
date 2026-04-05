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

#include <nlohmann/json.hpp>
#include <string>

namespace ros2_medkit_gateway {

/// Thin read-only wrapper over an HTTP request.
/// Hides the underlying HTTP library (currently cpp-httplib) from plugin code.
/// Constructed by the gateway per-request; plugins receive it by const reference.
class PluginRequest {
 public:
  /// Construct from opaque HTTP request pointer (gateway-internal).
  explicit PluginRequest(const void * impl);

  /// Extract a path parameter by regex capture group index.
  /// Index 0 is the full match; index 1 is the first capture group.
  std::string path_param(size_t index) const;

  /// Get a request header value by name. Returns empty string if not present.
  std::string header(const std::string & name) const;

  /// Full request path (e.g. "/api/v1/apps/my_app/data").
  const std::string & path() const;

  /// Request body (by reference - avoids copying large payloads).
  const std::string & body() const;

 private:
  const void * impl_;
};

/// Thin wrapper over an HTTP response.
/// Hides the underlying HTTP library from plugin code.
/// Constructed by the gateway per-request; plugins receive it by reference.
class PluginResponse {
 public:
  /// Construct from opaque HTTP response pointer (gateway-internal).
  explicit PluginResponse(void * impl);

  /// Send a JSON success response (HTTP 200).
  void send_json(const nlohmann::json & data);

  /// Send a SOVD-compliant error response.
  void send_error(int status, const std::string & error_code, const std::string & message,
                  const nlohmann::json & parameters = {});

 private:
  void * impl_;
};

}  // namespace ros2_medkit_gateway
