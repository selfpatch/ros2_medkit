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

#include <functional>
#include <httplib.h>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace openapi {

using HandlerFn = std::function<void(const httplib::Request &, httplib::Response &)>;

/// Fluent builder for a single route entry.
class RouteEntry {
 public:
  RouteEntry & tag(const std::string & t);
  RouteEntry & summary(const std::string & s);
  RouteEntry & description(const std::string & desc);
  RouteEntry & response(int status_code, const std::string & desc);
  RouteEntry & response(int status_code, const std::string & desc, const nlohmann::json & schema);
  RouteEntry & request_body(const std::string & desc, const nlohmann::json & schema);
  RouteEntry & path_param(const std::string & name, const std::string & desc);
  RouteEntry & query_param(const std::string & name, const std::string & desc, const std::string & type = "string");
  RouteEntry & deprecated();

 private:
  friend class RouteRegistry;
  std::string method_;
  std::string path_;        // OpenAPI path (with {param} style)
  std::string regex_path_;  // cpp-httplib regex path (with capture groups)
  std::string tag_;
  std::string summary_;
  std::string description_;
  HandlerFn handler_;
  bool deprecated_{false};

  struct ResponseInfo {
    std::string desc;
    nlohmann::json schema;
  };
  std::map<int, ResponseInfo> responses_;

  struct RequestBodyInfo {
    std::string desc;
    nlohmann::json schema;
  };
  std::optional<RequestBodyInfo> request_body_;

  std::vector<nlohmann::json> parameters_;
};

/// Central registry: single source of truth for routes + OpenAPI metadata.
/// Routes registered here are both served via cpp-httplib AND documented in OpenAPI.
class RouteRegistry {
 public:
  /// Register a GET route.
  RouteEntry & get(const std::string & openapi_path, HandlerFn handler);
  /// Register a POST route.
  RouteEntry & post(const std::string & openapi_path, HandlerFn handler);
  /// Register a PUT route.
  RouteEntry & put(const std::string & openapi_path, HandlerFn handler);
  /// Register a DELETE route (using "del" to avoid keyword conflict).
  RouteEntry & del(const std::string & openapi_path, HandlerFn handler);

  /// Register all routes with cpp-httplib server.
  void register_all(httplib::Server & server, const std::string & api_prefix) const;

  /// Generate OpenAPI paths object from all registered routes.
  nlohmann::json to_openapi_paths() const;

  /// Get all unique tags (for OpenAPI tags section).
  std::vector<std::string> tags() const;

  /// Number of registered routes.
  size_t size() const {
    return routes_.size();
  }

 private:
  /// Convert OpenAPI path to cpp-httplib regex path.
  /// e.g., "/apps/{app_id}/data/{data_id}" -> "/apps/([^/]+)/data/(.+)"
  static std::string to_regex_path(const std::string & openapi_path, const std::string & method);

  RouteEntry & add_route(const std::string & method, const std::string & openapi_path, HandlerFn handler);

  std::vector<RouteEntry> routes_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
