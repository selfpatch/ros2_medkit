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

#include "route_registry.hpp"

#include <algorithm>
#include <regex>
#include <set>
#include <string>

namespace ros2_medkit_gateway {
namespace openapi {

// -----------------------------------------------------------------------------
// RouteEntry fluent methods
// -----------------------------------------------------------------------------

RouteEntry & RouteEntry::tag(const std::string & t) {
  tag_ = t;
  return *this;
}

RouteEntry & RouteEntry::summary(const std::string & s) {
  summary_ = s;
  return *this;
}

RouteEntry & RouteEntry::description(const std::string & desc) {
  description_ = desc;
  return *this;
}

RouteEntry & RouteEntry::response(int status_code, const std::string & desc) {
  responses_[status_code] = {desc, {}};
  return *this;
}

RouteEntry & RouteEntry::response(int status_code, const std::string & desc, const nlohmann::json & schema) {
  responses_[status_code] = {desc, schema};
  return *this;
}

RouteEntry & RouteEntry::request_body(const std::string & desc, const nlohmann::json & schema) {
  request_body_ = RequestBodyInfo{desc, schema};
  return *this;
}

RouteEntry & RouteEntry::path_param(const std::string & name, const std::string & desc) {
  nlohmann::json param;
  param["name"] = name;
  param["in"] = "path";
  param["required"] = true;
  param["description"] = desc;
  param["schema"] = {{"type", "string"}};
  parameters_.push_back(std::move(param));
  return *this;
}

RouteEntry & RouteEntry::query_param(const std::string & name, const std::string & desc, const std::string & type) {
  nlohmann::json param;
  param["name"] = name;
  param["in"] = "query";
  param["required"] = false;
  param["description"] = desc;
  param["schema"] = {{"type", type}};
  parameters_.push_back(std::move(param));
  return *this;
}

RouteEntry & RouteEntry::deprecated() {
  deprecated_ = true;
  return *this;
}

// -----------------------------------------------------------------------------
// RouteRegistry route registration
// -----------------------------------------------------------------------------

RouteEntry & RouteRegistry::add_route(const std::string & method, const std::string & openapi_path, HandlerFn handler) {
  RouteEntry entry;
  entry.method_ = method;
  entry.path_ = openapi_path;
  entry.regex_path_ = to_regex_path(openapi_path, method);
  entry.handler_ = std::move(handler);
  routes_.push_back(std::move(entry));
  return routes_.back();
}

RouteEntry & RouteRegistry::get(const std::string & openapi_path, HandlerFn handler) {
  return add_route("get", openapi_path, std::move(handler));
}

RouteEntry & RouteRegistry::post(const std::string & openapi_path, HandlerFn handler) {
  return add_route("post", openapi_path, std::move(handler));
}

RouteEntry & RouteRegistry::put(const std::string & openapi_path, HandlerFn handler) {
  return add_route("put", openapi_path, std::move(handler));
}

RouteEntry & RouteRegistry::del(const std::string & openapi_path, HandlerFn handler) {
  return add_route("delete", openapi_path, std::move(handler));
}

// -----------------------------------------------------------------------------
// to_regex_path - convert OpenAPI {param} path to cpp-httplib regex
// -----------------------------------------------------------------------------

std::string RouteRegistry::to_regex_path(const std::string & openapi_path, const std::string & /*method*/) {
  // We need to convert {param} placeholders to regex capture groups.
  // Special cases:
  //   - {data_id} at the end of data paths -> (.+) (multi-segment, for slash-containing topic names)
  //   - {config_id} at the end of configuration paths -> (.+) (for slash-containing param names)
  //   - All other {param} -> ([^/]+) (single segment)
  //
  // The "end of path" check ensures only the LAST param on data/config paths gets (.+).

  std::string result;
  size_t i = 0;
  while (i < openapi_path.size()) {
    if (openapi_path[i] == '{') {
      auto close = openapi_path.find('}', i);
      if (close == std::string::npos) {
        result += openapi_path[i];
        ++i;
        continue;
      }
      std::string param_name = openapi_path.substr(i + 1, close - i - 1);
      bool is_last = (close + 1 >= openapi_path.size());

      // Use (.+) for the final segment on data and configuration item paths
      if (is_last && (param_name == "data_id" || param_name == "config_id")) {
        result += "(.+)";
      } else {
        result += "([^/]+)";
      }
      i = close + 1;
    } else {
      result += openapi_path[i];
      ++i;
    }
  }

  // Append $ anchor to ensure exact match
  result += "$";
  return result;
}

// -----------------------------------------------------------------------------
// register_all - register all routes with cpp-httplib server
// -----------------------------------------------------------------------------

void RouteRegistry::register_all(httplib::Server & server, const std::string & api_prefix) const {
  for (const auto & route : routes_) {
    std::string full_path = api_prefix + route.regex_path_;
    const auto & handler = route.handler_;

    if (route.method_ == "get") {
      server.Get(full_path.c_str(), handler);
    } else if (route.method_ == "post") {
      server.Post(full_path.c_str(), handler);
    } else if (route.method_ == "put") {
      server.Put(full_path.c_str(), handler);
    } else if (route.method_ == "delete") {
      server.Delete(full_path.c_str(), handler);
    }
  }
}

// -----------------------------------------------------------------------------
// to_openapi_paths - generate OpenAPI paths object
// -----------------------------------------------------------------------------

nlohmann::json RouteRegistry::to_openapi_paths() const {
  nlohmann::json paths = nlohmann::json::object();

  for (const auto & route : routes_) {
    nlohmann::json operation;

    if (!route.tag_.empty()) {
      operation["tags"] = nlohmann::json::array({route.tag_});
    }
    if (!route.summary_.empty()) {
      operation["summary"] = route.summary_;
    }
    if (!route.description_.empty()) {
      operation["description"] = route.description_;
    }
    if (route.deprecated_) {
      operation["deprecated"] = true;
    }

    // Parameters
    if (!route.parameters_.empty()) {
      operation["parameters"] = route.parameters_;
    }

    // Also extract path parameters from the path template and add them
    // if they were not explicitly added via path_param()
    {
      std::set<std::string> explicit_params;
      for (const auto & p : route.parameters_) {
        if (p.value("in", "") == "path") {
          explicit_params.insert(p.value("name", ""));
        }
      }

      // Find all {param} in the path
      std::string::size_type pos = 0;
      while ((pos = route.path_.find('{', pos)) != std::string::npos) {
        auto close = route.path_.find('}', pos);
        if (close == std::string::npos) {
          break;
        }
        std::string pname = route.path_.substr(pos + 1, close - pos - 1);
        if (explicit_params.find(pname) == explicit_params.end()) {
          // Auto-generate path parameter
          nlohmann::json param;
          param["name"] = pname;
          param["in"] = "path";
          param["required"] = true;
          param["schema"] = {{"type", "string"}};
          if (!operation.contains("parameters")) {
            operation["parameters"] = nlohmann::json::array();
          }
          operation["parameters"].push_back(std::move(param));
        }
        pos = close + 1;
      }
    }

    // Request body
    if (route.request_body_.has_value()) {
      operation["requestBody"]["description"] = route.request_body_->desc;
      if (!route.request_body_->schema.empty()) {
        operation["requestBody"]["content"]["application/json"]["schema"] = route.request_body_->schema;
      }
      operation["requestBody"]["required"] = true;
    }

    // Responses
    if (!route.responses_.empty()) {
      for (const auto & [code, info] : route.responses_) {
        std::string code_str = std::to_string(code);
        operation["responses"][code_str]["description"] = info.desc;
        if (!info.schema.empty()) {
          operation["responses"][code_str]["content"]["application/json"]["schema"] = info.schema;
        }
      }
    } else {
      // Default 200 response
      operation["responses"]["200"]["description"] = "Successful response";
    }

    // Add standard error responses
    operation["responses"]["400"]["description"] = "Bad request";
    operation["responses"]["404"]["description"] = "Not found";
    operation["responses"]["500"]["description"] = "Internal server error";

    // Check if any route has Authentication tag - add auth error responses
    bool has_auth_tag = false;
    for (const auto & r : routes_) {
      if (r.tag_ == "Authentication") {
        has_auth_tag = true;
        break;
      }
    }
    if (has_auth_tag) {
      operation["responses"]["401"]["description"] = "Unauthorized";
      operation["responses"]["403"]["description"] = "Forbidden";
    }

    paths[route.path_][route.method_] = std::move(operation);
  }

  return paths;
}

// -----------------------------------------------------------------------------
// tags - collect unique tags
// -----------------------------------------------------------------------------

std::vector<std::string> RouteRegistry::tags() const {
  std::set<std::string> tag_set;
  for (const auto & route : routes_) {
    if (!route.tag_.empty()) {
      tag_set.insert(route.tag_);
    }
  }
  return {tag_set.begin(), tag_set.end()};
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
