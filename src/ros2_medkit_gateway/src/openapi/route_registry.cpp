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
#include <set>
#include <string>
#include <unordered_map>

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

RouteEntry & RouteEntry::operation_id(const std::string & id) {
  operation_id_ = id;
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
      server.Get(full_path, handler);
    } else if (route.method_ == "post") {
      server.Post(full_path, handler);
    } else if (route.method_ == "put") {
      server.Put(full_path, handler);
    } else if (route.method_ == "delete") {
      server.Delete(full_path, handler);
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
          // Auto-generate path parameter with description
          static const std::unordered_map<std::string, std::string> kParamDescriptions = {
              {"area_id", "The area identifier"},
              {"component_id", "The component identifier"},
              {"app_id", "The app identifier"},
              {"function_id", "The function identifier"},
              {"data_id", "The data item identifier (ROS 2 topic name)"},
              {"operation_id", "The operation identifier"},
              {"execution_id", "The execution identifier"},
              {"config_id", "The configuration parameter identifier (ROS 2 parameter name)"},
              {"fault_code", "The fault code identifier"},
              {"subscription_id", "The cyclic subscription identifier"},
              {"category_id", "The bulk data category identifier"},
              {"file_id", "The bulk data file identifier"},
              {"update_id", "The software update identifier"},
              {"subarea_id", "The subarea identifier"},
              {"subcomponent_id", "The subcomponent identifier"},
              {"trigger_id", "The trigger identifier"},
              {"lock_id", "The lock identifier"},
              {"script_id", "The script identifier"},
          };
          nlohmann::json param;
          param["name"] = pname;
          param["in"] = "path";
          param["required"] = true;
          param["schema"] = {{"type", "string"}};
          auto desc_it = kParamDescriptions.find(pname);
          param["description"] = (desc_it != kParamDescriptions.end()) ? desc_it->second : "The " + pname + " value";
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

    // Add standard error responses as $ref to GenericError component.
    // Response-level $ref (not nested in content/schema) - the referenced
    // component is a complete response object with description and schema.
    auto add_error_ref = [&operation](const std::string & code) {
      if (!operation["responses"].contains(code)) {
        operation["responses"][code] = {{"$ref", "#/components/responses/GenericError"}};
      }
    };

    add_error_ref("400");
    add_error_ref("404");
    add_error_ref("500");

    if (auth_enabled_) {
      add_error_ref("401");
      add_error_ref("403");
    }

    // Use explicit operationId if set, otherwise auto-generate camelCase from path
    if (!route.operation_id_.empty()) {
      operation["operationId"] = route.operation_id_;
    } else {
      // Auto-generate: strip {param} segments, camelCase remaining path segments
      // e.g., GET /faults/stream -> getFaultsStream
      std::string op_id = route.method_;
      bool next_upper = false;
      bool in_param = false;
      for (char c : route.path_) {
        if (c == '{') {
          in_param = true;
          continue;
        }
        if (c == '}') {
          in_param = false;
          continue;
        }
        if (in_param) {
          continue;
        }
        if (c == '/' || c == '-') {
          next_upper = true;
        } else {
          if (next_upper) {
            op_id += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            next_upper = false;
          } else {
            op_id += c;
          }
        }
      }
      operation["operationId"] = op_id;
    }

    paths[route.path_][route.method_] = std::move(operation);
  }

  return paths;
}

// -----------------------------------------------------------------------------
// tags - collect unique tags
// -----------------------------------------------------------------------------

std::vector<std::string> RouteRegistry::to_endpoint_list(const std::string & api_prefix) const {
  std::vector<std::string> endpoints;
  endpoints.reserve(routes_.size());
  for (const auto & route : routes_) {
    // Uppercase the method
    std::string method = route.method_;
    std::transform(method.begin(), method.end(), method.begin(), [](unsigned char c) {
      return std::toupper(c);
    });
    std::string endpoint = method;
    endpoint += " ";
    endpoint += api_prefix;
    endpoint += route.path_;
    endpoints.push_back(std::move(endpoint));
  }
  return endpoints;
}

std::vector<std::string> RouteRegistry::tags() const {
  std::set<std::string> tag_set;
  for (const auto & route : routes_) {
    if (!route.tag_.empty()) {
      tag_set.insert(route.tag_);
    }
  }
  return {tag_set.begin(), tag_set.end()};
}

// -----------------------------------------------------------------------------
// validate_completeness - check all routes have required OpenAPI metadata
// -----------------------------------------------------------------------------

std::vector<ValidationIssue> RouteRegistry::validate_completeness() const {
  std::vector<ValidationIssue> issues;

  for (const auto & route : routes_) {
    std::string method_upper = route.method_;
    std::transform(method_upper.begin(), method_upper.end(), method_upper.begin(), [](unsigned char c) {
      return std::toupper(c);
    });
    std::string route_id = method_upper + " " + route.path_;

    // Every route must have a tag
    if (route.tag_.empty()) {
      issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing tag"});
    }

    // Check response schemas for non-DELETE methods
    if (route.method_ != "delete") {
      bool has_success_response_with_schema = false;
      for (const auto & [code, info] : route.responses_) {
        if (code >= 200 && code < 300 && !info.schema.empty()) {
          has_success_response_with_schema = true;
          break;
        }
      }

      // SSE endpoints use text/event-stream, not JSON schema - skip schema check
      // Convention: SSE endpoints have "SSE" or "stream" in summary
      bool is_sse = route.summary_.find("SSE") != std::string::npos ||
                    route.summary_.find("stream") != std::string::npos ||
                    route.summary_.find("Stream") != std::string::npos;

      // 204 No Content responses don't need a schema
      bool has_204 = route.responses_.count(204) > 0;

      // Endpoints that only return errors (e.g., 405) don't need success schemas
      bool has_only_error_responses = !route.responses_.empty();
      for (const auto & [code, info] : route.responses_) {
        if (code < 400) {
          has_only_error_responses = false;
          break;
        }
      }

      if (!has_success_response_with_schema && !is_sse && !has_204 && !has_only_error_responses) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing response schema for success (2xx)"});
      }
    } else {
      // DELETE must have an explicit response code
      if (route.responses_.empty()) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "DELETE missing explicit response code"});
      }
    }

    // POST/PUT must have request_body
    if ((route.method_ == "post" || route.method_ == "put") && !route.request_body_.has_value()) {
      // Exception: endpoints returning 405 (method not allowed) don't need request body
      bool is_405 = route.responses_.count(405) > 0;
      // Exception: PUT endpoints returning 204 (e.g., log config) don't need request body schema
      // if they also don't have a success schema - they may accept a body but it's optional
      if (!is_405) {
        issues.push_back({ValidationIssue::Severity::kError, route_id, "Missing request body definition"});
      }
    }

    // Warnings for missing summary/description
    if (route.summary_.empty()) {
      issues.push_back({ValidationIssue::Severity::kWarning, route_id, "Missing summary"});
    }
  }

  return issues;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
