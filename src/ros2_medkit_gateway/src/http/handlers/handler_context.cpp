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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

tl::expected<void, std::string> HandlerContext::validate_entity_id(const std::string & entity_id) const {
  // Check for empty string
  if (entity_id.empty()) {
    return tl::unexpected("Entity ID cannot be empty");
  }

  // Check length (reasonable limit to prevent abuse)
  if (entity_id.length() > 256) {
    return tl::unexpected("Entity ID too long (max 256 characters)");
  }

  // Validate characters according to naming conventions
  // Allow: alphanumeric (a-z, A-Z, 0-9), underscore (_), hyphen (-)
  // Reject: forward slash (conflicts with URL routing), special characters, escape sequences
  // Note: Hyphens are allowed in manifest entity IDs (e.g., "engine-ecu", "front-left-door")
  for (char c : entity_id) {
    bool is_alphanumeric = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    bool is_allowed_special = (c == '_' || c == '-');

    if (!is_alphanumeric && !is_allowed_special) {
      // For non-printable characters, show the character code
      std::string char_repr;
      if (c < 32 || c > 126) {
        std::ostringstream oss;
        oss << "0x" << std::hex << std::setfill('0') << std::setw(2)
            << static_cast<unsigned int>(static_cast<unsigned char>(c));
        char_repr = oss.str();
      } else {
        char_repr = std::string(1, c);
      }
      return tl::unexpected("Entity ID contains invalid character: '" + char_repr +
                            "'. Only alphanumeric, underscore and hyphen are allowed");
    }
  }

  return {};
}

tl::expected<std::string, std::string>
HandlerContext::get_component_namespace_path(const std::string & component_id) const {
  const auto cache = node_->get_entity_cache();
  for (const auto & component : cache.components) {
    if (component.id == component_id) {
      return component.namespace_path;
    }
  }
  return tl::unexpected("Component not found");
}

EntityInfo HandlerContext::get_entity_info(const std::string & entity_id) const {
  const auto cache = node_->get_entity_cache();
  EntityInfo info;
  info.id = entity_id;

  // Search components first
  for (const auto & component : cache.components) {
    if (component.id == entity_id) {
      info.type = EntityType::COMPONENT;
      info.namespace_path = component.namespace_path;
      info.fqn = component.fqn;
      info.id_field = "component_id";
      info.error_name = "Component";
      return info;
    }
  }

  // Search apps
  for (const auto & app : cache.apps) {
    if (app.id == entity_id) {
      info.type = EntityType::APP;
      // Apps use bound_fqn as namespace_path for fault filtering
      info.namespace_path = app.bound_fqn.value_or("");
      info.fqn = app.bound_fqn.value_or("");
      info.id_field = "app_id";
      info.error_name = "App";
      return info;
    }
  }

  // Search areas
  for (const auto & area : cache.areas) {
    if (area.id == entity_id) {
      info.type = EntityType::AREA;
      info.namespace_path = "";  // Areas don't have namespace_path
      info.fqn = "";
      info.id_field = "area_id";
      info.error_name = "Area";
      return info;
    }
  }

  // Search functions
  for (const auto & func : cache.functions) {
    if (func.id == entity_id) {
      info.type = EntityType::FUNCTION;
      info.namespace_path = "";
      info.fqn = "";
      info.id_field = "function_id";
      info.error_name = "Function";
      return info;
    }
  }

  // Not found - return UNKNOWN type
  info.type = EntityType::UNKNOWN;
  info.error_name = "Entity";
  return info;
}

void HandlerContext::set_cors_headers(httplib::Response & res, const std::string & origin) const {
  res.set_header("Access-Control-Allow-Origin", origin);

  // Use pre-built header strings from CorsConfig
  if (!cors_config_.methods_header.empty()) {
    res.set_header("Access-Control-Allow-Methods", cors_config_.methods_header);
  }
  if (!cors_config_.headers_header.empty()) {
    res.set_header("Access-Control-Allow-Headers", cors_config_.headers_header);
  }

  // Set credentials header if enabled
  if (cors_config_.allow_credentials) {
    res.set_header("Access-Control-Allow-Credentials", "true");
  }
}

bool HandlerContext::is_origin_allowed(const std::string & origin) const {
  // Check if origin matches any allowed origin
  // Note: Wildcard "*" is allowed here but credentials+wildcard is blocked at startup
  // (see gateway_node.cpp validation). When wildcard is used, we echo back the actual
  // origin for security, as browsers require exact origin match with credentials.
  for (const auto & allowed : cors_config_.allowed_origins) {
    if (allowed == "*" || allowed == origin) {
      return true;
    }
  }
  return false;
}

void HandlerContext::send_error(httplib::Response & res, httplib::StatusCode status, const std::string & error) {
  res.status = status;
  json error_json = {{"error", error}};
  res.set_content(error_json.dump(2), "application/json");
}

void HandlerContext::send_error(httplib::Response & res, httplib::StatusCode status, const std::string & error,
                                const json & extra_fields) {
  res.status = status;
  json error_json = {{"error", error}};
  // Merge extra fields into the error object
  for (const auto & [key, value] : extra_fields.items()) {
    error_json[key] = value;
  }
  res.set_content(error_json.dump(2), "application/json");
}

void HandlerContext::send_json(httplib::Response & res, const json & data) {
  res.set_content(data.dump(2), "application/json");
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
