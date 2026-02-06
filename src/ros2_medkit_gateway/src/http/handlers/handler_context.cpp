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
#include "ros2_medkit_gateway/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/models/entity_types.hpp"

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
  const auto & cache = node_->get_thread_safe_cache();
  auto component = cache.get_component(component_id);
  if (component) {
    return component->namespace_path;
  }
  return tl::unexpected("Component not found");
}

EntityInfo HandlerContext::get_entity_info(const std::string & entity_id, SovdEntityType expected_type) const {
  const auto & cache = node_->get_thread_safe_cache();
  EntityInfo info;
  info.id = entity_id;

  // If expected_type is specified, search ONLY in that collection
  // This prevents ID collisions (e.g., Area "powertrain" vs Component "powertrain")
  if (expected_type != SovdEntityType::UNKNOWN) {
    switch (expected_type) {
      case SovdEntityType::COMPONENT:
        if (auto component = cache.get_component(entity_id)) {
          info.type = EntityType::COMPONENT;
          info.namespace_path = component->namespace_path;
          info.fqn = component->fqn;
          info.id_field = "component_id";
          info.error_name = "Component";
          return info;
        }
        break;

      case SovdEntityType::APP:
        if (auto app = cache.get_app(entity_id)) {
          info.type = EntityType::APP;
          info.namespace_path = app->bound_fqn.value_or("");
          info.fqn = app->bound_fqn.value_or("");
          info.id_field = "app_id";
          info.error_name = "App";
          return info;
        }
        break;

      case SovdEntityType::AREA:
        if (auto area = cache.get_area(entity_id)) {
          info.type = EntityType::AREA;
          info.namespace_path = area->namespace_path;
          info.fqn = area->namespace_path;
          info.id_field = "area_id";
          info.error_name = "Area";
          return info;
        }
        break;

      case SovdEntityType::FUNCTION:
        if (auto func = cache.get_function(entity_id)) {
          info.type = EntityType::FUNCTION;
          info.namespace_path = "";
          info.fqn = "";
          info.id_field = "function_id";
          info.error_name = "Function";
          return info;
        }
        break;

      case SovdEntityType::SERVER:
      default:
        break;
    }
    // Not found in expected collection
    info.type = EntityType::UNKNOWN;
    info.error_name = "Entity";
    return info;
  }

  // No expected_type specified - search all collections in order (legacy behavior)
  // Search components first (O(1) lookup)
  if (auto component = cache.get_component(entity_id)) {
    info.type = EntityType::COMPONENT;
    info.namespace_path = component->namespace_path;
    info.fqn = component->fqn;
    info.id_field = "component_id";
    info.error_name = "Component";
    return info;
  }

  // Search apps (O(1) lookup)
  if (auto app = cache.get_app(entity_id)) {
    info.type = EntityType::APP;
    // Apps use bound_fqn as namespace_path for fault filtering
    info.namespace_path = app->bound_fqn.value_or("");
    info.fqn = app->bound_fqn.value_or("");
    info.id_field = "app_id";
    info.error_name = "App";
    return info;
  }

  // Search areas (O(1) lookup)
  if (auto area = cache.get_area(entity_id)) {
    info.type = EntityType::AREA;
    info.namespace_path = area->namespace_path;  // Use area's namespace for fault filtering
    info.fqn = area->namespace_path;
    info.id_field = "area_id";
    info.error_name = "Area";
    return info;
  }

  // Search functions (O(1) lookup)
  if (auto func = cache.get_function(entity_id)) {
    info.type = EntityType::FUNCTION;
    info.namespace_path = "";
    info.fqn = "";
    info.id_field = "function_id";
    info.error_name = "Function";
    return info;
  }

  // Not found - return UNKNOWN type
  info.type = EntityType::UNKNOWN;
  info.error_name = "Entity";
  return info;
}

std::optional<std::string> HandlerContext::validate_collection_access(const EntityInfo & entity,
                                                                      ResourceCollection collection) {
  auto caps = EntityCapabilities::for_type(entity.sovd_type());

  if (!caps.supports_collection(collection)) {
    return entity.error_name + " entities do not support " + to_string(collection) + " collection";
  }

  return std::nullopt;
}

std::optional<EntityInfo> HandlerContext::validate_entity_for_route(const httplib::Request & req,
                                                                    httplib::Response & res,
                                                                    const std::string & entity_id) const {
  // Step 1: Validate entity ID format
  auto validation_result = validate_entity_id(entity_id);
  if (!validation_result) {
    send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
               {{"details", validation_result.error()}, {"entity_id", entity_id}});
    return std::nullopt;
  }

  // Step 2: Get expected type from route path and look up entity
  auto expected_type = extract_entity_type_from_path(req.path);
  auto entity_info = get_entity_info(entity_id, expected_type);

  if (entity_info.type == EntityType::UNKNOWN) {
    // Step 3: Check if entity exists in ANY collection (for better error message)
    auto any_entity = get_entity_info(entity_id);
    if (any_entity.type != EntityType::UNKNOWN) {
      // Entity exists but wrong type for this route -> 400
      send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                 "Invalid entity type for route: expected " + to_string(expected_type) + ", got " +
                     to_string(any_entity.sovd_type()),
                 {{"entity_id", entity_id},
                  {"expected_type", to_string(expected_type)},
                  {"actual_type", to_string(any_entity.sovd_type())}});
    } else {
      // Entity doesn't exist at all -> 404
      send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found", {{"entity_id", entity_id}});
    }
    return std::nullopt;
  }

  return entity_info;
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

  // Expose headers that JavaScript needs access to (e.g., for file downloads)
  res.set_header("Access-Control-Expose-Headers", "Content-Disposition, Content-Length");

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

void HandlerContext::send_error(httplib::Response & res, httplib::StatusCode status, const std::string & error_code,
                                const std::string & message, const json & parameters) {
  res.status = status;
  json error_json;

  // Handle vendor-specific error codes (x-medkit-*)
  if (is_vendor_error_code(error_code)) {
    error_json["error_code"] = ERR_VENDOR_ERROR;
    error_json["vendor_code"] = error_code;
  } else {
    error_json["error_code"] = error_code;
  }

  error_json["message"] = message;

  // SOVD GenericError schema (7.4.2) requires additional info in 'parameters' field
  if (!parameters.empty()) {
    error_json["parameters"] = parameters;
  }

  res.set_content(error_json.dump(2), "application/json");
}

void HandlerContext::send_json(httplib::Response & res, const json & data) {
  res.set_content(data.dump(2), "application/json");
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
