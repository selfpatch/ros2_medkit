// Copyright 2025 mfaferek93
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

#include "ros2_medkit_gateway/rest_server.hpp"

#include <algorithm>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#include "ros2_medkit_gateway/exceptions.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {

// API version prefix for all endpoints
static constexpr const char * API_BASE_PATH = "/api/v1";

// Helper to build versioned endpoint path
inline std::string api_path(const std::string & endpoint) {
  return std::string(API_BASE_PATH) + endpoint;
}

RESTServer::RESTServer(GatewayNode * node, const std::string & host, int port, const CorsConfig & cors_config)
  : node_(node), host_(host), port_(port), cors_config_(cors_config) {
  server_ = std::make_unique<httplib::Server>();

  // Set up pre-routing handler for CORS (only if enabled)
  if (cors_config_.enabled) {
    server_->set_pre_routing_handler([this](const httplib::Request & req, httplib::Response & res) {
      std::string origin = req.get_header_value("Origin");
      bool origin_allowed = !origin.empty() && is_origin_allowed(origin);

      if (origin_allowed) {
        set_cors_headers(res, origin);
      }

      // Handle preflight OPTIONS requests
      // Return 204 for allowed origins, 403 for disallowed (prevents endpoint discovery)
      if (req.method == "OPTIONS") {
        if (origin_allowed) {
          res.set_header("Access-Control-Max-Age", std::to_string(cors_config_.max_age_seconds));
          res.status = StatusCode::NoContent_204;
        } else {
          res.status = StatusCode::Forbidden_403;
        }
        return httplib::Server::HandlerResponse::Handled;
      }
      return httplib::Server::HandlerResponse::Unhandled;
    });
  }

  setup_routes();
}

RESTServer::~RESTServer() {
  stop();
}

void RESTServer::setup_routes() {
  // Health check
  server_->Get(api_path("/health").c_str(), [this](const httplib::Request & req, httplib::Response & res) {
    handle_health(req, res);
  });

  // Root - server capabilities and entry points (REQ_INTEROP_010)
  server_->Get(api_path("/").c_str(), [this](const httplib::Request & req, httplib::Response & res) {
    handle_root(req, res);
  });

  // Version info (REQ_INTEROP_001)
  server_->Get(api_path("/version-info").c_str(), [this](const httplib::Request & req, httplib::Response & res) {
    handle_version_info(req, res);
  });

  // Areas
  server_->Get(api_path("/areas").c_str(), [this](const httplib::Request & req, httplib::Response & res) {
    handle_list_areas(req, res);
  });

  // Components
  server_->Get(api_path("/components").c_str(), [this](const httplib::Request & req, httplib::Response & res) {
    handle_list_components(req, res);
  });

  // Area components
  server_->Get((api_path("/areas") + R"(/([^/]+)/components)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_area_components(req, res);
               });

  // Component topic data (specific topic) - register before general route
  // Use (.+) for topic_name to accept slashes from percent-encoded URLs (%2F -> /)
  server_->Get((api_path("/components") + R"(/([^/]+)/data/(.+)$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_component_topic_data(req, res);
               });

  // Component data (all topics)
  server_->Get((api_path("/components") + R"(/([^/]+)/data$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_component_data(req, res);
               });

  // Component topic publish (PUT)
  // Use (.+) for topic_name to accept slashes from percent-encoded URLs (%2F -> /)
  server_->Put((api_path("/components") + R"(/([^/]+)/data/(.+)$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_component_topic_publish(req, res);
               });

  // Component operation (POST) - sync operations like service calls, async action goals
  server_->Post((api_path("/components") + R"(/([^/]+)/operations/([^/]+)$)").c_str(),
                [this](const httplib::Request & req, httplib::Response & res) {
                  handle_component_operation(req, res);
                });

  // Action status (GET) - get current status of an action goal
  server_->Get((api_path("/components") + R"(/([^/]+)/operations/([^/]+)/status$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_action_status(req, res);
               });

  // Action result (GET) - get result of a completed action goal
  server_->Get((api_path("/components") + R"(/([^/]+)/operations/([^/]+)/result$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_action_result(req, res);
               });

  // Action cancel (DELETE) - cancel a running action goal
  server_->Delete((api_path("/components") + R"(/([^/]+)/operations/([^/]+)$)").c_str(),
                  [this](const httplib::Request & req, httplib::Response & res) {
                    handle_action_cancel(req, res);
                  });

  // Configurations endpoints - SOVD Configurations API mapped to ROS2 parameters
  // List all configurations (parameters) for a component
  server_->Get((api_path("/components") + R"(/([^/]+)/configurations$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_list_configurations(req, res);
               });

  // Get specific configuration (parameter) - register before general route
  server_->Get((api_path("/components") + R"(/([^/]+)/configurations/([^/]+)$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_get_configuration(req, res);
               });

  // Set configuration (parameter)
  server_->Put((api_path("/components") + R"(/([^/]+)/configurations/([^/]+)$)").c_str(),
               [this](const httplib::Request & req, httplib::Response & res) {
                 handle_set_configuration(req, res);
               });

  // Delete configuration - not supported in ROS2 (returns 405)
  server_->Delete((api_path("/components") + R"(/([^/]+)/configurations/([^/]+)$)").c_str(),
                  [this](const httplib::Request & req, httplib::Response & res) {
                    handle_delete_configuration(req, res);
                  });
}

void RESTServer::start() {
  RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Starting REST server on %s:%d...", host_.c_str(), port_);

  server_->listen(host_.c_str(), port_);
}

void RESTServer::stop() {
  if (server_ && server_->is_running()) {
    RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Stopping REST server...");
    server_->stop();
  }
}

std::expected<void, std::string> RESTServer::validate_entity_id(const std::string & entity_id) const {
  // Check for empty string
  if (entity_id.empty()) {
    return std::unexpected("Entity ID cannot be empty");
  }

  // Check length (reasonable limit to prevent abuse)
  if (entity_id.length() > 256) {
    return std::unexpected("Entity ID too long (max 256 characters)");
  }

  // Validate characters according to ROS 2 naming conventions
  // Allow: alphanumeric (a-z, A-Z, 0-9), underscore (_)
  // Reject: hyphen (not allowed in ROS 2 names), forward slash (conflicts with URL routing),
  //         special characters, escape sequences
  for (char c : entity_id) {
    bool is_alphanumeric = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    bool is_allowed_special = (c == '_');

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
      return std::unexpected("Entity ID contains invalid character: '" + char_repr +
                             "'. Only alphanumeric and underscore are allowed");
    }
  }

  return {};
}

void RESTServer::handle_health(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    json response = {{"status", "healthy"}, {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}};

    res.set_content(response.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Internal server error"}}.dump(), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_health: %s", e.what());
  }
}

void RESTServer::handle_root(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    json response = {
        {"name", "ROS 2 Medkit Gateway"},
        {"version", "0.1.0"},
        {"api_base", API_BASE_PATH},
        {"endpoints",
         json::array({"GET /api/v1/health", "GET /api/v1/version-info", "GET /api/v1/areas", "GET /api/v1/components",
                      "GET /api/v1/areas/{area_id}/components", "GET /api/v1/components/{component_id}/data",
                      "GET /api/v1/components/{component_id}/data/{topic_name}",
                      "PUT /api/v1/components/{component_id}/data/{topic_name}",
                      "POST /api/v1/components/{component_id}/operations/{operation_name}",
                      "GET /api/v1/components/{component_id}/operations/{operation_name}/status",
                      "GET /api/v1/components/{component_id}/operations/{operation_name}/result",
                      "DELETE /api/v1/components/{component_id}/operations/{operation_name}",
                      "GET /api/v1/components/{component_id}/configurations",
                      "GET /api/v1/components/{component_id}/configurations/{param_name}",
                      "PUT /api/v1/components/{component_id}/configurations/{param_name}"})},
        {"capabilities",
         {{"discovery", true},
          {"data_access", true},
          {"operations", true},
          {"async_actions", true},
          {"configurations", true}}}};

    res.set_content(response.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Internal server error"}}.dump(), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_root: %s", e.what());
  }
}

void RESTServer::handle_version_info(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    json response = {{"status", "ROS 2 Medkit Gateway running"},
                     {"version", "0.1.0"},
                     {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}};

    res.set_content(response.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Internal server error"}}.dump(), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_version_info: %s", e.what());
  }
}

void RESTServer::handle_list_areas(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    const auto cache = node_->get_entity_cache();

    json areas_json = json::array();
    for (const auto & area : cache.areas) {
      areas_json.push_back(area.to_json());
    }

    res.set_content(areas_json.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Internal server error"}}.dump(), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_list_areas: %s", e.what());
  }
}

void RESTServer::handle_list_components(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    const auto cache = node_->get_entity_cache();

    json components_json = json::array();
    for (const auto & component : cache.components) {
      components_json.push_back(component.to_json());
    }

    res.set_content(components_json.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Internal server error"}}.dump(), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_list_components: %s", e.what());
  }
}

void RESTServer::handle_area_components(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract area_id from URL path
    if (req.matches.size() < 2) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    std::string area_id = req.matches[1];

    // Validate area_id
    auto validation_result = validate_entity_id(area_id);
    if (!validation_result) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          json{{"error", "Invalid area ID"}, {"details", validation_result.error()}, {"area_id", area_id}}.dump(2),
          "application/json");
      return;
    }

    const auto cache = node_->get_entity_cache();

    // Check if area exists
    bool area_exists = false;
    for (const auto & area : cache.areas) {
      if (area.id == area_id) {
        area_exists = true;
        break;
      }
    }

    if (!area_exists) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Area not found"}, {"area_id", area_id}}.dump(2), "application/json");
      return;
    }

    // Filter components by area
    json components_json = json::array();
    for (const auto & component : cache.components) {
      if (component.area == area_id) {
        components_json.push_back(component.to_json());
      }
    }

    res.set_content(components_json.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Internal server error"}}.dump(), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_area_components: %s", e.what());
  }
}

void RESTServer::handle_component_data(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  try {
    // Extract component_id from URL path
    if (req.matches.size() < 2) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];

    // Validate component_id
    auto validation_result = validate_entity_id(component_id);
    if (!validation_result) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"},
                           {"details", validation_result.error()},
                           {"component_id", component_id}}
                          .dump(2),
                      "application/json");
      return;
    }

    const auto cache = node_->get_entity_cache();

    // Find component in cache
    std::string component_namespace;
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        component_namespace = component.namespace_path;
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    // Get component data from DataAccessManager (with fallback to metadata)
    // Use namespace_path to find topics (topics are relative to namespace, not FQN)
    auto data_access_mgr = node_->get_data_access_manager();
    json component_data = data_access_mgr->get_component_data_with_fallback(component_namespace);

    res.set_content(component_data.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(
        json{{"error", "Failed to retrieve component data"}, {"details", e.what()}, {"component_id", component_id}}
            .dump(2),
        "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_component_data for component '%s': %s",
                 component_id.c_str(), e.what());
  }
}

void RESTServer::handle_component_topic_data(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string topic_name;
  try {
    // Extract component_id and topic_name from URL path
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    // e.g., "powertrain%2Fengine%2Ftemperature" -> "powertrain/engine/temperature"
    topic_name = req.matches[2];

    // Validate component_id
    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"},
                           {"details", component_validation.error()},
                           {"component_id", component_id}}
                          .dump(2),
                      "application/json");
      return;
    }

    // Skip topic_name validation - it may contain slashes after URL decoding
    // The actual validation happens when we try to find the topic in the ROS graph

    const auto cache = node_->get_entity_cache();

    // Find component in cache - only needed to verify it exists
    // We use the full topic path from the URL, not the component namespace
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    // cpp-httplib has already decoded %2F to / in topic_name
    // Now just add leading slash to make it a full ROS topic path
    // e.g., "powertrain/engine/temperature" -> "/powertrain/engine/temperature"
    std::string full_topic_path = "/" + topic_name;

    // Get topic data from DataAccessManager (with fallback to metadata if data unavailable)
    // Uses topic_sample_timeout_sec parameter (default: 1.0s)
    auto data_access_mgr = node_->get_data_access_manager();
    json topic_data = data_access_mgr->get_topic_sample_with_fallback(full_topic_path);

    res.set_content(topic_data.dump(2), "application/json");
  } catch (const TopicNotAvailableException & e) {
    // Topic doesn't exist or metadata retrieval failed
    res.status = StatusCode::NotFound_404;
    res.set_content(
        json{{"error", "Topic not found"}, {"component_id", component_id}, {"topic_name", topic_name}}.dump(2),
        "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Topic not available for component '%s', topic '%s': %s",
                 component_id.c_str(), topic_name.c_str(), e.what());
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to retrieve topic data"},
                         {"details", e.what()},
                         {"component_id", component_id},
                         {"topic_name", topic_name}}
                        .dump(2),
                    "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"),
                 "Error in handle_component_topic_data for component '%s', topic '%s': %s", component_id.c_str(),
                 topic_name.c_str(), e.what());
  }
}

void RESTServer::handle_component_topic_publish(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string topic_name;
  try {
    // Extract component_id and topic_name from URL path
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    // e.g., "chassis%2Fbrakes%2Fcommand" -> "chassis/brakes/command"
    topic_name = req.matches[2];

    // Validate component_id
    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"},
                           {"details", component_validation.error()},
                           {"component_id", component_id}}
                          .dump(2),
                      "application/json");
      return;
    }

    // Skip topic_name validation - it may contain slashes after URL decoding
    // The actual validation happens when we try to publish to the topic

    // Parse request body
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid JSON in request body"}, {"details", e.what()}}.dump(2),
                      "application/json");
      return;
    }

    // Validate required fields: type and data
    if (!body.contains("type") || !body["type"].is_string()) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Missing or invalid 'type' field"},
                           {"details", "Request body must contain 'type' string field"}}
                          .dump(2),
                      "application/json");
      return;
    }

    if (!body.contains("data")) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          json{{"error", "Missing 'data' field"}, {"details", "Request body must contain 'data' field"}}.dump(2),
          "application/json");
      return;
    }

    std::string msg_type = body["type"].get<std::string>();
    json data = body["data"];

    // Validate message type format (e.g., std_msgs/msg/Float32)
    // Expected format: package/msg/Type (exactly 2 slashes)
    size_t slash_count = std::count(msg_type.begin(), msg_type.end(), '/');
    size_t msg_pos = msg_type.find("/msg/");
    bool valid_format = (slash_count == 2) && (msg_pos != std::string::npos) &&
                        (msg_pos > 0) &&                    // package before /msg/
                        (msg_pos + 5 < msg_type.length());  // Type after /msg/

    if (!valid_format) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid message type format"},
                           {"details", "Message type should be in format: package/msg/Type"},
                           {"type", msg_type}}
                          .dump(2),
                      "application/json");
      return;
    }

    const auto cache = node_->get_entity_cache();

    // Find component in cache - only needed to verify it exists
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    // cpp-httplib has already decoded %2F to / in topic_name
    // Now just add leading slash to make it a full ROS topic path
    // e.g., "chassis/brakes/command" -> "/chassis/brakes/command"
    std::string full_topic_path = "/" + topic_name;

    // Publish data using DataAccessManager
    auto data_access_mgr = node_->get_data_access_manager();
    json result = data_access_mgr->publish_to_topic(full_topic_path, msg_type, data);

    // Add component info to result
    result["component_id"] = component_id;
    result["topic_name"] = topic_name;

    res.set_content(result.dump(2), "application/json");
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to publish to topic"},
                         {"details", e.what()},
                         {"component_id", component_id},
                         {"topic_name", topic_name}}
                        .dump(2),
                    "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"),
                 "Error in handle_component_topic_publish for component '%s', topic '%s': %s", component_id.c_str(),
                 topic_name.c_str(), e.what());
  }
}

void RESTServer::handle_component_operation(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string operation_name;
  try {
    // Extract component_id and operation_name from URL path
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate component_id
    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"},
                           {"details", component_validation.error()},
                           {"component_id", component_id}}
                          .dump(2),
                      "application/json");
      return;
    }

    // Validate operation_name
    auto operation_validation = validate_entity_id(operation_name);
    if (!operation_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid operation name"},
                           {"details", operation_validation.error()},
                           {"operation_name", operation_name}}
                          .dump(2),
                      "application/json");
      return;
    }

    // Parse request body (optional for services with no parameters)
    json body = json::object();
    if (!req.body.empty()) {
      try {
        body = json::parse(req.body);
      } catch (const json::parse_error & e) {
        res.status = StatusCode::BadRequest_400;
        res.set_content(json{{"error", "Invalid JSON in request body"}, {"details", e.what()}}.dump(2),
                        "application/json");
        return;
      }
    }

    // Extract optional type override and request data
    std::optional<std::string> service_type;
    json request_data = json::object();

    if (body.contains("type") && body["type"].is_string()) {
      service_type = body["type"].get<std::string>();
    }

    if (body.contains("request")) {
      request_data = body["request"];
    }

    const auto cache = node_->get_entity_cache();

    // Find component in cache
    std::string component_namespace;
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        component_namespace = component.namespace_path;
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    // Check if operation is a service or action
    auto operation_mgr = node_->get_operation_manager();
    auto discovery_mgr = node_->get_discovery_manager();

    // First, check if it's an action
    auto action_info = discovery_mgr->find_action(component_namespace, operation_name);
    if (action_info.has_value()) {
      // Extract goal data (from 'goal' field or root object)
      json goal_data = json::object();
      if (body.contains("goal")) {
        goal_data = body["goal"];
      } else if (!body.contains("type") && !body.contains("request")) {
        // If no 'type' or 'request', treat the whole body as goal (without system fields)
        goal_data = body;
      }

      std::optional<std::string> action_type;
      if (body.contains("type") && body["type"].is_string()) {
        action_type = body["type"].get<std::string>();
      }

      auto action_result =
          operation_mgr->send_component_action_goal(component_namespace, operation_name, action_type, goal_data);

      if (action_result.success && action_result.goal_accepted) {
        auto tracked = operation_mgr->get_tracked_goal(action_result.goal_id);
        std::string status_str = tracked ? action_status_to_string(tracked->status) : "accepted";

        json response = {{"status", "success"},
                         {"kind", "action"},
                         {"component_id", component_id},
                         {"operation", operation_name},
                         {"goal_id", action_result.goal_id},
                         {"goal_status", status_str}};
        res.set_content(response.dump(2), "application/json");
      } else if (action_result.success && !action_result.goal_accepted) {
        res.status = StatusCode::BadRequest_400;
        res.set_content(
            json{{"status", "rejected"},
                 {"kind", "action"},
                 {"component_id", component_id},
                 {"operation", operation_name},
                 {"error", action_result.error_message.empty() ? "Goal rejected" : action_result.error_message}}
                .dump(2),
            "application/json");
      } else {
        res.status = StatusCode::InternalServerError_500;
        res.set_content(json{{"status", "error"},
                             {"kind", "action"},
                             {"component_id", component_id},
                             {"operation", operation_name},
                             {"error", action_result.error_message}}
                            .dump(2),
                        "application/json");
      }
      return;
    }

    // Otherwise, it's a service call
    auto result =
        operation_mgr->call_component_service(component_namespace, operation_name, service_type, request_data);

    if (result.success) {
      json response = {{"status", "success"},
                       {"kind", "service"},
                       {"component_id", component_id},
                       {"operation", operation_name},
                       {"response", result.response}};
      res.set_content(response.dump(2), "application/json");
    } else {
      res.status = StatusCode::InternalServerError_500;
      res.set_content(json{{"status", "error"},
                           {"kind", "service"},
                           {"component_id", component_id},
                           {"operation", operation_name},
                           {"error", result.error_message}}
                          .dump(2),
                      "application/json");
    }
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to execute operation"},
                         {"details", e.what()},
                         {"component_id", component_id},
                         {"operation_name", operation_name}}
                        .dump(2),
                    "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"),
                 "Error in handle_component_operation for component '%s', operation '%s': %s", component_id.c_str(),
                 operation_name.c_str(), e.what());
  }
}

void RESTServer::handle_action_status(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string operation_name;
  try {
    // Extract component_id and operation_name from URL path
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate IDs
    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"}, {"details", component_validation.error()}}.dump(2),
                      "application/json");
      return;
    }

    auto operation_validation = validate_entity_id(operation_name);
    if (!operation_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid operation name"}, {"details", operation_validation.error()}}.dump(2),
                      "application/json");
      return;
    }

    auto operation_mgr = node_->get_operation_manager();

    // Check query parameters
    std::string goal_id;
    bool get_all = false;
    if (req.has_param("goal_id")) {
      goal_id = req.get_param_value("goal_id");
    }
    if (req.has_param("all") && req.get_param_value("all") == "true") {
      get_all = true;
    }

    // If specific goal_id provided, return that goal's status
    if (!goal_id.empty()) {
      auto goal_info = operation_mgr->get_tracked_goal(goal_id);
      if (!goal_info.has_value()) {
        res.status = StatusCode::NotFound_404;
        res.set_content(json{{"error", "Goal not found"}, {"goal_id", goal_id}}.dump(2), "application/json");
        return;
      }

      json response = {{"goal_id", goal_info->goal_id},
                       {"status", action_status_to_string(goal_info->status)},
                       {"action_path", goal_info->action_path},
                       {"action_type", goal_info->action_type}};
      if (!goal_info->last_feedback.is_null() && !goal_info->last_feedback.empty()) {
        response["last_feedback"] = goal_info->last_feedback;
      }
      res.set_content(response.dump(2), "application/json");
      return;
    }

    // No goal_id provided - find goals by action path
    // First, find the component to get its namespace
    auto discovery_mgr = node_->get_discovery_manager();
    auto components = discovery_mgr->discover_components();
    std::optional<Component> component;
    for (const auto & c : components) {
      if (c.id == component_id) {
        component = c;
        break;
      }
    }
    if (!component.has_value()) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    // Build the action path: namespace + operation_name
    std::string action_path = component->namespace_path + "/" + operation_name;

    if (get_all) {
      // Return all goals for this action
      auto goals = operation_mgr->get_goals_for_action(action_path);
      json goals_array = json::array();
      for (const auto & goal : goals) {
        json goal_json = {{"goal_id", goal.goal_id},
                          {"status", action_status_to_string(goal.status)},
                          {"action_path", goal.action_path},
                          {"action_type", goal.action_type}};
        if (!goal.last_feedback.is_null() && !goal.last_feedback.empty()) {
          goal_json["last_feedback"] = goal.last_feedback;
        }
        goals_array.push_back(goal_json);
      }
      json response = {{"action_path", action_path}, {"goals", goals_array}, {"count", goals.size()}};
      res.set_content(response.dump(2), "application/json");
    } else {
      // Return the most recent goal for this action
      auto goal_info = operation_mgr->get_latest_goal_for_action(action_path);
      if (!goal_info.has_value()) {
        res.status = StatusCode::NotFound_404;
        res.set_content(json{{"error", "No goals found for this action"}, {"action_path", action_path}}.dump(2),
                        "application/json");
        return;
      }

      json response = {{"goal_id", goal_info->goal_id},
                       {"status", action_status_to_string(goal_info->status)},
                       {"action_path", goal_info->action_path},
                       {"action_type", goal_info->action_type}};
      if (!goal_info->last_feedback.is_null() && !goal_info->last_feedback.empty()) {
        response["last_feedback"] = goal_info->last_feedback;
      }
      res.set_content(response.dump(2), "application/json");
    }
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to get action status"}, {"details", e.what()}}.dump(2), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_action_status: %s", e.what());
  }
}

void RESTServer::handle_action_result(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string operation_name;
  try {
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate IDs
    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"}, {"details", component_validation.error()}}.dump(2),
                      "application/json");
      return;
    }

    // Get goal_id from query parameter
    std::string goal_id;
    if (req.has_param("goal_id")) {
      goal_id = req.get_param_value("goal_id");
    }

    if (goal_id.empty()) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Missing goal_id query parameter"}}.dump(2), "application/json");
      return;
    }

    // Get tracked goal info to find action path and type
    auto operation_mgr = node_->get_operation_manager();
    auto goal_info = operation_mgr->get_tracked_goal(goal_id);

    if (!goal_info.has_value()) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Goal not found"}, {"goal_id", goal_id}}.dump(2), "application/json");
      return;
    }

    // Get the result (this may block until the action completes)
    auto result = operation_mgr->get_action_result(goal_info->action_path, goal_info->action_type, goal_id);

    if (result.success) {
      json response = {
          {"goal_id", goal_id}, {"status", action_status_to_string(result.status)}, {"result", result.result}};
      res.set_content(response.dump(2), "application/json");
    } else {
      res.status = StatusCode::InternalServerError_500;
      res.set_content(json{{"error", "Failed to get action result"}, {"details", result.error_message}}.dump(2),
                      "application/json");
    }
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to get action result"}, {"details", e.what()}}.dump(2), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_action_result: %s", e.what());
  }
}

void RESTServer::handle_action_cancel(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string operation_name;
  try {
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    operation_name = req.matches[2];

    // Validate IDs
    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"}, {"details", component_validation.error()}}.dump(2),
                      "application/json");
      return;
    }

    // Get goal_id from query parameter
    std::string goal_id;
    if (req.has_param("goal_id")) {
      goal_id = req.get_param_value("goal_id");
    }

    if (goal_id.empty()) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Missing goal_id query parameter"}}.dump(2), "application/json");
      return;
    }

    // Get tracked goal info to find action path
    auto operation_mgr = node_->get_operation_manager();
    auto goal_info = operation_mgr->get_tracked_goal(goal_id);

    if (!goal_info.has_value()) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Goal not found"}, {"goal_id", goal_id}}.dump(2), "application/json");
      return;
    }

    // Cancel the action
    auto result = operation_mgr->cancel_action_goal(goal_info->action_path, goal_id);

    if (result.success && result.return_code == 0) {
      json response = {{"status", "canceling"}, {"goal_id", goal_id}, {"message", "Cancel request accepted"}};
      res.set_content(response.dump(2), "application/json");
    } else {
      res.status = StatusCode::BadRequest_400;
      std::string error_msg;
      switch (result.return_code) {
        case 1:
          error_msg = "Cancel request rejected";
          break;
        case 2:
          error_msg = "Unknown goal ID";
          break;
        case 3:
          error_msg = "Goal already terminated";
          break;
        default:
          error_msg = result.error_message.empty() ? "Cancel failed" : result.error_message;
      }
      res.set_content(json{{"error", error_msg}, {"goal_id", goal_id}, {"return_code", result.return_code}}.dump(2),
                      "application/json");
    }
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to cancel action"}, {"details", e.what()}}.dump(2), "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_action_cancel: %s", e.what());
  }
}

void RESTServer::handle_list_configurations(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  try {
    if (req.matches.size() < 2) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];

    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"},
                           {"details", component_validation.error()},
                           {"component_id", component_id}}
                          .dump(2),
                      "application/json");
      return;
    }

    const auto cache = node_->get_entity_cache();

    // Find component to get its namespace and node name
    std::string node_name;
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        node_name = component.namespace_path + "/" + component.id;
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    auto config_mgr = node_->get_configuration_manager();
    auto result = config_mgr->list_parameters(node_name);

    if (result.success) {
      json response = {{"component_id", component_id}, {"node_name", node_name}, {"parameters", result.data}};
      res.set_content(response.dump(2), "application/json");
    } else {
      res.status = StatusCode::ServiceUnavailable_503;
      res.set_content(
          json{{"error", "Failed to list parameters"}, {"details", result.error_message}, {"node_name", node_name}}
              .dump(2),
          "application/json");
    }
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(
        json{{"error", "Failed to list configurations"}, {"details", e.what()}, {"component_id", component_id}}.dump(2),
        "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_list_configurations for component '%s': %s",
                 component_id.c_str(), e.what());
  }
}

void RESTServer::handle_get_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string param_name;
  try {
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    param_name = req.matches[2];

    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"},
                           {"details", component_validation.error()},
                           {"component_id", component_id}}
                          .dump(2),
                      "application/json");
      return;
    }

    // Parameter names may contain dots, so we use a more permissive validation
    if (param_name.empty() || param_name.length() > 256) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          json{{"error", "Invalid parameter name"}, {"details", "Parameter name is empty or too long"}}.dump(2),
          "application/json");
      return;
    }

    const auto cache = node_->get_entity_cache();

    std::string node_name;
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        node_name = component.namespace_path + "/" + component.id;
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    auto config_mgr = node_->get_configuration_manager();
    auto result = config_mgr->get_parameter(node_name, param_name);

    if (result.success) {
      json response = {{"component_id", component_id}, {"parameter", result.data}};
      res.set_content(response.dump(2), "application/json");
    } else {
      // Check if it's a "not found" error
      if (result.error_message.find("not found") != std::string::npos ||
          result.error_message.find("Parameter not found") != std::string::npos) {
        res.status = StatusCode::NotFound_404;
      } else {
        res.status = StatusCode::ServiceUnavailable_503;
      }
      res.set_content(json{{"error", "Failed to get parameter"},
                           {"details", result.error_message},
                           {"component_id", component_id},
                           {"param_name", param_name}}
                          .dump(2),
                      "application/json");
    }
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to get configuration"},
                         {"details", e.what()},
                         {"component_id", component_id},
                         {"param_name", param_name}}
                        .dump(2),
                    "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"),
                 "Error in handle_get_configuration for component '%s', param '%s': %s", component_id.c_str(),
                 param_name.c_str(), e.what());
  }
}

void RESTServer::handle_set_configuration(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string param_name;
  try {
    if (req.matches.size() < 3) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid request"}}.dump(2), "application/json");
      return;
    }

    component_id = req.matches[1];
    param_name = req.matches[2];

    auto component_validation = validate_entity_id(component_id);
    if (!component_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid component ID"},
                           {"details", component_validation.error()},
                           {"component_id", component_id}}
                          .dump(2),
                      "application/json");
      return;
    }

    if (param_name.empty() || param_name.length() > 256) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          json{{"error", "Invalid parameter name"}, {"details", "Parameter name is empty or too long"}}.dump(2),
          "application/json");
      return;
    }

    // Parse request body
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(json{{"error", "Invalid JSON in request body"}, {"details", e.what()}}.dump(2),
                      "application/json");
      return;
    }

    // Extract value from request body
    if (!body.contains("value")) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          json{{"error", "Missing 'value' field"}, {"details", "Request body must contain 'value' field"}}.dump(2),
          "application/json");
      return;
    }

    json value = body["value"];

    const auto cache = node_->get_entity_cache();

    std::string node_name;
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        node_name = component.namespace_path + "/" + component.id;
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      res.status = StatusCode::NotFound_404;
      res.set_content(json{{"error", "Component not found"}, {"component_id", component_id}}.dump(2),
                      "application/json");
      return;
    }

    auto config_mgr = node_->get_configuration_manager();
    auto result = config_mgr->set_parameter(node_name, param_name, value);

    if (result.success) {
      json response = {{"status", "success"}, {"component_id", component_id}, {"parameter", result.data}};
      res.set_content(response.dump(2), "application/json");
    } else {
      // Check if it's a read-only, not found, or service unavailable error
      if (result.error_message.find("read-only") != std::string::npos ||
          result.error_message.find("read only") != std::string::npos ||
          result.error_message.find("is read_only") != std::string::npos) {
        res.status = StatusCode::Forbidden_403;
      } else if (result.error_message.find("not found") != std::string::npos ||
                 result.error_message.find("Parameter not found") != std::string::npos) {
        res.status = StatusCode::NotFound_404;
      } else if (result.error_message.find("not available") != std::string::npos ||
                 result.error_message.find("service not available") != std::string::npos) {
        res.status = StatusCode::ServiceUnavailable_503;
      } else {
        res.status = StatusCode::BadRequest_400;
      }
      res.set_content(json{{"error", "Failed to set parameter"},
                           {"details", result.error_message},
                           {"component_id", component_id},
                           {"param_name", param_name}}
                          .dump(2),
                      "application/json");
    }
  } catch (const std::exception & e) {
    res.status = StatusCode::InternalServerError_500;
    res.set_content(json{{"error", "Failed to set configuration"},
                         {"details", e.what()},
                         {"component_id", component_id},
                         {"param_name", param_name}}
                        .dump(2),
                    "application/json");
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"),
                 "Error in handle_set_configuration for component '%s', param '%s': %s", component_id.c_str(),
                 param_name.c_str(), e.what());
  }
}

void RESTServer::handle_delete_configuration(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  // ROS2 does not support deleting parameters at runtime (they can be unset but not removed)
  // Return 405 Method Not Allowed per SOVD spec for unsupported operations
  res.status = StatusCode::MethodNotAllowed_405;
  res.set_header("Allow", "GET, PUT");
  res.set_content(
      json{{"error", "Method not allowed"},
           {"details", "Deleting configurations is not supported in ROS2. Parameters cannot be removed at runtime."}}
          .dump(2),
      "application/json");
}

void RESTServer::set_cors_headers(httplib::Response & res, const std::string & origin) const {
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

bool RESTServer::is_origin_allowed(const std::string & origin) const {
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

}  // namespace ros2_medkit_gateway
