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

RESTServer::RESTServer(GatewayNode * node, const std::string & host, int port) : node_(node), host_(host), port_(port) {
  server_ = std::make_unique<httplib::Server>();
  setup_routes();
}

RESTServer::~RESTServer() {
  stop();
}

void RESTServer::setup_routes() {
  // Health check
  server_->Get("/health", [this](const httplib::Request & req, httplib::Response & res) {
    handle_health(req, res);
  });

  // Root - server capabilities and entry points (REQ_INTEROP_010)
  server_->Get("/", [this](const httplib::Request & req, httplib::Response & res) {
    handle_root(req, res);
  });

  // Version info (REQ_INTEROP_001)
  server_->Get("/version-info", [this](const httplib::Request & req, httplib::Response & res) {
    handle_version_info(req, res);
  });

  // Areas
  server_->Get("/areas", [this](const httplib::Request & req, httplib::Response & res) {
    handle_list_areas(req, res);
  });

  // Components
  server_->Get("/components", [this](const httplib::Request & req, httplib::Response & res) {
    handle_list_components(req, res);
  });

  // Area components
  server_->Get(R"(/areas/([^/]+)/components)", [this](const httplib::Request & req, httplib::Response & res) {
    handle_area_components(req, res);
  });

  // Component topic data (specific topic) - register before general route
  server_->Get(R"(/components/([^/]+)/data/([^/]+)$)", [this](const httplib::Request & req, httplib::Response & res) {
    handle_component_topic_data(req, res);
  });

  // Component data (all topics)
  server_->Get(R"(/components/([^/]+)/data$)", [this](const httplib::Request & req, httplib::Response & res) {
    handle_component_data(req, res);
  });

  // Component topic publish (PUT)
  server_->Put(R"(/components/([^/]+)/data/([^/]+)$)", [this](const httplib::Request & req, httplib::Response & res) {
    handle_component_topic_publish(req, res);
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
        {"endpoints", json::array({"GET /health", "GET /version-info", "GET /areas", "GET /components",
                                   "GET /areas/{area_id}/components", "GET /components/{component_id}/data",
                                   "GET /components/{component_id}/data/{topic_name}",
                                   "PUT /components/{component_id}/data/{topic_name}"})},
        {"capabilities", {{"discovery", true}, {"data_access", true}}}};

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

    // Get component data from DataAccessManager
    // Use namespace_path to find topics (topics are relative to namespace, not FQN)
    auto data_access_mgr = node_->get_data_access_manager();
    json component_data = data_access_mgr->get_component_data(component_namespace);

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

    // Validate topic_name
    auto topic_validation = validate_entity_id(topic_name);
    if (!topic_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          json{{"error", "Invalid topic name"}, {"details", topic_validation.error()}, {"topic_name", topic_name}}.dump(
              2),
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

    // Construct full topic path: {namespace_path}/{topic_name}
    // Handle root namespace case to avoid double slash (//topic_name)
    std::string full_topic_path =
        (component_namespace == "/") ? "/" + topic_name : component_namespace + "/" + topic_name;

    // Get topic data from DataAccessManager
    auto data_access_mgr = node_->get_data_access_manager();
    json topic_data = data_access_mgr->get_topic_sample(full_topic_path);

    res.set_content(topic_data.dump(2), "application/json");
  } catch (const TopicNotAvailableException & e) {
    res.status = StatusCode::NotFound_404;
    res.set_content(
        json{{"error", "Topic not found or not publishing"}, {"component_id", component_id}, {"topic_name", topic_name}}
            .dump(2),
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

    // Validate topic_name
    auto topic_validation = validate_entity_id(topic_name);
    if (!topic_validation) {
      res.status = StatusCode::BadRequest_400;
      res.set_content(
          json{{"error", "Invalid topic name"}, {"details", topic_validation.error()}, {"topic_name", topic_name}}.dump(
              2),
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

    // Construct full topic path
    std::string full_topic_path =
        (component_namespace == "/") ? "/" + topic_name : component_namespace + "/" + topic_name;

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

}  // namespace ros2_medkit_gateway
