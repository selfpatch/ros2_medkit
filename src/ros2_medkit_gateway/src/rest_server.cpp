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
#include <iomanip>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {

RESTServer::RESTServer(GatewayNode* node, const std::string& host, int port)
    : node_(node), host_(host), port_(port)
{
    server_ = std::make_unique<httplib::Server>();
    setup_routes();
}

RESTServer::~RESTServer() {
    stop();
}

void RESTServer::setup_routes() {
    // Health check
    server_->Get("/health", [this](const httplib::Request& req, httplib::Response& res) {
        handle_health(req, res);
    });

    // Root
    server_->Get("/", [this](const httplib::Request& req, httplib::Response& res) {
        handle_root(req, res);
    });

    // Areas
    server_->Get("/areas", [this](const httplib::Request& req, httplib::Response& res) {
        handle_list_areas(req, res);
    });

    // Components
    server_->Get("/components", [this](const httplib::Request& req, httplib::Response& res) {
        handle_list_components(req, res);
    });

    // Area components
    server_->Get(R"(/areas/([^/]+)/components)", [this](const httplib::Request& req, httplib::Response& res) {
        handle_area_components(req, res);
    });

    // Component data
    server_->Get(R"(/components/([^/]+)/data)", [this](const httplib::Request& req, httplib::Response& res) {
        handle_component_data(req, res);
    });
}

void RESTServer::start() {
    RCLCPP_INFO(
        rclcpp::get_logger("rest_server"),
        "Starting REST server on %s:%d...",
        host_.c_str(),
        port_
    );

    server_->listen(host_.c_str(), port_);
}

void RESTServer::stop() {
    if (server_ && server_->is_running()) {
        RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Stopping REST server...");
        server_->stop();
    }
}

std::expected<void, std::string> RESTServer::validate_entity_id(
    const std::string& entity_id
) const {
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
        bool is_alphanumeric = (c >= 'a' && c <= 'z') ||
                              (c >= 'A' && c <= 'Z') ||
                              (c >= '0' && c <= '9');
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
            return std::unexpected(
                "Entity ID contains invalid character: '" + char_repr +
                "'. Only alphanumeric and underscore are allowed"
            );
        }
    }

    return {};
}

void RESTServer::handle_health(const httplib::Request& req, httplib::Response& res) {
    (void)req;  // Unused parameter

    try {
        json response = {
            {"status", "healthy"},
            {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
        };

        res.set_content(response.dump(2), "application/json");
    } catch (const std::exception& e) {
        res.status = StatusCode::InternalServerError_500;
        res.set_content(
            json{{"error", "Internal server error"}}.dump(),
            "application/json"
        );
        RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_health: %s", e.what());
    }
}

void RESTServer::handle_root(const httplib::Request& req, httplib::Response& res) {
    (void)req;  // Unused parameter

    try {
        json response = {
            {"status", "ROS 2 Medkit Gateway running"},
            {"version", "0.1.0"},
            {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
        };

        res.set_content(response.dump(2), "application/json");
    } catch (const std::exception& e) {
        res.status = StatusCode::InternalServerError_500;
        res.set_content(
            json{{"error", "Internal server error"}}.dump(),
            "application/json"
        );
        RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_root: %s", e.what());
    }
}

void RESTServer::handle_list_areas(const httplib::Request& req, httplib::Response& res) {
    (void)req;  // Unused parameter

    try {
        const auto cache = node_->get_entity_cache();

        json areas_json = json::array();
        for (const auto& area : cache.areas) {
            areas_json.push_back(area.to_json());
        }

        res.set_content(areas_json.dump(2), "application/json");
    } catch (const std::exception& e) {
        res.status = StatusCode::InternalServerError_500;
        res.set_content(
            json{{"error", "Internal server error"}}.dump(),
            "application/json"
        );
        RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Error in handle_list_areas: %s", e.what());
    }
}

void RESTServer::handle_list_components(const httplib::Request& req, httplib::Response& res) {
    (void)req;  // Unused parameter

    try {
        const auto cache = node_->get_entity_cache();

        json components_json = json::array();
        for (const auto& component : cache.components) {
            components_json.push_back(component.to_json());
        }

        res.set_content(components_json.dump(2), "application/json");
    } catch (const std::exception& e) {
        res.status = StatusCode::InternalServerError_500;
        res.set_content(
            json{{"error", "Internal server error"}}.dump(),
            "application/json"
        );
        RCLCPP_ERROR(
            rclcpp::get_logger("rest_server"),
            "Error in handle_list_components: %s",
            e.what()
        );
    }
}

void RESTServer::handle_area_components(const httplib::Request& req, httplib::Response& res) {
    try {
        // Extract area_id from URL path
        if (req.matches.size() < 2) {
            res.status = StatusCode::BadRequest_400;
            res.set_content(
                json{{"error", "Invalid request"}}.dump(2),
                "application/json"
            );
            return;
        }

        std::string area_id = req.matches[1];

        // Validate area_id
        auto validation_result = validate_entity_id(area_id);
        if (!validation_result) {
            res.status = StatusCode::BadRequest_400;
            res.set_content(
                json{
                    {"error", "Invalid area ID"},
                    {"details", validation_result.error()},
                    {"area_id", area_id}
                }.dump(2),
                "application/json"
            );
            return;
        }

        const auto cache = node_->get_entity_cache();

        // Check if area exists
        bool area_exists = false;
        for (const auto& area : cache.areas) {
            if (area.id == area_id) {
                area_exists = true;
                break;
            }
        }

        if (!area_exists) {
            res.status = StatusCode::NotFound_404;
            res.set_content(
                json{
                    {"error", "Area not found"},
                    {"area_id", area_id}
                }.dump(2),
                "application/json"
            );
            return;
        }

        // Filter components by area
        json components_json = json::array();
        for (const auto& component : cache.components) {
            if (component.area == area_id) {
                components_json.push_back(component.to_json());
            }
        }

        res.set_content(components_json.dump(2), "application/json");
    } catch (const std::exception& e) {
        res.status = StatusCode::InternalServerError_500;
        res.set_content(
            json{{"error", "Internal server error"}}.dump(),
            "application/json"
        );
        RCLCPP_ERROR(
            rclcpp::get_logger("rest_server"),
            "Error in handle_area_components: %s",
            e.what()
        );
    }
}

void RESTServer::handle_component_data(const httplib::Request& req, httplib::Response& res) {
    std::string component_id;
    try {
        // Extract component_id from URL path
        if (req.matches.size() < 2) {
            res.status = StatusCode::BadRequest_400;
            res.set_content(
                json{{"error", "Invalid request"}}.dump(2),
                "application/json"
            );
            return;
        }

        component_id = req.matches[1];

        // Validate component_id
        auto validation_result = validate_entity_id(component_id);
        if (!validation_result) {
            res.status = StatusCode::BadRequest_400;
            res.set_content(
                json{
                    {"error", "Invalid component ID"},
                    {"details", validation_result.error()},
                    {"component_id", component_id}
                }.dump(2),
                "application/json"
            );
            return;
        }

        const auto cache = node_->get_entity_cache();

        // Find component in cache
        std::string component_namespace;
        bool component_found = false;

        for (const auto& component : cache.components) {
            if (component.id == component_id) {
                component_namespace = component.fqn;
                component_found = true;
                break;
            }
        }

        if (!component_found) {
            res.status = StatusCode::NotFound_404;
            res.set_content(
                json{
                    {"error", "Component not found"},
                    {"component_id", component_id}
                }.dump(2),
                "application/json"
            );
            return;
        }

        // Get component data from DataAccessManager
        auto data_access_mgr = node_->get_data_access_manager();
        json component_data = data_access_mgr->get_component_data(component_namespace);

        res.set_content(component_data.dump(2), "application/json");
    } catch (const std::exception& e) {
        res.status = StatusCode::InternalServerError_500;
        res.set_content(
            json{
                {"error", "Failed to retrieve component data"},
                {"details", e.what()},
                {"component_id", component_id}
            }.dump(2),
            "application/json"
        );
        RCLCPP_ERROR(
            rclcpp::get_logger("rest_server"),
            "Error in handle_component_data for component '%s': %s",
            component_id.c_str(),
            e.what()
        );
    }
}

}  // namespace ros2_medkit_gateway
