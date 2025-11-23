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
#include "ros2_medkit_gateway/gateway_node.hpp"
#include <rclcpp/rclcpp.hpp>

using json = nlohmann::json;

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

void RESTServer::handle_health(const httplib::Request& req, httplib::Response& res) {
    (void)req;  // Unused parameter

    try {
        json response = {
            {"status", "healthy"},
            {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
        };

        res.set_content(response.dump(2), "application/json");
    } catch (const std::exception& e) {
        res.status = 500;
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
        res.status = 500;
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
        res.status = 500;
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
        res.status = 500;
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

}  // namespace ros2_medkit_gateway
