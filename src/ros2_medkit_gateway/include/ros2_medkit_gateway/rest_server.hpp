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

#pragma once

#include <httplib.h>

#include <expected>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {

class GatewayNode;

class RESTServer {
public:
    RESTServer(GatewayNode* node, const std::string& host, int port);
    ~RESTServer();

    void start();
    void stop();

private:
    void setup_routes();

    // Route handlers
    void handle_health(const httplib::Request& req, httplib::Response& res);
    void handle_root(const httplib::Request& req, httplib::Response& res);
    void handle_list_areas(const httplib::Request& req, httplib::Response& res);
    void handle_list_components(const httplib::Request& req, httplib::Response& res);
    void handle_area_components(const httplib::Request& req, httplib::Response& res);
    void handle_component_data(const httplib::Request& req, httplib::Response& res);

    // Helper methods
    std::expected<void, std::string> validate_entity_id(const std::string& entity_id) const;

    GatewayNode* node_;
    std::string host_;
    int port_;
    std::unique_ptr<httplib::Server> server_;
};

}  // namespace ros2_medkit_gateway
