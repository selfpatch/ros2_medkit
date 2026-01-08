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

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <tl/expected.hpp>
#include <vector>

#include "ros2_medkit_gateway/auth/auth_config.hpp"
#include "ros2_medkit_gateway/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/config.hpp"

namespace ros2_medkit_gateway {

class GatewayNode;

class RESTServer {
 public:
  RESTServer(GatewayNode * node, const std::string & host, int port, const CorsConfig & cors_config,
             const AuthConfig & auth_config);
  ~RESTServer();

  void start();
  void stop();

 private:
  void setup_routes();

  // Route handlers
  void handle_health(const httplib::Request & req, httplib::Response & res);
  void handle_root(const httplib::Request & req, httplib::Response & res);
  void handle_version_info(const httplib::Request & req, httplib::Response & res);
  void handle_list_areas(const httplib::Request & req, httplib::Response & res);
  void handle_list_components(const httplib::Request & req, httplib::Response & res);
  void handle_area_components(const httplib::Request & req, httplib::Response & res);
  void handle_component_data(const httplib::Request & req, httplib::Response & res);
  void handle_component_topic_data(const httplib::Request & req, httplib::Response & res);
  void handle_component_topic_publish(const httplib::Request & req, httplib::Response & res);
  void handle_component_operation(const httplib::Request & req, httplib::Response & res);
  void handle_list_operations(const httplib::Request & req, httplib::Response & res);
  void handle_action_status(const httplib::Request & req, httplib::Response & res);
  void handle_action_result(const httplib::Request & req, httplib::Response & res);
  void handle_action_cancel(const httplib::Request & req, httplib::Response & res);
  void handle_list_configurations(const httplib::Request & req, httplib::Response & res);
  void handle_get_configuration(const httplib::Request & req, httplib::Response & res);
  void handle_set_configuration(const httplib::Request & req, httplib::Response & res);
  void handle_delete_configuration(const httplib::Request & req, httplib::Response & res);
  void handle_delete_all_configurations(const httplib::Request & req, httplib::Response & res);

  // Fault endpoints
  void handle_list_all_faults(const httplib::Request & req, httplib::Response & res);
  void handle_list_faults(const httplib::Request & req, httplib::Response & res);
  void handle_get_fault(const httplib::Request & req, httplib::Response & res);
  void handle_clear_fault(const httplib::Request & req, httplib::Response & res);

  // Authentication endpoints (REQ_INTEROP_086, REQ_INTEROP_087)
  void handle_auth_authorize(const httplib::Request & req, httplib::Response & res);
  void handle_auth_token(const httplib::Request & req, httplib::Response & res);
  void handle_auth_revoke(const httplib::Request & req, httplib::Response & res);

  // Helper methods
  tl::expected<void, std::string> validate_entity_id(const std::string & entity_id) const;
  tl::expected<std::string, std::string> get_component_namespace_path(const std::string & component_id) const;
  void set_cors_headers(httplib::Response & res, const std::string & origin) const;
  bool is_origin_allowed(const std::string & origin) const;

  GatewayNode * node_;
  std::string host_;
  int port_;
  CorsConfig cors_config_;
  AuthConfig auth_config_;
  std::unique_ptr<AuthManager> auth_manager_;
  std::unique_ptr<AuthMiddleware> auth_middleware_;

  std::unique_ptr<httplib::Server> server_;
};

}  // namespace ros2_medkit_gateway
