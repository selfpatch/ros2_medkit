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

#include "ros2_medkit_gateway/http/rest_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "ros2_medkit_gateway/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

using httplib::StatusCode;

namespace ros2_medkit_gateway {

RESTServer::RESTServer(GatewayNode * node, const std::string & host, int port, const CorsConfig & cors_config,
                       const AuthConfig & auth_config, const TlsConfig & tls_config)
  : node_(node)
  , host_(host)
  , port_(port)
  , cors_config_(cors_config)
  , auth_config_(auth_config)
  , tls_config_(tls_config) {
  // Create HTTP/HTTPS server manager
  http_server_ = std::make_unique<HttpServerManager>(tls_config_);

  // Initialize auth manager and middleware if auth is enabled
  if (auth_config_.enabled) {
    auth_manager_ = std::make_unique<AuthManager>(auth_config_);
    auth_middleware_ = std::make_unique<AuthMiddleware>(auth_config_, auth_manager_.get());
    RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Authentication enabled - algorithm: %s, require_auth_for: %s",
                algorithm_to_string(auth_config_.jwt_algorithm).c_str(),
                auth_config_.require_auth_for == AuthRequirement::NONE    ? "none"
                : auth_config_.require_auth_for == AuthRequirement::WRITE ? "write"
                                                                          : "all");
  }

  // Create handler context and domain-specific handlers
  handler_ctx_ =
      std::make_unique<handlers::HandlerContext>(node_, cors_config_, auth_config_, tls_config_, auth_manager_.get());

  health_handlers_ = std::make_unique<handlers::HealthHandlers>(*handler_ctx_);
  area_handlers_ = std::make_unique<handlers::AreaHandlers>(*handler_ctx_);
  component_handlers_ = std::make_unique<handlers::ComponentHandlers>(*handler_ctx_);
  app_handlers_ = std::make_unique<handlers::AppHandlers>(*handler_ctx_);
  function_handlers_ = std::make_unique<handlers::FunctionHandlers>(*handler_ctx_);
  operation_handlers_ = std::make_unique<handlers::OperationHandlers>(*handler_ctx_);
  config_handlers_ = std::make_unique<handlers::ConfigHandlers>(*handler_ctx_);
  fault_handlers_ = std::make_unique<handlers::FaultHandlers>(*handler_ctx_);
  auth_handlers_ = std::make_unique<handlers::AuthHandlers>(*handler_ctx_);
  sse_fault_handler_ = std::make_unique<handlers::SSEFaultHandler>(*handler_ctx_);

  // Set up pre-routing handler for CORS and Authentication
  setup_pre_routing_handler();
  setup_routes();
}

void RESTServer::setup_pre_routing_handler() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    return;
  }

  // Set up pre-routing handler for CORS and Authentication
  // This handler runs before any route handler
  srv->set_pre_routing_handler([this](const httplib::Request & req, httplib::Response & res) {
    // Handle CORS if enabled
    if (cors_config_.enabled) {
      std::string origin = req.get_header_value("Origin");
      bool origin_allowed = !origin.empty() && is_origin_allowed(origin);

      if (origin_allowed) {
        set_cors_headers(res, origin);
        // Add Authorization header to allowed headers for CORS
        if (auth_config_.enabled) {
          std::string current_headers = res.get_header_value("Access-Control-Allow-Headers");
          if (!current_headers.empty() && current_headers.find("Authorization") == std::string::npos) {
            res.set_header("Access-Control-Allow-Headers", current_headers + ", Authorization");
          }
        }
      }

      // Handle preflight OPTIONS requests
      if (req.method == "OPTIONS") {
        if (origin_allowed) {
          res.set_header("Access-Control-Max-Age", std::to_string(cors_config_.max_age_seconds));
          res.status = StatusCode::NoContent_204;
        } else {
          res.status = StatusCode::Forbidden_403;
        }
        return httplib::Server::HandlerResponse::Handled;
      }
    }

    // Handle Authentication if enabled
    if (auth_middleware_ && auth_middleware_->is_enabled()) {
      // Use AuthMiddleware to process the request
      auto auth_request = AuthMiddleware::from_httplib_request(req);
      auto result = auth_middleware_->process(auth_request);

      if (!result.allowed) {
        AuthMiddleware::apply_to_response(result, res);
        return httplib::Server::HandlerResponse::Handled;
      }
    }

    return httplib::Server::HandlerResponse::Unhandled;
  });
}

RESTServer::~RESTServer() {
  stop();
}

void RESTServer::setup_routes() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    throw std::runtime_error("No server instance available for route setup");
  }

  // Health check
  srv->Get(api_path("/health"), [this](const httplib::Request & req, httplib::Response & res) {
    health_handlers_->handle_health(req, res);
  });

  // Root - server capabilities and entry points (REQ_INTEROP_010)
  srv->Get(api_path("/"), [this](const httplib::Request & req, httplib::Response & res) {
    health_handlers_->handle_root(req, res);
  });

  // Version info (REQ_INTEROP_001)
  srv->Get(api_path("/version-info"), [this](const httplib::Request & req, httplib::Response & res) {
    health_handlers_->handle_version_info(req, res);
  });

  // Areas
  srv->Get(api_path("/areas"), [this](const httplib::Request & req, httplib::Response & res) {
    area_handlers_->handle_list_areas(req, res);
  });

  // Apps - must register before /apps/{id} to avoid regex conflict
  srv->Get(api_path("/apps"), [this](const httplib::Request & req, httplib::Response & res) {
    app_handlers_->handle_list_apps(req, res);
  });

  // App data item (specific topic) - register before /apps/{id}/data
  srv->Get((api_path("/apps") + R"(/([^/]+)/data/(.+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             app_handlers_->handle_get_app_data_item(req, res);
           });

  // App data (all topics)
  srv->Get((api_path("/apps") + R"(/([^/]+)/data$)"), [this](const httplib::Request & req, httplib::Response & res) {
    app_handlers_->handle_get_app_data(req, res);
  });

  // App operations
  srv->Get((api_path("/apps") + R"(/([^/]+)/operations$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             operation_handlers_->handle_list_operations(req, res);
           });

  // App operation (POST) - sync operations like service calls, async action goals
  srv->Post((api_path("/apps") + R"(/([^/]+)/operations/([^/]+)$)"),
            [this](const httplib::Request & req, httplib::Response & res) {
              operation_handlers_->handle_component_operation(req, res);
            });

  // App action status (GET)
  srv->Get((api_path("/apps") + R"(/([^/]+)/operations/([^/]+)/status$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             operation_handlers_->handle_action_status(req, res);
           });

  // App action result (GET)
  srv->Get((api_path("/apps") + R"(/([^/]+)/operations/([^/]+)/result$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             operation_handlers_->handle_action_result(req, res);
           });

  // App action cancel (DELETE)
  srv->Delete((api_path("/apps") + R"(/([^/]+)/operations/([^/]+)$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                operation_handlers_->handle_action_cancel(req, res);
              });

  // App configurations - list all
  srv->Get((api_path("/apps") + R"(/([^/]+)/configurations$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             config_handlers_->handle_list_configurations(req, res);
           });

  // App configurations - get specific
  srv->Get((api_path("/apps") + R"(/([^/]+)/configurations/([^/]+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             config_handlers_->handle_get_configuration(req, res);
           });

  // App configurations - set
  srv->Put((api_path("/apps") + R"(/([^/]+)/configurations/([^/]+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             config_handlers_->handle_set_configuration(req, res);
           });

  // App configurations - delete single
  srv->Delete((api_path("/apps") + R"(/([^/]+)/configurations/([^/]+)$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                config_handlers_->handle_delete_configuration(req, res);
              });

  // App configurations - delete all
  srv->Delete((api_path("/apps") + R"(/([^/]+)/configurations$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                config_handlers_->handle_delete_all_configurations(req, res);
              });

  // Single app (capabilities) - must be after more specific routes
  srv->Get((api_path("/apps") + R"(/([^/]+)$)"), [this](const httplib::Request & req, httplib::Response & res) {
    app_handlers_->handle_get_app(req, res);
  });

  // Functions - list all functions
  srv->Get(api_path("/functions"), [this](const httplib::Request & req, httplib::Response & res) {
    function_handlers_->handle_list_functions(req, res);
  });

  // Function hosts
  srv->Get((api_path("/functions") + R"(/([^/]+)/hosts$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             function_handlers_->handle_function_hosts(req, res);
           });

  // Function data (aggregated from host apps)
  srv->Get((api_path("/functions") + R"(/([^/]+)/data$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             function_handlers_->handle_get_function_data(req, res);
           });

  // Function operations (aggregated from host apps)
  srv->Get((api_path("/functions") + R"(/([^/]+)/operations$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             function_handlers_->handle_list_function_operations(req, res);
           });

  // Single function (capabilities) - must be after more specific routes
  srv->Get((api_path("/functions") + R"(/([^/]+)$)"), [this](const httplib::Request & req, httplib::Response & res) {
    function_handlers_->handle_get_function(req, res);
  });

  // Components
  srv->Get(api_path("/components"), [this](const httplib::Request & req, httplib::Response & res) {
    component_handlers_->handle_list_components(req, res);
  });

  // Area components
  srv->Get((api_path("/areas") + R"(/([^/]+)/components)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             area_handlers_->handle_area_components(req, res);
           });

  // Area subareas (relationship endpoint)
  srv->Get((api_path("/areas") + R"(/([^/]+)/subareas$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             area_handlers_->handle_get_subareas(req, res);
           });

  // Area related-components (relationship endpoint)
  srv->Get((api_path("/areas") + R"(/([^/]+)/related-components$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             area_handlers_->handle_get_related_components(req, res);
           });

  // Single area (capabilities) - must be after more specific routes
  srv->Get((api_path("/areas") + R"(/([^/]+)$)"), [this](const httplib::Request & req, httplib::Response & res) {
    area_handlers_->handle_get_area(req, res);
  });

  // Component topic data (specific topic) - register before general route
  // Use (.+) for topic_name to accept slashes from percent-encoded URLs (%2F -> /)
  srv->Get((api_path("/components") + R"(/([^/]+)/data/(.+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             component_handlers_->handle_component_topic_data(req, res);
           });

  // Component data (all topics)
  srv->Get((api_path("/components") + R"(/([^/]+)/data$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             component_handlers_->handle_component_data(req, res);
           });

  // Component subcomponents (relationship endpoint)
  srv->Get((api_path("/components") + R"(/([^/]+)/subcomponents$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             component_handlers_->handle_get_subcomponents(req, res);
           });

  // Component related-apps (relationship endpoint)
  srv->Get((api_path("/components") + R"(/([^/]+)/related-apps$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             component_handlers_->handle_get_related_apps(req, res);
           });

  // Single component (capabilities) - must be after more specific routes
  srv->Get((api_path("/components") + R"(/([^/]+)$)"), [this](const httplib::Request & req, httplib::Response & res) {
    component_handlers_->handle_get_component(req, res);
  });

  // Component topic publish (PUT)
  // Use (.+) for topic_name to accept slashes from percent-encoded URLs (%2F -> /)
  srv->Put((api_path("/components") + R"(/([^/]+)/data/(.+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             component_handlers_->handle_component_topic_publish(req, res);
           });

  // Component operation (POST) - sync operations like service calls, async action goals
  srv->Post((api_path("/components") + R"(/([^/]+)/operations/([^/]+)$)"),
            [this](const httplib::Request & req, httplib::Response & res) {
              operation_handlers_->handle_component_operation(req, res);
            });

  // List component operations (GET) - list all services and actions for a component
  srv->Get((api_path("/components") + R"(/([^/]+)/operations$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             operation_handlers_->handle_list_operations(req, res);
           });

  // Action status (GET) - get current status of an action goal
  srv->Get((api_path("/components") + R"(/([^/]+)/operations/([^/]+)/status$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             operation_handlers_->handle_action_status(req, res);
           });

  // Action result (GET) - get result of a completed action goal
  srv->Get((api_path("/components") + R"(/([^/]+)/operations/([^/]+)/result$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             operation_handlers_->handle_action_result(req, res);
           });

  // Action cancel (DELETE) - cancel a running action goal
  srv->Delete((api_path("/components") + R"(/([^/]+)/operations/([^/]+)$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                operation_handlers_->handle_action_cancel(req, res);
              });

  // Configurations endpoints - SOVD Configurations API mapped to ROS2 parameters
  // List all configurations (parameters) for a component
  srv->Get((api_path("/components") + R"(/([^/]+)/configurations$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             config_handlers_->handle_list_configurations(req, res);
           });

  // Get specific configuration (parameter) - register before general route
  srv->Get((api_path("/components") + R"(/([^/]+)/configurations/([^/]+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             config_handlers_->handle_get_configuration(req, res);
           });

  // Set configuration (parameter)
  srv->Put((api_path("/components") + R"(/([^/]+)/configurations/([^/]+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             config_handlers_->handle_set_configuration(req, res);
           });

  // Delete (reset) single configuration to default value
  srv->Delete((api_path("/components") + R"(/([^/]+)/configurations/([^/]+)$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                config_handlers_->handle_delete_configuration(req, res);
              });

  // Delete (reset) all configurations to default values
  srv->Delete((api_path("/components") + R"(/([^/]+)/configurations$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                config_handlers_->handle_delete_all_configurations(req, res);
              });

  // Fault endpoints
  // SSE stream for real-time fault events - must be registered before /faults to avoid regex conflict
  srv->Get(api_path("/faults/stream"), [this](const httplib::Request & req, httplib::Response & res) {
    sse_fault_handler_->handle_stream(req, res);
  });

  // GET /faults - convenience API to retrieve all faults across the system
  // Useful for dashboards and monitoring tools that need a complete system health view
  srv->Get(api_path("/faults"), [this](const httplib::Request & req, httplib::Response & res) {
    fault_handlers_->handle_list_all_faults(req, res);
  });

  // List all faults for a component (REQ_INTEROP_012)
  srv->Get((api_path("/components") + R"(/([^/]+)/faults$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             fault_handlers_->handle_list_faults(req, res);
           });

  // List all faults for an app (same handler, entity-agnostic)
  srv->Get((api_path("/apps") + R"(/([^/]+)/faults$)"), [this](const httplib::Request & req, httplib::Response & res) {
    fault_handlers_->handle_list_faults(req, res);
  });

  // Get specific fault by code (REQ_INTEROP_013)
  srv->Get((api_path("/components") + R"(/([^/]+)/faults/([^/]+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             fault_handlers_->handle_get_fault(req, res);
           });

  // Get specific fault by code for an app
  srv->Get((api_path("/apps") + R"(/([^/]+)/faults/([^/]+)$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             fault_handlers_->handle_get_fault(req, res);
           });

  // Clear a fault (REQ_INTEROP_015)
  srv->Delete((api_path("/components") + R"(/([^/]+)/faults/([^/]+)$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                fault_handlers_->handle_clear_fault(req, res);
              });

  // Clear a fault for an app
  srv->Delete((api_path("/apps") + R"(/([^/]+)/faults/([^/]+)$)"),
              [this](const httplib::Request & req, httplib::Response & res) {
                fault_handlers_->handle_clear_fault(req, res);
              });

  // Snapshot endpoints for fault debugging
  // GET /faults/{fault_code}/snapshots - system-wide snapshot access
  srv->Get((api_path("/faults") + R"(/([^/]+)/snapshots$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             fault_handlers_->handle_get_snapshots(req, res);
           });

  // GET /components/{component_id}/faults/{fault_code}/snapshots - component-scoped snapshot access
  srv->Get((api_path("/components") + R"(/([^/]+)/faults/([^/]+)/snapshots$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             fault_handlers_->handle_get_component_snapshots(req, res);
           });

  // GET /apps/{app_id}/faults/{fault_code}/snapshots - app-scoped snapshot access
  srv->Get((api_path("/apps") + R"(/([^/]+)/faults/([^/]+)/snapshots$)"),
           [this](const httplib::Request & req, httplib::Response & res) {
             fault_handlers_->handle_get_component_snapshots(req, res);
           });

  // Authentication endpoints (REQ_INTEROP_086, REQ_INTEROP_087)
  // POST /auth/authorize - Authenticate and get tokens (client_credentials grant)
  srv->Post(api_path("/auth/authorize"), [this](const httplib::Request & req, httplib::Response & res) {
    auth_handlers_->handle_auth_authorize(req, res);
  });

  // POST /auth/token - Refresh access token
  srv->Post(api_path("/auth/token"), [this](const httplib::Request & req, httplib::Response & res) {
    auth_handlers_->handle_auth_token(req, res);
  });

  // POST /auth/revoke - Revoke a refresh token
  srv->Post(api_path("/auth/revoke"), [this](const httplib::Request & req, httplib::Response & res) {
    auth_handlers_->handle_auth_revoke(req, res);
  });
}

void RESTServer::start() {
  http_server_->listen(host_, port_);
}

void RESTServer::stop() {
  if (http_server_) {
    http_server_->stop();
  }
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
