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

#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <type_traits>

#include "ros2_medkit_gateway/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

#include "../openapi/route_registry.hpp"

namespace ros2_medkit_gateway {

namespace {

void set_internal_server_error(httplib::Response & res, const std::string & details) {
  nlohmann::json error;
  error["error_code"] = ERR_INTERNAL_ERROR;
  error["message"] = "Internal server error";
  error["parameters"] = nlohmann::json::object();
  error["parameters"]["details"] = details;

  res.status = 500;
  res.set_content(error.dump(2), "application/json");
}

struct HttplibExceptionHandlerAdapter {
  // Newer cpp-httplib versions pass std::exception_ptr.
  void operator()(const httplib::Request & /*req*/, httplib::Response & res, const std::exception_ptr & ep) const {
    try {
      if (ep) {
        std::rethrow_exception(ep);
      }
      RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unhandled empty exception_ptr");
      set_internal_server_error(res, "Unknown exception");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unhandled exception: %s", e.what());
      set_internal_server_error(res, "An internal error occurred");
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unknown exception caught");
      set_internal_server_error(res, "An internal error occurred");
    }
  }

  // Older cpp-httplib versions pass std::exception directly.
  void operator()(const httplib::Request & /*req*/, httplib::Response & res, const std::exception & e) const {
    RCLCPP_ERROR(rclcpp::get_logger("rest_server"), "Unhandled exception: %s", e.what());
    set_internal_server_error(res, "An internal error occurred");
  }
};

static_assert(std::is_constructible_v<httplib::Server::ExceptionHandler, HttplibExceptionHandlerAdapter>,
              "cpp-httplib exception handler signature changed; update HttplibExceptionHandlerAdapter");

}  // namespace

RESTServer::RESTServer(GatewayNode * node, const std::string & host, int port, const CorsConfig & cors_config,
                       const AuthConfig & auth_config, const RateLimitConfig & rate_limit_config,
                       const TlsConfig & tls_config)
  : node_(node)
  , host_(host)
  , port_(port)
  , cors_config_(cors_config)
  , auth_config_(auth_config)
  , tls_config_(tls_config) {
  // Create HTTP/HTTPS server manager
  http_server_ = std::make_unique<HttpServerManager>(tls_config_);

  // Set maximum payload size for uploads (cpp-httplib default is 8MB)
  auto * srv = http_server_->get_server();
  if (srv) {
    size_t max_payload = node_->get_bulk_data_store() ? node_->get_bulk_data_store()->max_upload_bytes() : 0;
    if (max_payload > 0) {
      srv->set_payload_max_length(max_payload);
      RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Max payload length set to %zu bytes", max_payload);
    }
  }

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

  // Initialize rate limiter if enabled
  if (rate_limit_config.enabled) {
    rate_limiter_ = std::make_unique<RateLimiter>(rate_limit_config);
    RCLCPP_INFO(rclcpp::get_logger("rest_server"), "Rate limiting enabled - global: %d rpm, per-client: %d rpm",
                rate_limit_config.global_requests_per_minute, rate_limit_config.client_requests_per_minute);
  }

  // Create handler context and domain-specific handlers
  handler_ctx_ = std::make_unique<handlers::HandlerContext>(node_, cors_config_, auth_config_, tls_config_,
                                                            auth_manager_.get(), node_->get_bulk_data_store());

  // Create route registry before handlers that need it (HealthHandlers, DocsHandlers).
  // Routes are populated later in setup_routes(), but both handlers only access
  // the registry lazily at request time, so the pointer is valid.
  route_registry_ = std::make_unique<openapi::RouteRegistry>();
  route_registry_->set_auth_enabled(auth_config_.enabled);

  health_handlers_ = std::make_unique<handlers::HealthHandlers>(*handler_ctx_, route_registry_.get());
  discovery_handlers_ = std::make_unique<handlers::DiscoveryHandlers>(*handler_ctx_);
  data_handlers_ = std::make_unique<handlers::DataHandlers>(*handler_ctx_);
  operation_handlers_ = std::make_unique<handlers::OperationHandlers>(*handler_ctx_);
  config_handlers_ = std::make_unique<handlers::ConfigHandlers>(*handler_ctx_);
  fault_handlers_ = std::make_unique<handlers::FaultHandlers>(*handler_ctx_);
  log_handlers_ = std::make_unique<handlers::LogHandlers>(*handler_ctx_);
  auth_handlers_ = std::make_unique<handlers::AuthHandlers>(*handler_ctx_);
  sse_client_tracker_ = node_->get_sse_client_tracker();
  sse_fault_handler_ = std::make_unique<handlers::SSEFaultHandler>(*handler_ctx_, sse_client_tracker_);
  bulkdata_handlers_ = std::make_unique<handlers::BulkDataHandlers>(*handler_ctx_);
  auto max_duration_sec = static_cast<int>(node_->get_parameter("sse.max_duration_sec").as_int());
  if (max_duration_sec <= 0) {
    RCLCPP_WARN(node_->get_logger(), "sse.max_duration_sec must be > 0, using default 3600");
    max_duration_sec = 3600;
  }
  cyclic_sub_handlers_ = std::make_unique<handlers::CyclicSubscriptionHandlers>(
      *handler_ctx_, *node_->get_subscription_manager(), *node_->get_sampler_registry(),
      *node_->get_transport_registry(), max_duration_sec);

  if (node_->get_update_manager()) {
    update_handlers_ = std::make_unique<handlers::UpdateHandlers>(*handler_ctx_, node_->get_update_manager());
  }

  lock_handlers_ = std::make_unique<handlers::LockHandlers>(*handler_ctx_, node_->get_lock_manager());

  // TODO(Task 9): Wire ScriptHandlers when GatewayNode::get_script_manager() is available
  // if (node_->get_script_manager()) {
  //   script_handlers_ =
  //       std::make_unique<handlers::ScriptHandlers>(*handler_ctx_, node_->get_script_manager());
  // }

  docs_handlers_ = std::make_unique<handlers::DocsHandlers>(*handler_ctx_, *node_, node_->get_plugin_manager(),
                                                            route_registry_.get());

  // Set up global error handlers for SOVD GenericError compliance
  setup_global_error_handlers();
  // Set up pre-routing handler for CORS and Authentication
  setup_pre_routing_handler();
  setup_routes();

  // Register plugin custom routes
  if (node_->get_plugin_manager()) {
    auto * plugin_srv = http_server_->get_server();
    if (plugin_srv) {
      node_->get_plugin_manager()->register_routes(*plugin_srv, API_BASE_PATH);
    }
  }
}

void RESTServer::setup_pre_routing_handler() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    return;
  }

  // Set up pre-routing handler for CORS and Authentication
  // This handler runs before any route handler
  srv->set_pre_routing_handler([this](const httplib::Request & req, httplib::Response & res) {
    // 1. Handle CORS (existing logic)
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

      // 2. Handle preflight OPTIONS requests
      if (req.method == "OPTIONS") {
        if (origin_allowed) {
          res.set_header("Access-Control-Max-Age", std::to_string(cors_config_.max_age_seconds));
          res.status = 204;
        } else {
          res.status = 403;
        }
        return httplib::Server::HandlerResponse::Handled;
      }
    }

    // 3. Rate limiting check. If rejected, return Handled (CORS headers already set)
    if (rate_limiter_ && rate_limiter_->is_enabled() && req.method != "OPTIONS") {
      auto rl_result = rate_limiter_->check(req.remote_addr, req.path);
      RateLimiter::apply_headers(rl_result, res);
      if (!rl_result.allowed) {
        RateLimiter::apply_rejection(rl_result, res);
        return httplib::Server::HandlerResponse::Handled;
      }
    }

    // 1. Handle CORS (existing logic)

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

void RESTServer::setup_global_error_handlers() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    return;
  }

  // Global error handler - catches HTTP errors like 404 Not Found
  // Only set error content if no content has been set by a handler
  srv->set_error_handler([](const httplib::Request & /*req*/, httplib::Response & res) {
    // If the response already has content (from a handler's send_error), don't overwrite it
    if (!res.body.empty()) {
      return;
    }

    nlohmann::json error;
    error["error_code"] = ERR_RESOURCE_NOT_FOUND;
    error["message"] = "Resource not found";
    error["parameters"] = nlohmann::json::object();
    error["parameters"]["status"] = res.status;

    res.set_content(error.dump(2), "application/json");
  });

  // Global exception handler - catches unhandled exceptions in route handlers
  srv->set_exception_handler(HttplibExceptionHandlerAdapter{});
}

RESTServer::~RESTServer() {
  stop();
}

void RESTServer::setup_routes() {
  httplib::Server * srv = http_server_->get_server();
  if (!srv) {
    throw std::runtime_error("No server instance available for route setup");
  }

  // === Docs routes - MUST be before data/config item routes to avoid (.+) capture collision ===
  // These use special regex patterns that don't map cleanly to OpenAPI {param} style,
  // so they are registered directly with the server rather than through the route registry.
  srv->Get(api_path("/docs"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_docs_root(req, res);
  });
  srv->Get((api_path("") + R"((.+)/docs$)"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_docs_any_path(req, res);
  });

#ifdef ENABLE_SWAGGER_UI
  // Swagger UI - interactive API documentation browser
  srv->Get(api_path("/swagger-ui"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_swagger_ui(req, res);
  });
  srv->Get(api_path(R"(/swagger-ui/([^/]+))"), [this](const httplib::Request & req, httplib::Response & res) {
    docs_handlers_->handle_swagger_asset(req, res);
  });
#endif

  auto & reg = *route_registry_;

  // === Server endpoints ===
  reg.get("/health",
          [this](auto & req, auto & res) {
            health_handlers_->handle_health(req, res);
          })
      .tag("Server")
      .summary("Health check")
      .description("Returns gateway health status.")
      .response(200, "Gateway is healthy");

  reg.get("/",
          [this](auto & req, auto & res) {
            health_handlers_->handle_root(req, res);
          })
      .tag("Server")
      .summary("API overview")
      .description("Returns gateway metadata, available endpoints, and capabilities.");

  reg.get("/version-info",
          [this](auto & req, auto & res) {
            health_handlers_->handle_version_info(req, res);
          })
      .tag("Server")
      .summary("SOVD version information")
      .description("Returns SOVD specification version and vendor info.");

  // === Discovery - entity collections ===
  reg.get("/areas",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_areas(req, res);
          })
      .tag("Discovery")
      .summary("List areas");

  reg.get("/apps",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_apps(req, res);
          })
      .tag("Discovery")
      .summary("List apps");

  reg.get("/components",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_components(req, res);
          })
      .tag("Discovery")
      .summary("List components");

  reg.get("/functions",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_functions(req, res);
          })
      .tag("Discovery")
      .summary("List functions");

  // === Per-entity-type resource routes ===
  // Entity types: areas, components, apps, functions
  // For each entity type, register data, operations, configurations, faults, logs, bulk-data,
  // and discovery relationship endpoints.

  // Helper lambdas for entity-type-specific discovery detail handlers
  using HandlerFn = openapi::HandlerFn;
  struct EntityHandlers {
    const char * type;
    const char * singular;
    HandlerFn detail_handler;
  };

  // clang-format off
  std::vector<EntityHandlers> entity_types = {
      {"areas", "area", [this](auto & req, auto & res) { discovery_handlers_->handle_get_area(req, res); }},
      {"components", "component", [this](auto & req, auto & res) { discovery_handlers_->handle_get_component(req, res); }},
      {"apps", "app", [this](auto & req, auto & res) { discovery_handlers_->handle_get_app(req, res); }},
      {"functions", "function", [this](auto & req, auto & res) { discovery_handlers_->handle_get_function(req, res); }},
  };
  // clang-format on

  for (const auto & et : entity_types) {
    std::string base = std::string("/") + et.type;
    std::string entity_path = base + "/{" + et.singular + "_id}";

    // --- Data ---
    // Data item (specific topic) - MUST be before data collection to avoid (.+) capture
    reg.get(entity_path + "/data/{data_id}",
            [this](auto & req, auto & res) {
              data_handlers_->handle_get_data_item(req, res);
            })
        .tag("Data")
        .summary(std::string("Get data item for ") + et.singular);

    reg.put(entity_path + "/data/{data_id}",
            [this](auto & req, auto & res) {
              data_handlers_->handle_put_data_item(req, res);
            })
        .tag("Data")
        .summary(std::string("Write data item for ") + et.singular);

    // Data-categories (apps only in original, but registered for all entity types)
    reg.get(entity_path + "/data-categories",
            [this](auto & req, auto & res) {
              data_handlers_->handle_data_categories(req, res);
            })
        .tag("Data")
        .summary(std::string("List data categories for ") + et.singular);

    // Data-groups
    reg.get(entity_path + "/data-groups",
            [this](auto & req, auto & res) {
              data_handlers_->handle_data_groups(req, res);
            })
        .tag("Data")
        .summary(std::string("List data groups for ") + et.singular);

    // Data collection (all topics)
    reg.get(entity_path + "/data",
            [this](auto & req, auto & res) {
              data_handlers_->handle_list_data(req, res);
            })
        .tag("Data")
        .summary(std::string("List data items for ") + et.singular);

    // --- Operations ---
    reg.get(entity_path + "/operations",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_list_operations(req, res);
            })
        .tag("Operations")
        .summary(std::string("List operations for ") + et.singular);

    reg.get(entity_path + "/operations/{operation_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_get_operation(req, res);
            })
        .tag("Operations")
        .summary(std::string("Get operation details for ") + et.singular);

    // Execution endpoints
    reg.post(entity_path + "/operations/{operation_id}/executions",
             [this](auto & req, auto & res) {
               operation_handlers_->handle_create_execution(req, res);
             })
        .tag("Operations")
        .summary(std::string("Start operation execution for ") + et.singular);

    reg.get(entity_path + "/operations/{operation_id}/executions",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_list_executions(req, res);
            })
        .tag("Operations")
        .summary(std::string("List operation executions for ") + et.singular);

    reg.get(entity_path + "/operations/{operation_id}/executions/{execution_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_get_execution(req, res);
            })
        .tag("Operations")
        .summary(std::string("Get execution status for ") + et.singular);

    reg.put(entity_path + "/operations/{operation_id}/executions/{execution_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_update_execution(req, res);
            })
        .tag("Operations")
        .summary(std::string("Update execution for ") + et.singular);

    reg.del(entity_path + "/operations/{operation_id}/executions/{execution_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_cancel_execution(req, res);
            })
        .tag("Operations")
        .summary(std::string("Cancel execution for ") + et.singular);

    // --- Configurations ---
    reg.get(entity_path + "/configurations",
            [this](auto & req, auto & res) {
              config_handlers_->handle_list_configurations(req, res);
            })
        .tag("Configuration")
        .summary(std::string("List configurations for ") + et.singular);

    reg.get(entity_path + "/configurations/{config_id}",
            [this](auto & req, auto & res) {
              config_handlers_->handle_get_configuration(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Get specific configuration for ") + et.singular);

    reg.put(entity_path + "/configurations/{config_id}",
            [this](auto & req, auto & res) {
              config_handlers_->handle_set_configuration(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Set configuration for ") + et.singular);

    reg.del(entity_path + "/configurations/{config_id}",
            [this](auto & req, auto & res) {
              config_handlers_->handle_delete_configuration(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Delete configuration for ") + et.singular);

    reg.del(entity_path + "/configurations",
            [this](auto & req, auto & res) {
              config_handlers_->handle_delete_all_configurations(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Delete all configurations for ") + et.singular);

    // --- Faults ---
    reg.get(entity_path + "/faults",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_list_faults(req, res);
            })
        .tag("Faults")
        .summary(std::string("List faults for ") + et.singular);

    reg.get(entity_path + "/faults/{fault_code}",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_get_fault(req, res);
            })
        .tag("Faults")
        .summary(std::string("Get specific fault for ") + et.singular);

    reg.del(entity_path + "/faults/{fault_code}",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_clear_fault(req, res);
            })
        .tag("Faults")
        .summary(std::string("Clear fault for ") + et.singular);

    reg.del(entity_path + "/faults",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_clear_all_faults(req, res);
            })
        .tag("Faults")
        .summary(std::string("Clear all faults for ") + et.singular);

    // --- Logs ---
    reg.get(entity_path + "/logs",
            [this](auto & req, auto & res) {
              log_handlers_->handle_get_logs(req, res);
            })
        .tag("Logs")
        .summary(std::string("Query log entries for ") + et.singular);

    reg.get(entity_path + "/logs/configuration",
            [this](auto & req, auto & res) {
              log_handlers_->handle_get_logs_configuration(req, res);
            })
        .tag("Logs")
        .summary(std::string("Get log configuration for ") + et.singular);

    reg.put(entity_path + "/logs/configuration",
            [this](auto & req, auto & res) {
              log_handlers_->handle_put_logs_configuration(req, res);
            })
        .tag("Logs")
        .summary(std::string("Update log configuration for ") + et.singular);

    // --- Bulk Data ---
    reg.get(entity_path + "/bulk-data",
            [this](auto & req, auto & res) {
              bulkdata_handlers_->handle_list_categories(req, res);
            })
        .tag("Bulk Data")
        .summary(std::string("List bulk-data categories for ") + et.singular);

    reg.get(entity_path + "/bulk-data/{category_id}",
            [this](auto & req, auto & res) {
              bulkdata_handlers_->handle_list_descriptors(req, res);
            })
        .tag("Bulk Data")
        .summary(std::string("List bulk-data descriptors for ") + et.singular);

    reg.get(entity_path + "/bulk-data/{category_id}/{file_id}",
            [this](auto & req, auto & res) {
              bulkdata_handlers_->handle_download(req, res);
            })
        .tag("Bulk Data")
        .summary(std::string("Download bulk-data file for ") + et.singular);

    // Upload: only for apps and components (405 for areas and functions)
    std::string et_type_str = et.type;
    if (et_type_str == "apps" || et_type_str == "components") {
      reg.post(entity_path + "/bulk-data/{category_id}",
               [this](auto & req, auto & res) {
                 bulkdata_handlers_->handle_upload(req, res);
               })
          .tag("Bulk Data")
          .summary(std::string("Upload bulk-data for ") + et.singular);

      reg.del(entity_path + "/bulk-data/{category_id}/{file_id}",
              [this](auto & req, auto & res) {
                bulkdata_handlers_->handle_delete(req, res);
              })
          .tag("Bulk Data")
          .summary(std::string("Delete bulk-data file for ") + et.singular);
    } else {
      reg.post(entity_path + "/bulk-data/{category_id}",
               [this](auto & /*req*/, auto & res) {
                 handlers::HandlerContext::send_error(res, 405, ERR_INVALID_REQUEST,
                                                      "Bulk data upload is only supported for components and apps");
               })
          .tag("Bulk Data")
          .summary(std::string("Upload bulk-data for ") + et.singular + " (not supported)")
          .response(405, "Method not allowed");

      reg.del(entity_path + "/bulk-data/{category_id}/{file_id}",
              [this](auto & /*req*/, auto & res) {
                handlers::HandlerContext::send_error(res, 405, ERR_INVALID_REQUEST,
                                                     "Bulk data deletion is only supported for components and apps");
              })
          .tag("Bulk Data")
          .summary(std::string("Delete bulk-data file for ") + et.singular + " (not supported)")
          .response(405, "Method not allowed");
    }

    // --- Cyclic Subscriptions (apps, components, and functions) ---
    if (et_type_str == "apps" || et_type_str == "components" || et_type_str == "functions") {
      // SSE events stream - registered before CRUD routes
      reg.get(entity_path + "/cyclic-subscriptions/{subscription_id}/events",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_events(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("SSE events for cyclic subscription on ") + et.singular);

      reg.post(entity_path + "/cyclic-subscriptions",
               [this](auto & req, auto & res) {
                 cyclic_sub_handlers_->handle_create(req, res);
               })
          .tag("Subscriptions")
          .summary(std::string("Create cyclic subscription for ") + et.singular);

      reg.get(entity_path + "/cyclic-subscriptions",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_list(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("List cyclic subscriptions for ") + et.singular);

      reg.get(entity_path + "/cyclic-subscriptions/{subscription_id}",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_get(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("Get cyclic subscription for ") + et.singular);

      reg.put(entity_path + "/cyclic-subscriptions/{subscription_id}",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_update(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("Update cyclic subscription for ") + et.singular);

      reg.del(entity_path + "/cyclic-subscriptions/{subscription_id}",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_delete(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("Delete cyclic subscription for ") + et.singular);
    }

    // --- Locking (components and apps only, per SOVD spec) ---
    if (et_type_str == "components" || et_type_str == "apps") {
      reg.post(entity_path + "/locks",
               [this](auto & req, auto & res) {
                 lock_handlers_->handle_acquire_lock(req, res);
               })
          .tag("Locking")
          .summary(std::string("Acquire lock on ") + et.singular);

      reg.get(entity_path + "/locks",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_list_locks(req, res);
              })
          .tag("Locking")
          .summary(std::string("List locks on ") + et.singular);

      reg.get(entity_path + "/locks/{lock_id}",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_get_lock(req, res);
              })
          .tag("Locking")
          .summary(std::string("Get lock details for ") + et.singular);

      reg.put(entity_path + "/locks/{lock_id}",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_extend_lock(req, res);
              })
          .tag("Locking")
          .summary(std::string("Extend lock on ") + et.singular);

      reg.del(entity_path + "/locks/{lock_id}",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_release_lock(req, res);
              })
          .tag("Locking")
          .summary(std::string("Release lock on ") + et.singular);
    }

    // --- Scripts (apps and components only) ---
    if (script_handlers_ && (et_type_str == "apps" || et_type_str == "components")) {
      reg.post(entity_path + "/scripts",
               [this](auto & req, auto & res) {
                 script_handlers_->handle_upload_script(req, res);
               })
          .tag("Scripts")
          .summary(std::string("Upload diagnostic script for ") + et.singular)
          .response(201, "Script uploaded");

      reg.get(entity_path + "/scripts",
              [this](auto & req, auto & res) {
                script_handlers_->handle_list_scripts(req, res);
              })
          .tag("Scripts")
          .summary(std::string("List scripts for ") + et.singular);

      reg.get(entity_path + "/scripts/{script_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_get_script(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Get script metadata for ") + et.singular);

      reg.del(entity_path + "/scripts/{script_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_delete_script(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Delete script for ") + et.singular)
          .response(204, "Script deleted");

      reg.post(entity_path + "/scripts/{script_id}/executions",
               [this](auto & req, auto & res) {
                 script_handlers_->handle_start_execution(req, res);
               })
          .tag("Scripts")
          .summary(std::string("Start script execution for ") + et.singular)
          .response(202, "Execution started");

      reg.get(entity_path + "/scripts/{script_id}/executions/{execution_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_get_execution(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Get execution status for ") + et.singular);

      reg.put(entity_path + "/scripts/{script_id}/executions/{execution_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_control_execution(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Terminate script execution for ") + et.singular);

      reg.del(entity_path + "/scripts/{script_id}/executions/{execution_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_delete_execution(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Remove completed execution for ") + et.singular)
          .response(204, "Execution removed");
    }

    // --- Discovery relationship endpoints (entity-type-specific) ---
    if (et_type_str == "areas") {
      reg.get(entity_path + "/components",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_area_components(req, res);
              })
          .tag("Discovery")
          .summary("List components in area");

      reg.get(entity_path + "/subareas",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_subareas(req, res);
              })
          .tag("Discovery")
          .summary("List subareas");

      reg.get(entity_path + "/contains",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_contains(req, res);
              })
          .tag("Discovery")
          .summary("List entities contained in area");
    }

    if (et_type_str == "components") {
      reg.get(entity_path + "/subcomponents",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_subcomponents(req, res);
              })
          .tag("Discovery")
          .summary("List subcomponents");

      reg.get(entity_path + "/hosts",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_hosts(req, res);
              })
          .tag("Discovery")
          .summary("List component hosts");

      reg.get(entity_path + "/depends-on",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_component_depends_on(req, res);
              })
          .tag("Discovery")
          .summary("List component dependencies");
    }

    if (et_type_str == "apps") {
      reg.get(entity_path + "/depends-on",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_app_depends_on(req, res);
              })
          .tag("Discovery")
          .summary("List app dependencies");
    }

    if (et_type_str == "functions") {
      reg.get(entity_path + "/hosts",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_function_hosts(req, res);
              })
          .tag("Discovery")
          .summary("List function hosts");
    }

    // Single entity detail (capabilities) - must be LAST for this entity type
    reg.get(entity_path, et.detail_handler).tag("Discovery").summary(std::string("Get ") + et.singular + " details");
  }

  // === Nested entities - subareas bulk-data ===
  reg.get("/areas/{area_id}/subareas/{subarea_id}/bulk-data",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_categories(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data categories for subarea");

  reg.get("/areas/{area_id}/subareas/{subarea_id}/bulk-data/{category_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_descriptors(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data descriptors for subarea");

  reg.get("/areas/{area_id}/subareas/{subarea_id}/bulk-data/{category_id}/{file_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_download(req, res);
          })
      .tag("Bulk Data")
      .summary("Download bulk-data file for subarea");

  // === Nested entities - subcomponents bulk-data ===
  reg.get("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_categories(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data categories for subcomponent");

  reg.get("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data/{category_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_descriptors(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data descriptors for subcomponent");

  reg.get("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data/{category_id}/{file_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_download(req, res);
          })
      .tag("Bulk Data")
      .summary("Download bulk-data file for subcomponent");

  // === Global faults ===
  // SSE stream - must be before /faults to avoid regex conflict
  reg.get("/faults/stream",
          [this](auto & req, auto & res) {
            sse_fault_handler_->handle_stream(req, res);
          })
      .tag("Faults")
      .summary("Stream fault events (SSE)")
      .description("Server-Sent Events stream for real-time fault notifications.");

  reg.get("/faults",
          [this](auto & req, auto & res) {
            fault_handlers_->handle_list_all_faults(req, res);
          })
      .tag("Faults")
      .summary("List all faults globally")
      .description("Retrieve all faults across the system.");

  reg.del("/faults",
          [this](auto & req, auto & res) {
            fault_handlers_->handle_clear_all_faults_global(req, res);
          })
      .tag("Faults")
      .summary("Clear all faults globally");

  // === Software Updates ===
  // Always register for OpenAPI documentation. Lambdas guard against null update_handlers_.
  auto update_501 = [](auto & /*req*/, auto & res) {
    handlers::HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Software updates not available");
  };

  reg.get("/updates", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_list_updates(req, res);
          })
                                       : HandlerFn(update_501))
      .tag("Updates")
      .summary("List software updates");

  reg.post("/updates", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
             update_handlers_->handle_register_update(req, res);
           })
                                        : HandlerFn(update_501))
      .tag("Updates")
      .summary("Register a software update");

  reg.get("/updates/{update_id}/status", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_get_status(req, res);
          })
                                                          : HandlerFn(update_501))
      .tag("Updates")
      .summary("Get update status");

  reg.put("/updates/{update_id}/prepare", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_prepare(req, res);
          })
                                                           : HandlerFn(update_501))
      .tag("Updates")
      .summary("Prepare update for execution");

  reg.put("/updates/{update_id}/execute", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_execute(req, res);
          })
                                                           : HandlerFn(update_501))
      .tag("Updates")
      .summary("Execute update");

  reg.put("/updates/{update_id}/automated", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_automated(req, res);
          })
                                                             : HandlerFn(update_501))
      .tag("Updates")
      .summary("Run automated update");

  reg.get("/updates/{update_id}", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_get_update(req, res);
          })
                                                   : HandlerFn(update_501))
      .tag("Updates")
      .summary("Get update details");

  reg.del("/updates/{update_id}", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_delete_update(req, res);
          })
                                                   : HandlerFn(update_501))
      .tag("Updates")
      .summary("Delete update");

  // === Authentication ===
  reg.post("/auth/authorize",
           [this](auto & req, auto & res) {
             auth_handlers_->handle_auth_authorize(req, res);
           })
      .tag("Authentication")
      .summary("Authorize client")
      .description("Authenticate and obtain authorization tokens.");

  reg.post("/auth/token",
           [this](auto & req, auto & res) {
             auth_handlers_->handle_auth_token(req, res);
           })
      .tag("Authentication")
      .summary("Obtain access token")
      .description("Exchange credentials or refresh token for a JWT access token.");

  reg.post("/auth/revoke",
           [this](auto & req, auto & res) {
             auth_handlers_->handle_auth_revoke(req, res);
           })
      .tag("Authentication")
      .summary("Revoke token")
      .description("Revoke an access or refresh token.");

  // Register all routes with cpp-httplib
  route_registry_->register_all(*srv, API_BASE_PATH);
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

  // Expose headers that JavaScript needs access to (e.g., for file downloads)
  res.set_header("Access-Control-Expose-Headers",
                 "Content-Disposition, Content-Length, X-RateLimit-Limit, X-RateLimit-Remaining, "
                 "X-RateLimit-Reset, Retry-After");

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
