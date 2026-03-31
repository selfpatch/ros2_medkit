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
#include "../openapi/schema_builder.hpp"

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

  if (node_->get_script_manager()) {
    script_handlers_ = std::make_unique<handlers::ScriptHandlers>(*handler_ctx_, node_->get_script_manager());
  }

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

void RESTServer::set_trigger_handlers(TriggerManager & trigger_mgr) {
  trigger_handlers_ = std::make_unique<handlers::TriggerHandlers>(*handler_ctx_, trigger_mgr, sse_client_tracker_);
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
  using SB = openapi::SchemaBuilder;

  auto capitalize = [](const std::string & s) -> std::string {
    if (s.empty()) {
      return s;
    }
    std::string result = s;
    result[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(result[0])));
    return result;
  };

  // === Server endpoints ===
  reg.get("/health",
          [this](auto & req, auto & res) {
            health_handlers_->handle_health(req, res);
          })
      .tag("Server")
      .summary("Health check")
      .description("Returns gateway health status.")
      .response(200, "Gateway is healthy", SB::ref("HealthStatus"))
      .operation_id("getHealth");

  reg.get("/",
          [this](auto & req, auto & res) {
            health_handlers_->handle_root(req, res);
          })
      .tag("Server")
      .summary("API overview")
      .description("Returns gateway metadata, available endpoints, and capabilities.")
      .response(200, "API metadata", SB::ref("RootOverview"))
      .operation_id("getRoot");

  reg.get("/version-info",
          [this](auto & req, auto & res) {
            health_handlers_->handle_version_info(req, res);
          })
      .tag("Server")
      .summary("SOVD version information")
      .description("Returns SOVD specification version and vendor info.")
      .response(200, "Version info", SB::ref("VersionInfo"))
      .operation_id("getVersionInfo");

  // === Discovery - entity collections ===
  reg.get("/areas",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_areas(req, res);
          })
      .tag("Discovery")
      .summary("List areas")
      .description("Lists all discovered areas in the system.")
      .response(200, "Area list", SB::ref("EntityList"))
      .operation_id("listAreas");

  reg.get("/apps",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_apps(req, res);
          })
      .tag("Discovery")
      .summary("List apps")
      .description("Lists all discovered apps (ROS 2 nodes) in the system.")
      .response(200, "App list", SB::ref("EntityList"))
      .operation_id("listApps");

  reg.get("/components",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_components(req, res);
          })
      .tag("Discovery")
      .summary("List components")
      .description("Lists all discovered components in the system.")
      .response(200, "Component list", SB::ref("EntityList"))
      .operation_id("listComponents");

  reg.get("/functions",
          [this](auto & req, auto & res) {
            discovery_handlers_->handle_list_functions(req, res);
          })
      .tag("Discovery")
      .summary("List functions")
      .description("Lists all discovered functions in the system.")
      .response(200, "Function list", SB::ref("EntityList"))
      .operation_id("listFunctions");

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
        .summary(std::string("Get data item for ") + et.singular)
        .description(std::string("Returns the latest value from a ROS 2 topic for this ") + et.singular + ".")
        .response(200, "Data value", SB::generic_object_schema())
        .operation_id(std::string("get") + capitalize(et.singular) + "DataItem");

    reg.put(entity_path + "/data/{data_id}",
            [this](auto & req, auto & res) {
              data_handlers_->handle_put_data_item(req, res);
            })
        .tag("Data")
        .summary(std::string("Write data item for ") + et.singular)
        .description(std::string("Publishes a value to a ROS 2 topic on this ") + et.singular + ".")
        .request_body("Data value to write", SB::ref("DataWriteRequest"))
        .response(200, "Written value", SB::generic_object_schema())
        .operation_id(std::string("put") + capitalize(et.singular) + "DataItem");

    // Data-categories (returns 501 - not yet implemented)
    reg.get(entity_path + "/data-categories",
            [this](auto & req, auto & res) {
              data_handlers_->handle_data_categories(req, res);
            })
        .tag("Data")
        .summary(std::string("List data categories for ") + et.singular)
        .description(std::string("Lists available data categories for this ") + et.singular + ".")
        .response(200, "Category list", SB::ref("BulkDataCategoryList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "DataCategories");

    // Data-groups (returns 501 - not yet implemented)
    reg.get(entity_path + "/data-groups",
            [this](auto & req, auto & res) {
              data_handlers_->handle_data_groups(req, res);
            })
        .tag("Data")
        .summary(std::string("List data groups for ") + et.singular)
        .description(std::string("Lists available data groups for this ") + et.singular + ".")
        .response(200, "Group list", SB::items_wrapper(SB::generic_object_schema()))
        .operation_id(std::string("list") + capitalize(et.singular) + "DataGroups");

    // Data collection (all topics)
    reg.get(entity_path + "/data",
            [this](auto & req, auto & res) {
              data_handlers_->handle_list_data(req, res);
            })
        .tag("Data")
        .summary(std::string("List data items for ") + et.singular)
        .description(std::string("Lists all data items (ROS 2 topics) available on this ") + et.singular + ".")
        .response(200, "Data item list", SB::ref("DataItemList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "Data");

    // --- Operations ---
    reg.get(entity_path + "/operations",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_list_operations(req, res);
            })
        .tag("Operations")
        .summary(std::string("List operations for ") + et.singular)
        .description(std::string("Lists all ROS 2 services and actions available on this ") + et.singular + ".")
        .response(200, "Operation list", SB::ref("OperationItemList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "Operations");

    reg.get(entity_path + "/operations/{operation_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_get_operation(req, res);
            })
        .tag("Operations")
        .summary(std::string("Get operation details for ") + et.singular)
        .description(std::string("Returns operation details including request/response schema for this ") +
                     et.singular + ".")
        .response(200, "Operation details", SB::ref("OperationDetail"))
        .operation_id(std::string("get") + capitalize(et.singular) + "Operation");

    // Execution endpoints
    reg.post(entity_path + "/operations/{operation_id}/executions",
             [this](auto & req, auto & res) {
               operation_handlers_->handle_create_execution(req, res);
             })
        .tag("Operations")
        .summary(std::string("Start operation execution for ") + et.singular)
        .description("Starts a new execution. Returns 200 for synchronous, 202 for asynchronous operations.")
        .request_body("Operation parameters", SB::generic_object_schema())
        .response(200, "Synchronous result", SB::generic_object_schema())
        .response(202, "Asynchronous execution started", SB::ref("OperationExecution"))
        .operation_id(std::string("execute") + capitalize(et.singular) + "Operation");

    reg.get(entity_path + "/operations/{operation_id}/executions",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_list_executions(req, res);
            })
        .tag("Operations")
        .summary(std::string("List operation executions for ") + et.singular)
        .description(std::string("Lists all executions of an operation on this ") + et.singular + ".")
        .response(200, "Execution list", SB::ref("OperationExecutionList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "Executions");

    reg.get(entity_path + "/operations/{operation_id}/executions/{execution_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_get_execution(req, res);
            })
        .tag("Operations")
        .summary(std::string("Get execution status for ") + et.singular)
        .description("Returns the current status and result of a specific execution.")
        .response(200, "Execution status", SB::ref("OperationExecution"))
        .operation_id(std::string("get") + capitalize(et.singular) + "Execution");

    reg.put(entity_path + "/operations/{operation_id}/executions/{execution_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_update_execution(req, res);
            })
        .tag("Operations")
        .summary(std::string("Update execution for ") + et.singular)
        .description("Sends a control command to a running execution.")
        .request_body("Execution control", SB::ref("ExecutionUpdateRequest"))
        .response(200, "Updated execution", SB::ref("OperationExecution"))
        .operation_id(std::string("update") + capitalize(et.singular) + "Execution");

    reg.del(entity_path + "/operations/{operation_id}/executions/{execution_id}",
            [this](auto & req, auto & res) {
              operation_handlers_->handle_cancel_execution(req, res);
            })
        .tag("Operations")
        .summary(std::string("Cancel execution for ") + et.singular)
        .description("Cancels a running execution.")
        .response(204, "Execution cancelled")
        .operation_id(std::string("cancel") + capitalize(et.singular) + "Execution");

    // --- Configurations ---
    reg.get(entity_path + "/configurations",
            [this](auto & req, auto & res) {
              config_handlers_->handle_list_configurations(req, res);
            })
        .tag("Configuration")
        .summary(std::string("List configurations for ") + et.singular)
        .description(std::string("Lists all ROS 2 node parameters for this ") + et.singular + ".")
        .response(200, "Configuration list", SB::ref("ConfigurationMetaDataList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "Configurations");

    reg.get(entity_path + "/configurations/{config_id}",
            [this](auto & req, auto & res) {
              config_handlers_->handle_get_configuration(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Get specific configuration for ") + et.singular)
        .description(std::string("Returns a specific ROS 2 node parameter for this ") + et.singular + ".")
        .response(200, "Configuration parameter", SB::ref("ConfigurationReadValue"))
        .operation_id(std::string("get") + capitalize(et.singular) + "Configuration");

    reg.put(entity_path + "/configurations/{config_id}",
            [this](auto & req, auto & res) {
              config_handlers_->handle_set_configuration(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Set configuration for ") + et.singular)
        .description(std::string("Sets a ROS 2 node parameter value for this ") + et.singular + ".")
        .request_body("Configuration value", SB::ref("ConfigurationWriteValue"))
        .response(200, "Updated configuration", SB::ref("ConfigurationReadValue"))
        .operation_id(std::string("set") + capitalize(et.singular) + "Configuration");

    reg.del(entity_path + "/configurations/{config_id}",
            [this](auto & req, auto & res) {
              config_handlers_->handle_delete_configuration(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Delete configuration for ") + et.singular)
        .description(std::string("Resets a configuration parameter to its default for this ") + et.singular + ".")
        .response(204, "Configuration deleted")
        .operation_id(std::string("delete") + capitalize(et.singular) + "Configuration");

    reg.del(entity_path + "/configurations",
            [this](auto & req, auto & res) {
              config_handlers_->handle_delete_all_configurations(req, res);
            })
        .tag("Configuration")
        .summary(std::string("Delete all configurations for ") + et.singular)
        .description(std::string("Resets all configuration parameters for this ") + et.singular + ".")
        .response(204, "All configurations deleted")
        .response(207, "Partial success - some nodes failed", SB::ref("ConfigurationDeleteMultiStatus"))
        .operation_id(std::string("deleteAll") + capitalize(et.singular) + "Configurations");

    // --- Faults ---
    reg.get(entity_path + "/faults",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_list_faults(req, res);
            })
        .tag("Faults")
        .summary(std::string("List faults for ") + et.singular)
        .description(std::string("Returns all active faults reported by this ") + et.singular + ".")
        .response(200, "Fault list", SB::ref("FaultList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "Faults");

    reg.get(entity_path + "/faults/{fault_code}",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_get_fault(req, res);
            })
        .tag("Faults")
        .summary(std::string("Get specific fault for ") + et.singular)
        .description("Returns fault details including SOVD status, environment data, and rosbag snapshots.")
        .response(200, "Fault detail", SB::ref("FaultDetail"))
        .operation_id(std::string("get") + capitalize(et.singular) + "Fault");

    reg.del(entity_path + "/faults/{fault_code}",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_clear_fault(req, res);
            })
        .tag("Faults")
        .summary(std::string("Clear fault for ") + et.singular)
        .description(std::string("Clears a specific fault for this ") + et.singular + ".")
        .response(204, "Fault cleared")
        .operation_id(std::string("clear") + capitalize(et.singular) + "Fault");

    reg.del(entity_path + "/faults",
            [this](auto & req, auto & res) {
              fault_handlers_->handle_clear_all_faults(req, res);
            })
        .tag("Faults")
        .summary(std::string("Clear all faults for ") + et.singular)
        .description(std::string("Clears all faults for this ") + et.singular + ".")
        .response(204, "All faults cleared")
        .operation_id(std::string("clearAll") + capitalize(et.singular) + "Faults");

    // --- Logs ---
    reg.get(entity_path + "/logs",
            [this](auto & req, auto & res) {
              log_handlers_->handle_get_logs(req, res);
            })
        .tag("Logs")
        .summary(std::string("Query log entries for ") + et.singular)
        .description(std::string("Queries application log entries for this ") + et.singular + ".")
        .response(200, "Log entries", SB::ref("LogEntryList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "Logs");

    reg.get(entity_path + "/logs/configuration",
            [this](auto & req, auto & res) {
              log_handlers_->handle_get_logs_configuration(req, res);
            })
        .tag("Logs")
        .summary(std::string("Get log configuration for ") + et.singular)
        .description(std::string("Returns the log filter configuration for this ") + et.singular + ".")
        .response(200, "Log configuration", SB::ref("LogConfiguration"))
        .operation_id(std::string("get") + capitalize(et.singular) + "LogConfiguration");

    reg.put(entity_path + "/logs/configuration",
            [this](auto & req, auto & res) {
              log_handlers_->handle_put_logs_configuration(req, res);
            })
        .tag("Logs")
        .summary(std::string("Update log configuration for ") + et.singular)
        .description(std::string("Updates the log severity filter and max entries for this ") + et.singular + ".")
        .request_body("Log configuration", SB::ref("LogConfiguration"))
        .response(204, "Configuration updated")
        .operation_id(std::string("set") + capitalize(et.singular) + "LogConfiguration");

    // --- Bulk Data ---
    reg.get(entity_path + "/bulk-data",
            [this](auto & req, auto & res) {
              bulkdata_handlers_->handle_list_categories(req, res);
            })
        .tag("Bulk Data")
        .summary(std::string("List bulk-data categories for ") + et.singular)
        .description(std::string("Lists bulk-data categories (e.g., rosbag snapshots) for this ") + et.singular + ".")
        .response(200, "Category list", SB::ref("BulkDataCategoryList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "BulkDataCategories");

    reg.get(entity_path + "/bulk-data/{category_id}",
            [this](auto & req, auto & res) {
              bulkdata_handlers_->handle_list_descriptors(req, res);
            })
        .tag("Bulk Data")
        .summary(std::string("List bulk-data descriptors for ") + et.singular)
        .description(std::string("Lists downloadable files in a bulk-data category for this ") + et.singular + ".")
        .response(200, "Descriptor list", SB::ref("BulkDataDescriptorList"))
        .operation_id(std::string("list") + capitalize(et.singular) + "BulkDataDescriptors");

    reg.get(entity_path + "/bulk-data/{category_id}/{file_id}",
            [this](auto & req, auto & res) {
              bulkdata_handlers_->handle_download(req, res);
            })
        .tag("Bulk Data")
        .summary(std::string("Download bulk-data file for ") + et.singular)
        .description("Downloads a bulk-data file (binary content).")
        .response(200, "File content", SB::binary_schema())
        .operation_id(std::string("download") + capitalize(et.singular) + "BulkData");

    // Upload: only for apps and components (405 for areas and functions)
    std::string et_type_str = et.type;
    if (et_type_str == "apps" || et_type_str == "components") {
      reg.post(entity_path + "/bulk-data/{category_id}",
               [this](auto & req, auto & res) {
                 bulkdata_handlers_->handle_upload(req, res);
               })
          .tag("Bulk Data")
          .summary(std::string("Upload bulk-data for ") + et.singular)
          .description(std::string("Uploads a file to a bulk-data category for this ") + et.singular + ".")
          .request_body("File to upload", SB::binary_schema(), "multipart/form-data")
          .response(201, "File uploaded", SB::ref("BulkDataDescriptor"))
          .operation_id(std::string("upload") + capitalize(et.singular) + "BulkData");

      reg.del(entity_path + "/bulk-data/{category_id}/{file_id}",
              [this](auto & req, auto & res) {
                bulkdata_handlers_->handle_delete(req, res);
              })
          .tag("Bulk Data")
          .summary(std::string("Delete bulk-data file for ") + et.singular)
          .description(std::string("Deletes a bulk-data file for this ") + et.singular + ".")
          .response(204, "File deleted")
          .operation_id(std::string("delete") + capitalize(et.singular) + "BulkData");
    } else {
      reg.post(entity_path + "/bulk-data/{category_id}",
               [this](auto & /*req*/, auto & res) {
                 handlers::HandlerContext::send_error(res, 405, ERR_INVALID_REQUEST,
                                                      "Bulk data upload is only supported for components and apps");
               })
          .tag("Bulk Data")
          .summary(std::string("Upload bulk-data for ") + et.singular + " (not supported)")
          .description("Bulk data upload is not supported for this entity type.")
          .response(405, "Method not allowed")
          .hidden();  // Always returns 405 - exclude from OpenAPI spec and generated clients

      reg.del(entity_path + "/bulk-data/{category_id}/{file_id}",
              [this](auto & /*req*/, auto & res) {
                handlers::HandlerContext::send_error(res, 405, ERR_INVALID_REQUEST,
                                                     "Bulk data deletion is only supported for components and apps");
              })
          .tag("Bulk Data")
          .summary(std::string("Delete bulk-data file for ") + et.singular + " (not supported)")
          .description("Bulk data deletion is not supported for this entity type.")
          .response(405, "Method not allowed")
          .hidden();  // Always returns 405 - exclude from OpenAPI spec and generated clients
    }

    // --- Triggers (ALL entity types - x-medkit extension beyond SOVD) ---
    {
      auto trigger_501 = [](auto & /*req*/, auto & res) {
        handlers::HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Triggers not available");
      };

      // SSE events stream - registered before CRUD routes
      reg.get(entity_path + "/triggers/{trigger_id}/events",
              [this, trigger_501](auto & req, auto & res) {
                if (!trigger_handlers_) {
                  trigger_501(req, res);
                  return;
                }
                trigger_handlers_->handle_events(req, res);
              })
          .tag("Triggers")
          .summary(std::string("SSE events stream for trigger on ") + et.singular)
          .description(std::string("Server-Sent Events stream for trigger notifications on this ") + et.singular + ".")
          .operation_id(std::string("stream") + capitalize(et.singular) + "TriggerEvents");

      reg.post(entity_path + "/triggers",
               [this, trigger_501](auto & req, auto & res) {
                 if (!trigger_handlers_) {
                   trigger_501(req, res);
                   return;
                 }
                 trigger_handlers_->handle_create(req, res);
               })
          .tag("Triggers")
          .summary(std::string("Create trigger for ") + et.singular)
          .description(std::string("Creates a new event trigger for this ") + et.singular + ".")
          .request_body("Trigger configuration", SB::ref("TriggerCreateRequest"))
          .response(201, "Trigger created", SB::ref("Trigger"))
          .operation_id(std::string("create") + capitalize(et.singular) + "Trigger");

      reg.get(entity_path + "/triggers",
              [this, trigger_501](auto & req, auto & res) {
                if (!trigger_handlers_) {
                  trigger_501(req, res);
                  return;
                }
                trigger_handlers_->handle_list(req, res);
              })
          .tag("Triggers")
          .summary(std::string("List triggers for ") + et.singular)
          .description(std::string("Lists all triggers configured for this ") + et.singular + ".")
          .response(200, "Trigger list", SB::ref("TriggerList"))
          .operation_id(std::string("list") + capitalize(et.singular) + "Triggers");

      reg.get(entity_path + "/triggers/{trigger_id}",
              [this, trigger_501](auto & req, auto & res) {
                if (!trigger_handlers_) {
                  trigger_501(req, res);
                  return;
                }
                trigger_handlers_->handle_get(req, res);
              })
          .tag("Triggers")
          .summary(std::string("Get trigger for ") + et.singular)
          .description(std::string("Returns details of a specific trigger on this ") + et.singular + ".")
          .response(200, "Trigger details", SB::ref("Trigger"))
          .operation_id(std::string("get") + capitalize(et.singular) + "Trigger");

      reg.put(entity_path + "/triggers/{trigger_id}",
              [this, trigger_501](auto & req, auto & res) {
                if (!trigger_handlers_) {
                  trigger_501(req, res);
                  return;
                }
                trigger_handlers_->handle_update(req, res);
              })
          .tag("Triggers")
          .summary(std::string("Update trigger for ") + et.singular)
          .description(std::string("Updates a trigger configuration on this ") + et.singular + ".")
          .request_body("Trigger update", SB::ref("TriggerUpdateRequest"))
          .response(200, "Updated trigger", SB::ref("Trigger"))
          .operation_id(std::string("update") + capitalize(et.singular) + "Trigger");

      reg.del(entity_path + "/triggers/{trigger_id}",
              [this, trigger_501](auto & req, auto & res) {
                if (!trigger_handlers_) {
                  trigger_501(req, res);
                  return;
                }
                trigger_handlers_->handle_delete(req, res);
              })
          .tag("Triggers")
          .summary(std::string("Delete trigger for ") + et.singular)
          .description(std::string("Deletes a trigger from this ") + et.singular + ".")
          .response(204, "Trigger deleted")
          .operation_id(std::string("delete") + capitalize(et.singular) + "Trigger");
    }

    // --- Cyclic Subscriptions (apps, components, and functions) ---
    if (et_type_str == "apps" || et_type_str == "components" || et_type_str == "functions") {
      // SSE events stream - registered before CRUD routes
      reg.get(entity_path + "/cyclic-subscriptions/{subscription_id}/events",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_events(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("SSE events stream for cyclic subscription on ") + et.singular)
          .description(std::string("Server-Sent Events stream for subscription data on this ") + et.singular + ".")
          .operation_id(std::string("stream") + capitalize(et.singular) + "SubscriptionEvents");

      reg.post(entity_path + "/cyclic-subscriptions",
               [this](auto & req, auto & res) {
                 cyclic_sub_handlers_->handle_create(req, res);
               })
          .tag("Subscriptions")
          .summary(std::string("Create cyclic subscription for ") + et.singular)
          .description(std::string("Creates a new cyclic data subscription for this ") + et.singular + ".")
          .request_body("Subscription configuration", SB::ref("CyclicSubscriptionCreateRequest"))
          .response(201, "Subscription created", SB::ref("CyclicSubscription"))
          .operation_id(std::string("create") + capitalize(et.singular) + "Subscription");

      reg.get(entity_path + "/cyclic-subscriptions",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_list(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("List cyclic subscriptions for ") + et.singular)
          .description(std::string("Lists all cyclic subscriptions for this ") + et.singular + ".")
          .response(200, "Subscription list", SB::ref("CyclicSubscriptionList"))
          .operation_id(std::string("list") + capitalize(et.singular) + "Subscriptions");

      reg.get(entity_path + "/cyclic-subscriptions/{subscription_id}",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_get(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("Get cyclic subscription for ") + et.singular)
          .description(std::string("Returns details of a specific subscription on this ") + et.singular + ".")
          .response(200, "Subscription details", SB::ref("CyclicSubscription"))
          .operation_id(std::string("get") + capitalize(et.singular) + "Subscription");

      reg.put(entity_path + "/cyclic-subscriptions/{subscription_id}",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_update(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("Update cyclic subscription for ") + et.singular)
          .description(std::string("Updates a subscription configuration on this ") + et.singular + ".")
          .request_body("Subscription update", SB::ref("CyclicSubscription"))
          .response(200, "Updated subscription", SB::ref("CyclicSubscription"))
          .operation_id(std::string("update") + capitalize(et.singular) + "Subscription");

      reg.del(entity_path + "/cyclic-subscriptions/{subscription_id}",
              [this](auto & req, auto & res) {
                cyclic_sub_handlers_->handle_delete(req, res);
              })
          .tag("Subscriptions")
          .summary(std::string("Delete cyclic subscription for ") + et.singular)
          .description(std::string("Deletes a cyclic subscription from this ") + et.singular + ".")
          .response(204, "Subscription deleted")
          .operation_id(std::string("delete") + capitalize(et.singular) + "Subscription");
    }

    // --- Locking (components and apps only, per SOVD spec) ---
    if (et_type_str == "components" || et_type_str == "apps") {
      reg.post(entity_path + "/locks",
               [this](auto & req, auto & res) {
                 lock_handlers_->handle_acquire_lock(req, res);
               })
          .tag("Locking")
          .summary(std::string("Acquire lock on ") + et.singular)
          .description(std::string("Acquires an exclusive lock on this ") + et.singular + ".")
          .request_body("Lock parameters", SB::ref("AcquireLockRequest"))
          .header_param("X-Client-Id", "Unique client identifier for lock ownership")
          .response(201, "Lock acquired", SB::ref("Lock"))
          .operation_id(std::string("acquire") + capitalize(et.singular) + "Lock");

      reg.get(entity_path + "/locks",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_list_locks(req, res);
              })
          .tag("Locking")
          .summary(std::string("List locks on ") + et.singular)
          .description(std::string("Lists all active locks on this ") + et.singular + ".")
          .header_param("X-Client-Id", "When provided, the 'owned' field indicates whether this client owns the lock",
                        false)
          .response(200, "Lock list", SB::ref("LockList"))
          .operation_id(std::string("list") + capitalize(et.singular) + "Locks");

      reg.get(entity_path + "/locks/{lock_id}",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_get_lock(req, res);
              })
          .tag("Locking")
          .summary(std::string("Get lock details for ") + et.singular)
          .description(std::string("Returns details of a specific lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "When provided, the 'owned' field indicates whether this client owns the lock",
                        false)
          .response(200, "Lock details", SB::ref("Lock"))
          .operation_id(std::string("get") + capitalize(et.singular) + "Lock");

      reg.put(entity_path + "/locks/{lock_id}",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_extend_lock(req, res);
              })
          .tag("Locking")
          .summary(std::string("Extend lock on ") + et.singular)
          .description(std::string("Extends the expiration of a lock on this ") + et.singular + ".")
          .request_body("Lock extension", SB::ref("ExtendLockRequest"))
          .header_param("X-Client-Id", "Unique client identifier for lock ownership")
          .response(204, "Lock extended")
          .operation_id(std::string("extend") + capitalize(et.singular) + "Lock");

      reg.del(entity_path + "/locks/{lock_id}",
              [this](auto & req, auto & res) {
                lock_handlers_->handle_release_lock(req, res);
              })
          .tag("Locking")
          .summary(std::string("Release lock on ") + et.singular)
          .description(std::string("Releases a lock on this ") + et.singular + ".")
          .header_param("X-Client-Id", "Unique client identifier for lock ownership")
          .response(204, "Lock released")
          .operation_id(std::string("release") + capitalize(et.singular) + "Lock");
    }

    // --- Scripts (apps and components only) ---
    if (script_handlers_ && (et_type_str == "apps" || et_type_str == "components")) {
      reg.post(entity_path + "/scripts",
               [this](auto & req, auto & res) {
                 script_handlers_->handle_upload_script(req, res);
               })
          .tag("Scripts")
          .summary(std::string("Upload diagnostic script for ") + et.singular)
          .description(std::string("Uploads a diagnostic script for this ") + et.singular + ".")
          .request_body("Script file", SB::binary_schema(), "multipart/form-data")
          .response(201, "Script uploaded", SB::ref("ScriptUploadResponse"))
          .operation_id(std::string("upload") + capitalize(et.singular) + "Script");

      reg.get(entity_path + "/scripts",
              [this](auto & req, auto & res) {
                script_handlers_->handle_list_scripts(req, res);
              })
          .tag("Scripts")
          .summary(std::string("List scripts for ") + et.singular)
          .description(std::string("Lists all diagnostic scripts for this ") + et.singular + ".")
          .response(200, "Script list", SB::ref("ScriptMetadataList"))
          .operation_id(std::string("list") + capitalize(et.singular) + "Scripts");

      reg.get(entity_path + "/scripts/{script_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_get_script(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Get script metadata for ") + et.singular)
          .description(std::string("Returns metadata of a specific script for this ") + et.singular + ".")
          .response(200, "Script metadata", SB::ref("ScriptMetadata"))
          .operation_id(std::string("get") + capitalize(et.singular) + "Script");

      reg.del(entity_path + "/scripts/{script_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_delete_script(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Delete script for ") + et.singular)
          .description(std::string("Deletes a diagnostic script from this ") + et.singular + ".")
          .response(204, "Script deleted")
          .operation_id(std::string("delete") + capitalize(et.singular) + "Script");

      reg.post(entity_path + "/scripts/{script_id}/executions",
               [this](auto & req, auto & res) {
                 script_handlers_->handle_start_execution(req, res);
               })
          .tag("Scripts")
          .summary(std::string("Start script execution for ") + et.singular)
          .description(std::string("Starts execution of a diagnostic script on this ") + et.singular + ".")
          .request_body("Execution parameters", SB::generic_object_schema())
          .response(202, "Execution started", SB::ref("ScriptExecution"))
          .operation_id(std::string("start") + capitalize(et.singular) + "ScriptExecution");

      reg.get(entity_path + "/scripts/{script_id}/executions/{execution_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_get_execution(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Get execution status for ") + et.singular)
          .description("Returns the current status of a script execution.")
          .response(200, "Execution status", SB::ref("ScriptExecution"))
          .operation_id(std::string("get") + capitalize(et.singular) + "ScriptExecution");

      reg.put(entity_path + "/scripts/{script_id}/executions/{execution_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_control_execution(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Terminate script execution for ") + et.singular)
          .description("Sends a control command (e.g., terminate) to a running script execution.")
          .request_body("Execution control", SB::ref("ScriptControlRequest"))
          .response(200, "Execution updated", SB::ref("ScriptExecution"))
          .operation_id(std::string("control") + capitalize(et.singular) + "ScriptExecution");

      reg.del(entity_path + "/scripts/{script_id}/executions/{execution_id}",
              [this](auto & req, auto & res) {
                script_handlers_->handle_delete_execution(req, res);
              })
          .tag("Scripts")
          .summary(std::string("Remove completed execution for ") + et.singular)
          .description("Removes a completed script execution record.")
          .response(204, "Execution removed")
          .operation_id(std::string("remove") + capitalize(et.singular) + "ScriptExecution");
    }

    // --- Discovery relationship endpoints (entity-type-specific) ---
    if (et_type_str == "areas") {
      reg.get(entity_path + "/components",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_area_components(req, res);
              })
          .tag("Discovery")
          .summary("List components in area")
          .description("Lists components belonging to this area.")
          .response(200, "Component list", SB::ref("EntityList"))
          .operation_id("listAreaComponents");

      reg.get(entity_path + "/subareas",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_subareas(req, res);
              })
          .tag("Discovery")
          .summary("List subareas")
          .description("Lists subareas within this area.")
          .response(200, "Subarea list", SB::ref("EntityList"))
          .operation_id("listSubareas");

      reg.get(entity_path + "/contains",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_contains(req, res);
              })
          .tag("Discovery")
          .summary("List entities contained in area")
          .description("Lists all entities contained in this area.")
          .response(200, "Contained entities", SB::ref("EntityList"))
          .operation_id("listAreaContains");
    }

    if (et_type_str == "components") {
      reg.get(entity_path + "/subcomponents",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_subcomponents(req, res);
              })
          .tag("Discovery")
          .summary("List subcomponents")
          .description("Lists subcomponents of this component.")
          .response(200, "Subcomponent list", SB::ref("EntityList"))
          .operation_id("listSubcomponents");

      reg.get(entity_path + "/hosts",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_get_hosts(req, res);
              })
          .tag("Discovery")
          .summary("List component hosts")
          .description("Lists apps hosted by this component.")
          .response(200, "Host list", SB::ref("EntityList"))
          .operation_id("listComponentHosts");

      reg.get(entity_path + "/depends-on",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_component_depends_on(req, res);
              })
          .tag("Discovery")
          .summary("List component dependencies")
          .description("Lists components this component depends on.")
          .response(200, "Dependency list", SB::ref("EntityList"))
          .operation_id("listComponentDependencies");
    }

    if (et_type_str == "apps") {
      reg.get(entity_path + "/is-located-on",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_app_is_located_on(req, res);
              })
          .tag("Discovery")
          .summary("Get app host component")
          .description("Returns the component hosting this app.")
          .response(200, "Host component", SB::ref("EntityDetail"))
          .operation_id("getAppHost");

      reg.get(entity_path + "/depends-on",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_app_depends_on(req, res);
              })
          .tag("Discovery")
          .summary("List app dependencies")
          .description("Lists apps this app depends on.")
          .response(200, "Dependency list", SB::ref("EntityList"))
          .operation_id("listAppDependencies");
    }

    if (et_type_str == "functions") {
      reg.get(entity_path + "/hosts",
              [this](auto & req, auto & res) {
                discovery_handlers_->handle_function_hosts(req, res);
              })
          .tag("Discovery")
          .summary("List function hosts")
          .description("Lists components hosting this function.")
          .response(200, "Host list", SB::ref("EntityList"))
          .operation_id("listFunctionHosts");
    }

    // Single entity detail (capabilities) - must be LAST for this entity type
    reg.get(entity_path, et.detail_handler)
        .tag("Discovery")
        .summary(std::string("Get ") + et.singular + " details")
        .description(std::string("Returns ") + et.singular + " details with capabilities and resource collection URIs.")
        .response(200, "Entity details with capabilities", SB::ref("EntityDetail"))
        .operation_id(std::string("get") + capitalize(et.singular));
  }

  // === Nested entities - subareas bulk-data ===
  reg.get("/areas/{area_id}/subareas/{subarea_id}/bulk-data",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_categories(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data categories for subarea")
      .description("Lists bulk-data categories for a subarea.")
      .response(200, "Category list", SB::ref("BulkDataCategoryList"))
      .operation_id("listSubareaBulkDataCategories");

  reg.get("/areas/{area_id}/subareas/{subarea_id}/bulk-data/{category_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_descriptors(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data descriptors for subarea")
      .description("Lists bulk-data descriptors for a subarea.")
      .response(200, "Descriptor list", SB::ref("BulkDataDescriptorList"))
      .operation_id("listSubareaBulkDataDescriptors");

  reg.get("/areas/{area_id}/subareas/{subarea_id}/bulk-data/{category_id}/{file_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_download(req, res);
          })
      .tag("Bulk Data")
      .summary("Download bulk-data file for subarea")
      .description("Downloads a bulk-data file for a subarea.")
      .response(200, "File content", SB::binary_schema())
      .operation_id("downloadSubareaBulkData");

  // === Nested entities - subcomponents bulk-data ===
  reg.get("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_categories(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data categories for subcomponent")
      .description("Lists bulk-data categories for a subcomponent.")
      .response(200, "Category list", SB::ref("BulkDataCategoryList"))
      .operation_id("listSubcomponentBulkDataCategories");

  reg.get("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data/{category_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_list_descriptors(req, res);
          })
      .tag("Bulk Data")
      .summary("List bulk-data descriptors for subcomponent")
      .description("Lists bulk-data descriptors for a subcomponent.")
      .response(200, "Descriptor list", SB::ref("BulkDataDescriptorList"))
      .operation_id("listSubcomponentBulkDataDescriptors");

  reg.get("/components/{component_id}/subcomponents/{subcomponent_id}/bulk-data/{category_id}/{file_id}",
          [this](auto & req, auto & res) {
            bulkdata_handlers_->handle_download(req, res);
          })
      .tag("Bulk Data")
      .summary("Download bulk-data file for subcomponent")
      .description("Downloads a bulk-data file for a subcomponent.")
      .response(200, "File content", SB::binary_schema())
      .operation_id("downloadSubcomponentBulkData");

  // === Global faults ===
  // SSE stream - must be before /faults to avoid regex conflict
  reg.get("/faults/stream",
          [this](auto & req, auto & res) {
            sse_fault_handler_->handle_stream(req, res);
          })
      .tag("Faults")
      .summary("Stream fault events (SSE)")
      .description("Server-Sent Events stream for real-time fault notifications.")
      .operation_id("streamFaults");

  reg.get("/faults",
          [this](auto & req, auto & res) {
            fault_handlers_->handle_list_all_faults(req, res);
          })
      .tag("Faults")
      .summary("List all faults globally")
      .description("Retrieve all faults across the system.")
      .response(200, "All faults", SB::ref("FaultList"))
      .operation_id("listAllFaults");

  reg.del("/faults",
          [this](auto & req, auto & res) {
            fault_handlers_->handle_clear_all_faults_global(req, res);
          })
      .tag("Faults")
      .summary("Clear all faults globally")
      .description("Clears all faults across the entire system.")
      .response(204, "All faults cleared")
      .operation_id("clearAllFaults");

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
      .summary("List software updates")
      .description("Lists all registered software updates.")
      .response(200, "Update list", SB::ref("UpdateList"))
      .operation_id("listUpdates");

  reg.post("/updates", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
             update_handlers_->handle_register_update(req, res);
           })
                                        : HandlerFn(update_501))
      .tag("Updates")
      .summary("Register a software update")
      .description("Registers a new software update descriptor.")
      .request_body("Update descriptor", SB::generic_object_schema())
      .response(201, "Update registered", SB::generic_object_schema())
      .operation_id("registerUpdate");

  reg.get("/updates/{update_id}/status", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_get_status(req, res);
          })
                                                          : HandlerFn(update_501))
      .tag("Updates")
      .summary("Get update status")
      .description("Returns the current status and progress of an update.")
      .response(200, "Update status", SB::ref("UpdateStatus"))
      .operation_id("getUpdateStatus");

  reg.put("/updates/{update_id}/prepare", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_prepare(req, res);
          })
                                                           : HandlerFn(update_501))
      .tag("Updates")
      .summary("Prepare update for execution")
      .description("Prepares an update for execution (downloads, validates).")
      .request_body("Prepare parameters", SB::generic_object_schema())
      .response(202, "Update preparation started")
      .operation_id("prepareUpdate");

  reg.put("/updates/{update_id}/execute", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_execute(req, res);
          })
                                                           : HandlerFn(update_501))
      .tag("Updates")
      .summary("Execute update")
      .description("Starts executing a prepared update.")
      .request_body("Execute parameters", SB::generic_object_schema())
      .response(202, "Update execution started")
      .operation_id("executeUpdate");

  reg.put("/updates/{update_id}/automated", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_automated(req, res);
          })
                                                             : HandlerFn(update_501))
      .tag("Updates")
      .summary("Run automated update")
      .description("Runs a fully automated update (prepare + execute).")
      .request_body("Automated parameters", SB::generic_object_schema())
      .response(202, "Automated update started")
      .operation_id("automateUpdate");

  reg.get("/updates/{update_id}", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_get_update(req, res);
          })
                                                   : HandlerFn(update_501))
      .tag("Updates")
      .summary("Get update details")
      .description("Returns details of a specific update.")
      .response(200, "Update details", SB::generic_object_schema())
      .operation_id("getUpdate");

  reg.del("/updates/{update_id}", update_handlers_ ? HandlerFn([this](auto & req, auto & res) {
            update_handlers_->handle_delete_update(req, res);
          })
                                                   : HandlerFn(update_501))
      .tag("Updates")
      .summary("Delete update")
      .description("Removes an update registration.")
      .response(204, "Update deleted")
      .operation_id("deleteUpdate");

  // === Authentication ===
  reg.post("/auth/authorize",
           [this](auto & req, auto & res) {
             auth_handlers_->handle_auth_authorize(req, res);
           })
      .tag("Authentication")
      .summary("Authorize client")
      .description("Authenticate and obtain authorization tokens.")
      .request_body("Client credentials", SB::ref("AuthCredentials"))
      .response(200, "Authorization tokens", SB::ref("AuthTokenResponse"))
      .operation_id("authorize");

  reg.post("/auth/token",
           [this](auto & req, auto & res) {
             auth_handlers_->handle_auth_token(req, res);
           })
      .tag("Authentication")
      .summary("Obtain access token")
      .description("Exchange credentials or refresh token for a JWT access token.")
      .request_body("Token request credentials", SB::ref("AuthCredentials"))
      .response(200, "Access token", SB::ref("AuthTokenResponse"))
      .operation_id("getToken");

  reg.post("/auth/revoke",
           [this](auto & req, auto & res) {
             auth_handlers_->handle_auth_revoke(req, res);
           })
      .tag("Authentication")
      .summary("Revoke token")
      .description("Revoke an access or refresh token.")
      .request_body("Token to revoke", SB::generic_object_schema())
      .response(200, "Token revoked", SB::generic_object_schema())
      .operation_id("revokeToken");

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
