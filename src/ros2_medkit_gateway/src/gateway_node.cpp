// Copyright 2025 bburda, mfaferek93
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

#include "ros2_medkit_gateway/gateway_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <unordered_set>

#include "ros2_medkit_gateway/http/handlers/sse_transport_provider.hpp"

using namespace std::chrono_literals;

namespace ros2_medkit_gateway {

namespace {

/// Convert an rclcpp::Parameter to a nlohmann::json value
nlohmann::json parameter_to_json(const rclcpp::Parameter & param) {
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return param.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return param.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return param.as_string();
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return param.as_byte_array();
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return param.as_bool_array();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return param.as_integer_array();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return param.as_double_array();
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return param.as_string_array();
    default:
      return nullptr;
  }
}

/// Extract per-plugin config from YAML parameter overrides.
/// Scans for keys matching "plugins.<name>.<key>" (excluding ".path")
/// and builds a flat JSON object: {"<key>": value, ...}
nlohmann::json extract_plugin_config(const std::vector<rclcpp::Parameter> & overrides,
                                     const std::string & plugin_name) {
  auto config = nlohmann::json::object();
  std::string prefix = "plugins." + plugin_name + ".";
  std::string path_key = prefix + "path";

  for (const auto & param : overrides) {
    const auto & name = param.get_name();
    if (name.rfind(prefix, 0) == 0 && name != path_key) {
      auto key = name.substr(prefix.size());
      config[key] = parameter_to_json(param);
    }
  }
  return config;
}

}  // namespace

GatewayNode::GatewayNode() : Node("ros2_medkit_gateway") {
  RCLCPP_INFO(get_logger(), "Initializing ROS 2 Medkit Gateway...");

  // Declare parameters with defaults
  declare_parameter("server.host", "127.0.0.1");
  declare_parameter("server.port", 8080);
  declare_parameter("refresh_interval_ms", 10000);
  declare_parameter("cors.allowed_origins", std::vector<std::string>{});
  declare_parameter("cors.allowed_methods", std::vector<std::string>{"GET", "PUT", "POST", "DELETE", "OPTIONS"});
  declare_parameter("cors.allowed_headers", std::vector<std::string>{"Content-Type", "Accept"});
  declare_parameter("cors.allow_credentials", false);
  declare_parameter("cors.max_age_seconds", 86400);

  // SSE (Server-Sent Events) parameters
  declare_parameter("sse.max_clients", 10);         // Limit concurrent SSE connections to prevent resource exhaustion
  declare_parameter("sse.max_subscriptions", 100);  // Maximum active cyclic subscriptions across all entities
  declare_parameter("sse.max_duration_sec", 3600);  // Maximum subscription duration in seconds (1 hour default)

  // Log management parameters
  declare_parameter("logs.buffer_size",
                    200);  // Ring buffer capacity per node; entries exceeding this are dropped (oldest first)

  // TLS/HTTPS parameters
  declare_parameter("server.tls.enabled", false);
  declare_parameter("server.tls.cert_file", "");
  declare_parameter("server.tls.key_file", "");
  declare_parameter("server.tls.ca_file", "");
  declare_parameter("server.tls.min_version", "1.2");
  // TODO(future): Add mutual_tls parameter when implemented
  // declare_parameter("server.tls.mutual_tls", false);

  // Authentication parameters (REQ_INTEROP_086, REQ_INTEROP_087)
  declare_parameter("auth.enabled", false);
  declare_parameter("auth.jwt_secret", "");
  declare_parameter("auth.jwt_public_key", "");
  declare_parameter("auth.jwt_algorithm", "HS256");
  declare_parameter("auth.token_expiry_seconds", 3600);
  declare_parameter("auth.refresh_token_expiry_seconds", 86400);
  declare_parameter("auth.require_auth_for", "write");
  declare_parameter("auth.issuer", "ros2_medkit_gateway");
  declare_parameter("auth.clients", std::vector<std::string>{});

  // Rate limiting parameters
  declare_parameter("rate_limiting.enabled", false);
  declare_parameter("rate_limiting.global_requests_per_minute", 600);
  declare_parameter("rate_limiting.client_requests_per_minute", 60);
  declare_parameter("rate_limiting.endpoint_limits", std::vector<std::string>{});
  declare_parameter("rate_limiting.client_cleanup_interval_seconds", 300);
  declare_parameter("rate_limiting.client_max_idle_seconds", 600);

  // Discovery mode parameters
  declare_parameter("discovery.mode", "runtime_only");  // runtime_only, manifest_only, hybrid
  declare_parameter("discovery.manifest_path", "");
  declare_parameter("discovery.manifest_strict_validation", true);

  // Software updates parameters
  declare_parameter("updates.enabled", false);

  // Plugin framework parameters
  declare_parameter("plugins", std::vector<std::string>{});

  // Bulk data storage parameters
  declare_parameter("bulk_data.storage_dir", "/tmp/ros2_medkit_bulk_data");
  declare_parameter("bulk_data.max_upload_size", 104857600);  // 100MB
  declare_parameter("bulk_data.categories", std::vector<std::string>{});

  // Runtime (heuristic) discovery options
  // These control how nodes are mapped to SOVD entities in runtime mode
  declare_parameter("discovery.runtime.create_synthetic_components", true);
  declare_parameter("discovery.runtime.grouping_strategy", "namespace");
  declare_parameter("discovery.runtime.synthetic_component_name_pattern", "{area}");
  declare_parameter("discovery.runtime.topic_only_policy", "create_component");
  declare_parameter("discovery.runtime.min_topics_for_component", 1);

  // Merge pipeline configuration (hybrid mode only)
  declare_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_areas", true);
  declare_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_components", true);
  declare_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_apps", true);
  declare_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_functions", false);
  declare_parameter("discovery.merge_pipeline.gap_fill.namespace_whitelist", std::vector<std::string>{});
  declare_parameter("discovery.merge_pipeline.gap_fill.namespace_blacklist", std::vector<std::string>{});

  // Per-layer merge policy overrides (optional, empty string = use layer default)
  for (const auto & layer : {"manifest", "runtime"}) {
    for (const auto & fg : {"identity", "hierarchy", "live_data", "status", "metadata"}) {
      declare_parameter(std::string("discovery.merge_pipeline.layers.") + layer + "." + fg, std::string(""));
    }
  }

  // Get parameter values
  server_host_ = get_parameter("server.host").as_string();
  server_port_ = static_cast<int>(get_parameter("server.port").as_int());
  refresh_interval_ms_ = static_cast<int>(get_parameter("refresh_interval_ms").as_int());

  // Build CORS configuration using builder pattern
  // Throws std::invalid_argument if configuration is invalid
  cors_config_ = CorsConfigBuilder()
                     .with_origins(get_parameter("cors.allowed_origins").as_string_array())
                     .with_methods(get_parameter("cors.allowed_methods").as_string_array())
                     .with_headers(get_parameter("cors.allowed_headers").as_string_array())
                     .with_credentials(get_parameter("cors.allow_credentials").as_bool())
                     .with_max_age(static_cast<int>(get_parameter("cors.max_age_seconds").as_int()))
                     .build();

  // Validate port range
  if (server_port_ < 1024 || server_port_ > 65535) {
    RCLCPP_ERROR(get_logger(), "Invalid port %d. Must be between 1024-65535. Using default 8080.", server_port_);
    server_port_ = 8080;
  }

  // Validate host
  if (server_host_.empty()) {
    RCLCPP_WARN(get_logger(), "Empty host specified. Using default 127.0.0.1");
    server_host_ = "127.0.0.1";
  }

  // Warn if binding to all interfaces
  if (server_host_ == "0.0.0.0") {
    RCLCPP_WARN(get_logger(), "Binding to 0.0.0.0 - REST API accessible from ALL network interfaces!");
  }

  // Validate refresh interval
  if (refresh_interval_ms_ < 100 || refresh_interval_ms_ > 60000) {
    RCLCPP_WARN(get_logger(), "Invalid refresh interval %dms. Must be between 100-60000ms. Using default 10000ms.",
                refresh_interval_ms_);
    refresh_interval_ms_ = 10000;
  }

  // Log configuration
  RCLCPP_INFO(get_logger(), "Configuration: REST API at %s:%d, refresh interval: %dms", server_host_.c_str(),
              server_port_, refresh_interval_ms_);

  if (cors_config_.enabled) {
    std::string origins_str;
    for (const auto & origin : cors_config_.allowed_origins) {
      if (!origins_str.empty()) {
        origins_str += ", ";
      }
      origins_str += origin;
    }

    std::string methods_str;
    for (const auto & method : cors_config_.allowed_methods) {
      if (!methods_str.empty()) {
        methods_str += ", ";
      }
      methods_str += method;
    }

    RCLCPP_INFO(get_logger(), "CORS enabled - origins: [%s], methods: [%s], credentials: %s, max_age: %ds",
                origins_str.c_str(), methods_str.c_str(), cors_config_.allow_credentials ? "true" : "false",
                cors_config_.max_age_seconds);
  } else {
    RCLCPP_INFO(get_logger(), "CORS: disabled (no configuration provided)");
  }

  // Build TLS/HTTPS configuration
  bool tls_enabled = get_parameter("server.tls.enabled").as_bool();
  if (tls_enabled) {
    try {
      tls_config_ = TlsConfigBuilder()
                        .with_enabled(true)
                        .with_cert_file(get_parameter("server.tls.cert_file").as_string())
                        .with_key_file(get_parameter("server.tls.key_file").as_string())
                        .with_ca_file(get_parameter("server.tls.ca_file").as_string())
                        .with_min_version(get_parameter("server.tls.min_version").as_string())
                        // TODO(future): Add .with_mutual_tls() when implemented
                        .build();
      // Note: HttpServerManager will log TLS configuration details
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Invalid TLS configuration: %s. TLS disabled.", e.what());
      tls_config_ = TlsConfig{};  // Disabled
    }
  } else {
    RCLCPP_INFO(get_logger(), "TLS/HTTPS: disabled");
    tls_config_ = TlsConfig{};
  }

  // Build Authentication configuration (REQ_INTEROP_086, REQ_INTEROP_087)
  bool auth_enabled = get_parameter("auth.enabled").as_bool();
  if (auth_enabled) {
    try {
      AuthConfigBuilder auth_builder;
      auth_builder.with_enabled(true)
          .with_jwt_secret(get_parameter("auth.jwt_secret").as_string())
          .with_jwt_public_key(get_parameter("auth.jwt_public_key").as_string())
          .with_algorithm(string_to_algorithm(get_parameter("auth.jwt_algorithm").as_string()))
          .with_token_expiry(static_cast<int>(get_parameter("auth.token_expiry_seconds").as_int()))
          .with_refresh_token_expiry(static_cast<int>(get_parameter("auth.refresh_token_expiry_seconds").as_int()))
          .with_require_auth_for(string_to_auth_requirement(get_parameter("auth.require_auth_for").as_string()))
          .with_issuer(get_parameter("auth.issuer").as_string());

      // Parse clients from configuration
      // Format: "client_id:client_secret:role" (e.g., "admin:secret123:admin")
      auto clients = get_parameter("auth.clients").as_string_array();
      for (const auto & client_str : clients) {
        if (client_str.empty()) {
          continue;
        }
        // Parse "client_id:client_secret:role"
        size_t first_colon = client_str.find(':');
        size_t last_colon = client_str.rfind(':');
        if (first_colon != std::string::npos && last_colon != std::string::npos && first_colon != last_colon) {
          std::string client_id = client_str.substr(0, first_colon);
          std::string client_secret = client_str.substr(first_colon + 1, last_colon - first_colon - 1);
          std::string role_str = client_str.substr(last_colon + 1);
          try {
            UserRole role = string_to_role(role_str);
            auth_builder.add_client(client_id, client_secret, role);
            RCLCPP_INFO(get_logger(), "Registered client '%s' with role '%s'", client_id.c_str(), role_str.c_str());
          } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Invalid role '%s' for client '%s': %s", role_str.c_str(), client_id.c_str(),
                        e.what());
          }
        } else {
          RCLCPP_WARN(get_logger(), "Invalid client format: '%s'. Expected 'client_id:client_secret:role'",
                      client_str.c_str());
        }
      }

      auth_config_ = auth_builder.build();
      RCLCPP_INFO(get_logger(), "Authentication enabled - algorithm: %s, require_auth_for: %s",
                  algorithm_to_string(auth_config_.jwt_algorithm).c_str(),
                  get_parameter("auth.require_auth_for").as_string().c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Invalid authentication configuration: %s. Authentication disabled.", e.what());
      auth_config_ = AuthConfig{};  // Disabled
    }
  } else {
    RCLCPP_INFO(get_logger(), "Authentication: disabled");
    auth_config_ = AuthConfig{};
  }

  // Build Rate Limiting configuration
  bool rate_limit_enabled = get_parameter("rate_limiting.enabled").as_bool();
  if (rate_limit_enabled) {
    try {
      RateLimitConfigBuilder rl_builder;
      rl_builder.with_enabled(true)
          .with_global_rpm(static_cast<int>(get_parameter("rate_limiting.global_requests_per_minute").as_int()))
          .with_client_rpm(static_cast<int>(get_parameter("rate_limiting.client_requests_per_minute").as_int()))
          .with_cleanup_interval(
              static_cast<int>(get_parameter("rate_limiting.client_cleanup_interval_seconds").as_int()))
          .with_max_idle(static_cast<int>(get_parameter("rate_limiting.client_max_idle_seconds").as_int()));

      // Parse endpoint limits from "pattern:rpm" format
      auto endpoint_strs = get_parameter("rate_limiting.endpoint_limits").as_string_array();
      for (const auto & entry : endpoint_strs) {
        if (entry.empty()) {
          continue;
        }
        size_t colon = entry.rfind(':');
        if (colon != std::string::npos && colon > 0) {
          std::string pattern = entry.substr(0, colon);
          try {
            int rpm = std::stoi(entry.substr(colon + 1));
            rl_builder.add_endpoint_limit(pattern, rpm);
          } catch (...) {
            RCLCPP_WARN(get_logger(), "Non-numeric RPM in endpoint_limit: '%s'", entry.c_str());
            continue;
          }
        } else {
          RCLCPP_WARN(get_logger(), "Invalid endpoint_limit format: '%s'. Expected 'pattern:rpm'", entry.c_str());
        }
      }

      rate_limit_config_ = rl_builder.build();
      RCLCPP_INFO(get_logger(), "Rate limiting enabled - global: %d rpm, per-client: %d rpm",
                  rate_limit_config_.global_requests_per_minute, rate_limit_config_.client_requests_per_minute);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Invalid rate limiting configuration: %s. Rate limiting disabled.", e.what());
      rate_limit_config_ = RateLimitConfig{};
    }
  } else {
    RCLCPP_INFO(get_logger(), "Rate limiting: disabled");
    rate_limit_config_ = RateLimitConfig{};
  }

  // Initialize managers
  discovery_mgr_ = std::make_unique<DiscoveryManager>(this);

  // Configure and initialize discovery manager
  DiscoveryConfig discovery_config;

  auto mode_str = get_parameter("discovery.mode").as_string();
  discovery_config.mode = parse_discovery_mode(mode_str);
  if (mode_str != "runtime_only" && mode_str != "manifest_only" && mode_str != "hybrid") {
    RCLCPP_WARN(get_logger(), "Unknown discovery.mode '%s', defaulting to 'runtime_only'", mode_str.c_str());
  }

  discovery_config.manifest_path = get_parameter("discovery.manifest_path").as_string();
  discovery_config.manifest_strict_validation = get_parameter("discovery.manifest_strict_validation").as_bool();

  // Runtime discovery options
  discovery_config.runtime.create_synthetic_components =
      get_parameter("discovery.runtime.create_synthetic_components").as_bool();

  auto grouping_str = get_parameter("discovery.runtime.grouping_strategy").as_string();
  discovery_config.runtime.grouping = parse_grouping_strategy(grouping_str);
  if (grouping_str != "none" && grouping_str != "namespace") {
    RCLCPP_WARN(get_logger(), "Unknown grouping_strategy '%s', defaulting to 'none'", grouping_str.c_str());
  }

  discovery_config.runtime.synthetic_component_name_pattern =
      get_parameter("discovery.runtime.synthetic_component_name_pattern").as_string();

  auto topic_policy_str = get_parameter("discovery.runtime.topic_only_policy").as_string();
  discovery_config.runtime.topic_only_policy = parse_topic_only_policy(topic_policy_str);
  if (topic_policy_str != "ignore" && topic_policy_str != "create_component" &&
      topic_policy_str != "create_area_only") {
    RCLCPP_WARN(get_logger(), "Unknown topic_only_policy '%s', defaulting to 'create_component'",
                topic_policy_str.c_str());
  }
  discovery_config.runtime.min_topics_for_component =
      static_cast<int>(get_parameter("discovery.runtime.min_topics_for_component").as_int());

  // Merge pipeline gap-fill configuration (hybrid mode)
  discovery_config.merge_pipeline.gap_fill.allow_heuristic_areas =
      get_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_areas").as_bool();
  discovery_config.merge_pipeline.gap_fill.allow_heuristic_components =
      get_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_components").as_bool();
  discovery_config.merge_pipeline.gap_fill.allow_heuristic_apps =
      get_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_apps").as_bool();
  discovery_config.merge_pipeline.gap_fill.allow_heuristic_functions =
      get_parameter("discovery.merge_pipeline.gap_fill.allow_heuristic_functions").as_bool();
  discovery_config.merge_pipeline.gap_fill.namespace_whitelist =
      get_parameter("discovery.merge_pipeline.gap_fill.namespace_whitelist").as_string_array();
  discovery_config.merge_pipeline.gap_fill.namespace_blacklist =
      get_parameter("discovery.merge_pipeline.gap_fill.namespace_blacklist").as_string_array();

  // Read per-layer merge policy overrides
  for (const auto & layer : {"manifest", "runtime"}) {
    for (const auto & fg : {"identity", "hierarchy", "live_data", "status", "metadata"}) {
      auto val = get_parameter(std::string("discovery.merge_pipeline.layers.") + layer + "." + fg).as_string();
      if (!val.empty()) {
        discovery_config.merge_pipeline.layer_policies[layer][fg] = val;
      }
    }
  }

  if (!discovery_mgr_->initialize(discovery_config)) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize discovery manager");
    throw std::runtime_error("Discovery initialization failed");
  }

  data_access_mgr_ = std::make_unique<DataAccessManager>(this);
  operation_mgr_ = std::make_unique<OperationManager>(this, discovery_mgr_.get());
  config_mgr_ = std::make_unique<ConfigurationManager>(this);
  fault_mgr_ = std::make_unique<FaultManager>(this);

  // Initialize bulk data store
  auto bd_storage_dir = get_parameter("bulk_data.storage_dir").as_string();
  auto bd_max_upload = static_cast<size_t>(get_parameter("bulk_data.max_upload_size").as_int());
  auto bd_categories = get_parameter("bulk_data.categories").as_string_array();
  bd_categories.erase(std::remove_if(bd_categories.begin(), bd_categories.end(),
                                     [](const auto & item) {
                                       return item.empty();
                                     }),
                      bd_categories.end());

  bulk_data_store_ = std::make_unique<BulkDataStore>(bd_storage_dir, bd_max_upload, bd_categories);
  RCLCPP_INFO(get_logger(), "Bulk data store: dir=%s, max_upload=%zuB, categories=%zu", bd_storage_dir.c_str(),
              bd_max_upload, bd_categories.size());

  // Initialize subscription manager for cyclic subscriptions
  auto max_subscriptions = static_cast<size_t>(get_parameter("sse.max_subscriptions").as_int());
  subscription_mgr_ = std::make_unique<SubscriptionManager>(max_subscriptions);
  RCLCPP_INFO(get_logger(), "Subscription manager: max_subscriptions=%zu", max_subscriptions);

  // Create SSE client tracker (shared between SseTransportProvider and SSEFaultHandler)
  auto max_sse_clients = static_cast<size_t>(get_parameter("sse.max_clients").as_int());
  sse_client_tracker_ = std::make_shared<SSEClientTracker>(max_sse_clients);

  // Initialize resource sampler and transport registries
  sampler_registry_ = std::make_unique<ResourceSamplerRegistry>();
  transport_registry_ = std::make_unique<TransportRegistry>();

  // Initialize plugin manager
  plugin_mgr_ = std::make_unique<PluginManager>();
  plugin_mgr_->set_registries(*sampler_registry_, *transport_registry_);
  auto plugin_names = get_parameter("plugins").as_string_array();
  plugin_names.erase(std::remove_if(plugin_names.begin(), plugin_names.end(),
                                    [](const auto & item) {
                                      return item.empty();
                                    }),
                     plugin_names.end());
  if (!plugin_names.empty()) {
    std::vector<PluginConfig> configs;
    // Plugin name validation: alphanumeric, underscore, hyphen only (max 256 chars)
    auto is_valid_plugin_name = [](const std::string & name) -> bool {
      if (name.empty() || name.size() > 256) {
        return false;
      }
      return std::all_of(name.begin(), name.end(), [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
      });
    };
    std::unordered_set<std::string> seen_names;
    for (const auto & pname : plugin_names) {
      if (!seen_names.insert(pname).second) {
        RCLCPP_WARN(get_logger(), "Duplicate plugin name '%s' - skipping", pname.c_str());
        continue;
      }
      if (!is_valid_plugin_name(pname)) {
        RCLCPP_ERROR(get_logger(),
                     "Invalid plugin name '%s': must be alphanumeric, underscore, or hyphen (max 256 chars)",
                     pname.c_str());
        continue;
      }
      auto path_param = "plugins." + pname + ".path";
      declare_parameter(path_param, std::string(""));
      auto path = get_parameter(path_param).as_string();
      if (path.empty()) {
        RCLCPP_ERROR(get_logger(), "Plugin '%s' has no path configured", pname.c_str());
        continue;
      }
      auto plugin_config = extract_plugin_config(get_node_options().parameter_overrides(), pname);
      if (!plugin_config.empty()) {
        RCLCPP_INFO(get_logger(), "Plugin '%s' config: %zu key(s)", pname.c_str(), plugin_config.size());
      }
      configs.push_back({pname, path, std::move(plugin_config)});
    }
    auto loaded = plugin_mgr_->load_plugins(configs);
    plugin_mgr_->configure_plugins();
    plugin_ctx_ = make_gateway_plugin_context(this, fault_mgr_.get());
    plugin_mgr_->set_context(*plugin_ctx_);
    RCLCPP_INFO(get_logger(), "Loaded %zu plugin(s)", loaded);

    // Register IntrospectionProvider plugins as pipeline layers (hybrid mode only)
    if (discovery_mgr_->get_mode() == DiscoveryMode::HYBRID) {
      auto providers = plugin_mgr_->get_named_introspection_providers();
      for (auto & [name, provider] : providers) {
        discovery_mgr_->add_plugin_layer(name, provider);
      }
      if (!providers.empty()) {
        discovery_mgr_->refresh_pipeline();
      }
    }
  }

  // Initialize log manager (subscribes to /rosout, delegates to plugin if available)
  static constexpr int kMinBufferSize = 1;
  static constexpr int kMaxBufferSize = 100000;
  auto raw_buffer_size = get_parameter("logs.buffer_size").as_int();
  auto clamped =
      std::clamp(raw_buffer_size, static_cast<int64_t>(kMinBufferSize), static_cast<int64_t>(kMaxBufferSize));
  if (clamped != raw_buffer_size) {
    RCLCPP_WARN(get_logger(), "logs.buffer_size %" PRId64 " clamped to %" PRId64, raw_buffer_size, clamped);
  }
  auto log_buffer_size = static_cast<size_t>(clamped);
  log_mgr_ = std::make_unique<LogManager>(this, plugin_mgr_.get(), log_buffer_size);

  // Initialize update manager
  auto updates_enabled = get_parameter("updates.enabled").as_bool();
  if (updates_enabled) {
    update_mgr_ = std::make_unique<UpdateManager>();
    auto * update_provider = plugin_mgr_->get_update_provider();
    if (update_provider) {
      update_mgr_->set_backend(update_provider);
      RCLCPP_INFO(get_logger(), "Update backend provided by plugin");
    } else {
      RCLCPP_WARN(get_logger(), "Updates enabled but no UpdateProvider plugin loaded");
    }
  }

  // Register built-in resource samplers
  sampler_registry_->register_sampler(
      "data",
      [this](const std::string & /*entity_id*/,
             const std::string & resource_path) -> tl::expected<nlohmann::json, std::string> {
        auto * dam = get_data_access_manager();
        if (!dam) {
          return tl::make_unexpected(std::string("DataAccessManager not available"));
        }
        auto * native_sampler = dam->get_native_sampler();
        if (!native_sampler) {
          return tl::make_unexpected(std::string("Native topic sampler unavailable"));
        }
        auto sample = native_sampler->sample_topic(resource_path, dam->get_topic_sample_timeout());
        if (sample.has_data && sample.data.has_value()) {
          nlohmann::json payload;
          payload["id"] = resource_path;
          payload["data"] = *sample.data;
          return payload;
        }
        return tl::make_unexpected(std::string("Topic data not available: " + resource_path));
      },
      true);

  sampler_registry_->register_sampler(
      "faults",
      [this](const std::string & entity_id,
             const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
        auto * fault_mgr = get_fault_manager();
        if (!fault_mgr) {
          return tl::make_unexpected(std::string("FaultManager not available"));
        }
        // Determine source_id for fault filtering based on entity type
        const auto & cache = get_thread_safe_cache();
        auto entity_ref = cache.find_entity(entity_id);
        std::string source_id;
        if (entity_ref) {
          if (entity_ref->type == SovdEntityType::APP) {
            auto app = cache.get_app(entity_id);
            if (app) {
              source_id = app->bound_fqn.value_or("");
            }
          } else if (entity_ref->type == SovdEntityType::COMPONENT) {
            auto comp = cache.get_component(entity_id);
            if (comp) {
              source_id = comp->namespace_path;
            }
          }
          // AREA and FUNCTION: leave source_id empty (returns all faults)
          // Guard against TOCTOU: entity found but vanished before get_app/get_component
          if (source_id.empty() && entity_ref->type != SovdEntityType::AREA &&
              entity_ref->type != SovdEntityType::FUNCTION) {
            return tl::make_unexpected(std::string("Entity no longer available: " + entity_id));
          }
        }
        auto result = fault_mgr->list_faults(source_id);
        if (!result.success) {
          return tl::make_unexpected(result.error_message);
        }
        return result.data;
      },
      true);

  sampler_registry_->register_sampler(
      "configurations",
      [this](const std::string & entity_id,
             const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
        auto * config_mgr = get_configuration_manager();
        if (!config_mgr) {
          return tl::make_unexpected(std::string("ConfigurationManager not available"));
        }
        const auto & cache = get_thread_safe_cache();
        auto configs = cache.get_entity_configurations(entity_id);
        nlohmann::json items = nlohmann::json::array();
        for (const auto & node : configs.nodes) {
          auto result = config_mgr->list_parameters(node.node_fqn);
          if (result.success && result.data.is_array()) {
            for (auto & param : result.data) {
              items.push_back(std::move(param));
            }
          }
        }
        nlohmann::json payload;
        payload["items"] = std::move(items);
        return payload;
      },
      true);

  sampler_registry_->register_sampler(
      "logs",
      [this](const std::string & entity_id,
             const std::string & /*resource_path*/) -> tl::expected<nlohmann::json, std::string> {
        auto * log_mgr = get_log_manager();
        if (!log_mgr) {
          return tl::make_unexpected(std::string("LogManager not available"));
        }
        const auto & cache = get_thread_safe_cache();
        auto configs = cache.get_entity_configurations(entity_id);
        std::vector<std::string> node_fqns;
        node_fqns.reserve(configs.nodes.size());
        for (const auto & node : configs.nodes) {
          node_fqns.push_back(node.node_fqn);
        }
        auto result = log_mgr->get_logs(node_fqns, false, "", "", entity_id);
        if (!result.has_value()) {
          return tl::make_unexpected(result.error());
        }
        nlohmann::json payload;
        payload["items"] = std::move(*result);
        return payload;
      },
      true);

  // Register update status sampler (server-level, uses UpdateManager)
  if (update_mgr_) {
    sampler_registry_->register_sampler(
        "updates",
        [this](const std::string & /*entity_id*/,
               const std::string & resource_path) -> tl::expected<nlohmann::json, std::string> {
          auto * mgr = get_update_manager();
          if (!mgr || !mgr->has_backend()) {
            return tl::make_unexpected(std::string("Update backend not available"));
          }
          auto result = mgr->get_status(resource_path);
          if (!result) {
            return tl::make_unexpected(result.error().message);
          }
          return update_status_to_json(*result);
        },
        true);
  }

  RCLCPP_INFO(get_logger(), "Registered built-in resource samplers: data, faults, configurations, logs%s",
              update_mgr_ ? ", updates" : "");

  // Register built-in SSE transport
  transport_registry_->register_transport(
      std::make_unique<SseTransportProvider>(*subscription_mgr_, sse_client_tracker_));

  // Wire subscription lifecycle -> transport cleanup
  subscription_mgr_->set_on_removed([this](const CyclicSubscriptionInfo & info) {
    if (auto * transport = transport_registry_->get_transport(info.protocol)) {
      transport->stop(info.id);
    }
  });

  // Connect topic sampler to discovery manager for component-topic mapping
  discovery_mgr_->set_topic_sampler(data_access_mgr_->get_native_sampler());

  // Connect type introspection for operation schema enrichment
  discovery_mgr_->set_type_introspection(data_access_mgr_->get_type_introspection());

  // Initial discovery
  refresh_cache();

  // Setup periodic refresh with configurable interval
  refresh_timer_ = create_wall_timer(std::chrono::milliseconds(refresh_interval_ms_), [this]() {
    refresh_cache();
  });

  // Setup periodic cleanup of old action goals (every 60 seconds, remove goals older than 5 minutes)
  cleanup_timer_ = create_wall_timer(60s, [this]() {
    operation_mgr_->cleanup_old_goals(std::chrono::seconds(300));
  });

  // Setup periodic cleanup of expired cyclic subscriptions (every 30 seconds)
  subscription_cleanup_timer_ = create_wall_timer(30s, [this]() {
    size_t removed = subscription_mgr_->cleanup_expired();
    if (removed > 0) {
      RCLCPP_DEBUG(get_logger(), "Cleaned up %zu expired cyclic subscriptions", removed);
    }
  });

  // Start REST server with configured host, port, CORS, auth, and TLS
  rest_server_ = std::make_unique<RESTServer>(this, server_host_, server_port_, cors_config_, auth_config_,
                                              rate_limit_config_, tls_config_);
  start_rest_server();

  std::string protocol = tls_config_.enabled ? "HTTPS" : "HTTP";
  RCLCPP_INFO(get_logger(), "ROS 2 Medkit Gateway ready on %s://%s:%d", protocol.c_str(), server_host_.c_str(),
              server_port_);
}

GatewayNode::~GatewayNode() {
  RCLCPP_INFO(get_logger(), "Shutting down ROS 2 Medkit Gateway...");
  // 1. Stop REST server (kills HTTP connections, SSE streams exit)
  stop_rest_server();
  // 2. Shutdown subscriptions via transport registry (calls sub_mgr.shutdown(),
  //    which triggers on_removed -> transport->stop() for each active subscription)
  if (transport_registry_) {
    transport_registry_->shutdown_all(*subscription_mgr_);
  }
  // 3. Shutdown plugins
  if (plugin_mgr_) {
    plugin_mgr_->shutdown_all();
  }
  // 4. Normal member destruction (managers safe - all transports stopped)
}

const ThreadSafeEntityCache & GatewayNode::get_thread_safe_cache() const {
  return thread_safe_cache_;
}

DataAccessManager * GatewayNode::get_data_access_manager() const {
  return data_access_mgr_.get();
}

OperationManager * GatewayNode::get_operation_manager() const {
  return operation_mgr_.get();
}

DiscoveryManager * GatewayNode::get_discovery_manager() const {
  return discovery_mgr_.get();
}

ConfigurationManager * GatewayNode::get_configuration_manager() const {
  return config_mgr_.get();
}

FaultManager * GatewayNode::get_fault_manager() const {
  return fault_mgr_.get();
}

LogManager * GatewayNode::get_log_manager() const {
  return log_mgr_.get();
}

BulkDataStore * GatewayNode::get_bulk_data_store() const {
  return bulk_data_store_.get();
}

SubscriptionManager * GatewayNode::get_subscription_manager() const {
  return subscription_mgr_.get();
}

UpdateManager * GatewayNode::get_update_manager() const {
  return update_mgr_.get();
}

PluginManager * GatewayNode::get_plugin_manager() const {
  return plugin_mgr_.get();
}

ResourceSamplerRegistry * GatewayNode::get_sampler_registry() const {
  return sampler_registry_.get();
}

TransportRegistry * GatewayNode::get_transport_registry() const {
  return transport_registry_.get();
}

std::shared_ptr<SSEClientTracker> GatewayNode::get_sse_client_tracker() const {
  return sse_client_tracker_;
}

void GatewayNode::refresh_cache() {
  RCLCPP_DEBUG(get_logger(), "Refreshing entity cache...");

  try {
    // Refresh topic map first (rebuilds the cached map, triggers pipeline in hybrid mode)
    discovery_mgr_->refresh_topic_map();

    // Discover entities - in HYBRID mode the pipeline merges all sources,
    // in RUNTIME_ONLY mode we manually merge node + topic components
    auto areas = discovery_mgr_->discover_areas();
    auto apps = discovery_mgr_->discover_apps();
    auto functions = discovery_mgr_->discover_functions();

    std::vector<Component> all_components;
    if (discovery_mgr_->get_mode() == DiscoveryMode::HYBRID) {
      // Pipeline already merges node and topic components
      all_components = discovery_mgr_->discover_components();
    } else {
      auto node_components = discovery_mgr_->discover_components();
      auto topic_components = discovery_mgr_->discover_topic_components();
      all_components.reserve(node_components.size() + topic_components.size());
      all_components.insert(all_components.end(), node_components.begin(), node_components.end());
      all_components.insert(all_components.end(), topic_components.begin(), topic_components.end());
    }

    // Capture sizes for logging
    const size_t area_count = areas.size();
    const size_t component_count = all_components.size();
    const size_t app_count = apps.size();
    const size_t function_count = functions.size();

    // Update ThreadSafeEntityCache with copies
    thread_safe_cache_.update_all(areas, all_components, apps, functions);

    // Update topic type cache (avoids expensive ROS graph queries on /data requests)
    if (data_access_mgr_) {
      auto native_sampler = data_access_mgr_->get_native_sampler();
      auto all_topics = native_sampler->discover_all_topics();
      std::unordered_map<std::string, std::string> topic_types;
      topic_types.reserve(all_topics.size());
      for (const auto & topic : all_topics) {
        if (!topic.type.empty()) {
          topic_types[topic.name] = topic.type;
        }
      }
      thread_safe_cache_.update_topic_types(std::move(topic_types));
    }

    RCLCPP_DEBUG(get_logger(), "Cache refreshed: %zu areas, %zu components, %zu apps, %zu functions", area_count,
                 component_count, app_count, function_count);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to refresh cache: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to refresh cache: unknown exception");
  }
}

void GatewayNode::start_rest_server() {
  server_thread_ = std::make_unique<std::thread>([this]() {
    {
      std::lock_guard<std::mutex> lock(server_mutex_);
      server_running_ = true;
    }
    server_cv_.notify_all();

    try {
      rest_server_->start();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "REST server failed to start: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "REST server failed to start: unknown exception");
    }

    {
      std::lock_guard<std::mutex> lock(server_mutex_);
      server_running_ = false;
    }
    server_cv_.notify_all();
  });

  // Wait for server to start
  std::unique_lock<std::mutex> lock(server_mutex_);
  server_cv_.wait(lock, [this] {
    return server_running_.load();
  });
}

void GatewayNode::stop_rest_server() {
  if (rest_server_) {
    rest_server_->stop();
  }

  // Wait for server thread to finish
  if (server_thread_ && server_thread_->joinable()) {
    std::unique_lock<std::mutex> lock(server_mutex_);
    server_cv_.wait(lock, [this] {
      return !server_running_.load();
    });
    server_thread_->join();
  }
}

}  // namespace ros2_medkit_gateway
