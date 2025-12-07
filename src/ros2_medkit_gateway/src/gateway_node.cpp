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

#include <chrono>

using namespace std::chrono_literals;

namespace ros2_medkit_gateway {

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

  // Get parameter values
  server_host_ = get_parameter("server.host").as_string();
  server_port_ = get_parameter("server.port").as_int();
  refresh_interval_ms_ = get_parameter("refresh_interval_ms").as_int();

  // Build CORS configuration using builder pattern
  // Throws std::invalid_argument if configuration is invalid
  cors_config_ = CorsConfigBuilder()
                     .with_origins(get_parameter("cors.allowed_origins").as_string_array())
                     .with_methods(get_parameter("cors.allowed_methods").as_string_array())
                     .with_headers(get_parameter("cors.allowed_headers").as_string_array())
                     .with_credentials(get_parameter("cors.allow_credentials").as_bool())
                     .with_max_age(get_parameter("cors.max_age_seconds").as_int())
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

  // Initialize managers
  discovery_mgr_ = std::make_unique<DiscoveryManager>(this);
  data_access_mgr_ = std::make_unique<DataAccessManager>(this);
  operation_mgr_ = std::make_unique<OperationManager>(this, discovery_mgr_.get());
  config_mgr_ = std::make_unique<ConfigurationManager>(this);

  // Connect topic sampler to discovery manager for component-topic mapping
  discovery_mgr_->set_topic_sampler(data_access_mgr_->get_native_sampler());

  // Initial discovery
  refresh_cache();

  // Setup periodic refresh with configurable interval
  refresh_timer_ =
      create_wall_timer(std::chrono::milliseconds(refresh_interval_ms_), std::bind(&GatewayNode::refresh_cache, this));

  // Setup periodic cleanup of old action goals (every 60 seconds, remove goals older than 5 minutes)
  cleanup_timer_ = create_wall_timer(60s, [this]() {
    operation_mgr_->cleanup_old_goals(std::chrono::seconds(300));
  });

  // Start REST server with configured host, port and CORS
  rest_server_ = std::make_unique<RESTServer>(this, server_host_, server_port_, cors_config_);
  start_rest_server();

  RCLCPP_INFO(get_logger(), "ROS 2 Medkit Gateway ready on %s:%d", server_host_.c_str(), server_port_);
}

GatewayNode::~GatewayNode() {
  RCLCPP_INFO(get_logger(), "Shutting down ROS 2 Medkit Gateway...");
  stop_rest_server();
}

EntityCache GatewayNode::get_entity_cache() const {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return entity_cache_;
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

void GatewayNode::refresh_cache() {
  RCLCPP_DEBUG(get_logger(), "Refreshing entity cache...");

  try {
    // Refresh topic map first (rebuilds the cached map)
    discovery_mgr_->refresh_topic_map();

    // Discover data outside the lock to minimize lock time
    auto areas = discovery_mgr_->discover_areas();
    auto components = discovery_mgr_->discover_components();
    auto timestamp = std::chrono::system_clock::now();

    // Capture sizes before move for logging
    const size_t area_count = areas.size();
    const size_t component_count = components.size();

    // Lock only for the actual cache update
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      entity_cache_.areas = std::move(areas);
      entity_cache_.components = std::move(components);
      entity_cache_.last_update = timestamp;
    }

    RCLCPP_DEBUG(get_logger(), "Cache refreshed: %zu areas, %zu components", area_count, component_count);
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
