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
  declare_parameter("refresh_interval_ms", 2000);

  // Get parameter values
  server_host_ = get_parameter("server.host").as_string();
  server_port_ = get_parameter("server.port").as_int();
  refresh_interval_ms_ = get_parameter("refresh_interval_ms").as_int();

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
    RCLCPP_WARN(get_logger(), "Invalid refresh interval %dms. Must be between 100-60000ms. Using default 2000ms.",
                refresh_interval_ms_);
    refresh_interval_ms_ = 2000;
  }

  // Log configuration
  RCLCPP_INFO(get_logger(), "Configuration: REST API at %s:%d, refresh interval: %dms", server_host_.c_str(),
              server_port_, refresh_interval_ms_);

  // Initialize managers
  discovery_mgr_ = std::make_unique<DiscoveryManager>(this);
  data_access_mgr_ = std::make_unique<DataAccessManager>(this);

  // Initial discovery
  refresh_cache();

  // Setup periodic refresh with configurable interval
  refresh_timer_ =
      create_wall_timer(std::chrono::milliseconds(refresh_interval_ms_), std::bind(&GatewayNode::refresh_cache, this));

  // Start REST server with configured host and port
  rest_server_ = std::make_unique<RESTServer>(this, server_host_, server_port_);
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

void GatewayNode::refresh_cache() {
  RCLCPP_DEBUG(get_logger(), "Refreshing entity cache...");

  try {
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
