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

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/data_access_manager.hpp"
#include "ros2_medkit_gateway/discovery_manager.hpp"
#include "ros2_medkit_gateway/models.hpp"
#include "ros2_medkit_gateway/operation_manager.hpp"
#include "ros2_medkit_gateway/rest_server.hpp"

namespace ros2_medkit_gateway {

class GatewayNode : public rclcpp::Node {
 public:
  GatewayNode();
  ~GatewayNode();

  // Thread-safe accessors for REST server
  EntityCache get_entity_cache() const;

  /**
   * @brief Get the DataAccessManager instance
   * @return Raw pointer to DataAccessManager (valid for lifetime of GatewayNode)
   * @note The returned pointer is valid as long as the GatewayNode exists.
   *       REST server is stopped before GatewayNode destruction to ensure safe access.
   */
  DataAccessManager * get_data_access_manager() const;

  /**
   * @brief Get the OperationManager instance
   * @return Raw pointer to OperationManager (valid for lifetime of GatewayNode)
   */
  OperationManager * get_operation_manager() const;

  /**
   * @brief Get the DiscoveryManager instance
   * @return Raw pointer to DiscoveryManager (valid for lifetime of GatewayNode)
   */
  DiscoveryManager * get_discovery_manager() const;

 private:
  void refresh_cache();
  void start_rest_server();
  void stop_rest_server();

  // Configuration parameters
  std::string server_host_;
  int server_port_;
  int refresh_interval_ms_;
  CorsConfig cors_config_;

  // Managers
  std::unique_ptr<DiscoveryManager> discovery_mgr_;
  std::unique_ptr<DataAccessManager> data_access_mgr_;
  std::unique_ptr<OperationManager> operation_mgr_;
  std::unique_ptr<RESTServer> rest_server_;

  // Cache with thread safety
  mutable std::mutex cache_mutex_;
  EntityCache entity_cache_;

  // Timer for periodic refresh
  rclcpp::TimerBase::SharedPtr refresh_timer_;

  // Timer for periodic cleanup of old action goals
  rclcpp::TimerBase::SharedPtr cleanup_timer_;

  // REST server thread management
  std::unique_ptr<std::thread> server_thread_;
  std::atomic<bool> server_running_{false};
  std::mutex server_mutex_;
  std::condition_variable server_cv_;
};

}  // namespace ros2_medkit_gateway
