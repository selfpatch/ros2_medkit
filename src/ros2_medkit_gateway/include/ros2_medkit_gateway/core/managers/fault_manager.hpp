// Copyright 2026 bburda
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

#include <chrono>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/faults/fault_types.hpp"
#include "ros2_medkit_gateway/core/transports/fault_service_transport.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief Application service for fault-management operations.
 *
 * Pure C++; ROS-side I/O is performed by the injected FaultServiceTransport
 * adapter (typically Ros2FaultServiceTransport). All
 * `rclcpp::Client<ros2_medkit_msgs::srv::*>` instances, the seven per-client
 * mutexes, and the message-to-JSON conversion helpers live in the adapter.
 *
 * The manager body delegates each of the eight SOVD operations to the
 * transport and returns its neutral FaultResult / FaultWithEnvJsonResult
 * structures. The handler-facing public API is preserved verbatim across
 * the refactor; the only behaviour change is that get_fault_with_env now
 * returns the response body as JSON ("fault" + "environment_data") rather
 * than as raw message types - the transport performs the conversion.
 */
class FaultManager {
 public:
  /**
   * @param transport Concrete FaultServiceTransport adapter. Manager takes
   *                  shared ownership.
   */
  explicit FaultManager(std::shared_ptr<FaultServiceTransport> transport);

  ~FaultManager() = default;

  FaultManager(const FaultManager &) = delete;
  FaultManager & operator=(const FaultManager &) = delete;
  FaultManager(FaultManager &&) = delete;
  FaultManager & operator=(FaultManager &&) = delete;

  /// Report a fault from a component.
  /// @param fault_code Unique fault identifier
  /// @param severity Fault severity (0=INFO, 1=WARN, 2=ERROR, 3=CRITICAL)
  /// @param description Human-readable description
  /// @param source_id Component identifier (namespace path)
  FaultResult report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                           const std::string & source_id);

  /// Get all faults, optionally filtered by component (prefix match on source_id).
  FaultResult list_faults(const std::string & source_id = "", bool include_prefailed = true,
                          bool include_confirmed = true, bool include_cleared = false, bool include_healed = false,
                          bool include_muted = false, bool include_clusters = false);

  /// Get a specific fault by code with environment data, returned as JSON.
  /// `data` carries `{ "fault": {...}, "environment_data": {...} }`. The
  /// rosbag-snapshot bulk_data_uri is intentionally NOT included; per-request
  /// URL building belongs to the handler that knows the entity path.
  FaultWithEnvJsonResult get_fault_with_env(const std::string & fault_code, const std::string & source_id = "");

  /// Get a specific fault by code (JSON result - "fault" body only).
  FaultResult get_fault(const std::string & fault_code, const std::string & source_id = "");

  /// Clear a fault.
  FaultResult clear_fault(const std::string & fault_code);

  /// Get snapshots for a fault (optional topic filter).
  FaultResult get_snapshots(const std::string & fault_code, const std::string & topic = "");

  /// Get rosbag file info for a fault.
  FaultResult get_rosbag(const std::string & fault_code);

  /// Get all rosbag files for an entity (batch operation).
  FaultResult list_rosbags(const std::string & entity_fqn);

  /// Check if fault manager services are available.
  bool is_available() const;

  /// Wait for fault manager services to become available (forwards to transport).
  bool wait_for_services(std::chrono::duration<double> timeout);

 private:
  std::shared_ptr<FaultServiceTransport> transport_;
};

}  // namespace ros2_medkit_gateway
