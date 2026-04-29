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
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/faults/fault_types.hpp"

namespace ros2_medkit_gateway {

/// Port for the seven services provided by the ros2_medkit_fault_manager
/// package. The transport wraps `rclcpp::Client<ros2_medkit_msgs::srv::*>`,
/// converts message types to JSON internally, and returns the neutral
/// FaultResult / FaultWithEnvJsonResult structs already filled.
class FaultServiceTransport {
 public:
  FaultServiceTransport() = default;
  FaultServiceTransport(const FaultServiceTransport &) = delete;
  FaultServiceTransport & operator=(const FaultServiceTransport &) = delete;
  FaultServiceTransport(FaultServiceTransport &&) = delete;
  FaultServiceTransport & operator=(FaultServiceTransport &&) = delete;
  virtual ~FaultServiceTransport() = default;

  virtual FaultResult report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                                   const std::string & source_id) = 0;

  virtual FaultResult list_faults(const std::string & source_id, bool include_prefailed, bool include_confirmed,
                                  bool include_cleared, bool include_healed, bool include_muted,
                                  bool include_clusters) = 0;

  virtual FaultWithEnvJsonResult get_fault_with_env(const std::string & fault_code, const std::string & source_id) = 0;

  virtual FaultResult get_fault(const std::string & fault_code, const std::string & source_id) = 0;

  virtual FaultResult clear_fault(const std::string & fault_code) = 0;

  virtual FaultResult get_snapshots(const std::string & fault_code, const std::string & topic) = 0;

  virtual FaultResult get_rosbag(const std::string & fault_code) = 0;

  virtual FaultResult list_rosbags(const std::string & entity_fqn) = 0;

  virtual bool wait_for_services(std::chrono::duration<double> timeout) = 0;

  virtual bool is_available() const = 0;
};

}  // namespace ros2_medkit_gateway
