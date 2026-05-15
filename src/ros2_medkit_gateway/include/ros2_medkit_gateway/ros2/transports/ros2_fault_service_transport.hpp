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
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_medkit_gateway/core/transports/fault_service_transport.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/get_rosbag.hpp"
#include "ros2_medkit_msgs/srv/get_snapshots.hpp"
#include "ros2_medkit_msgs/srv/list_faults.hpp"
#include "ros2_medkit_msgs/srv/list_rosbags.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing FaultServiceTransport.
 *
 * Owns seven `rclcpp::Client<ros2_medkit_msgs::srv::*>` instances scoped to a
 * dedicated `MutuallyExclusive` callback group, plus a private
 * `SingleThreadedExecutor` that drives that group. Each RPC blocks on
 * `executor_.spin_until_future_complete()` so the client's pending-request
 * cleanup and the response destruction happen on the calling thread instead
 * of racing with whichever external executor spins the host node.
 *
 * Performs the ros2_medkit_msgs <-> JSON translation internally, returning
 * neutral FaultResult and FaultWithEnvJsonResult structures so the FaultManager
 * body lives in the ROS-free build layer.
 */
class Ros2FaultServiceTransport : public FaultServiceTransport {
 public:
  /**
   * @param node Non-owning ROS node used for client creation and to derive
   *             the configured fault_manager namespace.
   */
  explicit Ros2FaultServiceTransport(rclcpp::Node * node);

  ~Ros2FaultServiceTransport() override;

  Ros2FaultServiceTransport(const Ros2FaultServiceTransport &) = delete;
  Ros2FaultServiceTransport & operator=(const Ros2FaultServiceTransport &) = delete;
  Ros2FaultServiceTransport(Ros2FaultServiceTransport &&) = delete;
  Ros2FaultServiceTransport & operator=(Ros2FaultServiceTransport &&) = delete;

  FaultResult report_fault(const std::string & fault_code, uint8_t severity, const std::string & description,
                           const std::string & source_id) override;

  FaultResult list_faults(const std::string & source_id, bool include_prefailed, bool include_confirmed,
                          bool include_cleared, bool include_healed, bool include_muted,
                          bool include_clusters) override;

  FaultWithEnvJsonResult get_fault_with_env(const std::string & fault_code, const std::string & source_id) override;

  FaultResult get_fault(const std::string & fault_code, const std::string & source_id) override;

  FaultResult clear_fault(const std::string & fault_code, bool skip_correlation_auto_clear) override;

  FaultResult get_snapshots(const std::string & fault_code, const std::string & topic) override;

  FaultResult get_rosbag(const std::string & fault_code) override;

  FaultResult list_rosbags(const std::string & entity_fqn) override;

  bool wait_for_services(std::chrono::duration<double> timeout) override;

  bool is_available() const override;

 private:
  rclcpp::Node * node_;

  /// Dedicated callback group for all fault service clients. Created with
  /// `automatically_add_to_executor_with_node = false` so the host node's
  /// executor does not spin these clients - we drive them ourselves.
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  /// Private single-threaded executor that owns `callback_group_` and is
  /// driven inline via `spin_until_future_complete()` on each RPC.
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetFault>::SharedPtr get_fault_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::ListFaults>::SharedPtr list_faults_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_fault_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetSnapshots>::SharedPtr get_snapshots_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::GetRosbag>::SharedPtr get_rosbag_client_;
  rclcpp::Client<ros2_medkit_msgs::srv::ListRosbags>::SharedPtr list_rosbags_client_;

  double service_timeout_sec_{5.0};
  std::string fault_manager_base_path_{"/fault_manager"};

  /// Serialises access to `executor_`. `SingleThreadedExecutor::spin_until_future_complete()`
  /// is not safe for concurrent callers on the same executor, so all RPCs take
  /// this mutex for the duration of the synchronous spin. Replaces the seven
  /// per-client mutexes that previously gated each method.
  mutable std::mutex executor_mutex_;
};

}  // namespace ros2_medkit_gateway::ros2
