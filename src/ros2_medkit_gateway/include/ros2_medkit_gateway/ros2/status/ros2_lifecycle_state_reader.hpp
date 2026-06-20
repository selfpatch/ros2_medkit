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
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/status/lifecycle_state_reader.hpp"

namespace ros2_medkit_gateway {

/// LifecycleStateReader backed by lifecycle_msgs/srv/GetState. The GetState client
/// runs on a private node driven by a private SingleThreadedExecutor that is spun
/// inline on the calling thread (no background spin), so it never races the host
/// gateway node's MultiThreadedExecutor (the private-node/private-executor idea is
/// borrowed from ros2_fault_service_transport.cpp; unlike that transport, the target
/// service path varies per app, so the client is created per call rather than once
/// in the constructor). Calls are serialized by an internal mutex: a slow or
/// unreachable lifecycle node holds the mutex for up to ~2x the timeout and blocks
/// other status reads, so the default timeout is kept short.
class Ros2LifecycleStateReader : public LifecycleStateReader {
 public:
  explicit Ros2LifecycleStateReader(rclcpp::Node * host,
                                    std::chrono::duration<double> timeout = std::chrono::milliseconds(500));
  ~Ros2LifecycleStateReader() override;
  Ros2LifecycleStateReader(const Ros2LifecycleStateReader &) = delete;
  Ros2LifecycleStateReader & operator=(const Ros2LifecycleStateReader &) = delete;
  Ros2LifecycleStateReader(Ros2LifecycleStateReader &&) = delete;
  Ros2LifecycleStateReader & operator=(Ros2LifecycleStateReader &&) = delete;

  std::optional<std::string> get_state(const std::string & get_state_service_path) override;

 private:
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::chrono::duration<double> timeout_;
  std::mutex mutex_;
};

}  // namespace ros2_medkit_gateway
