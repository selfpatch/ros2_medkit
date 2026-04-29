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
#include <functional>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/operations/operation_types.hpp"

namespace ros2_medkit_gateway {

class ActionTransport {
 public:
  /// Status update from a status-topic subscription. The action_path and the
  /// new status for one tracked goal. Manager updates its own tracking map.
  using StatusCallback =
      std::function<void(const std::string & action_path, const std::string & goal_id, ActionGoalStatus status)>;

  ActionTransport() = default;
  ActionTransport(const ActionTransport &) = delete;
  ActionTransport & operator=(const ActionTransport &) = delete;
  ActionTransport(ActionTransport &&) = delete;
  ActionTransport & operator=(ActionTransport &&) = delete;
  virtual ~ActionTransport() = default;

  virtual ActionSendGoalResult send_goal(const std::string & action_path, const std::string & action_type,
                                         const json & goal, std::chrono::duration<double> timeout) = 0;

  virtual ActionCancelResult cancel_goal(const std::string & action_path, const std::string & goal_id,
                                         std::chrono::duration<double> timeout) = 0;

  virtual ActionGetResultResult get_result(const std::string & action_path, const std::string & action_type,
                                           const std::string & goal_id, std::chrono::duration<double> timeout) = 0;

  /// Subscribe to status updates for an action path. The callback fires when
  /// any tracked goal's status changes. Subsequent calls for the same path
  /// are no-ops (idempotent).
  virtual void subscribe_status(const std::string & action_path, StatusCallback callback) = 0;

  virtual void unsubscribe_status(const std::string & action_path) = 0;
};

}  // namespace ros2_medkit_gateway
