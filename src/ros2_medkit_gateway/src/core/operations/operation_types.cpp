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

#include "ros2_medkit_gateway/core/operations/operation_types.hpp"

namespace ros2_medkit_gateway {

std::string action_status_to_string(ActionGoalStatus status) {
  switch (status) {
    case ActionGoalStatus::ACCEPTED:
      return "accepted";
    case ActionGoalStatus::EXECUTING:
      return "executing";
    case ActionGoalStatus::CANCELING:
      return "canceling";
    case ActionGoalStatus::SUCCEEDED:
      return "succeeded";
    case ActionGoalStatus::CANCELED:
      return "canceled";
    case ActionGoalStatus::ABORTED:
      return "aborted";
    case ActionGoalStatus::UNKNOWN:
    default:
      return "unknown";
  }
}

}  // namespace ros2_medkit_gateway
