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

#include "ros2_medkit_fault_manager/ros2/ros_log_sink.hpp"

#include "rcutils/logging_macros.h"

namespace ros2_medkit_fault_manager::ros2 {

void RosLogSink::log(core::LogLevel level, const std::string & logger, const std::string & message) {
  switch (level) {
    case core::LogLevel::kDebug:
      RCUTILS_LOG_DEBUG_NAMED(logger.c_str(), "%s", message.c_str());
      break;
    case core::LogLevel::kInfo:
      RCUTILS_LOG_INFO_NAMED(logger.c_str(), "%s", message.c_str());
      break;
    case core::LogLevel::kWarn:
      RCUTILS_LOG_WARN_NAMED(logger.c_str(), "%s", message.c_str());
      break;
    case core::LogLevel::kError:
      RCUTILS_LOG_ERROR_NAMED(logger.c_str(), "%s", message.c_str());
      break;
  }
}

}  // namespace ros2_medkit_fault_manager::ros2
