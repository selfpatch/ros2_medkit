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

#include <string>

#include "ros2_medkit_fault_manager/core/log_sink.hpp"

namespace ros2_medkit_fault_manager::ros2 {

/// Forwards core log lines to rcutils with the same level and logger name.
class RosLogSink : public core::LogSink {
 public:
  void log(core::LogLevel level, const std::string & logger, const std::string & message) override;
};

}  // namespace ros2_medkit_fault_manager::ros2
