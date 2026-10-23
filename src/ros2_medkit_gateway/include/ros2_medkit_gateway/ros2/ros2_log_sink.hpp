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
#include <string_view>

#include "ros2_medkit_gateway/core/logging.hpp"

namespace ros2_medkit_gateway {

/// Forwards log lines to rcutils under the same logger name, so per-name level
/// control (`--log-level <name>:=debug`) and `/rosout` behave as with rclcpp.
class Ros2LogSink : public log::Sink {
 public:
  bool enabled(const std::string & logger, log::Level level) const override;
  void write(const std::string & logger, log::Level level, std::string_view message,
             const log::Location & location) override;
};

}  // namespace ros2_medkit_gateway
