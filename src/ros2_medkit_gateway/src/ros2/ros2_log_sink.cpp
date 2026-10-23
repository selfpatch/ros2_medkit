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

#include "ros2_medkit_gateway/ros2/ros2_log_sink.hpp"

#include <rcutils/logging.h>

namespace ros2_medkit_gateway {

bool Ros2LogSink::enabled(std::string_view logger, log::Level level) const {
  return rcutils_logging_logger_is_enabled_for(std::string(logger).c_str(), static_cast<int>(level));
}

void Ros2LogSink::write(std::string_view logger, log::Level level, std::string_view message,
                        const log::Location & location) {
  const rcutils_log_location_t rcutils_location = {location.function, location.file,
                                                   static_cast<size_t>(location.line)};
  rcutils_log(&rcutils_location, static_cast<int>(level), std::string(logger).c_str(), "%.*s",
              static_cast<int>(message.size()), message.data());
}

}  // namespace ros2_medkit_gateway
