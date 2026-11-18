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

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace ros2_medkit_fault_manager::core {

enum class LogLevel { kDebug, kInfo, kWarn, kError, kFatal };

/// Receives log lines from the ROS-free core. `logger` names the source. log() is called from several
/// threads at once (core callers and capture pool workers); implementations must be thread-safe.
class LogSink {
 public:
  LogSink() = default;
  LogSink(const LogSink &) = delete;
  LogSink & operator=(const LogSink &) = delete;
  LogSink(LogSink &&) = delete;
  LogSink & operator=(LogSink &&) = delete;
  virtual ~LogSink() = default;

  virtual void log(LogLevel level, const std::string & logger, const std::string & message) = 0;
};

/// printf-style helper: formats the message and hands it to the sink.
#if defined(__GNUC__)
__attribute__((format(printf, 4, 5)))
#endif
inline void
logf(LogSink & sink, LogLevel level, const std::string & logger, const char * fmt, ...) {
  va_list args;
  va_start(args, fmt);
  va_list args_copy;
  va_copy(args_copy, args);
  const int len = std::vsnprintf(nullptr, 0, fmt, args);
  va_end(args);
  std::string message;
  if (len > 0) {
    std::vector<char> buf(static_cast<size_t>(len) + 1);
    std::vsnprintf(buf.data(), buf.size(), fmt, args_copy);
    message.assign(buf.data(), static_cast<size_t>(len));
  }
  va_end(args_copy);
  sink.log(level, logger, message);
}

}  // namespace ros2_medkit_fault_manager::core
