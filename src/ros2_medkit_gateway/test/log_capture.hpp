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

// LogCapture - records rcutils log output for tests that assert what was
// logged, or how often.

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <iterator>
#include <mutex>
#include <string>
#include <vector>

#include "rcutils/logging.h"

namespace ros2_medkit_gateway::test {

/// Captures every log line while alive and puts the previous output handler
/// back on every exit path, so a failing case cannot swallow the output of
/// later ones or leave rclcpp's handler replaced.
///
/// The handler is a plain C function pointer with no user-data slot, so the
/// live capture is reached through a static. The pointer is atomic because
/// the handler is process-global and other threads log too.
class LogCapture {
 public:
  LogCapture() : previous_(rcutils_logging_get_output_handler()) {
    active().store(this);
    rcutils_logging_set_output_handler(&LogCapture::handler);
  }
  ~LogCapture() {
    rcutils_logging_set_output_handler(previous_);
    active().store(nullptr);
  }
  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;
  LogCapture(LogCapture &&) = delete;
  LogCapture & operator=(LogCapture &&) = delete;

  /// Captured lines, each as "<logger name>: <message>", that contain `needle`.
  std::vector<std::string> matching(const std::string & needle) const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<std::string> out;
    std::copy_if(lines_.begin(), lines_.end(), std::back_inserter(out), [&needle](const std::string & line) {
      return line.find(needle) != std::string::npos;
    });
    return out;
  }

 private:
  static std::atomic<LogCapture *> & active() {
    static std::atomic<LogCapture *> current{nullptr};
    return current;
  }

  static void handler(const rcutils_log_location_t * /*location*/, int /*severity*/, const char * name,
                      rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args) {
    char buf[1024];
    va_list copy;
    va_copy(copy, *args);
    // The format string arrives through the handler signature, so there is no
    // literal to check. GCC exempts va_list formatters from
    // -Wformat-nonliteral, clang does not. Scoped to the single call.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    vsnprintf(buf, sizeof(buf), format, copy);
#pragma GCC diagnostic pop
    va_end(copy);
    LogCapture * capture = active().load();
    if (capture == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lk(capture->mutex_);
    capture->lines_.push_back(std::string(name != nullptr ? name : "") + ": " + buf);
  }

  rcutils_logging_output_handler_t previous_;
  mutable std::mutex mutex_;
  std::vector<std::string> lines_;
};

}  // namespace ros2_medkit_gateway::test
