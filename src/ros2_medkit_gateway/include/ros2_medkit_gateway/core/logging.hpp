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

#include <memory>
#include <string>
#include <string_view>

namespace ros2_medkit_gateway::log {

/// Severity values match the rcutils severities.
enum class Level { kDebug = 10, kInfo = 20, kWarn = 30, kError = 40, kFatal = 50 };

/// Call site of a log line.
struct Location {
  const char * file;
  const char * function;
  int line;
};

/// Destination of all log lines. Must be thread-safe.
class Sink {
 public:
  Sink() = default;
  Sink(const Sink &) = delete;
  Sink & operator=(const Sink &) = delete;
  Sink(Sink &&) = delete;
  Sink & operator=(Sink &&) = delete;
  virtual ~Sink() = default;

  virtual bool enabled(std::string_view logger, Level level) const = 0;
  virtual void write(std::string_view logger, Level level, std::string_view message, const Location & location) = 0;
};

/// Replaces the active sink. Call it once at startup, before other threads log.
/// A null sink restores the default sink, which writes "[LEVEL] [name]: message"
/// to stderr for INFO and above.
void set_sink(std::shared_ptr<Sink> sink);

/// A named log source. Cheap to copy.
class Logger {
 public:
  explicit Logger(std::string name);
  const std::string & name() const {
    return name_;
  }
  bool enabled(Level level) const;

 private:
  std::string name_;
};

namespace detail {
/// Process-wide sink slot. Default visibility, so a plugin that links this code
/// binds to the executable's slot.
__attribute__((visibility("default"))) std::shared_ptr<Sink> & sink_slot();

void emit(const Logger & logger, Level level, const Location & location, const char * fmt, ...)
    __attribute__((format(printf, 4, 5)));
}  // namespace detail

}  // namespace ros2_medkit_gateway::log

// The level is checked before the arguments are evaluated or formatted.
#define MEDKIT_LOG_AT(logger, level, ...)                                                                           \
  do {                                                                                                              \
    const auto & medkit_log_ref_ = (logger);                                                                        \
    if (medkit_log_ref_.enabled(level)) {                                                                           \
      ::ros2_medkit_gateway::log::detail::emit(                                                                     \
          medkit_log_ref_, level, ::ros2_medkit_gateway::log::Location{__FILE__, __func__, __LINE__}, __VA_ARGS__); \
    }                                                                                                               \
  } while (0)

#define MEDKIT_LOG_DEBUG(logger, ...) MEDKIT_LOG_AT(logger, ::ros2_medkit_gateway::log::Level::kDebug, __VA_ARGS__)
#define MEDKIT_LOG_INFO(logger, ...) MEDKIT_LOG_AT(logger, ::ros2_medkit_gateway::log::Level::kInfo, __VA_ARGS__)
#define MEDKIT_LOG_WARN(logger, ...) MEDKIT_LOG_AT(logger, ::ros2_medkit_gateway::log::Level::kWarn, __VA_ARGS__)
#define MEDKIT_LOG_ERROR(logger, ...) MEDKIT_LOG_AT(logger, ::ros2_medkit_gateway::log::Level::kError, __VA_ARGS__)
#define MEDKIT_LOG_FATAL(logger, ...) MEDKIT_LOG_AT(logger, ::ros2_medkit_gateway::log::Level::kFatal, __VA_ARGS__)
