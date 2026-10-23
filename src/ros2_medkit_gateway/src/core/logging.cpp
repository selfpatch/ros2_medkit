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

#include "ros2_medkit_gateway/core/logging.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway::log {

namespace {

const char * level_name(Level level) {
  switch (level) {
    case Level::kDebug:
      return "DEBUG";
    case Level::kInfo:
      return "INFO";
    case Level::kWarn:
      return "WARN";
    case Level::kError:
      return "ERROR";
    case Level::kFatal:
      return "FATAL";
  }
  return "UNKNOWN";
}

class StderrSink : public Sink {
 public:
  bool enabled(std::string_view /*logger*/, Level level) const override {
    return level >= Level::kInfo;
  }
  void write(std::string_view logger, Level level, std::string_view message, const Location & /*location*/) override {
    std::fprintf(stderr, "[%s] [%.*s]: %.*s\n", level_name(level), static_cast<int>(logger.size()), logger.data(),
                 static_cast<int>(message.size()), message.data());
  }
};

std::shared_ptr<Sink> current_sink() {
  return std::atomic_load(&detail::sink_slot());
}

}  // namespace

void set_sink(std::shared_ptr<Sink> sink) {
  if (!sink) {
    sink = std::make_shared<StderrSink>();
  }
  std::atomic_store(&detail::sink_slot(), std::move(sink));
}

Logger::Logger(std::string name) : name_(std::move(name)) {
}

bool Logger::enabled(Level level) const {
  return current_sink()->enabled(name_, level);
}

namespace detail {

std::shared_ptr<Sink> & sink_slot() {
  static std::shared_ptr<Sink> slot = std::make_shared<StderrSink>();
  return slot;
}

void emit(const Logger & logger, Level level, const Location & location, const char * fmt, ...) {
  std::va_list args;
  va_start(args, fmt);
  std::va_list copy;
  va_copy(copy, args);
  const int needed = std::vsnprintf(nullptr, 0, fmt, copy);
  va_end(copy);
  std::vector<char> buffer(needed > 0 ? static_cast<size_t>(needed) + 1 : 1, '\0');
  if (needed > 0) {
    std::vsnprintf(buffer.data(), buffer.size(), fmt, args);
  }
  va_end(args);
  current_sink()->write(logger.name(), level,
                        std::string_view(buffer.data(), needed > 0 ? static_cast<size_t>(needed) : 0), location);
}

}  // namespace detail

}  // namespace ros2_medkit_gateway::log
