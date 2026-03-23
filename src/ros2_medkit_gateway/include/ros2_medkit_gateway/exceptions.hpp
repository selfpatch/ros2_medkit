// Copyright 2025 mfaferek93
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

#include <stdexcept>
#include <string>

namespace ros2_medkit_gateway {

/// Exception thrown when a topic is not available or times out
class TopicNotAvailableException : public std::runtime_error {
 public:
  explicit TopicNotAvailableException(const std::string & topic)
    : std::runtime_error("Topic not available: " + topic), topic_(topic) {
  }

  const std::string & topic() const noexcept {
    return topic_;
  }

 private:
  std::string topic_;
};

/// Exception thrown when a required command (e.g., ros2 CLI) is not available
class CommandNotAvailableException : public std::runtime_error {
 public:
  explicit CommandNotAvailableException(const std::string & command)
    : std::runtime_error("Command not available: " + command), command_(command) {
  }

  const std::string & command() const noexcept {
    return command_;
  }

 private:
  std::string command_;
};

}  // namespace ros2_medkit_gateway
