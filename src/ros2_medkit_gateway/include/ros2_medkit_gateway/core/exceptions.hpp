// Copyright 2025-2026 mfaferek93, bburda
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

#include "ros2_medkit_gateway/core/models/error_info.hpp"

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

/// Exception thrown when a provider returns a non-404 error from its ErrorInfo
/// contract and the caller needs to preserve the original http_status / code
/// on the way up to the handler. TopicNotAvailableException is only appropriate
/// for 404-class errors; everything else must go through this type so 5xx is
/// not collapsed to 404.
class ProviderErrorException : public std::runtime_error {
 public:
  explicit ProviderErrorException(ErrorInfo info) : std::runtime_error(info.message), info_(std::move(info)) {
  }

  const ErrorInfo & info() const noexcept {
    return info_;
  }

 private:
  ErrorInfo info_;
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
