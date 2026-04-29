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

#include <functional>

#include "ros2_medkit_gateway/core/log_types.hpp"

namespace ros2_medkit_gateway {

/// Port: source of /rosout-like log entries. The adapter
/// (`Ros2LogSource`) subscribes to /rosout and delivers each message as a
/// neutral LogEntry, normalising logger name (no leading slash) and SOVD
/// severity. Manager keeps the ring-buffer storage; the source merely
/// produces entries.
class LogSource {
 public:
  using EntryCallback = std::function<void(const LogEntry & entry)>;

  LogSource() = default;
  LogSource(const LogSource &) = delete;
  LogSource & operator=(const LogSource &) = delete;
  LogSource(LogSource &&) = delete;
  LogSource & operator=(LogSource &&) = delete;
  virtual ~LogSource() = default;

  /// Start delivering entries. Idempotent. Calling start() twice replaces the
  /// previous callback.
  virtual void start(EntryCallback callback) = 0;

  /// Stop delivering entries. Idempotent. After stop() returns, the callback
  /// is guaranteed not to fire again.
  virtual void stop() = 0;
};

}  // namespace ros2_medkit_gateway
