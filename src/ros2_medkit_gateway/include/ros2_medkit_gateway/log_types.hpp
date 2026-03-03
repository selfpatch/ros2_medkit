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

#include <cstddef>
#include <cstdint>
#include <string>

namespace ros2_medkit_gateway {

/// Per-entity log configuration (query-time result cap and severity filter)
///
/// Note on max_entries: per the SOVD spec this should be a storage limit;
/// in this implementation it is a query-time result cap (most recent N entries
/// returned). A LogProvider plugin can implement true storage-limit semantics.
struct LogConfig {
  std::string severity_filter = "debug";  ///< Minimum severity to include in query results
  size_t max_entries = 10000;             ///< Maximum entries returned per GET /logs request
};

/// A single log entry stored in the ring buffer
struct LogEntry {
  int64_t id;              ///< Monotonically increasing server-assigned ID (starts at 1)
  int64_t stamp_sec;       ///< Seconds component of log timestamp
  uint32_t stamp_nanosec;  ///< Nanoseconds component of log timestamp
  uint8_t level;           ///< ROS 2 log level (10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL)
  std::string name;        ///< Logger name from /rosout — FQN WITHOUT leading slash
                           ///< e.g. "powertrain/engine/temp_sensor" (NOT "/powertrain/...")
  std::string msg;         ///< Human-readable log message
  std::string function;    ///< Source function name (may be empty)
  std::string file;        ///< Source file path (may be empty)
  uint32_t line;           ///< Source line number (0 if unknown)
};

}  // namespace ros2_medkit_gateway
