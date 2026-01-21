// Copyright 2026 mfaferek93
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

#include "ros2_medkit_fault_manager/correlation/types.hpp"

#include <algorithm>
#include <stdexcept>

namespace ros2_medkit_fault_manager {
namespace correlation {

CorrelationMode string_to_mode(const std::string & mode_str) {
  std::string lower = mode_str;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "hierarchical") {
    return CorrelationMode::HIERARCHICAL;
  }
  if (lower == "auto_cluster" || lower == "autocluster" || lower == "cluster") {
    return CorrelationMode::AUTO_CLUSTER;
  }
  throw std::invalid_argument("Invalid correlation mode: " + mode_str + ". Valid values: hierarchical, auto_cluster");
}

std::string mode_to_string(CorrelationMode mode) {
  switch (mode) {
    case CorrelationMode::HIERARCHICAL:
      return "hierarchical";
    case CorrelationMode::AUTO_CLUSTER:
      return "auto_cluster";
    default:
      return "unknown";
  }
}

Representative string_to_representative(const std::string & rep_str) {
  std::string lower = rep_str;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "first") {
    return Representative::FIRST;
  }
  if (lower == "most_recent" || lower == "mostrecent" || lower == "recent") {
    return Representative::MOST_RECENT;
  }
  if (lower == "highest_severity" || lower == "highestseverity" || lower == "severity") {
    return Representative::HIGHEST_SEVERITY;
  }
  throw std::invalid_argument("Invalid representative: " + rep_str +
                              ". Valid values: first, most_recent, highest_severity");
}

std::string representative_to_string(Representative rep) {
  switch (rep) {
    case Representative::FIRST:
      return "first";
    case Representative::MOST_RECENT:
      return "most_recent";
    case Representative::HIGHEST_SEVERITY:
      return "highest_severity";
    default:
      return "unknown";
  }
}

std::string severity_to_string(uint8_t severity) {
  switch (severity) {
    case 0:  // SEVERITY_INFO
      return "INFO";
    case 1:  // SEVERITY_WARN
      return "WARNING";
    case 2:  // SEVERITY_ERROR
      return "ERROR";
    case 3:  // SEVERITY_CRITICAL
      return "CRITICAL";
    default:
      return "UNKNOWN";
  }
}

int severity_rank(uint8_t severity) {
  // Direct mapping: severity value is already the rank (0-3)
  return (severity <= 3) ? static_cast<int>(severity) : 0;
}

int severity_rank(const std::string & severity) {
  if (severity == "CRITICAL" || severity == "3") {
    return 3;
  }
  if (severity == "ERROR" || severity == "2") {
    return 2;
  }
  if (severity == "WARN" || severity == "WARNING" || severity == "1") {
    return 1;
  }
  // INFO or unknown
  return 0;
}

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
