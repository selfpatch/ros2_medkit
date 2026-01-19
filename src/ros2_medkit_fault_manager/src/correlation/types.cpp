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

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
