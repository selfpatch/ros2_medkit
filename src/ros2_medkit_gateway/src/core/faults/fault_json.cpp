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

#include "ros2_medkit_gateway/core/faults/fault_json.hpp"

namespace ros2_medkit_gateway {

namespace fd = ros2_medkit::fault_detection;

namespace {

/// Seconds as sec + nanosec / 1e9 with the time normalised so nanosec is
/// in [0, 1e9), the form builtin_interfaces/Time carries.
double to_seconds(int64_t ns) {
  constexpr int64_t kNsPerSec = 1000000000LL;
  int64_t sec = ns / kNsPerSec;
  int64_t rem = ns % kNsPerSec;
  if (rem < 0) {
    --sec;
    rem += kNsPerSec;
  }
  return static_cast<double>(sec) + static_cast<double>(rem) / 1e9;
}

}  // namespace

nlohmann::json fault_record_to_json(const fd::FaultRecord & fault) {
  nlohmann::json j;
  j["fault_code"] = fault.fault_code;
  j["severity"] = fault.severity;
  j["description"] = fault.description;
  j["first_occurred"] = to_seconds(fault.first_occurred_ns);
  j["last_occurred"] = to_seconds(fault.last_occurred_ns);
  // Omitted while zero: the fault never reported a PASSED event.
  if (fault.last_passed_ns != 0) {
    j["last_passed"] = to_seconds(fault.last_passed_ns);
  }
  j["occurrence_count"] = fault.occurrence_count;
  j["status"] = fault.status;
  j["reporting_sources"] = fault.reporting_sources;

  switch (fault.severity) {
    case fd::severity::kInfo:
      j["severity_label"] = "INFO";
      break;
    case fd::severity::kWarn:
      j["severity_label"] = "WARN";
      break;
    case fd::severity::kError:
      j["severity_label"] = "ERROR";
      break;
    case fd::severity::kCritical:
      j["severity_label"] = "CRITICAL";
      break;
    default:
      j["severity_label"] = "UNKNOWN";
      break;
  }

  return j;
}

}  // namespace ros2_medkit_gateway
