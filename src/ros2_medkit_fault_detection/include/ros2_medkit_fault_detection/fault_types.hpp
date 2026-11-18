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

// Neutral fault types shared by the fault reporter and the fault manager.
// Std only. Values mirror ros2_medkit_msgs Fault, FaultEvent and ReportFault.

#include <cstdint>
#include <string>
#include <vector>

namespace ros2_medkit::fault_detection {

/// Values of ReportFault.srv EVENT_FAILED and EVENT_PASSED.
enum class ReportKind : uint8_t { kFailed = 0, kPassed = 1 };

/// Values of Fault.msg SEVERITY_*.
namespace severity {
inline constexpr uint8_t kInfo = 0;
inline constexpr uint8_t kWarn = 1;
inline constexpr uint8_t kError = 2;
inline constexpr uint8_t kCritical = 3;
}  // namespace severity

/// Values of Fault.msg STATUS_*.
namespace fault_status {
inline constexpr const char * kPrefailed = "PREFAILED";
inline constexpr const char * kPrepassed = "PREPASSED";
inline constexpr const char * kConfirmed = "CONFIRMED";
inline constexpr const char * kHealed = "HEALED";
inline constexpr const char * kCleared = "CLEARED";
}  // namespace fault_status

/// Values of FaultEvent.msg EVENT_*.
namespace fault_event_type {
inline constexpr const char * kConfirmed = "fault_confirmed";
inline constexpr const char * kCleared = "fault_cleared";
inline constexpr const char * kUpdated = "fault_updated";
}  // namespace fault_event_type

/// One report sent to the fault manager.
struct FaultReport {
  std::string fault_code;
  ReportKind kind{ReportKind::kFailed};
  uint8_t severity{0};
  std::string description;
  std::string source_id;
};

/// Fault.msg field for field. A time of 0 ns means never.
struct FaultRecord {
  std::string fault_code;
  uint8_t severity{0};
  std::string description;
  int64_t first_occurred_ns{0};
  int64_t last_occurred_ns{0};
  int64_t last_passed_ns{0};
  uint32_t occurrence_count{0};
  std::string status;
  std::vector<std::string> reporting_sources;
};

/// FaultEvent.msg field for field.
struct FaultEventRecord {
  std::string event_type;
  FaultRecord fault;
  int64_t timestamp_ns{0};
  std::vector<std::string> auto_cleared_codes;
};

}  // namespace ros2_medkit::fault_detection
