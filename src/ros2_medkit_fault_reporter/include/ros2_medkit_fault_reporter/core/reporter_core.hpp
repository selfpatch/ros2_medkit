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

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "ros2_medkit_fault_detection/fault_types.hpp"
#include "ros2_medkit_fault_reporter/local_filter.hpp"

namespace ros2_medkit_fault_reporter::core {

/// Receives each report that passed the filter.
using ReportSink = std::function<void(const ros2_medkit::fault_detection::FaultReport &)>;

/// Filtering and reporting logic of the fault reporter, without ROS.
class ReporterCore {
 public:
  /// Throws std::invalid_argument when sink is empty.
  ReporterCore(std::string source_id, FilterConfig filter, ReportSink sink);

  /// FAILED report. Returns true when it reached the sink. An empty code is not forwarded.
  bool report(const std::string & fault_code, uint8_t severity, const std::string & description);

  /// PASSED report, filtered by the threshold. Returns true when it reached the sink.
  bool report_passed(const std::string & fault_code);

  const LocalFilter & filter() const {
    return filter_;
  }

 private:
  std::string source_id_;
  LocalFilter filter_;
  ReportSink sink_;
};

/// Builds a valid filter config from raw parameter values. A threshold above INT_MAX and a bypass
/// severity outside [0, 255] fall back to their defaults and append a message to warnings (when not
/// null). A threshold below 1 becomes 1. A window that is not finite and positive becomes 1.0.
FilterConfig validate_filter_params(bool enabled, int64_t threshold, double window_sec, int64_t bypass_severity,
                                    std::vector<std::string> * warnings);

}  // namespace ros2_medkit_fault_reporter::core
