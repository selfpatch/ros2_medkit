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

#include "ros2_medkit_fault_reporter/core/reporter_core.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace ros2_medkit_fault_reporter::core {

namespace fd = ros2_medkit::fault_detection;

ReporterCore::ReporterCore(std::string source_id, FilterConfig filter, ReportSink sink)
  : source_id_(std::move(source_id)), filter_(filter), sink_(std::move(sink)) {
  if (!sink_) {
    throw std::invalid_argument("ReporterCore needs a sink");
  }
}

bool ReporterCore::report(const std::string & fault_code, uint8_t severity, const std::string & description) {
  if (fault_code.empty() || !filter_.should_forward(fault_code, severity)) {
    return false;
  }
  {
    fd::FaultReport report;
    report.fault_code = fault_code;
    report.kind = fd::ReportKind::kFailed;
    report.severity = severity;
    report.description = description;
    report.source_id = source_id_;
    sink_(report);
  }
  return true;
}

bool ReporterCore::report_passed(const std::string & fault_code) {
  if (fault_code.empty() || !filter_.should_forward_passed(fault_code)) {
    return false;
  }
  {
    fd::FaultReport report;
    report.fault_code = fault_code;
    report.kind = fd::ReportKind::kPassed;
    report.source_id = source_id_;
    sink_(report);
  }
  return true;
}

FilterConfig validate_filter_params(bool enabled, int64_t threshold, double window_sec, int64_t bypass_severity,
                                    std::vector<std::string> * warnings) {
  FilterConfig defaults;
  FilterConfig config;
  config.enabled = enabled;

  if (threshold > std::numeric_limits<int>::max()) {
    config.default_threshold = defaults.default_threshold;
    if (warnings != nullptr) {
      warnings->push_back("fault_reporter.local_filtering.default_threshold " + std::to_string(threshold) +
                          " is above " + std::to_string(std::numeric_limits<int>::max()) + ", using " +
                          std::to_string(defaults.default_threshold));
    }
  } else {
    config.default_threshold = threshold < 1 ? 1 : static_cast<int>(threshold);
  }

  // "<= 0" alone is false for NaN, so test the valid range and negate it.
  const bool window_valid = std::isfinite(window_sec) && window_sec > 0.0;
  config.default_window_sec = window_valid ? window_sec : 1.0;

  if (bypass_severity < 0 || bypass_severity > 255) {
    config.bypass_severity = defaults.bypass_severity;
    if (warnings != nullptr) {
      warnings->push_back("fault_reporter.local_filtering.bypass_severity " + std::to_string(bypass_severity) +
                          " is outside [0, 255], using " + std::to_string(defaults.bypass_severity));
    }
  } else {
    config.bypass_severity = static_cast<uint8_t>(bypass_severity);
  }
  return config;
}

}  // namespace ros2_medkit_fault_reporter::core
