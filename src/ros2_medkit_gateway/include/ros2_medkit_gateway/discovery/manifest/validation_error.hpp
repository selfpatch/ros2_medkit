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

#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Severity level for validation errors
 */
enum class ValidationSeverity {
  ERROR,   ///< Manifest is invalid, cannot proceed
  WARNING  ///< Issue detected, but can proceed
};

/**
 * @brief A single validation error or warning
 */
struct ValidationError {
  std::string rule_id;          ///< e.g., "R001"
  ValidationSeverity severity;  ///< ERROR or WARNING
  std::string message;          ///< Human-readable description
  std::string path;             ///< YAML path where error occurred

  /**
   * @brief Convert to human-readable string
   */
  std::string to_string() const {
    std::string sev = (severity == ValidationSeverity::ERROR) ? "ERROR" : "WARNING";
    return "[" + rule_id + "] " + sev + ": " + message + " (at " + path + ")";
  }
};

/**
 * @brief Result of manifest validation
 */
struct ValidationResult {
  bool is_valid{true};
  std::vector<ValidationError> errors;
  std::vector<ValidationError> warnings;

  bool has_errors() const {
    return !errors.empty();
  }
  bool has_warnings() const {
    return !warnings.empty();
  }

  /**
   * @brief Add an error (manifest is invalid)
   */
  void add_error(const std::string & rule, const std::string & msg, const std::string & path) {
    errors.push_back({rule, ValidationSeverity::ERROR, msg, path});
    is_valid = false;
  }

  /**
   * @brief Add a warning (manifest is valid but has issues)
   */
  void add_warning(const std::string & rule, const std::string & msg, const std::string & path) {
    warnings.push_back({rule, ValidationSeverity::WARNING, msg, path});
  }

  /**
   * @brief Get summary string
   */
  std::string summary() const {
    return std::to_string(errors.size()) + " errors, " + std::to_string(warnings.size()) + " warnings";
  }
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway

