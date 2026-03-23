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

#pragma once

#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/correlation/types.hpp"

namespace ros2_medkit_fault_manager {
namespace correlation {

/// Result of configuration validation
struct ValidationResult {
  bool valid{true};
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  /// Add an error (makes result invalid)
  void add_error(const std::string & msg) {
    valid = false;
    errors.push_back(msg);
  }

  /// Add a warning (result stays valid)
  void add_warning(const std::string & msg) {
    warnings.push_back(msg);
  }
};

/// Parse correlation configuration from YAML file
/// @param config_file Path to the YAML configuration file
/// @return Parsed configuration (enabled=false if parsing failed)
/// @throws std::runtime_error if file cannot be read or YAML is malformed
CorrelationConfig parse_config_file(const std::string & config_file);

/// Parse correlation configuration from YAML string
/// @param yaml_content YAML content as string
/// @return Parsed configuration
/// @throws std::runtime_error if YAML is malformed
CorrelationConfig parse_config_string(const std::string & yaml_content);

/// Validate a parsed correlation configuration
/// Checks:
/// - All referenced pattern IDs exist
/// - No duplicate pattern or rule IDs
/// - Required fields are present
/// - window_ms and min_count are reasonable
/// @param config Configuration to validate
/// @return Validation result with errors and warnings
ValidationResult validate_config(const CorrelationConfig & config);

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
