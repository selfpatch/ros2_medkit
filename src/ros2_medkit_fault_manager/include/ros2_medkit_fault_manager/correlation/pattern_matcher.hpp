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

#include <map>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/correlation/types.hpp"

namespace ros2_medkit_fault_manager {
namespace correlation {

/// Matches fault codes against patterns with wildcard support
///
/// Supports wildcard patterns where '*' matches any sequence of characters.
/// Examples:
/// - "MOTOR_COMM_*" matches "MOTOR_COMM_FL", "MOTOR_COMM_FR", etc.
/// - "*_TIMEOUT" matches "SENSOR_TIMEOUT", "MOTOR_TIMEOUT", etc.
/// - "MOTOR_*_FL" matches "MOTOR_COMM_FL", "MOTOR_DRIVE_FL", etc.
/// - "*_COMM_*" matches "MOTOR_COMM_FL", "SENSOR_COMM_REAR", etc.
class PatternMatcher {
 public:
  /// Create pattern matcher from configuration
  /// @param patterns Map of pattern ID to pattern definition
  explicit PatternMatcher(const std::map<std::string, FaultPattern> & patterns);

  /// Check if a fault code matches a specific pattern by ID
  /// @param fault_code The fault code to check
  /// @param pattern_id The pattern ID to match against
  /// @return true if the fault code matches the pattern
  bool matches(const std::string & fault_code, const std::string & pattern_id) const;

  /// Check if a fault code matches any code in a list (with wildcard support)
  /// @param fault_code The fault code to check
  /// @param codes List of codes/patterns to match against
  /// @return true if the fault code matches any code in the list
  bool matches_any(const std::string & fault_code, const std::vector<std::string> & codes) const;

  /// Find all pattern IDs that match a fault code
  /// @param fault_code The fault code to check
  /// @return List of pattern IDs that match
  std::vector<std::string> find_matching_patterns(const std::string & fault_code) const;

  /// Check if a fault code matches a single wildcard pattern
  /// @param fault_code The fault code to check
  /// @param pattern The pattern (may contain '*' wildcards)
  /// @return true if the fault code matches the pattern
  static bool matches_wildcard(const std::string & fault_code, const std::string & pattern);

 private:
  /// Compiled pattern (pattern string -> regex)
  struct CompiledPattern {
    std::string original;
    std::regex regex;
    bool has_wildcard;
  };

  /// Compile a wildcard pattern to regex
  static CompiledPattern compile_pattern(const std::string & pattern);

  /// Get or compile pattern regex
  const CompiledPattern & get_compiled(const std::string & pattern) const;

  /// Pattern definitions (pattern_id -> pattern)
  std::map<std::string, FaultPattern> patterns_;

  /// Compiled pattern cache (pattern string -> compiled)
  mutable std::map<std::string, CompiledPattern> compiled_cache_;

  /// Mutex for thread-safe cache access
  mutable std::mutex cache_mutex_;
};

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
