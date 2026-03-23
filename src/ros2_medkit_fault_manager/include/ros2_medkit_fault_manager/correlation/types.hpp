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

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ros2_medkit_fault_manager {
namespace correlation {

/// Reusable fault pattern definition
/// Patterns are referenced by ID in correlation rules
struct FaultPattern {
  /// Pattern identifier (used in rules to reference this pattern)
  std::string id;

  /// List of fault codes to match (supports wildcard '*')
  /// Example: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
  std::vector<std::string> codes;
};

/// Correlation mode determines how faults are grouped
enum class CorrelationMode {
  /// Root cause -> symptoms hierarchy
  /// When root cause is detected, subsequent matching faults are muted as symptoms
  HIERARCHICAL,

  /// Auto-detect clusters of related faults
  /// When min_count faults matching patterns occur within window_ms, they form a cluster
  AUTO_CLUSTER
};

/// How to select the representative fault in an auto-cluster
enum class Representative {
  /// First fault that triggered the cluster
  FIRST,
  /// Most recently reported fault
  MOST_RECENT,
  /// Fault with highest severity
  HIGHEST_SEVERITY
};

/// A single correlation rule
struct CorrelationRule {
  /// Unique rule identifier
  std::string id;

  /// Human-readable rule name
  std::string name;

  /// Correlation mode
  CorrelationMode mode{CorrelationMode::HIERARCHICAL};

  // === Hierarchical mode fields ===

  /// Fault codes that act as root causes (supports wildcard)
  std::vector<std::string> root_cause_codes;

  /// Pattern IDs that define symptoms (references FaultPattern.id)
  std::vector<std::string> symptom_pattern_ids;

  /// Inline symptom codes (not pattern references, direct codes with wildcard support)
  /// Used when symptoms are defined with codes: [...] instead of pattern: xxx
  std::vector<std::string> inline_symptom_codes;

  /// Whether to mute (not publish) symptom faults
  bool mute_symptoms{true};

  /// Whether clearing root cause also clears its symptoms
  bool auto_clear_with_root{true};

  // === Auto-cluster mode fields ===

  /// Pattern IDs to match for clustering
  std::vector<std::string> match_pattern_ids;

  /// Minimum number of faults to trigger a cluster
  uint32_t min_count{3};

  /// Whether to show cluster as single item in fault list
  bool show_as_single{true};

  /// How to select the representative fault
  Representative representative{Representative::HIGHEST_SEVERITY};

  // === Common fields ===

  /// Time window in milliseconds for correlating faults
  uint32_t window_ms{500};
};

/// Complete correlation configuration
struct CorrelationConfig {
  /// Whether correlation is enabled
  bool enabled{false};

  /// Default time window for rules that don't specify one
  uint32_t default_window_ms{500};

  /// Reusable patterns (pattern_id -> pattern)
  std::map<std::string, FaultPattern> patterns;

  /// Correlation rules
  std::vector<CorrelationRule> rules;
};

/// Convert string to CorrelationMode
/// @throws std::invalid_argument if mode string is invalid
CorrelationMode string_to_mode(const std::string & mode_str);

/// Convert CorrelationMode to string
std::string mode_to_string(CorrelationMode mode);

/// Convert string to Representative
/// @throws std::invalid_argument if representative string is invalid
Representative string_to_representative(const std::string & rep_str);

/// Convert Representative to string
std::string representative_to_string(Representative rep);

/// Convert numeric severity (from Fault.msg) to string
/// @param severity Numeric severity (0=INFO, 1=WARN, 2=ERROR, 3=CRITICAL)
/// @return String representation ("INFO", "WARNING", "ERROR", "CRITICAL", or "UNKNOWN")
std::string severity_to_string(uint8_t severity);

/// Get numeric rank for severity comparison (higher = more severe)
/// @param severity Numeric severity (0=INFO, 1=WARN, 2=ERROR, 3=CRITICAL)
/// @return Rank for comparison (0-3)
int severity_rank(uint8_t severity);

/// Get numeric rank for severity comparison from string
/// @param severity String severity ("INFO", "WARNING", "ERROR", "CRITICAL")
/// @return Rank for comparison (0-3)
int severity_rank(const std::string & severity);

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
