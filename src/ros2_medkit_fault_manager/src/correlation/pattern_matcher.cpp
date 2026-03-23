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

#include "ros2_medkit_fault_manager/correlation/pattern_matcher.hpp"

#include <sstream>

namespace ros2_medkit_fault_manager {
namespace correlation {

PatternMatcher::PatternMatcher(const std::map<std::string, FaultPattern> & patterns) : patterns_(patterns) {
  // Pre-compile all patterns for better performance
  for (const auto & [pattern_id, pattern] : patterns_) {
    for (const auto & code : pattern.codes) {
      if (compiled_cache_.find(code) == compiled_cache_.end()) {
        compiled_cache_[code] = compile_pattern(code);
      }
    }
  }
}

bool PatternMatcher::matches(const std::string & fault_code, const std::string & pattern_id) const {
  auto it = patterns_.find(pattern_id);
  if (it == patterns_.end()) {
    return false;
  }

  return matches_any(fault_code, it->second.codes);
}

bool PatternMatcher::matches_any(const std::string & fault_code, const std::vector<std::string> & codes) const {
  for (const auto & code : codes) {
    const auto & compiled = get_compiled(code);

    if (compiled.has_wildcard) {
      if (std::regex_match(fault_code, compiled.regex)) {
        return true;
      }
    } else {
      // Exact match (faster than regex)
      if (fault_code == code) {
        return true;
      }
    }
  }
  return false;
}

std::vector<std::string> PatternMatcher::find_matching_patterns(const std::string & fault_code) const {
  std::vector<std::string> result;

  for (const auto & [pattern_id, pattern] : patterns_) {
    if (matches_any(fault_code, pattern.codes)) {
      result.push_back(pattern_id);
    }
  }

  return result;
}

bool PatternMatcher::matches_wildcard(const std::string & fault_code, const std::string & pattern) {
  // Quick path: no wildcard
  if (pattern.find('*') == std::string::npos) {
    return fault_code == pattern;
  }

  // Convert wildcard pattern to regex and match
  auto compiled = compile_pattern(pattern);
  return std::regex_match(fault_code, compiled.regex);
}

PatternMatcher::CompiledPattern PatternMatcher::compile_pattern(const std::string & pattern) {
  CompiledPattern result;
  result.original = pattern;
  result.has_wildcard = (pattern.find('*') != std::string::npos);

  if (!result.has_wildcard) {
    // No wildcard - use simple string comparison (regex not needed)
    return result;
  }

  // Convert wildcard pattern to regex:
  // - Escape regex special characters
  // - Replace '*' with '.*'
  std::ostringstream regex_str;
  regex_str << "^";  // Anchor to start

  for (char c : pattern) {
    switch (c) {
      case '*':
        regex_str << ".*";
        break;
      // Escape regex special characters
      case '.':
      case '+':
      case '?':
      case '[':
      case ']':
      case '(':
      case ')':
      case '{':
      case '}':
      case '|':
      case '^':
      case '$':
      case '\\':
        regex_str << '\\' << c;
        break;
      default:
        regex_str << c;
        break;
    }
  }

  regex_str << "$";  // Anchor to end

  result.regex = std::regex(regex_str.str());
  return result;
}

const PatternMatcher::CompiledPattern & PatternMatcher::get_compiled(const std::string & pattern) const {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  auto it = compiled_cache_.find(pattern);
  if (it != compiled_cache_.end()) {
    return it->second;
  }

  // Compile and cache
  compiled_cache_[pattern] = compile_pattern(pattern);
  return compiled_cache_[pattern];
}

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
