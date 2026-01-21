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

#include "ros2_medkit_fault_manager/correlation/config_parser.hpp"

#include <fstream>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace ros2_medkit_fault_manager {
namespace correlation {

namespace {

/// Helper to get optional string from YAML node
std::string get_string(const YAML::Node & node, const std::string & key, const std::string & default_value = "") {
  if (node[key] && node[key].IsScalar()) {
    return node[key].as<std::string>();
  }
  return default_value;
}

/// Helper to get optional bool from YAML node
bool get_bool(const YAML::Node & node, const std::string & key, bool default_value = false) {
  if (node[key] && node[key].IsScalar()) {
    return node[key].as<bool>();
  }
  return default_value;
}

/// Helper to get optional uint32 from YAML node
uint32_t get_uint32(const YAML::Node & node, const std::string & key, uint32_t default_value = 0) {
  if (node[key] && node[key].IsScalar()) {
    return node[key].as<uint32_t>();
  }
  return default_value;
}

/// Helper to get string array from YAML node
std::vector<std::string> get_string_array(const YAML::Node & node, const std::string & key) {
  std::vector<std::string> result;
  if (node[key] && node[key].IsSequence()) {
    for (const auto & item : node[key]) {
      if (item.IsScalar()) {
        result.push_back(item.as<std::string>());
      }
    }
  }
  return result;
}

/// Parse a single pattern from YAML
FaultPattern parse_pattern(const std::string & id, const YAML::Node & node) {
  FaultPattern pattern;
  pattern.id = id;
  pattern.codes = get_string_array(node, "codes");
  return pattern;
}

/// Parse patterns section from YAML
std::map<std::string, FaultPattern> parse_patterns(const YAML::Node & patterns_node) {
  std::map<std::string, FaultPattern> patterns;
  if (!patterns_node || !patterns_node.IsMap()) {
    return patterns;
  }

  for (const auto & item : patterns_node) {
    std::string id = item.first.as<std::string>();
    patterns[id] = parse_pattern(id, item.second);
  }
  return patterns;
}

/// Parse symptom references (can be pattern: xxx or codes: [...])
void parse_symptom_refs(const YAML::Node & symptoms_node, CorrelationRule & rule) {
  if (!symptoms_node || !symptoms_node.IsSequence()) {
    return;
  }

  for (const auto & symptom : symptoms_node) {
    if (symptom.IsMap()) {
      // Format: - pattern: pattern_id
      if (symptom["pattern"]) {
        rule.symptom_pattern_ids.push_back(symptom["pattern"].as<std::string>());
      }
      // Format: - codes: [CODE1, CODE2]
      else if (symptom["codes"] && symptom["codes"].IsSequence()) {
        for (const auto & code : symptom["codes"]) {
          if (code.IsScalar()) {
            rule.inline_symptom_codes.push_back(code.as<std::string>());
          }
        }
      }
    }
  }
}

/// Parse match references for auto-cluster
void parse_match_refs(const YAML::Node & match_node, CorrelationRule & rule) {
  if (!match_node || !match_node.IsSequence()) {
    return;
  }

  for (const auto & item : match_node) {
    if (item.IsMap() && item["pattern"]) {
      rule.match_pattern_ids.push_back(item["pattern"].as<std::string>());
    }
  }
}

/// Parse a single rule from YAML
CorrelationRule parse_rule(const YAML::Node & node, uint32_t default_window_ms) {
  CorrelationRule rule;

  rule.id = get_string(node, "id");
  rule.name = get_string(node, "name", rule.id);

  // Parse mode
  std::string mode_str = get_string(node, "mode", "hierarchical");
  rule.mode = string_to_mode(mode_str);

  // Common fields
  rule.window_ms = get_uint32(node, "window_ms", default_window_ms);

  if (rule.mode == CorrelationMode::HIERARCHICAL) {
    // Parse root cause
    if (node["root_cause"] && node["root_cause"]["codes"]) {
      rule.root_cause_codes = get_string_array(node["root_cause"], "codes");
    }

    // Parse symptoms
    parse_symptom_refs(node["symptoms"], rule);

    rule.mute_symptoms = get_bool(node, "mute_symptoms", true);
    rule.auto_clear_with_root = get_bool(node, "auto_clear_with_root", true);
  } else {
    // AUTO_CLUSTER mode
    parse_match_refs(node["match"], rule);

    rule.min_count = get_uint32(node, "min_count", 3);
    rule.show_as_single = get_bool(node, "show_as_single", true);

    std::string rep_str = get_string(node, "representative", "highest_severity");
    rule.representative = string_to_representative(rep_str);
  }

  return rule;
}

/// Parse rules section from YAML
std::vector<CorrelationRule> parse_rules(const YAML::Node & rules_node, uint32_t default_window_ms) {
  std::vector<CorrelationRule> rules;
  if (!rules_node || !rules_node.IsSequence()) {
    return rules;
  }

  for (const auto & rule_node : rules_node) {
    rules.push_back(parse_rule(rule_node, default_window_ms));
  }
  return rules;
}

}  // namespace

CorrelationConfig parse_config_file(const std::string & config_file) {
  std::ifstream file(config_file);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open correlation config file: " + config_file);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return parse_config_string(buffer.str());
}

CorrelationConfig parse_config_string(const std::string & yaml_content) {
  CorrelationConfig config;

  YAML::Node root = YAML::Load(yaml_content);

  // Look for correlation section
  YAML::Node corr_node = root["correlation"];
  if (!corr_node) {
    // No correlation section - return disabled config
    config.enabled = false;
    return config;
  }

  config.enabled = get_bool(corr_node, "enabled", false);
  if (!config.enabled) {
    return config;
  }

  config.default_window_ms = get_uint32(corr_node, "default_window_ms", 500);

  // Parse patterns
  config.patterns = parse_patterns(corr_node["patterns"]);

  // Parse rules
  config.rules = parse_rules(corr_node["rules"], config.default_window_ms);

  return config;
}

ValidationResult validate_config(const CorrelationConfig & config) {
  ValidationResult result;

  if (!config.enabled) {
    return result;  // Nothing to validate if disabled
  }

  // Collect all pattern IDs for reference checking
  std::set<std::string> pattern_ids;
  for (const auto & [id, pattern] : config.patterns) {
    if (pattern_ids.count(id) > 0) {
      result.add_error("Duplicate pattern ID: " + id);
    }
    pattern_ids.insert(id);

    if (pattern.codes.empty()) {
      result.add_warning("Pattern '" + id + "' has no codes defined");
    }
  }

  // Validate rules
  std::set<std::string> rule_ids;
  for (const auto & rule : config.rules) {
    // Check for duplicate rule IDs
    if (rule_ids.count(rule.id) > 0) {
      result.add_error("Duplicate rule ID: " + rule.id);
    }
    rule_ids.insert(rule.id);

    // Check rule has an ID
    if (rule.id.empty()) {
      result.add_error("Rule missing 'id' field");
    }

    if (rule.mode == CorrelationMode::HIERARCHICAL) {
      // Check root cause is defined
      if (rule.root_cause_codes.empty()) {
        result.add_error("Hierarchical rule '" + rule.id + "' has no root_cause codes");
      }

      // Check symptom patterns exist
      for (const auto & pattern_id : rule.symptom_pattern_ids) {
        if (pattern_ids.count(pattern_id) == 0) {
          result.add_error("Rule '" + rule.id + "' references unknown pattern: " + pattern_id);
        }
      }

      if (rule.symptom_pattern_ids.empty() && rule.inline_symptom_codes.empty()) {
        result.add_warning("Hierarchical rule '" + rule.id + "' has no symptoms defined");
      }
    } else {
      // AUTO_CLUSTER mode
      // Check match patterns exist
      for (const auto & pattern_id : rule.match_pattern_ids) {
        if (pattern_ids.count(pattern_id) == 0) {
          result.add_error("Rule '" + rule.id + "' references unknown pattern: " + pattern_id);
        }
      }

      if (rule.match_pattern_ids.empty()) {
        result.add_warning("Auto-cluster rule '" + rule.id + "' has no match patterns defined");
      }

      if (rule.min_count == 0) {
        result.add_error("Rule '" + rule.id + "' has min_count=0, must be at least 1");
      } else if (rule.min_count < 2) {
        result.add_warning("Rule '" + rule.id + "' has min_count < 2, which may cause false clusters");
      }
    }

    // Validate window_ms
    if (rule.window_ms == 0) {
      result.add_error("Rule '" + rule.id + "' has window_ms=0");
    } else if (rule.window_ms > 60000) {
      result.add_warning("Rule '" + rule.id + "' has window_ms > 60s, which may be too long");
    }
  }

  if (config.rules.empty()) {
    result.add_warning("Correlation is enabled but no rules are defined");
  }

  return result;
}

}  // namespace correlation
}  // namespace ros2_medkit_fault_manager
