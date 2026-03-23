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

#include <gtest/gtest.h>

#include "ros2_medkit_fault_manager/correlation/config_parser.hpp"
#include "ros2_medkit_fault_manager/correlation/types.hpp"

using namespace ros2_medkit_fault_manager::correlation;

class ConfigParserTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }
};

// ============================================================================
// Type conversion tests
// ============================================================================

TEST_F(ConfigParserTest, StringToModeHierarchical) {
  EXPECT_EQ(CorrelationMode::HIERARCHICAL, string_to_mode("hierarchical"));
  EXPECT_EQ(CorrelationMode::HIERARCHICAL, string_to_mode("HIERARCHICAL"));
  EXPECT_EQ(CorrelationMode::HIERARCHICAL, string_to_mode("Hierarchical"));
}

TEST_F(ConfigParserTest, StringToModeAutoCluster) {
  EXPECT_EQ(CorrelationMode::AUTO_CLUSTER, string_to_mode("auto_cluster"));
  EXPECT_EQ(CorrelationMode::AUTO_CLUSTER, string_to_mode("autocluster"));
  EXPECT_EQ(CorrelationMode::AUTO_CLUSTER, string_to_mode("cluster"));
}

TEST_F(ConfigParserTest, StringToModeInvalid) {
  EXPECT_THROW(string_to_mode("invalid"), std::invalid_argument);
  EXPECT_THROW(string_to_mode(""), std::invalid_argument);
}

TEST_F(ConfigParserTest, StringToRepresentative) {
  EXPECT_EQ(Representative::FIRST, string_to_representative("first"));
  EXPECT_EQ(Representative::MOST_RECENT, string_to_representative("most_recent"));
  EXPECT_EQ(Representative::MOST_RECENT, string_to_representative("recent"));
  EXPECT_EQ(Representative::HIGHEST_SEVERITY, string_to_representative("highest_severity"));
  EXPECT_EQ(Representative::HIGHEST_SEVERITY, string_to_representative("severity"));
}

TEST_F(ConfigParserTest, StringToRepresentativeInvalid) {
  EXPECT_THROW(string_to_representative("invalid"), std::invalid_argument);
}

// ============================================================================
// Config parsing tests
// ============================================================================

TEST_F(ConfigParserTest, ParseEmptyConfig) {
  const std::string yaml = R"(
# Empty config - no correlation section
)";
  auto config = parse_config_string(yaml);
  EXPECT_FALSE(config.enabled);
}

TEST_F(ConfigParserTest, ParseDisabledConfig) {
  const std::string yaml = R"(
correlation:
  enabled: false
)";
  auto config = parse_config_string(yaml);
  EXPECT_FALSE(config.enabled);
}

TEST_F(ConfigParserTest, ParseMinimalConfig) {
  const std::string yaml = R"(
correlation:
  enabled: true
  default_window_ms: 1000
)";
  auto config = parse_config_string(yaml);
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(1000u, config.default_window_ms);
  EXPECT_TRUE(config.patterns.empty());
  EXPECT_TRUE(config.rules.empty());
}

TEST_F(ConfigParserTest, ParsePatterns) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
    sensor_issues:
      codes: ["SENSOR_*"]
)";
  auto config = parse_config_string(yaml);
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(2u, config.patterns.size());

  ASSERT_TRUE(config.patterns.count("motor_errors") > 0);
  EXPECT_EQ("motor_errors", config.patterns.at("motor_errors").id);
  EXPECT_EQ(2u, config.patterns.at("motor_errors").codes.size());
  EXPECT_EQ("MOTOR_COMM_*", config.patterns.at("motor_errors").codes[0]);
  EXPECT_EQ("MOTOR_TIMEOUT_*", config.patterns.at("motor_errors").codes[1]);

  ASSERT_TRUE(config.patterns.count("sensor_issues") > 0);
  EXPECT_EQ(1u, config.patterns.at("sensor_issues").codes.size());
}

TEST_F(ConfigParserTest, ParseHierarchicalRule) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*"]
  rules:
    - id: estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: motor_errors
      window_ms: 1000
      mute_symptoms: true
      auto_clear_with_root: true
)";
  auto config = parse_config_string(yaml);
  EXPECT_TRUE(config.enabled);
  ASSERT_EQ(1u, config.rules.size());

  const auto & rule = config.rules[0];
  EXPECT_EQ("estop_cascade", rule.id);
  EXPECT_EQ("E-Stop Cascade", rule.name);
  EXPECT_EQ(CorrelationMode::HIERARCHICAL, rule.mode);
  ASSERT_EQ(1u, rule.root_cause_codes.size());
  EXPECT_EQ("ESTOP_001", rule.root_cause_codes[0]);
  ASSERT_EQ(1u, rule.symptom_pattern_ids.size());
  EXPECT_EQ("motor_errors", rule.symptom_pattern_ids[0]);
  EXPECT_EQ(1000u, rule.window_ms);
  EXPECT_TRUE(rule.mute_symptoms);
  EXPECT_TRUE(rule.auto_clear_with_root);
}

TEST_F(ConfigParserTest, ParseHierarchicalRuleWithInlineCodes) {
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: inline_test
      name: "Inline Codes Test"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
        - codes: ["DRIVE_*"]
      window_ms: 1000
)";
  auto config = parse_config_string(yaml);
  EXPECT_TRUE(config.enabled);
  ASSERT_EQ(1u, config.rules.size());

  const auto & rule = config.rules[0];
  EXPECT_EQ("inline_test", rule.id);
  EXPECT_EQ(CorrelationMode::HIERARCHICAL, rule.mode);
  EXPECT_TRUE(rule.symptom_pattern_ids.empty());  // No pattern references
  ASSERT_EQ(3u, rule.inline_symptom_codes.size());
  EXPECT_EQ("MOTOR_COMM_*", rule.inline_symptom_codes[0]);
  EXPECT_EQ("MOTOR_TIMEOUT_*", rule.inline_symptom_codes[1]);
  EXPECT_EQ("DRIVE_*", rule.inline_symptom_codes[2]);

  // Validate should pass - inline codes count as symptoms
  auto result = validate_config(config);
  EXPECT_TRUE(result.valid) << "Errors: " << (result.errors.empty() ? "none" : result.errors[0]);
}

TEST_F(ConfigParserTest, ParseHierarchicalRuleMixedSymptoms) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: mixed_test
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: sensor_errors
        - codes: ["MOTOR_*", "DRIVE_*"]
)";
  auto config = parse_config_string(yaml);
  ASSERT_EQ(1u, config.rules.size());

  const auto & rule = config.rules[0];
  ASSERT_EQ(1u, rule.symptom_pattern_ids.size());
  EXPECT_EQ("sensor_errors", rule.symptom_pattern_ids[0]);
  ASSERT_EQ(2u, rule.inline_symptom_codes.size());
  EXPECT_EQ("MOTOR_*", rule.inline_symptom_codes[0]);
  EXPECT_EQ("DRIVE_*", rule.inline_symptom_codes[1]);

  auto result = validate_config(config);
  EXPECT_TRUE(result.valid);
}

TEST_F(ConfigParserTest, ParseAutoClusterRule) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    comm_errors:
      codes: ["*_COMM_*", "*_TIMEOUT"]
  rules:
    - id: comm_storm
      name: "Communication Storm"
      mode: auto_cluster
      match:
        - pattern: comm_errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: highest_severity
)";
  auto config = parse_config_string(yaml);
  ASSERT_EQ(1u, config.rules.size());

  const auto & rule = config.rules[0];
  EXPECT_EQ("comm_storm", rule.id);
  EXPECT_EQ(CorrelationMode::AUTO_CLUSTER, rule.mode);
  ASSERT_EQ(1u, rule.match_pattern_ids.size());
  EXPECT_EQ("comm_errors", rule.match_pattern_ids[0]);
  EXPECT_EQ(3u, rule.min_count);
  EXPECT_EQ(500u, rule.window_ms);
  EXPECT_TRUE(rule.show_as_single);
  EXPECT_EQ(Representative::HIGHEST_SEVERITY, rule.representative);
}

TEST_F(ConfigParserTest, ParseMultipleRules) {
  const std::string yaml = R"(
correlation:
  enabled: true
  default_window_ms: 500
  patterns:
    motor_errors:
      codes: ["MOTOR_*"]
    sensor_issues:
      codes: ["SENSOR_*"]
  rules:
    - id: rule1
      mode: hierarchical
      root_cause:
        codes: ["ROOT1"]
      symptoms:
        - pattern: motor_errors
    - id: rule2
      mode: auto_cluster
      match:
        - pattern: sensor_issues
      min_count: 2
)";
  auto config = parse_config_string(yaml);
  EXPECT_EQ(2u, config.patterns.size());
  EXPECT_EQ(2u, config.rules.size());
  EXPECT_EQ("rule1", config.rules[0].id);
  EXPECT_EQ(CorrelationMode::HIERARCHICAL, config.rules[0].mode);
  EXPECT_EQ("rule2", config.rules[1].id);
  EXPECT_EQ(CorrelationMode::AUTO_CLUSTER, config.rules[1].mode);
}

TEST_F(ConfigParserTest, ParseDefaultValues) {
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: minimal_rule
      root_cause:
        codes: ["TEST"]
)";
  auto config = parse_config_string(yaml);
  ASSERT_EQ(1u, config.rules.size());

  const auto & rule = config.rules[0];
  // Check default values
  EXPECT_EQ(CorrelationMode::HIERARCHICAL, rule.mode);  // default
  EXPECT_EQ(500u, rule.window_ms);                      // default from config.default_window_ms
  EXPECT_TRUE(rule.mute_symptoms);                      // default
  EXPECT_TRUE(rule.auto_clear_with_root);               // default
}

// ============================================================================
// Validation tests
// ============================================================================

TEST_F(ConfigParserTest, ValidateValidConfig) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    motor_errors:
      codes: ["MOTOR_*"]
  rules:
    - id: test_rule
      root_cause:
        codes: ["ESTOP"]
      symptoms:
        - pattern: motor_errors
)";
  auto config = parse_config_string(yaml);
  auto result = validate_config(config);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.errors.empty());
}

TEST_F(ConfigParserTest, ValidateUnknownPatternReference) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    motor_errors:
      codes: ["MOTOR_*"]
  rules:
    - id: test_rule
      root_cause:
        codes: ["ESTOP"]
      symptoms:
        - pattern: nonexistent_pattern
)";
  auto config = parse_config_string(yaml);
  auto result = validate_config(config);
  EXPECT_FALSE(result.valid);
  ASSERT_EQ(1u, result.errors.size());
  EXPECT_NE(std::string::npos, result.errors[0].find("nonexistent_pattern"));
}

TEST_F(ConfigParserTest, ValidateDuplicateRuleId) {
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: same_id
      root_cause:
        codes: ["A"]
    - id: same_id
      root_cause:
        codes: ["B"]
)";
  auto config = parse_config_string(yaml);
  auto result = validate_config(config);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.errors.empty());
}

TEST_F(ConfigParserTest, ValidateHierarchicalNoRootCause) {
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: broken_rule
      mode: hierarchical
      # missing root_cause
)";
  auto config = parse_config_string(yaml);
  auto result = validate_config(config);
  EXPECT_FALSE(result.valid);
}

TEST_F(ConfigParserTest, ValidateZeroWindowMs) {
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: test_rule
      root_cause:
        codes: ["A"]
      window_ms: 0
)";
  auto config = parse_config_string(yaml);
  auto result = validate_config(config);
  EXPECT_FALSE(result.valid);
}

TEST_F(ConfigParserTest, ValidateWarningLongWindow) {
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: test_rule
      root_cause:
        codes: ["A"]
      window_ms: 120000
)";
  auto config = parse_config_string(yaml);
  auto result = validate_config(config);
  EXPECT_TRUE(result.valid);  // Still valid, just a warning
  EXPECT_FALSE(result.warnings.empty());
}

TEST_F(ConfigParserTest, ValidateDisabledConfigSkipsValidation) {
  const std::string yaml = R"(
correlation:
  enabled: false
)";
  auto config = parse_config_string(yaml);
  auto result = validate_config(config);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.errors.empty());
  EXPECT_TRUE(result.warnings.empty());
}

// ============================================================================
// Full config example test
// ============================================================================

TEST_F(ConfigParserTest, ParseFullExampleConfig) {
  const std::string yaml = R"(
correlation:
  enabled: true
  default_window_ms: 500

  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
    sensor_issues:
      codes: ["SENSOR_*_TIMEOUT", "SENSOR_*_DISCONNECT"]
    drive_faults:
      codes: ["DRIVE_*"]

  rules:
    - id: estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: motor_errors
        - pattern: drive_faults
      window_ms: 1000
      mute_symptoms: true
      auto_clear_with_root: true

    - id: ota_expected
      name: "OTA Expected Timeouts"
      mode: hierarchical
      root_cause:
        codes: ["OTA_UPDATE_ACTIVE"]
      symptoms:
        - pattern: motor_errors
        - pattern: sensor_issues
      window_ms: 5000
      mute_symptoms: true

    - id: comm_storm
      name: "Communication Storm"
      mode: auto_cluster
      match:
        - pattern: motor_errors
        - pattern: sensor_issues
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: highest_severity
)";

  auto config = parse_config_string(yaml);
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(500u, config.default_window_ms);
  EXPECT_EQ(3u, config.patterns.size());
  EXPECT_EQ(3u, config.rules.size());

  // Validate
  auto result = validate_config(config);
  EXPECT_TRUE(result.valid) << "Errors: " << (result.errors.empty() ? "none" : result.errors[0]);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
