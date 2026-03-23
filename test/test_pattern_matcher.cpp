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

#include "ros2_medkit_fault_manager/correlation/pattern_matcher.hpp"

using namespace ros2_medkit_fault_manager::correlation;

class PatternMatcherTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }
};

// ============================================================================
// Static matches_wildcard tests
// ============================================================================

TEST_F(PatternMatcherTest, ExactMatch) {
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "MOTOR_COMM_FL"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "MOTOR_COMM_FR"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "MOTOR_COMM"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_COMM", "MOTOR_COMM_FL"));
}

TEST_F(PatternMatcherTest, WildcardSuffix) {
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "MOTOR_COMM_*"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FR", "MOTOR_COMM_*"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_REAR_LEFT", "MOTOR_COMM_*"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("SENSOR_COMM_FL", "MOTOR_COMM_*"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_TIMEOUT", "MOTOR_COMM_*"));
}

TEST_F(PatternMatcherTest, WildcardPrefix) {
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "*_COMM_FL"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("SENSOR_COMM_FL", "*_COMM_FL"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("FRONT_MOTOR_COMM_FL", "*_COMM_FL"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_COMM_FR", "*_COMM_FL"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_TIMEOUT_FL", "*_COMM_FL"));
}

TEST_F(PatternMatcherTest, WildcardMiddle) {
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "MOTOR_*_FL"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_TIMEOUT_FL", "MOTOR_*_FL"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_DRIVE_COMM_FL", "MOTOR_*_FL"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_COMM_FR", "MOTOR_*_FL"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("SENSOR_COMM_FL", "MOTOR_*_FL"));
}

TEST_F(PatternMatcherTest, MultipleWildcards) {
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "*_COMM_*"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("SENSOR_COMM_REAR", "*_COMM_*"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("A_COMM_B", "*_COMM_*"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR_TIMEOUT_FL", "*_COMM_*"));
}

TEST_F(PatternMatcherTest, WildcardOnly) {
  EXPECT_TRUE(PatternMatcher::matches_wildcard("ANYTHING", "*"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("", "*"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "*"));
}

TEST_F(PatternMatcherTest, WildcardEdgeCases) {
  // Empty pattern
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTOR", ""));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("", ""));

  // Multiple consecutive wildcards
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR_COMM_FL", "MOTOR**FL"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("ABC", "***"));
}

TEST_F(PatternMatcherTest, SpecialRegexCharacters) {
  // Ensure regex special characters are escaped
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR.COMM", "MOTOR.COMM"));
  EXPECT_FALSE(PatternMatcher::matches_wildcard("MOTORXCOMM", "MOTOR.COMM"));  // . should not be regex dot

  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR[1]", "MOTOR[1]"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR+COMM", "MOTOR+COMM"));
  EXPECT_TRUE(PatternMatcher::matches_wildcard("MOTOR?COMM", "MOTOR?COMM"));
}

// ============================================================================
// PatternMatcher class tests
// ============================================================================

TEST_F(PatternMatcherTest, MatchesPatternById) {
  std::map<std::string, FaultPattern> patterns;
  patterns["motor_errors"] = {"motor_errors", {"MOTOR_COMM_*", "MOTOR_TIMEOUT_*"}};
  patterns["sensor_issues"] = {"sensor_issues", {"SENSOR_*"}};

  PatternMatcher matcher(patterns);

  // motor_errors pattern
  EXPECT_TRUE(matcher.matches("MOTOR_COMM_FL", "motor_errors"));
  EXPECT_TRUE(matcher.matches("MOTOR_COMM_FR", "motor_errors"));
  EXPECT_TRUE(matcher.matches("MOTOR_TIMEOUT_DRIVE", "motor_errors"));
  EXPECT_FALSE(matcher.matches("SENSOR_COMM_FL", "motor_errors"));

  // sensor_issues pattern
  EXPECT_TRUE(matcher.matches("SENSOR_LIDAR", "sensor_issues"));
  EXPECT_TRUE(matcher.matches("SENSOR_IMU_TIMEOUT", "sensor_issues"));
  EXPECT_FALSE(matcher.matches("MOTOR_COMM_FL", "sensor_issues"));

  // Unknown pattern
  EXPECT_FALSE(matcher.matches("MOTOR_COMM_FL", "nonexistent"));
}

TEST_F(PatternMatcherTest, MatchesAnyFromList) {
  std::map<std::string, FaultPattern> patterns;  // Empty - not used
  PatternMatcher matcher(patterns);

  std::vector<std::string> codes = {"MOTOR_COMM_*", "DRIVE_*", "ESTOP_001"};

  EXPECT_TRUE(matcher.matches_any("MOTOR_COMM_FL", codes));
  EXPECT_TRUE(matcher.matches_any("DRIVE_FAULT", codes));
  EXPECT_TRUE(matcher.matches_any("ESTOP_001", codes));
  EXPECT_FALSE(matcher.matches_any("SENSOR_TIMEOUT", codes));
  EXPECT_FALSE(matcher.matches_any("ESTOP_002", codes));
}

TEST_F(PatternMatcherTest, FindMatchingPatterns) {
  std::map<std::string, FaultPattern> patterns;
  patterns["motor_errors"] = {"motor_errors", {"MOTOR_*"}};
  patterns["comm_errors"] = {"comm_errors", {"*_COMM_*"}};
  patterns["timeout_errors"] = {"timeout_errors", {"*_TIMEOUT"}};

  PatternMatcher matcher(patterns);

  // MOTOR_COMM_FL matches motor_errors and comm_errors
  auto matches = matcher.find_matching_patterns("MOTOR_COMM_FL");
  EXPECT_EQ(2u, matches.size());
  EXPECT_NE(std::find(matches.begin(), matches.end(), "motor_errors"), matches.end());
  EXPECT_NE(std::find(matches.begin(), matches.end(), "comm_errors"), matches.end());

  // SENSOR_TIMEOUT matches timeout_errors only
  matches = matcher.find_matching_patterns("SENSOR_TIMEOUT");
  EXPECT_EQ(1u, matches.size());
  EXPECT_EQ("timeout_errors", matches[0]);

  // UNKNOWN matches nothing
  matches = matcher.find_matching_patterns("UNKNOWN_ERROR");
  EXPECT_TRUE(matches.empty());
}

TEST_F(PatternMatcherTest, EmptyPatterns) {
  std::map<std::string, FaultPattern> patterns;
  PatternMatcher matcher(patterns);

  EXPECT_FALSE(matcher.matches("MOTOR_COMM_FL", "any_pattern"));
  EXPECT_TRUE(matcher.find_matching_patterns("MOTOR_COMM_FL").empty());
}

TEST_F(PatternMatcherTest, PatternWithNoCodes) {
  std::map<std::string, FaultPattern> patterns;
  patterns["empty_pattern"] = {"empty_pattern", {}};  // No codes

  PatternMatcher matcher(patterns);

  EXPECT_FALSE(matcher.matches("MOTOR_COMM_FL", "empty_pattern"));
}

TEST_F(PatternMatcherTest, ExactMatchPerformance) {
  // Exact matches should be fast (no regex)
  std::map<std::string, FaultPattern> patterns;
  patterns["exact"] = {"exact", {"MOTOR_COMM_FL", "MOTOR_COMM_FR", "MOTOR_COMM_RL", "MOTOR_COMM_RR"}};

  PatternMatcher matcher(patterns);

  // These should use fast string comparison, not regex
  EXPECT_TRUE(matcher.matches("MOTOR_COMM_FL", "exact"));
  EXPECT_TRUE(matcher.matches("MOTOR_COMM_FR", "exact"));
  EXPECT_FALSE(matcher.matches("MOTOR_COMM_FRONT", "exact"));
}

TEST_F(PatternMatcherTest, MixedExactAndWildcard) {
  std::map<std::string, FaultPattern> patterns;
  patterns["mixed"] = {"mixed", {"ESTOP_001", "MOTOR_*", "SENSOR_LIDAR"}};

  PatternMatcher matcher(patterns);

  EXPECT_TRUE(matcher.matches("ESTOP_001", "mixed"));     // Exact
  EXPECT_TRUE(matcher.matches("MOTOR_COMM", "mixed"));    // Wildcard
  EXPECT_TRUE(matcher.matches("SENSOR_LIDAR", "mixed"));  // Exact
  EXPECT_FALSE(matcher.matches("ESTOP_002", "mixed"));
  EXPECT_FALSE(matcher.matches("SENSOR_IMU", "mixed"));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
