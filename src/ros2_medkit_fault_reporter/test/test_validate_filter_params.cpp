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

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "ros2_medkit_fault_reporter/core/reporter_core.hpp"

using ros2_medkit_fault_reporter::FilterConfig;
using ros2_medkit_fault_reporter::core::validate_filter_params;

namespace {

constexpr int64_t kIntMax = std::numeric_limits<int>::max();

FilterConfig validate(int64_t threshold, double window, int64_t bypass, std::vector<std::string> * warnings) {
  return validate_filter_params(true, threshold, window, bypass, warnings);
}

}  // namespace

TEST(ValidateFilterParams, ValidValuesPassThroughWithoutWarnings) {
  std::vector<std::string> warnings;
  const auto config = validate(5, 2.5, 3, &warnings);
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.default_threshold, 5);
  EXPECT_DOUBLE_EQ(config.default_window_sec, 2.5);
  EXPECT_EQ(config.bypass_severity, 3);
  EXPECT_TRUE(warnings.empty());
}

TEST(ValidateFilterParams, EnabledFlagIsCopied) {
  const auto config = validate_filter_params(false, 3, 10.0, 2, nullptr);
  EXPECT_FALSE(config.enabled);
}

TEST(ValidateFilterParams, ThresholdIntMaxIsKept) {
  std::vector<std::string> warnings;
  EXPECT_EQ(validate(kIntMax, 10.0, 2, &warnings).default_threshold, std::numeric_limits<int>::max());
  EXPECT_TRUE(warnings.empty());
}

TEST(ValidateFilterParams, ThresholdAboveIntMaxFallsBackToDefaultWithWarning) {
  std::vector<std::string> warnings;
  const auto config = validate(kIntMax + 1, 10.0, 2, &warnings);
  EXPECT_EQ(config.default_threshold, FilterConfig{}.default_threshold);
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_EQ(warnings[0], "fault_reporter.local_filtering.default_threshold 2147483648 is above 2147483647, using 3");
}

TEST(ValidateFilterParams, ThresholdFarAboveIntMaxDoesNotWrapIntoTheValidBand) {
  std::vector<std::string> warnings;
  // 2^32 + 1 narrows to 1 as an int.
  EXPECT_EQ(validate(4294967297LL, 10.0, 2, &warnings).default_threshold, FilterConfig{}.default_threshold);
  EXPECT_EQ(warnings.size(), 1u);
}

TEST(ValidateFilterParams, ThresholdZeroAndNegativeBecomeOne) {
  for (const int64_t threshold : {int64_t{0}, int64_t{-1}, std::numeric_limits<int64_t>::min()}) {
    std::vector<std::string> warnings;
    EXPECT_EQ(validate(threshold, 10.0, 2, &warnings).default_threshold, 1) << threshold;
    EXPECT_TRUE(warnings.empty()) << threshold;
  }
}

TEST(ValidateFilterParams, BypassSeverityEdgesInsideTheRangeAreKept) {
  for (const int64_t bypass : {int64_t{0}, int64_t{255}}) {
    std::vector<std::string> warnings;
    EXPECT_EQ(validate(3, 10.0, bypass, &warnings).bypass_severity, static_cast<uint8_t>(bypass));
    EXPECT_TRUE(warnings.empty()) << bypass;
  }
}

TEST(ValidateFilterParams, BypassSeverityOutsideTheRangeFallsBackToDefaultWithWarning) {
  const struct {
    int64_t value;
    const char * text;
  } cases[] = {{-1, "-1"}, {256, "256"}};
  for (const auto & c : cases) {
    std::vector<std::string> warnings;
    EXPECT_EQ(validate(3, 10.0, c.value, &warnings).bypass_severity, FilterConfig{}.bypass_severity) << c.text;
    ASSERT_EQ(warnings.size(), 1u) << c.text;
    EXPECT_EQ(warnings[0], std::string("fault_reporter.local_filtering.bypass_severity ") + c.text +
                               " is outside [0, 255], using 2");
  }
}

TEST(ValidateFilterParams, WindowThatIsNotFiniteAndPositiveBecomesOneSecond) {
  const double bad[] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity(), 0.0, -1.0};
  for (const double window : bad) {
    EXPECT_DOUBLE_EQ(validate(3, window, 2, nullptr).default_window_sec, 1.0) << window;
  }
}

TEST(ValidateFilterParams, BothFallbacksAppendTwoWarningsInOrder) {
  std::vector<std::string> warnings;
  validate(kIntMax + 1, 10.0, 256, &warnings);
  ASSERT_EQ(warnings.size(), 2u);
  EXPECT_NE(warnings[0].find("default_threshold"), std::string::npos);
  EXPECT_NE(warnings[1].find("bypass_severity"), std::string::npos);
}

TEST(ValidateFilterParams, NullWarningsPointerIsAccepted) {
  const auto config = validate(kIntMax + 1, 10.0, 256, nullptr);
  EXPECT_EQ(config.default_threshold, FilterConfig{}.default_threshold);
  EXPECT_EQ(config.bypass_severity, FilterConfig{}.bypass_severity);
}
