// Copyright 2026 selfpatch
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

#include <memory>
#include <optional>

#include "ros2_medkit_gateway/condition_evaluator.hpp"

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

// ===========================================================================
// OnChangeEvaluator Tests
// ===========================================================================

TEST(ConditionEvaluator, OnChange_DifferentValues) {
  OnChangeEvaluator eval;
  json prev = 10;
  json curr = 20;
  EXPECT_TRUE(eval.evaluate(prev, curr, json::object()));
}

TEST(ConditionEvaluator, OnChange_SameValues) {
  OnChangeEvaluator eval;
  json prev = 42;
  json curr = 42;
  EXPECT_FALSE(eval.evaluate(prev, curr, json::object()));
}

TEST(ConditionEvaluator, OnChange_NoPrevious) {
  OnChangeEvaluator eval;
  EXPECT_TRUE(eval.evaluate(std::nullopt, json(100), json::object()));
}

TEST(ConditionEvaluator, OnChange_DifferentTypes) {
  OnChangeEvaluator eval;
  json prev = "hello";
  json curr = 42;
  EXPECT_TRUE(eval.evaluate(prev, curr, json::object()));
}

TEST(ConditionEvaluator, OnChange_SameStrings) {
  OnChangeEvaluator eval;
  json prev = "hello";
  json curr = "hello";
  EXPECT_FALSE(eval.evaluate(prev, curr, json::object()));
}

TEST(ConditionEvaluator, OnChange_ValidateParams) {
  OnChangeEvaluator eval;
  // OnChange has no required params
  auto result = eval.validate_params(json::object());
  EXPECT_TRUE(result.has_value());
}

// ===========================================================================
// OnChangeToEvaluator Tests
// ===========================================================================

TEST(ConditionEvaluator, OnChangeTo_MatchesTarget) {
  OnChangeToEvaluator eval;
  json prev = 10;
  json curr = 42;
  json params = {{"target_value", 42}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, OnChangeTo_NoMatch) {
  OnChangeToEvaluator eval;
  json prev = 10;
  json curr = 20;
  json params = {{"target_value", 42}};
  EXPECT_FALSE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, OnChangeTo_AlreadyAtTarget) {
  // Was already at target - no change, should not fire
  OnChangeToEvaluator eval;
  json prev = 42;
  json curr = 42;
  json params = {{"target_value", 42}};
  EXPECT_FALSE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, OnChangeTo_NoPreviousMatchesTarget) {
  // First evaluation, current matches target - fire
  OnChangeToEvaluator eval;
  json params = {{"target_value", "error"}};
  EXPECT_TRUE(eval.evaluate(std::nullopt, json("error"), params));
}

TEST(ConditionEvaluator, OnChangeTo_NoPreviousNoMatch) {
  // First evaluation, current does not match target
  OnChangeToEvaluator eval;
  json params = {{"target_value", "error"}};
  EXPECT_FALSE(eval.evaluate(std::nullopt, json("ok"), params));
}

TEST(ConditionEvaluator, OnChangeTo_StringTarget) {
  OnChangeToEvaluator eval;
  json prev = "idle";
  json curr = "error";
  json params = {{"target_value", "error"}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, OnChangeTo_ValidateParams_MissingTarget) {
  OnChangeToEvaluator eval;
  auto result = eval.validate_params(json::object());
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("target_value"), std::string::npos);
}

TEST(ConditionEvaluator, OnChangeTo_ValidateParams_Valid) {
  OnChangeToEvaluator eval;
  json params = {{"target_value", 42}};
  auto result = eval.validate_params(params);
  EXPECT_TRUE(result.has_value());
}

// ===========================================================================
// EnterRangeEvaluator Tests
// ===========================================================================

TEST(ConditionEvaluator, EnterRange_OutsideToInside) {
  EnterRangeEvaluator eval;
  json prev = 5;
  json curr = 15;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, EnterRange_InsideToInside) {
  EnterRangeEvaluator eval;
  json prev = 12;
  json curr = 18;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_FALSE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, EnterRange_OutsideToOutside) {
  EnterRangeEvaluator eval;
  json prev = 5;
  json curr = 25;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_FALSE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, EnterRange_AtLowerBound) {
  // Exactly at lower bound is "inside"
  EnterRangeEvaluator eval;
  json prev = 5;
  json curr = 10;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, EnterRange_AtUpperBound) {
  // Exactly at upper bound is "inside"
  EnterRangeEvaluator eval;
  json prev = 25;
  json curr = 20;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, EnterRange_NoPrevious) {
  // No previous value - need transition, return false
  EnterRangeEvaluator eval;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_FALSE(eval.evaluate(std::nullopt, json(15), params));
}

TEST(ConditionEvaluator, EnterRange_ValidateParams_Valid) {
  EnterRangeEvaluator eval;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  auto result = eval.validate_params(params);
  EXPECT_TRUE(result.has_value());
}

TEST(ConditionEvaluator, EnterRange_ValidateParams_MissingLower) {
  EnterRangeEvaluator eval;
  json params = {{"upper_bound", 20}};
  auto result = eval.validate_params(params);
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("lower_bound"), std::string::npos);
}

TEST(ConditionEvaluator, EnterRange_ValidateParams_MissingUpper) {
  EnterRangeEvaluator eval;
  json params = {{"lower_bound", 10}};
  auto result = eval.validate_params(params);
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("upper_bound"), std::string::npos);
}

TEST(ConditionEvaluator, EnterRange_ValidateParams_LowerGreaterThanUpper) {
  EnterRangeEvaluator eval;
  json params = {{"lower_bound", 30}, {"upper_bound", 10}};
  auto result = eval.validate_params(params);
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("lower_bound"), std::string::npos);
}

TEST(ConditionEvaluator, EnterRange_ValidateParams_EqualBounds) {
  // Equal bounds is valid (single-point range)
  EnterRangeEvaluator eval;
  json params = {{"lower_bound", 15}, {"upper_bound", 15}};
  auto result = eval.validate_params(params);
  EXPECT_TRUE(result.has_value());
}

TEST(ConditionEvaluator, EnterRange_FloatingPoint) {
  EnterRangeEvaluator eval;
  json prev = 0.5;
  json curr = 1.5;
  json params = {{"lower_bound", 1.0}, {"upper_bound", 2.0}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

// ===========================================================================
// LeaveRangeEvaluator Tests
// ===========================================================================

TEST(ConditionEvaluator, LeaveRange_InsideToOutside) {
  LeaveRangeEvaluator eval;
  json prev = 15;
  json curr = 25;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, LeaveRange_OutsideToOutside) {
  LeaveRangeEvaluator eval;
  json prev = 5;
  json curr = 25;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_FALSE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, LeaveRange_InsideToInside) {
  LeaveRangeEvaluator eval;
  json prev = 12;
  json curr = 18;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_FALSE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, LeaveRange_LeavesBelow) {
  LeaveRangeEvaluator eval;
  json prev = 15;
  json curr = 5;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_TRUE(eval.evaluate(prev, curr, params));
}

TEST(ConditionEvaluator, LeaveRange_NoPrevious) {
  // No previous value - need transition, return false
  LeaveRangeEvaluator eval;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  EXPECT_FALSE(eval.evaluate(std::nullopt, json(25), params));
}

TEST(ConditionEvaluator, LeaveRange_ValidateParams_Valid) {
  LeaveRangeEvaluator eval;
  json params = {{"lower_bound", 10}, {"upper_bound", 20}};
  auto result = eval.validate_params(params);
  EXPECT_TRUE(result.has_value());
}

TEST(ConditionEvaluator, LeaveRange_ValidateParams_LowerGreaterThanUpper) {
  LeaveRangeEvaluator eval;
  json params = {{"lower_bound", 30}, {"upper_bound", 10}};
  auto result = eval.validate_params(params);
  EXPECT_FALSE(result.has_value());
}

// ===========================================================================
// ConditionRegistry Tests
// ===========================================================================

TEST(ConditionEvaluator, Registry_RegisterAndGet) {
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  auto eval = registry.get("OnChange");
  ASSERT_NE(eval, nullptr);
}

TEST(ConditionEvaluator, Registry_Has) {
  ConditionRegistry registry;
  EXPECT_FALSE(registry.has("OnChange"));
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  EXPECT_TRUE(registry.has("OnChange"));
}

TEST(ConditionEvaluator, Registry_UnknownReturnsNullptr) {
  ConditionRegistry registry;
  EXPECT_EQ(registry.get("nonexistent"), nullptr);
}

TEST(ConditionEvaluator, Registry_MultipleEvaluators) {
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  registry.register_condition("OnChangeTo", std::make_shared<OnChangeToEvaluator>());
  registry.register_condition("EnterRange", std::make_shared<EnterRangeEvaluator>());
  registry.register_condition("LeaveRange", std::make_shared<LeaveRangeEvaluator>());

  EXPECT_TRUE(registry.has("OnChange"));
  EXPECT_TRUE(registry.has("OnChangeTo"));
  EXPECT_TRUE(registry.has("EnterRange"));
  EXPECT_TRUE(registry.has("LeaveRange"));
}

TEST(ConditionEvaluator, Registry_DuplicateThrows) {
  ConditionRegistry registry;
  registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  EXPECT_THROW(registry.register_condition("OnChange", std::make_shared<OnChangeEvaluator>()), std::runtime_error);
}

// ===========================================================================
// Plugin Extension Tests
// ===========================================================================

/// Custom evaluator for testing plugin extensibility
class TestCustomEvaluator : public ConditionEvaluator {
 public:
  bool evaluate(const std::optional<nlohmann::json> & /*previous*/, const nlohmann::json & current,
                const nlohmann::json & params) const override {
    // Simple custom logic: fire when current equals "threshold" param
    if (!params.contains("threshold")) {
      return false;
    }
    return current.is_number() && current.get<double>() > params["threshold"].get<double>();
  }

  tl::expected<void, std::string> validate_params(const nlohmann::json & params) const override {
    if (!params.contains("threshold") || !params["threshold"].is_number()) {
      return tl::make_unexpected(std::string("Missing or non-numeric 'threshold' parameter"));
    }
    return {};
  }
};

TEST(ConditionEvaluator, PluginExtension_RegisterCustom) {
  ConditionRegistry registry;
  registry.register_condition("x-test", std::make_shared<TestCustomEvaluator>());
  EXPECT_TRUE(registry.has("x-test"));

  auto eval = registry.get("x-test");
  ASSERT_NE(eval, nullptr);

  json params = {{"threshold", 50.0}};
  EXPECT_TRUE(eval->evaluate(std::nullopt, json(100.0), params));
  EXPECT_FALSE(eval->evaluate(std::nullopt, json(10.0), params));
}

TEST(ConditionEvaluator, PluginExtension_ValidateCustomParams) {
  TestCustomEvaluator eval;

  auto valid = eval.validate_params({{"threshold", 42}});
  EXPECT_TRUE(valid.has_value());

  auto invalid = eval.validate_params(json::object());
  EXPECT_FALSE(invalid.has_value());
  EXPECT_NE(invalid.error().find("threshold"), std::string::npos);
}
