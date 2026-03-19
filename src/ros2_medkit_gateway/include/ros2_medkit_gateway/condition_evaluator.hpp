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

#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Abstract base class for trigger condition evaluators.
///
/// Each evaluator implements a single condition type (e.g., OnChange, EnterRange).
/// The evaluate() method determines whether a trigger should fire based on the
/// previous and current values of a monitored resource.
///
/// SOVD defines 4 standard condition types. Plugins can register custom evaluators
/// via the ConditionRegistry using vendor-specific "x-" prefixed names.
class ConditionEvaluator {
 public:
  virtual ~ConditionEvaluator() = default;

  /// Evaluate whether the condition is met.
  /// @pre validate_params(params) must have returned success before calling evaluate.
  /// @param previous The previous value (nullopt on first evaluation)
  /// @param current The current value
  /// @param params Condition-specific parameters (e.g., target_value, bounds)
  /// @return true if the condition is met and the trigger should fire
  virtual bool evaluate(const std::optional<nlohmann::json> & previous, const nlohmann::json & current,
                        const nlohmann::json & params) const = 0;

  /// Validate condition parameters before a trigger is created.
  /// @param params The parameters to validate
  /// @return void on success, error string on failure
  virtual tl::expected<void, std::string> validate_params(const nlohmann::json & params) const = 0;
};

// ===========================================================================
// Built-in Evaluators
// ===========================================================================

/// Fires when the current value differs from the previous value.
/// First evaluation (no previous) always fires.
class OnChangeEvaluator : public ConditionEvaluator {
 public:
  bool evaluate(const std::optional<nlohmann::json> & previous, const nlohmann::json & current,
                const nlohmann::json & /*params*/) const override {
    if (!previous.has_value()) {
      return true;
    }
    return *previous != current;
  }

  tl::expected<void, std::string> validate_params(const nlohmann::json & /*params*/) const override {
    return {};
  }
};

/// Fires when the current value equals a target AND differs from the previous.
/// First evaluation (no previous) checks target only.
/// Requires params: {"target_value": <any JSON value>}
class OnChangeToEvaluator : public ConditionEvaluator {
 public:
  bool evaluate(const std::optional<nlohmann::json> & previous, const nlohmann::json & current,
                const nlohmann::json & params) const override {
    const auto & target = params["target_value"];
    if (current != target) {
      return false;
    }
    // Current matches target - fire if no previous or previous was different
    if (!previous.has_value()) {
      return true;
    }
    return *previous != current;
  }

  tl::expected<void, std::string> validate_params(const nlohmann::json & params) const override {
    if (!params.contains("target_value")) {
      return tl::make_unexpected(std::string("Missing required parameter 'target_value'"));
    }
    return {};
  }
};

namespace detail {

/// Shared validation for range-based condition parameters.
inline tl::expected<void, std::string> validate_range_params(const nlohmann::json & params) {
  if (!params.contains("lower_bound") || !params["lower_bound"].is_number()) {
    return tl::make_unexpected(std::string("Missing or non-numeric 'lower_bound' parameter"));
  }
  if (!params.contains("upper_bound") || !params["upper_bound"].is_number()) {
    return tl::make_unexpected(std::string("Missing or non-numeric 'upper_bound' parameter"));
  }
  double lo = params["lower_bound"].get<double>();
  double hi = params["upper_bound"].get<double>();
  if (lo > hi) {
    return tl::make_unexpected(std::string("'lower_bound' must be <= 'upper_bound'"));
  }
  return {};
}

}  // namespace detail

/// Fires when a numeric value transitions from outside [lower_bound, upper_bound]
/// to inside the range. Both bounds are inclusive.
/// First evaluation (no previous) returns false (needs transition).
/// Requires params: {"lower_bound": number, "upper_bound": number}
class EnterRangeEvaluator : public ConditionEvaluator {
 public:
  bool evaluate(const std::optional<nlohmann::json> & previous, const nlohmann::json & current,
                const nlohmann::json & params) const override {
    if (!previous.has_value()) {
      return false;
    }
    if (!previous->is_number() || !current.is_number()) {
      return false;
    }
    double lo = params["lower_bound"].get<double>();
    double hi = params["upper_bound"].get<double>();
    double prev_val = previous->get<double>();
    double curr_val = current.get<double>();

    bool was_outside = prev_val < lo || prev_val > hi;
    bool is_inside = curr_val >= lo && curr_val <= hi;
    return was_outside && is_inside;
  }

  tl::expected<void, std::string> validate_params(const nlohmann::json & params) const override {
    return detail::validate_range_params(params);
  }
};

/// Fires when a numeric value transitions from inside [lower_bound, upper_bound]
/// to outside the range. Both bounds are inclusive.
/// First evaluation (no previous) returns false (needs transition).
/// Requires params: {"lower_bound": number, "upper_bound": number}
class LeaveRangeEvaluator : public ConditionEvaluator {
 public:
  bool evaluate(const std::optional<nlohmann::json> & previous, const nlohmann::json & current,
                const nlohmann::json & params) const override {
    if (!previous.has_value()) {
      return false;
    }
    if (!previous->is_number() || !current.is_number()) {
      return false;
    }
    double lo = params["lower_bound"].get<double>();
    double hi = params["upper_bound"].get<double>();
    double prev_val = previous->get<double>();
    double curr_val = current.get<double>();

    bool was_inside = prev_val >= lo && prev_val <= hi;
    bool is_outside = curr_val < lo || curr_val > hi;
    return was_inside && is_outside;
  }

  tl::expected<void, std::string> validate_params(const nlohmann::json & params) const override {
    return detail::validate_range_params(params);
  }
};

// ===========================================================================
// Condition Registry
// ===========================================================================

/// Thread-safe registry for condition evaluators.
///
/// Stores shared_ptr<ConditionEvaluator> keyed by condition type name.
/// Built-in SOVD types: "OnChange", "OnChangeTo", "EnterRange", "LeaveRange".
/// Plugins register custom evaluators with "x-" prefixed names.
class ConditionRegistry {
 public:
  /// Register a condition evaluator. Throws if name already registered.
  void register_condition(const std::string & name, std::shared_ptr<ConditionEvaluator> evaluator) {
    std::unique_lock lock(mutex_);
    auto [it, inserted] = evaluators_.emplace(name, std::move(evaluator));
    if (!inserted) {
      throw std::runtime_error("Condition evaluator already registered: " + name);
    }
  }

  /// Get a condition evaluator by name. Returns nullptr if not found.
  std::shared_ptr<ConditionEvaluator> get(const std::string & name) const {
    std::shared_lock lock(mutex_);
    auto it = evaluators_.find(name);
    if (it == evaluators_.end()) {
      return nullptr;
    }
    return it->second;
  }

  /// Check if a condition evaluator is registered.
  bool has(const std::string & name) const {
    std::shared_lock lock(mutex_);
    return evaluators_.count(name) > 0;
  }

 private:
  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<ConditionEvaluator>> evaluators_;
};

}  // namespace ros2_medkit_gateway
