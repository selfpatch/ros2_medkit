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

#include "ros2_medkit_gateway/core/fault_trigger_engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <utility>

namespace ros2_medkit_gateway {

namespace {

constexpr double kEqualEpsilon = 1e-9;

std::string json_string(const nlohmann::json & j, const char * key) {
  if (j.contains(key) && j[key].is_string()) {
    return j[key].get<std::string>();
  }
  return {};
}

}  // namespace

FaultTriggerEngine::FaultTriggerEngine(std::string storage_path, ValueFetcher fetcher, FaultReportFn report,
                                       FaultClearFn clear, LogFn log, DataPointNamesFn data_point_names)
  : storage_path_(std::move(storage_path))
  , fetcher_(std::move(fetcher))
  , report_(std::move(report))
  , clear_(std::move(clear))
  , log_(std::move(log))
  , data_point_names_(std::move(data_point_names)) {
  load();
}

bool FaultTriggerEngine::valid_operator(const std::string & op) {
  static const std::array<const char *, 5> kOps{">", "<", ">=", "<=", "=="};
  return std::any_of(kOps.begin(), kOps.end(), [&op](const char * o) {
    return op == o;
  });
}

bool FaultTriggerEngine::valid_severity(const std::string & sev) {
  static const std::array<const char *, 4> kSev{"INFO", "WARNING", "ERROR", "CRITICAL"};
  return std::any_of(kSev.begin(), kSev.end(), [&sev](const char * s) {
    return sev == s;
  });
}

bool FaultTriggerEngine::compare(double value, const std::string & op, double threshold) {
  if (op == ">") {
    return value > threshold;
  }
  if (op == "<") {
    return value < threshold;
  }
  if (op == ">=") {
    return value >= threshold;
  }
  if (op == "<=") {
    return value <= threshold;
  }
  if (op == "==") {
    return std::fabs(value - threshold) <= kEqualEpsilon;
  }
  return false;
}

nlohmann::json FaultTriggerEngine::rule_to_json(const FaultTriggerRule & r) {
  return nlohmann::json{{"id", r.id},
                        {"data_name", r.data_name},
                        {"operator", r.op},
                        {"threshold", r.threshold},
                        {"fault_code", r.fault_code},
                        {"severity", r.severity},
                        {"active", r.active}};
}

std::vector<FaultTriggerRule> FaultTriggerEngine::list(const std::string & app_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<FaultTriggerRule> out;
  for (const auto & r : rules_) {
    if (r.app_id == app_id) {
      out.push_back(r);
    }
  }
  return out;
}

tl::expected<FaultTriggerRule, std::pair<int, std::string>> FaultTriggerEngine::create(const std::string & app_id,
                                                                                       const nlohmann::json & body) {
  if (!body.is_object()) {
    return tl::make_unexpected(std::make_pair(400, std::string("request body must be a JSON object")));
  }

  FaultTriggerRule rule;
  rule.app_id = app_id;
  rule.data_name = json_string(body, "data_name");
  rule.op = json_string(body, "operator");
  rule.fault_code = json_string(body, "fault_code");
  rule.severity = json_string(body, "severity");
  rule.active = body.contains("active") && body["active"].is_boolean() ? body["active"].get<bool>() : true;

  if (rule.data_name.empty()) {
    return tl::make_unexpected(std::make_pair(400, std::string("'data_name' is required")));
  }
  if (!valid_operator(rule.op)) {
    return tl::make_unexpected(std::make_pair(400, std::string("'operator' must be one of >, <, >=, <=, ==")));
  }
  if (!body.contains("threshold") || !body["threshold"].is_number()) {
    return tl::make_unexpected(std::make_pair(400, std::string("'threshold' must be a number")));
  }
  rule.threshold = body["threshold"].get<double>();
  if (rule.fault_code.empty()) {
    return tl::make_unexpected(std::make_pair(400, std::string("'fault_code' is required")));
  }
  if (!valid_severity(rule.severity)) {
    return tl::make_unexpected(
        std::make_pair(400, std::string("'severity' must be one of INFO, WARNING, ERROR, CRITICAL")));
  }

  // A rule bound to a data point the app does not have can never fire, yet it
  // would list as active - a silently dead alarm. Reject it at creation when
  // the app's points are enumerable; nullopt (entity unreadable right now)
  // skips the check so a transient PLC outage does not block rule creation.
  if (data_point_names_) {
    const auto names = data_point_names_(rule.app_id);
    if (names.has_value() && std::find(names->begin(), names->end(), rule.data_name) == names->end()) {
      std::string available;
      constexpr size_t kMaxListed = 20;
      for (size_t i = 0; i < names->size() && i < kMaxListed; ++i) {
        available += (i == 0 ? "" : ", ") + (*names)[i];
      }
      if (names->size() > kMaxListed) {
        available += ", ...";
      }
      return tl::make_unexpected(
          std::make_pair(400, "data point '" + rule.data_name + "' does not exist on app '" + rule.app_id + "'" +
                                  (available.empty() ? std::string{} : " (available: " + available + ")")));
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // fault_code is the PRIMARY KEY of the fault-manager store, so two rules
  // sharing one code would fight over the same stored fault (one rule's clear
  // erases the other's assertion) and make per-app delete/clear ambiguous.
  const auto dup = std::find_if(rules_.begin(), rules_.end(), [&](const FaultTriggerRule & r) {
    return r.fault_code == rule.fault_code;
  });
  if (dup != rules_.end()) {
    return tl::make_unexpected(std::make_pair(
        409, "fault_code '" + rule.fault_code + "' is already used by rule '" + dup->id + "' on app '" + dup->app_id +
                 "' - fault codes are global to the fault store, pick a distinct one"));
  }
  rule.id = "ftr_" + std::to_string(next_seq_++);
  rules_.push_back(rule);
  save_locked();
  if (log_) {
    log_("fault-trigger created: " + rule.id + " on " + app_id + "/" + rule.data_name + " " + rule.op + " " +
         std::to_string(rule.threshold) + " -> " + rule.fault_code + " (" + rule.severity + ")");
  }
  return rule;
}

bool FaultTriggerEngine::remove(const std::string & app_id, const std::string & id) {
  std::string fault_code;
  bool was_crossed = false;
  bool found = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = std::find_if(rules_.begin(), rules_.end(), [&](const FaultTriggerRule & r) {
      return r.app_id == app_id && r.id == id;
    });
    if (it == rules_.end()) {
      return false;
    }
    fault_code = it->fault_code;
    was_crossed = it->crossed;
    rules_.erase(it);
    save_locked();
    found = true;
  }
  // Clear outside the lock: the fault report path may re-enter unrelated code.
  if (found && was_crossed && clear_) {
    clear_(app_id, fault_code);
  }
  return found;
}

void FaultTriggerEngine::evaluate_once() {
  // Snapshot the active rules under the lock, evaluate the (blocking) value
  // fetch + fault report outside it, then fold the new latch state back in.
  struct Pending {
    std::string id;
    std::string app_id;
    std::string data_name;
    std::string op;
    double threshold;
    std::string fault_code;
    std::string severity;
    bool crossed;
  };
  std::vector<Pending> work;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & r : rules_) {
      if (r.active) {
        work.push_back({r.id, r.app_id, r.data_name, r.op, r.threshold, r.fault_code, r.severity, r.crossed});
      }
    }
  }
  if (work.empty()) {
    return;
  }

  for (auto & w : work) {
    const std::optional<double> value = fetcher_ ? fetcher_(w.app_id, w.data_name) : std::nullopt;
    if (!value.has_value()) {
      continue;  // unreadable this poll: hold state, never false-clear a live fault
    }
    const bool now_crossed = compare(*value, w.op, w.threshold);
    if (now_crossed) {
      if (report_) {
        const std::string desc = w.data_name + " = " + std::to_string(*value) + " " + w.op + " " +
                                 std::to_string(w.threshold) + " (fault-trigger " + w.id + ")";
        // Re-report every poll while crossed: level-triggered, so the fault
        // confirms regardless of the manager's debounce threshold and stays
        // asserted until the value recovers.
        report_(w.app_id, w.fault_code, w.severity, desc);
      }
      w.crossed = true;
    } else {
      if (w.crossed && clear_) {
        clear_(w.app_id, w.fault_code);
      }
      w.crossed = false;
    }
  }

  // Fold latch state back into the live rules (still identified by id; a rule
  // deleted mid-evaluation is simply skipped).
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & w : work) {
    auto it = std::find_if(rules_.begin(), rules_.end(), [&](FaultTriggerRule & r) {
      return r.id == w.id && r.app_id == w.app_id;
    });
    if (it != rules_.end()) {
      it->crossed = w.crossed;
    }
  }
}

void FaultTriggerEngine::load() {
  if (storage_path_.empty()) {
    return;
  }
  std::ifstream in(storage_path_);
  if (!in.is_open()) {
    return;  // first run: no store yet
  }
  nlohmann::json j;
  try {
    in >> j;
  } catch (const nlohmann::json::exception & e) {
    if (log_) {
      log_("fault-trigger store '" + storage_path_ + "' is not valid JSON (" + e.what() + "); starting empty");
    }
    return;
  }
  if (!j.is_array()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & item : j) {
    if (!item.is_object()) {
      continue;
    }
    FaultTriggerRule r;
    r.id = json_string(item, "id");
    r.app_id = json_string(item, "app_id");
    r.data_name = json_string(item, "data_name");
    r.op = json_string(item, "operator");
    r.threshold = item.contains("threshold") && item["threshold"].is_number() ? item["threshold"].get<double>() : 0.0;
    r.fault_code = json_string(item, "fault_code");
    r.severity = json_string(item, "severity");
    r.active = item.contains("active") && item["active"].is_boolean() ? item["active"].get<bool>() : true;
    if (r.id.empty() || r.app_id.empty() || r.data_name.empty() || !valid_operator(r.op) || r.fault_code.empty() ||
        !valid_severity(r.severity)) {
      continue;  // drop malformed persisted rows rather than crash on them
    }
    rules_.push_back(r);
    // Keep id generation past any persisted "ftr_<n>" so restarts don't collide.
    if (r.id.rfind("ftr_", 0) == 0) {
      try {
        const uint64_t n = std::stoull(r.id.substr(4));
        next_seq_ = std::max(next_seq_, n + 1);
      } catch (const std::exception &) {
        // non-numeric suffix: leave next_seq_ as is
      }
    }
  }
  if (log_) {
    log_("fault-trigger engine loaded " + std::to_string(rules_.size()) + " rule(s) from " + storage_path_);
  }
}

void FaultTriggerEngine::save_locked() const {
  if (storage_path_.empty()) {
    return;
  }
  nlohmann::json j = nlohmann::json::array();
  for (const auto & r : rules_) {
    nlohmann::json row = rule_to_json(r);
    row["app_id"] = r.app_id;  // store the owning entity too (not in the REST response)
    j.push_back(std::move(row));
  }
  const std::string tmp = storage_path_ + ".tmp";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out.is_open()) {
    if (log_) {
      log_("fault-trigger store '" + storage_path_ + "' not writable; rules will not persist");
    }
    return;
  }
  out << j.dump(2);
  out.close();
  std::rename(tmp.c_str(), storage_path_.c_str());
}

}  // namespace ros2_medkit_gateway
