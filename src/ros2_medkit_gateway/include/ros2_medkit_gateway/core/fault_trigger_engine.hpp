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

#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// One runtime threshold rule bound to a discovered data point (issue #235
/// Tier-1a/1b, config-less). No node_map: the rule targets an app's data point
/// by its discovered ``data_name`` and fires/clears a SOVD fault when the live
/// value crosses ``threshold`` under ``op``.
struct FaultTriggerRule {
  std::string id;
  std::string app_id;     ///< SOVD entity id that owns the data point (reporting source)
  std::string data_name;  ///< discovered data point resource id
  std::string op;         ///< one of ">", "<", ">=", "<=", "=="
  double threshold{0.0};
  std::string fault_code;
  std::string severity;  ///< "INFO" | "WARNING" | "ERROR" | "CRITICAL"
  bool active{true};

  /// Runtime latch (not part of the persisted rule contract): true while the
  /// fault is currently asserted. Reporting is LEVEL-triggered - evaluate_once
  /// re-reports every poll while the value stays crossed (so the fault confirms
  /// past the manager's debounce and stays asserted); `crossed` only gates the
  /// auto-clear on the falling edge.
  bool crossed{false};
};

/// Runtime threshold-rule engine. Rules are evaluated every poll against the
/// live discovered value; on a threshold cross the engine reports a fault
/// (which the existing EntityFreezeFrameCapture snapshots), and on un-cross it
/// auto-clears. Rules persist to a small JSON store so they survive a restart
/// and are editable at runtime over REST.
///
/// All I/O is injected (value fetch + fault report/clear) so the CRUD,
/// validation, comparison and edge-detection logic is unit-testable without a
/// live PLC, ROS graph, or HTTP server.
class FaultTriggerEngine {
 public:
  /// Fetch the current numeric value of ``app_id``'s ``data_name`` data point.
  /// ``std::nullopt`` when the point is unreadable (PLC down, unknown name, or
  /// non-numeric) - the rule is then left in its current state, never cleared.
  using ValueFetcher = std::function<std::optional<double>(const std::string & app_id, const std::string & data_name)>;

  /// Report a FAILED fault for (``app_id`` as reporting source, ``fault_code``).
  using FaultReportFn = std::function<void(const std::string & app_id, const std::string & fault_code,
                                           const std::string & severity, const std::string & description)>;

  /// Report the PASSED (cleared) event for (``app_id``, ``fault_code``).
  using FaultClearFn = std::function<void(const std::string & app_id, const std::string & fault_code)>;

  /// Neutral log sink (keeps this layer ROS-free); may be null.
  using LogFn = std::function<void(const std::string & message)>;

  /// Enumerate the discovered data-point names of ``app_id``. ``std::nullopt``
  /// when they cannot be enumerated right now (entity not data-routed, PLC
  /// unreadable); create() then skips the existence check instead of blocking
  /// rule creation on a transient outage.
  using DataPointNamesFn = std::function<std::optional<std::vector<std::string>>(const std::string & app_id)>;

  /// @param storage_path JSON store path; empty disables persistence (in-memory)
  FaultTriggerEngine(std::string storage_path, ValueFetcher fetcher, FaultReportFn report, FaultClearFn clear,
                     LogFn log = nullptr, DataPointNamesFn data_point_names = nullptr);

  // --- REST-facing CRUD (thread-safe) ---

  /// All rules bound to ``app_id`` (persisted contract fields only).
  std::vector<FaultTriggerRule> list(const std::string & app_id) const;

  /// Validate + create a rule from a POST body. Returns the created rule on
  /// success, an (http_status, message) pair on validation failure.
  tl::expected<FaultTriggerRule, std::pair<int, std::string>> create(const std::string & app_id,
                                                                     const nlohmann::json & body);

  /// Delete the rule ``id`` under ``app_id``. Clears any fault it currently
  /// asserts. Returns false when no such rule exists.
  bool remove(const std::string & app_id, const std::string & id);

  /// Evaluate every active rule once against the live value and fire/clear
  /// faults on threshold edges. Called from the gateway poll timer.
  void evaluate_once();

  // --- Pure helpers (unit-testable) ---

  static bool valid_operator(const std::string & op);
  static bool valid_severity(const std::string & sev);
  static bool compare(double value, const std::string & op, double threshold);

  /// Serialize a rule to the wire contract (persisted + REST response shape).
  static nlohmann::json rule_to_json(const FaultTriggerRule & r);

 private:
  void load();
  void save_locked() const;

  mutable std::mutex mutex_;
  std::string storage_path_;
  ValueFetcher fetcher_;
  FaultReportFn report_;
  FaultClearFn clear_;
  LogFn log_;
  DataPointNamesFn data_point_names_;
  std::vector<FaultTriggerRule> rules_;
  uint64_t next_seq_{1};  ///< monotonic suffix for id generation
};

}  // namespace ros2_medkit_gateway
