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

#include <cstdint>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

class ThreadSafeEntityCache;

namespace faults {

/// True when `source` equals `owned` or is a path child of it (`<owned>/...`).
/// The one rule for "this reporting source belongs to this owner".
bool source_in_scope(const std::string & source, const std::string & owned);

/// The fault-reporting source `app` owns, or "" when it owns none. An external
/// app (a plugin-introspected asset with no ROS binding) reports under its bare
/// entity id; every other app owns its `effective_fqn()`. An unbound
/// non-external app owns nothing - granting it its bare id would let it claim
/// faults it never reported.
std::string app_owned_source(const App & app);

/// `app_owned_source` of the cached app `app_id`, or "" when it is not cached.
std::string resolve_app_source_fqn(const ThreadSafeEntityCache & cache, const std::string & app_id);

/// Resolve the set of fault-reporting sources that fall within an entity's
/// scope by walking the entity cache. Each hosted app contributes what
/// `resolve_app_source_fqn` yields (bare entity id for an external app,
/// effective FQN otherwise):
///   - APP: the app's own source, if it owns one
///   - COMPONENT: every hosted app's source, plus the component's own id when
///     the component is external (protocol bridges report under that id)
///   - AREA: every source under the area and its (recursive) subareas
///   - FUNCTION: every source hosted directly or via a hosted component
/// Returns an empty set for SERVER / UNKNOWN.
///
/// Shared by the HTTP fault handlers (`GET /{entity}/faults`) and the ROS 2
/// plugin-context fault path so both agree on entity -> source-set resolution.
std::set<std::string> resolve_entity_source_fqns(const ThreadSafeEntityCache & cache, SovdEntityType type,
                                                 const std::string & entity_id);

/// True when `fault` has at least one reporting source and *every* reporting
/// source is within `source_fqns` (exact match or a path-boundary prefix).
/// An empty scope set or an empty/absent source list returns false.
bool fault_in_source_scope(const nlohmann::json & fault, const std::set<std::string> & source_fqns);

/// Subset of `faults_array` whose faults satisfy `fault_in_source_scope`.
nlohmann::json filter_faults_by_sources(const nlohmann::json & faults_array, const std::set<std::string> & source_fqns);

/// One fault record paired with the reporting source that owns it.
struct ScopedFault {
  nlohmann::json fault;
  std::string owner;
};

/// The first reporting source of `fault`, or "" when the record names none. It
/// is the owner of a per_source record, and a source that reaches a shared one.
std::string record_owner(const nlohmann::json & fault);

/// Every record of `fault_code` in `faults_array` whose owner lies inside
/// `source_fqns`, ordered by owner so the answer does not depend on the order
/// the store listed them in.
///
/// Returns the records rather than a count or a single pick, because the
/// consumers disagree about what several of them mean: a per-entity fault route
/// refuses to act on an ambiguous address, while a rosbag download asks each of
/// their owners whether it holds the recording.
std::vector<ScopedFault> records_of_code_in_scope(const nlohmann::json & faults_array, const std::string & fault_code,
                                                  const std::set<std::string> & source_fqns);

/// The records of `fault_code` inside `source_fqns` that a per-entity route
/// resolves a code in its URL over.
///
/// `listing` is a fault-manager listing read with every status and with muted
/// records included: its `faults` array holds every record, and each
/// `muted_faults` entry names one the correlation engine hides by its
/// `fault_code` and `source_id`.
///
/// The candidates come in three tiers, and the first tier holding any record of
/// the code decides:
///   1. the records the entity's default fault list shows: PREFAILED and
///      CONFIRMED, not muted
///   2. the muted records of those statuses, which that list would show but for
///      the correlation engine
///   3. the records the list shows only when asked for status=cleared or
///      status=healed: CLEARED, HEALED and PREPASSED, muted or not
///
/// So a record hidden as a symptom, or one a source cleared, stays addressable
/// by its code while it is the only one, and never makes the record a client
/// read off the list ambiguous.
std::vector<ScopedFault> addressable_records(const nlohmann::json & listing, const std::string & fault_code,
                                             const std::set<std::string> & source_fqns);

/// Owned source of every cached app, rebuilt when the cache generation moves.
/// Thread-safe. One instance per caller that resolves record owners per event.
class AppSourceIndex {
 public:
  /// Id of the app whose scope holds `owner`, the record's reporting source,
  /// i.e. whose `GET /apps/{id}/faults?status=all` lists the record. With
  /// nested node paths the app owning the longest source wins, then the
  /// smallest id. "" when no app holds it.
  std::string find_owner(const ThreadSafeEntityCache & cache, const std::string & owner);

 private:
  std::mutex mutex_;
  bool built_{false};
  uint64_t generation_{0};
  /// (owned source, app id), one entry per app that owns a source.
  std::vector<std::pair<std::string, std::string>> sources_;
};

}  // namespace faults
}  // namespace ros2_medkit_gateway
