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

#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"

#include <algorithm>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {
namespace faults {

namespace {

void collect_app_fqn(const ThreadSafeEntityCache & cache, const std::string & app_id, std::set<std::string> & out) {
  auto fqn = resolve_app_source_fqn(cache, app_id);
  if (fqn.empty()) {
    return;
  }
  out.insert(std::move(fqn));
}

void collect_component_app_fqns(const ThreadSafeEntityCache & cache, const std::string & comp_id,
                                std::set<std::string> & out) {
  // An external component is a reporting source in its own right, even when
  // child apps also contribute: protocol bridge plugins raise faults (e.g.
  // PLC_COMMS_LOST) with the component's own id as source_id, and those must
  // not vanish from /components/<id>/faults just because the component hosts
  // apps. A non-external component never claims its bare id - an unbound ROS
  // component must not own faults it never reported (#516 guardrail).
  auto comp = cache.get_component(comp_id);
  if (comp && comp->external.value_or(false)) {
    out.insert(comp_id);
  }
  for (const auto & app_id : cache.get_apps_for_component(comp_id)) {
    collect_app_fqn(cache, app_id, out);
  }
}

void collect_area_app_fqns(const ThreadSafeEntityCache & cache, const std::string & area_id,
                           std::set<std::string> & out) {
  // BFS over (area, subareas...) so a top-level area whose components live in
  // nested subareas still resolves to the union of every descendant's apps.
  // Without the recursion, e.g. `/areas/powertrain/...` returns an empty set
  // when components are attached to `engine` (subarea of `powertrain`).
  std::vector<std::string> pending = {area_id};
  std::set<std::string> visited;
  while (!pending.empty()) {
    auto current = std::move(pending.back());
    pending.pop_back();
    if (!visited.insert(current).second) {
      continue;
    }
    for (const auto & comp_id : cache.get_components_for_area(current)) {
      collect_component_app_fqns(cache, comp_id, out);
    }
    for (const auto & sub_id : cache.get_subareas(current)) {
      pending.push_back(sub_id);
    }
  }
}

void collect_function_app_fqns(const ThreadSafeEntityCache & cache, const std::string & function_id,
                               std::set<std::string> & out) {
  // Function.hosts can contain either App IDs or Component IDs; the indexed
  // lookups in the cache only resolve the App-host case (function_to_apps_),
  // so we walk the raw `hosts` list and dispatch per host kind ourselves.
  auto func = cache.get_function(function_id);
  if (!func) {
    return;
  }
  for (const auto & host_id : func->hosts) {
    if (cache.get_app(host_id)) {
      collect_app_fqn(cache, host_id, out);
    } else if (cache.get_component(host_id)) {
      collect_component_app_fqns(cache, host_id, out);
    }
    // Unknown host - silently skip; it would have been flagged by manifest validation.
  }
}

bool source_matches_scope(const std::string & src, const std::set<std::string> & scope_fqns) {
  return std::any_of(scope_fqns.begin(), scope_fqns.end(), [&src](const std::string & fqn) {
    return source_in_scope(src, fqn);
  });
}

/// The owner the fault manager gives a shared record (Fault.msg `owner`).
constexpr const char * kSharedOwner = "*";

/// Rank of a record status: the higher, the more advanced the fault.
int status_rank(const std::string & status) {
  if (status == "CONFIRMED") {
    return 5;
  }
  if (status == "PREFAILED") {
    return 4;
  }
  if (status == "PREPASSED") {
    return 3;
  }
  if (status == "HEALED") {
    return 2;
  }
  if (status == "CLEARED") {
    return 1;
  }
  return 0;
}

double number_or(const nlohmann::json & j, const char * key, double fallback) {
  const auto it = j.find(key);
  return it != j.end() && it->is_number() ? it->get<double>() : fallback;
}

std::string owner_field(const nlohmann::json & fault) {
  const auto it = fault.find("owner");
  return it != fault.end() && it->is_string() ? it->get<std::string>() : std::string{};
}

}  // namespace

bool source_in_scope(const std::string & source, const std::string & owned) {
  if (owned.empty()) {
    return false;
  }
  if (source == owned) {
    return true;
  }
  return source.size() > owned.size() && source.compare(0, owned.size(), owned) == 0 && source[owned.size()] == '/';
}

std::string app_owned_source(const App & app) {
  // External assets introspected by a protocol plugin (e.g. a PLC over OPC UA)
  // report faults to the fault_manager under their own entity id, so that id is
  // their sole fault-scope owner. This must be checked before effective_fqn():
  // a manifest may declare a stray ros_binding on an external app, and the
  // derived FQN would otherwise shadow the bare id and drop its faults from
  // every rollup (#517, neighbor case).
  if (app.external.value_or(false)) {
    return app.id;
  }
  // A non-external app that merely failed to bind owns nothing: granting an
  // unbound ROS app its bare id would let it claim faults it never reported.
  return app.effective_fqn();
}

std::string resolve_app_source_fqn(const ThreadSafeEntityCache & cache, const std::string & app_id) {
  auto app = cache.get_app(app_id);
  if (!app) {
    return "";
  }
  return app_owned_source(*app);
}

std::set<std::string> resolve_entity_source_fqns(const ThreadSafeEntityCache & cache, SovdEntityType type,
                                                 const std::string & entity_id) {
  std::set<std::string> fqns;
  switch (type) {
    case SovdEntityType::APP:
      collect_app_fqn(cache, entity_id, fqns);
      break;
    case SovdEntityType::COMPONENT:
      collect_component_app_fqns(cache, entity_id, fqns);
      break;
    case SovdEntityType::AREA:
      collect_area_app_fqns(cache, entity_id, fqns);
      break;
    case SovdEntityType::FUNCTION:
      collect_function_app_fqns(cache, entity_id, fqns);
      break;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      break;
  }
  return fqns;
}

std::vector<std::string> hosted_entities_of(const ThreadSafeEntityCache & cache, const std::string & source_id) {
  auto entity = cache.find_entity(source_id);
  if (!entity || entity->type != SovdEntityType::COMPONENT) {
    return {};
  }
  return cache.get_apps_for_component(source_id);
}

bool fault_in_source_scope(const nlohmann::json & fault, const std::set<std::string> & source_fqns) {
  if (source_fqns.empty()) {
    return false;
  }
  const auto sources = fault.find("reporting_sources");
  if (sources == fault.end() || !sources->is_array()) {
    return false;
  }
  return std::any_of(sources->begin(), sources->end(), [&source_fqns](const nlohmann::json & src) {
    return src.is_string() && source_matches_scope(src.get<std::string>(), source_fqns);
  });
}

nlohmann::json filter_faults_by_sources(const nlohmann::json & faults_array,
                                        const std::set<std::string> & source_fqns) {
  nlohmann::json filtered = nlohmann::json::array();
  for (const auto & fault : faults_array) {
    if (fault_in_source_scope(fault, source_fqns)) {
      filtered.push_back(fault);
    }
  }
  return filtered;
}

std::string record_owner(const nlohmann::json & fault) {
  if (!fault.contains("reporting_sources") || !fault["reporting_sources"].is_array()) {
    return {};
  }
  const auto & sources = fault["reporting_sources"];
  if (sources.empty() || !sources.front().is_string()) {
    return {};
  }
  return sources.front().get<std::string>();
}

std::string stored_owner(const nlohmann::json & fault) {
  auto owner = owner_field(fault);
  return owner.empty() ? record_owner(fault) : owner;
}

bool is_shared_record(const nlohmann::json & fault) {
  const auto owner = owner_field(fault);
  return owner.empty() || owner == kSharedOwner;
}

nlohmann::json record_ref(const nlohmann::json & record, const AppOfSource & app_of) {
  nlohmann::json ref = nlohmann::json::object();
  if (is_shared_record(record)) {
    ref["reporting_sources"] = record.value("reporting_sources", nlohmann::json::array());
  } else {
    const auto owner = stored_owner(record);
    auto app_id = app_of ? app_of(owner) : std::string{};
    if (!app_id.empty()) {
      ref["app_id"] = std::move(app_id);
    }
    ref["source_id"] = owner;
  }
  ref["status"] = record.value("status", std::string{});
  return ref;
}

nlohmann::json record_item(const nlohmann::json & record, const AppOfSource & app_of) {
  nlohmann::json item = record;
  item.erase("owner");
  item.erase("source_id");
  if (!is_shared_record(record)) {
    item["x-medkit"] = {{"records", nlohmann::json::array({record_ref(record, app_of)})}};
  }
  return item;
}

const nlohmann::json & representative_record(const std::vector<nlohmann::json> & records) {
  const nlohmann::json * best = &records.front();
  const auto better = [](const nlohmann::json & a, const nlohmann::json & b) {
    const bool a_confirmed = a.value("status", std::string{}) == "CONFIRMED";
    const bool b_confirmed = b.value("status", std::string{}) == "CONFIRMED";
    if (a_confirmed != b_confirmed) {
      return a_confirmed;
    }
    return number_or(a, "last_occurred", 0.0) > number_or(b, "last_occurred", 0.0);
  };
  for (const auto & record : records) {
    if (better(record, *best)) {
      best = &record;
    }
  }
  return *best;
}

nlohmann::json entry_item(const std::vector<nlohmann::json> & records, const AppOfSource & app_of) {
  if (records.size() == 1) {
    return record_item(records.front(), app_of);
  }
  const auto & representative = representative_record(records);
  const nlohmann::json * most_severe = &records.front();
  const nlohmann::json * most_advanced = &records.front();
  std::set<std::string> sources;
  uint64_t occurrences = 0;
  std::optional<double> first_occurred;
  std::optional<double> last_occurred;
  std::optional<double> last_passed;
  nlohmann::json refs = nlohmann::json::array();
  for (const auto & record : records) {
    if (record.value("severity", 0) > most_severe->value("severity", 0)) {
      most_severe = &record;
    }
    if (status_rank(record.value("status", std::string{})) >
        status_rank(most_advanced->value("status", std::string{}))) {
      most_advanced = &record;
    }
    occurrences += record.value("occurrence_count", uint64_t{0});
    for (const auto & source : record.value("reporting_sources", nlohmann::json::array())) {
      if (source.is_string()) {
        sources.insert(source.get<std::string>());
      }
    }
    if (record.contains("first_occurred")) {
      const double t = number_or(record, "first_occurred", 0.0);
      first_occurred = first_occurred ? std::min(*first_occurred, t) : t;
    }
    if (record.contains("last_occurred")) {
      const double t = number_or(record, "last_occurred", 0.0);
      last_occurred = last_occurred ? std::max(*last_occurred, t) : t;
    }
    if (record.contains("last_passed")) {
      const double t = number_or(record, "last_passed", 0.0);
      last_passed = last_passed ? std::max(*last_passed, t) : t;
    }
    refs.push_back(record_ref(record, app_of));
  }

  nlohmann::json item = nlohmann::json::object();
  item["fault_code"] = representative.value("fault_code", std::string{});
  item["description"] = representative.value("description", std::string{});
  item["severity"] = most_severe->value("severity", 0);
  item["severity_label"] = most_severe->value("severity_label", std::string{"UNKNOWN"});
  item["status"] = most_advanced->value("status", std::string{});
  item["occurrence_count"] = occurrences;
  item["reporting_sources"] = std::vector<std::string>(sources.begin(), sources.end());
  if (first_occurred) {
    item["first_occurred"] = *first_occurred;
  }
  if (last_occurred) {
    item["last_occurred"] = *last_occurred;
  }
  if (last_passed) {
    item["last_passed"] = *last_passed;
  }
  item["x-medkit"] = {{"records", std::move(refs)}};
  return item;
}

std::vector<std::vector<nlohmann::json>> group_by_code(const nlohmann::json & faults) {
  std::vector<std::vector<nlohmann::json>> groups;
  if (!faults.is_array()) {
    return groups;
  }
  std::unordered_map<std::string, size_t> index;
  for (const auto & fault : faults) {
    if (!fault.is_object()) {
      continue;
    }
    const auto [it, inserted] = index.emplace(fault.value("fault_code", std::string{}), groups.size());
    if (inserted) {
      groups.emplace_back();
    }
    groups[it->second].push_back(fault);
  }
  for (auto & group : groups) {
    std::stable_sort(group.begin(), group.end(), [](const nlohmann::json & a, const nlohmann::json & b) {
      return stored_owner(a) < stored_owner(b);
    });
  }
  return groups;
}

std::string AppSourceIndex::find_owner(const ThreadSafeEntityCache & cache, const std::string & owner) {
  if (owner.empty()) {
    return "";
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // Read before collecting: a mutation in between leaves a newer generation,
  // so the next call rebuilds instead of keeping a stale index.
  const auto generation = cache.generation();
  if (!built_ || generation != generation_) {
    sources_.clear();
    for (const auto & app : cache.get_apps()) {
      auto owned = app_owned_source(app);
      if (!owned.empty()) {
        sources_.emplace_back(std::move(owned), app.id);
      }
    }
    generation_ = generation;
    built_ = true;
  }

  const std::pair<std::string, std::string> * best = nullptr;
  for (const auto & entry : sources_) {
    if (!source_in_scope(owner, entry.first)) {
      continue;
    }
    if (best == nullptr || entry.first.size() > best->first.size() ||
        (entry.first.size() == best->first.size() && entry.second < best->second)) {
      best = &entry;
    }
  }
  return best != nullptr ? best->second : "";
}

}  // namespace faults
}  // namespace ros2_medkit_gateway
