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

#include "ros2_medkit_gateway/core/logs/log_scope.hpp"

#include <algorithm>
#include <cstdlib>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {
namespace logs {

namespace {

using json = nlohmann::json;

// Entry ids are "log_<n>" with a monotonic n (LogManager::entry_to_json).
unsigned long long entry_id_num(const json & item) {
  const std::string id = item.value("id", "");
  const std::size_t off = id.rfind("log_", 0) == 0 ? 4 : 0;
  return std::strtoull(id.c_str() + off, nullptr, 10);
}

}  // namespace

LogScope resolve_log_scope(const ThreadSafeEntityCache & cache, SovdEntityType type, const std::string & entity_id,
                           const std::string & entity_fqn, std::set<std::string> * resolved_sources) {
  auto sources = faults::resolve_entity_source_fqns(cache, type, entity_id);
  if (resolved_sources) {
    *resolved_sources = sources;
  }

  LogScope scope;
  scope.exact.assign(sources.begin(), sources.end());
  // Legacy APP fallback: an app that is not in the cache (or resolves to no
  // source) keeps the entity.fqn exact query it always had.
  if (type == SovdEntityType::APP && scope.exact.empty() && !entity_fqn.empty()) {
    scope.exact.push_back(entity_fqn);
  }
  // Functions are pure aggregated views - no namespace fallback. APP has a
  // single source. Only grouping entities own a namespace prefix.
  if (type == SovdEntityType::COMPONENT || type == SovdEntityType::AREA) {
    scope.prefix = entity_fqn;
  }
  return scope;
}

bool log_scope_contains(const LogScope & scope, const std::string & logger_name) {
  for (const auto & fqn : scope.exact) {
    if (LogManager::logger_matches(logger_name, fqn, false)) {
      return true;
    }
  }
  return !scope.prefix.empty() && LogManager::logger_matches(logger_name, scope.prefix, true);
}

std::string entity_log_fqn(const ThreadSafeEntityCache & cache, SovdEntityType type, const std::string & entity_id) {
  switch (type) {
    case SovdEntityType::COMPONENT:
      if (auto comp = cache.get_component(entity_id)) {
        return comp->fqn;
      }
      break;
    case SovdEntityType::AREA:
      if (auto area = cache.get_area(entity_id)) {
        return area->namespace_path;
      }
      break;
    case SovdEntityType::APP:
      if (auto app = cache.get_app(entity_id)) {
        return app->effective_fqn();
      }
      break;
    case SovdEntityType::FUNCTION:
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      break;
  }
  return "";
}

std::optional<std::vector<json>> restore_replay_entries(LogManager & log_mgr, const ThreadSafeEntityCache & cache,
                                                        SovdEntityType type, const std::string & entity_id) {
  if (!cache.has_entity(type, entity_id)) {
    return std::nullopt;
  }
  std::unordered_set<std::string> markers;
  auto listed = query_entity_logs(log_mgr, cache, type, entity_id, entity_log_fqn(cache, type, entity_id), "", "",
                                  nullptr, &markers);
  if (!listed || !listed->is_array()) {
    return std::nullopt;
  }
  std::vector<json> entries;
  for (const auto & item : *listed) {
    if (markers.count(item.value("id", "")) == 0) {
      entries.push_back(item);
    }
  }
  return entries;
}

std::string LoggerOwnerIndex::find_owner(const ThreadSafeEntityCache & cache, const std::string & logger_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Read before collecting: a mutation in between leaves a newer generation,
  // so the next call rebuilds.
  const auto generation = cache.generation();
  if (!built_ || generation != generation_) {
    owners_.clear();
    for (const auto & app : cache.get_apps()) {
      // An app's log scope is its one source, matched exactly in either form.
      const auto source = LogManager::normalize_fqn(faults::resolve_app_source_fqn(cache, app.id));
      if (source.empty()) {
        continue;
      }
      std::string dotted = source;
      std::replace(dotted.begin(), dotted.end(), '/', '.');
      for (const auto & form : {source, dotted}) {
        auto [it, inserted] = owners_.emplace(form, app.id);
        if (!inserted && it->second != app.id) {
          it->second.clear();
        }
      }
    }
    generation_ = generation;
    built_ = true;
  }
  auto it = owners_.find(logger_name);
  return it != owners_.end() ? it->second : "";
}

bool entity_lists_log_entry(const LogManager & log_mgr, const ThreadSafeEntityCache & cache, SovdEntityType type,
                            const std::string & entity_id, const std::string & logger_name, uint8_t level) {
  const auto scope = resolve_log_scope(cache, type, entity_id, entity_log_fqn(cache, type, entity_id));
  if (!log_scope_contains(scope, logger_name)) {
    return false;
  }
  // get_logs() drops entries below the entity's severity_filter.
  auto cfg = log_mgr.get_config(entity_id);
  return cfg && level >= LogManager::severity_to_level(cfg->severity_filter);
}

bool entity_lists_log_change(const LogManager & log_mgr, const ThreadSafeEntityCache & cache, SovdEntityType type,
                             const std::string & entity_id, const json & entry) {
  if (!entry.is_object()) {
    return false;
  }
  const auto ctx = entry.find("context");
  if (ctx == entry.end() || !ctx->is_object()) {
    return false;
  }
  const auto node = ctx->find("node");
  if (node == ctx->end() || !node->is_string()) {
    return false;
  }
  return entity_lists_log_entry(log_mgr, cache, type, entity_id, node->get<std::string>(), log_mgr.entry_level(entry));
}

tl::expected<json, std::string> query_entity_logs(LogManager & log_mgr, const ThreadSafeEntityCache & cache,
                                                  SovdEntityType type, const std::string & entity_id,
                                                  const std::string & entity_fqn, const std::string & min_severity,
                                                  const std::string & context_filter,
                                                  std::set<std::string> * resolved_sources,
                                                  std::unordered_set<std::string> * marker_ids) {
  const auto scope = resolve_log_scope(cache, type, entity_id, entity_fqn, resolved_sources);

  json merged = json::array();
  if (!scope.exact.empty()) {
    auto result = log_mgr.get_logs(scope.exact, false, min_severity, context_filter, entity_id, marker_ids);
    if (!result) {
      return tl::make_unexpected(result.error());
    }
    merged = std::move(*result);
  }

  if (!scope.prefix.empty()) {
    auto result = log_mgr.get_logs({scope.prefix}, true, min_severity, context_filter, entity_id, marker_ids);
    if (!result) {
      return tl::make_unexpected(result.error());
    }
    // Dedupe: an app FQN under the entity namespace is hit by both queries.
    std::unordered_set<std::string> seen;
    for (const auto & item : merged) {
      seen.insert(item.value("id", ""));
    }
    for (auto & item : *result) {
      if (seen.insert(item.value("id", "")).second) {
        merged.push_back(std::move(item));
      }
    }
    std::sort(merged.begin(), merged.end(), [](const json & a, const json & b) {
      return entry_id_num(a) < entry_id_num(b);
    });
    // Each get_logs call caps individually; re-cap the union to the entity
    // budget, keeping the most recent entries.
    auto cfg = log_mgr.get_config(entity_id);
    if (cfg && merged.size() > cfg->max_entries) {
      merged.erase(merged.begin(), merged.begin() + static_cast<std::ptrdiff_t>(merged.size() - cfg->max_entries));
    }
  }

  return merged;
}

}  // namespace logs
}  // namespace ros2_medkit_gateway
