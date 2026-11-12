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
#include <optional>
#include <set>
#include <string>
#include <tl/expected.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/models/entity_types.hpp"

namespace ros2_medkit_gateway {

class LogManager;
class ThreadSafeEntityCache;

namespace logs {

/// Log sources of one entity, as the log routes query them.
struct LogScope {
  /// Matched exactly (slash or dot form).
  std::vector<std::string> exact;
  /// Namespace prefix, COMPONENT and AREA only. Empty when there is none.
  std::string prefix;
};

/// Build the scope `query_entity_logs` queries. `resolved_sources` (optional
/// out) receives the exact-match source set.
LogScope resolve_log_scope(const ThreadSafeEntityCache & cache, SovdEntityType type, const std::string & entity_id,
                           const std::string & entity_fqn, std::set<std::string> * resolved_sources = nullptr);

/// True when entries of `logger_name` fall inside `scope`
/// (`LogManager::logger_matches` per source).
bool log_scope_contains(const LogScope & scope, const std::string & logger_name);

/// The fqn the log routes pass for an entity: component fqn, area namespace,
/// app effective fqn. Empty for FUNCTION and for an entity not in the cache.
std::string entity_log_fqn(const ThreadSafeEntityCache & cache, SovdEntityType type, const std::string & entity_id);

/// True when the log routes' rule admits an entry of `logger_name` at
/// `level`: the logger is in the entity's scope and `level` is at or above the
/// entity's configured severity_filter. With a LogProvider plugin, GET
/// returns the plugin's entries instead.
bool entity_lists_log_entry(const LogManager & log_mgr, const ThreadSafeEntityCache & cache, SovdEntityType type,
                            const std::string & entity_id, const std::string & logger_name, uint8_t level);

/// `entity_lists_log_entry` for the value of a "logs" change: the logger name
/// is `context.node`, the level is `LogManager::entry_level`. False for a value
/// that carries no logger name.
bool entity_lists_log_change(const LogManager & log_mgr, const ThreadSafeEntityCache & cache, SovdEntityType type,
                             const std::string & entity_id, const nlohmann::json & entry);

/// Entries a restored logs trigger on the entity replays: what
/// `GET /{entity}/logs` lists now, oldest first, less the trigger markers
/// (`LogManager::add_log_entry`). nullopt while the entity of that type is not
/// in the cache or the listing fails; the replay is then retried by the next
/// sweep or live change.
std::optional<std::vector<nlohmann::json>> restore_replay_entries(LogManager & log_mgr,
                                                                  const ThreadSafeEntityCache & cache,
                                                                  SovdEntityType type, const std::string & entity_id);

/// Loggers the log scope of each cached app holds, rebuilt when the cache
/// generation moves. Thread-safe.
class LoggerOwnerIndex {
 public:
  /// Id of the one app whose log scope holds `logger_name`, or "" when no app
  /// or more than one does.
  std::string find_owner(const ThreadSafeEntityCache & cache, const std::string & logger_name);

 private:
  std::mutex mutex_;
  bool built_{false};
  uint64_t generation_{0};
  /// Logger name in slash and dot form -> owning app id, "" when shared.
  std::unordered_map<std::string, std::string> owners_;
};

/// Query every log entry within an entity's scope.
///
/// Exact-match sources come from `faults::resolve_entity_source_fqns`, so the
/// scope rule matches faults and bulk-data: an external app stores entries
/// under its bare entity id, an external component also owns entries stored
/// under its own id, areas recurse subareas and functions follow component
/// hosts. COMPONENT and AREA additionally run `entity_fqn` as a namespace
/// prefix query - hosted-app resolution must not disable it, or a manifest
/// component that hosts an external app loses the ROS node entries under its
/// namespace. APP falls back to `entity_fqn` (exact) when the resolver yields
/// nothing. Both result sets are merged, deduped by entry id, sorted by id
/// ascending and re-capped to the entity's configured max_entries.
///
/// `resolved_sources` (optional out) receives the exact-match source set for
/// response metadata. `marker_ids` (optional out) receives the ids of the
/// listed trigger markers. Shared by the HTTP log handlers and the cyclic
/// subscription "logs" sampler so both agree on entity -> log-source scoping.
tl::expected<nlohmann::json, std::string>
query_entity_logs(LogManager & log_mgr, const ThreadSafeEntityCache & cache, SovdEntityType type,
                  const std::string & entity_id, const std::string & entity_fqn, const std::string & min_severity,
                  const std::string & context_filter, std::set<std::string> * resolved_sources = nullptr,
                  std::unordered_set<std::string> * marker_ids = nullptr);

}  // namespace logs
}  // namespace ros2_medkit_gateway
