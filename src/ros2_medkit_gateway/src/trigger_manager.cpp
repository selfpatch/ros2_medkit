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

#include "ros2_medkit_gateway/trigger_manager.hpp"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

namespace ros2_medkit_gateway {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

TriggerManager::TriggerManager(ResourceChangeNotifier & notifier, ConditionRegistry & conditions, TriggerStore & store,
                               const TriggerConfig & config)
  : notifier_(notifier), conditions_(conditions), store_(store), config_(config) {
  // Subscribe to all resource changes (empty filter = catch-all)
  NotifierFilter filter;
  notifier_sub_id_ = notifier_.subscribe(filter, [this](const ResourceChange & change) {
    on_resource_change(change);
  });
}

TriggerManager::~TriggerManager() {
  shutdown();
}

void TriggerManager::shutdown() {
  if (shutdown_flag_.exchange(true)) {
    return;  // Already shut down
  }

  notifier_.unsubscribe(notifier_sub_id_);

  std::lock_guard<std::mutex> lock(triggers_mutex_);
  for (auto & [id, state] : triggers_) {
    state->active.store(false);
    state->cv.notify_all();
  }
}

// ---------------------------------------------------------------------------
// ID generation
// ---------------------------------------------------------------------------

std::string TriggerManager::generate_id() {
  uint64_t id = next_id_.fetch_add(1);
  return "trig_" + std::to_string(id);
}

// ---------------------------------------------------------------------------
// Dispatch index helpers
// ---------------------------------------------------------------------------

std::string TriggerManager::dispatch_key(const std::string & collection, const std::string & entity_id) {
  return collection + ":" + entity_id;
}

void TriggerManager::add_to_dispatch_index(const std::string & trigger_id, const std::string & collection,
                                           const std::string & entity_id) {
  // Exact match key
  dispatch_index_[dispatch_key(collection, entity_id)].insert(trigger_id);
  // Catch-all key (for hierarchy triggers)
  dispatch_index_[dispatch_key(collection, "")].insert(trigger_id);
}

void TriggerManager::remove_from_dispatch_index(const std::string & trigger_id, const std::string & collection,
                                                const std::string & entity_id) {
  auto remove_from = [&](const std::string & key) {
    auto it = dispatch_index_.find(key);
    if (it != dispatch_index_.end()) {
      it->second.erase(trigger_id);
      if (it->second.empty()) {
        dispatch_index_.erase(it);
      }
    }
  };
  remove_from(dispatch_key(collection, entity_id));
  remove_from(dispatch_key(collection, ""));
}

// ---------------------------------------------------------------------------
// Hierarchy matching
// ---------------------------------------------------------------------------

void TriggerManager::set_entity_children_fn(EntityChildrenFn fn) {
  std::lock_guard<std::mutex> lock(hierarchy_mutex_);
  entity_children_fn_ = std::move(fn);
}

bool TriggerManager::matches_entity(const TriggerState & state, const std::string & notification_entity_id) const {
  // Direct match
  if (state.info.entity_id == notification_entity_id) {
    return true;
  }

  // Hierarchy match: check if notification entity is a descendant
  std::lock_guard<std::mutex> lock(hierarchy_mutex_);
  if (!entity_children_fn_) {
    return false;
  }

  auto children = entity_children_fn_(state.info.entity_id, state.info.entity_type);
  return std::find(children.begin(), children.end(), notification_entity_id) != children.end();
}

// ---------------------------------------------------------------------------
// Expiry check
// ---------------------------------------------------------------------------

bool TriggerManager::is_expired(const TriggerState & state) const {
  if (!state.info.expires_at.has_value()) {
    return false;
  }
  return std::chrono::system_clock::now() >= state.info.expires_at.value();
}

// ---------------------------------------------------------------------------
// ISO 8601 formatting
// ---------------------------------------------------------------------------

std::string TriggerManager::to_iso8601(const std::chrono::system_clock::time_point & tp) {
  auto time_t_val = std::chrono::system_clock::to_time_t(tp);
  std::tm tm_val{};
  gmtime_r(&time_t_val, &tm_val);

  std::ostringstream oss;
  oss << std::put_time(&tm_val, "%Y-%m-%dT%H:%M:%S") << "Z";
  return oss.str();
}

// ---------------------------------------------------------------------------
// CRUD
// ---------------------------------------------------------------------------

tl::expected<TriggerInfo, std::string> TriggerManager::create(const TriggerCreateRequest & req) {
  // Validate condition type
  auto evaluator = conditions_.get(req.condition_type);
  if (!evaluator) {
    return tl::make_unexpected("Unknown condition type: " + req.condition_type);
  }

  // Validate condition params
  auto validation = evaluator->validate_params(req.condition_params);
  if (!validation.has_value()) {
    return tl::make_unexpected(validation.error());
  }

  std::lock_guard<std::mutex> lock(triggers_mutex_);

  // Check capacity
  if (triggers_.size() >= static_cast<size_t>(config_.max_triggers)) {
    return tl::make_unexpected("Maximum trigger capacity (" + std::to_string(config_.max_triggers) + ") reached");
  }

  auto now = std::chrono::system_clock::now();

  // Build TriggerInfo
  auto state = std::make_shared<TriggerState>();
  state->info.id = generate_id();
  state->info.entity_id = req.entity_id;
  state->info.entity_type = req.entity_type;
  state->info.resource_uri = req.resource_uri;
  state->info.collection = req.collection;
  state->info.resource_path = req.resource_path;
  state->info.path = req.path;
  state->info.condition_type = req.condition_type;
  state->info.condition_params = req.condition_params;
  state->info.protocol = req.protocol;
  state->info.multishot = req.multishot;
  state->info.persistent = req.persistent;
  state->info.lifetime_sec = req.lifetime_sec;
  state->info.log_settings = req.log_settings;
  state->info.status = TriggerStatus::ACTIVE;
  state->info.created_at = now;

  if (req.lifetime_sec.has_value()) {
    state->info.expires_at = now + std::chrono::seconds(req.lifetime_sec.value());
  }

  // Persist if requested
  if (req.persistent) {
    auto save_result = store_.save(state->info);
    if (!save_result.has_value()) {
      return tl::make_unexpected("Failed to persist trigger: " + save_result.error());
    }
  }

  // TODO(Task 10): log_settings integration with LogManager

  auto info_copy = state->info;
  add_to_dispatch_index(state->info.id, state->info.collection, state->info.entity_id);
  triggers_[state->info.id] = std::move(state);

  return info_copy;
}

std::optional<TriggerInfo> TriggerManager::get(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return std::nullopt;
    }
    state = it->second;
  }
  std::lock_guard<std::mutex> sub_lock(state->mtx);
  return state->info;
}

std::vector<TriggerInfo> TriggerManager::list(const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(triggers_mutex_);
  std::vector<TriggerInfo> result;
  for (const auto & [id, state] : triggers_) {
    std::lock_guard<std::mutex> sub_lock(state->mtx);
    if (state->info.entity_id == entity_id) {
      result.push_back(state->info);
    }
  }
  return result;
}

tl::expected<TriggerInfo, std::string> TriggerManager::update(const std::string & trigger_id, int new_lifetime) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return tl::make_unexpected("Trigger not found: " + trigger_id);
    }
    state = it->second;
  }

  std::lock_guard<std::mutex> sub_lock(state->mtx);

  state->info.lifetime_sec = new_lifetime;
  state->info.expires_at = std::chrono::system_clock::now() + std::chrono::seconds(new_lifetime);

  // Persist if needed
  if (state->info.persistent) {
    nlohmann::json fields;
    fields["lifetime_sec"] = new_lifetime;
    fields["expires_at"] = to_iso8601(state->info.expires_at.value());
    (void)store_.update(trigger_id, fields);
  }

  return state->info;
}

bool TriggerManager::remove(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  std::string collection;
  std::string entity_id;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return false;
    }
    state = it->second;
    {
      std::lock_guard<std::mutex> sub_lock(state->mtx);
      collection = state->info.collection;
      entity_id = state->info.entity_id;
    }
    remove_from_dispatch_index(trigger_id, collection, entity_id);
    triggers_.erase(it);
  }

  // Mark inactive and wake any waiting SSE stream
  state->active.store(false);
  state->cv.notify_all();

  // Remove from persistent store if applicable
  {
    std::lock_guard<std::mutex> sub_lock(state->mtx);
    if (state->info.persistent) {
      (void)store_.remove(trigger_id);
    }
  }

  if (on_removed_) {
    on_removed_(trigger_id);
  }

  return true;
}

// ---------------------------------------------------------------------------
// SSE synchronization
// ---------------------------------------------------------------------------

bool TriggerManager::wait_for_event(const std::string & trigger_id, std::chrono::milliseconds timeout) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return true;  // Already removed - signal caller to stop
    }
    state = it->second;
  }

  std::unique_lock<std::mutex> sub_lock(state->mtx);
  bool woken = state->cv.wait_for(sub_lock, timeout, [&]() {
    return state->pending_event.has_value() || !state->active.load() || shutdown_flag_.load();
  });
  return woken;
}

bool TriggerManager::is_active(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return false;
    }
    state = it->second;
  }

  std::lock_guard<std::mutex> sub_lock(state->mtx);

  if (!state->active.load()) {
    return false;
  }

  if (state->info.status == TriggerStatus::TERMINATED) {
    return false;
  }

  // Check expiry
  if (is_expired(*state)) {
    state->info.status = TriggerStatus::TERMINATED;
    state->active.store(false);
    state->cv.notify_all();
    if (state->info.persistent) {
      nlohmann::json fields;
      fields["status"] = "TERMINATED";
      (void)store_.update(state->info.id, fields);
    }
    return false;
  }

  return true;
}

std::optional<nlohmann::json> TriggerManager::consume_pending_event(const std::string & trigger_id) {
  std::shared_ptr<TriggerState> state;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);
    auto it = triggers_.find(trigger_id);
    if (it == triggers_.end()) {
      return std::nullopt;
    }
    state = it->second;
  }

  std::lock_guard<std::mutex> sub_lock(state->mtx);
  if (!state->pending_event.has_value()) {
    return std::nullopt;
  }

  auto event = std::move(state->pending_event.value());
  state->pending_event.reset();
  return event;
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void TriggerManager::set_on_removed(OnRemovedCallback callback) {
  on_removed_ = std::move(callback);
}

// ---------------------------------------------------------------------------
// Persistent trigger loading
// ---------------------------------------------------------------------------

void TriggerManager::load_persistent_triggers() {
  if (config_.on_restart_behavior != "restore") {
    return;
  }

  auto load_result = store_.load_all();
  if (!load_result.has_value()) {
    return;
  }

  std::lock_guard<std::mutex> lock(triggers_mutex_);
  for (auto & info : load_result.value()) {
    if (info.status != TriggerStatus::ACTIVE) {
      continue;
    }

    // Check if expired
    if (info.expires_at.has_value() && std::chrono::system_clock::now() >= info.expires_at.value()) {
      nlohmann::json fields;
      fields["status"] = "TERMINATED";
      (void)store_.update(info.id, fields);
      continue;
    }

    auto state = std::make_shared<TriggerState>();
    state->info = std::move(info);

    // Restore previous value state if available
    auto state_result = store_.load_state(state->info.id);
    if (state_result.has_value() && state_result.value().has_value()) {
      state->previous_value = state_result.value().value();
      state->has_previous_value = true;
    }

    // Update next_id_ to avoid collisions
    // IDs are "trig_N" - extract N and ensure next_id_ is beyond it
    auto underscore_pos = state->info.id.find('_');
    if (underscore_pos != std::string::npos) {
      try {
        auto loaded_id = std::stoull(state->info.id.substr(underscore_pos + 1));
        uint64_t current = next_id_.load();
        while (current <= loaded_id && !next_id_.compare_exchange_weak(current, loaded_id + 1)) {
        }
      } catch (...) {
        // Non-numeric suffix - skip ID adjustment
      }
    }

    add_to_dispatch_index(state->info.id, state->info.collection, state->info.entity_id);
    triggers_[state->info.id] = std::move(state);
  }
}

// ---------------------------------------------------------------------------
// Resource change evaluation
// ---------------------------------------------------------------------------

void TriggerManager::on_resource_change(const ResourceChange & change) {
  if (shutdown_flag_.load()) {
    return;
  }

  // Collect candidate trigger IDs from the dispatch index
  std::vector<std::pair<std::string, std::shared_ptr<TriggerState>>> candidates;
  {
    std::lock_guard<std::mutex> lock(triggers_mutex_);

    // Gather trigger IDs from both exact match and catch-all keys
    std::unordered_set<std::string> candidate_ids;

    auto exact_key = dispatch_key(change.collection, change.entity_id);
    auto it_exact = dispatch_index_.find(exact_key);
    if (it_exact != dispatch_index_.end()) {
      candidate_ids.insert(it_exact->second.begin(), it_exact->second.end());
    }

    auto catchall_key = dispatch_key(change.collection, "");
    auto it_catchall = dispatch_index_.find(catchall_key);
    if (it_catchall != dispatch_index_.end()) {
      candidate_ids.insert(it_catchall->second.begin(), it_catchall->second.end());
    }

    // Resolve IDs to states
    for (const auto & tid : candidate_ids) {
      auto it = triggers_.find(tid);
      if (it != triggers_.end()) {
        candidates.emplace_back(tid, it->second);
      }
    }
  }

  // Evaluate each candidate trigger
  for (auto & [trigger_id, state] : candidates) {
    std::unique_lock<std::mutex> sub_lock(state->mtx);

    // Skip terminated or inactive triggers
    if (state->info.status != TriggerStatus::ACTIVE || !state->active.load()) {
      continue;
    }

    // Check expiry
    if (is_expired(*state)) {
      state->info.status = TriggerStatus::TERMINATED;
      state->active.store(false);
      sub_lock.unlock();
      state->cv.notify_all();
      if (state->info.persistent) {
        nlohmann::json fields;
        fields["status"] = "TERMINATED";
        (void)store_.update(trigger_id, fields);
      }
      continue;
    }

    // Check resource_path match (if trigger specifies a specific resource)
    if (!state->info.resource_path.empty() && state->info.resource_path != change.resource_path) {
      continue;
    }

    // Check entity hierarchy match
    // Release sub_lock before calling matches_entity (it acquires hierarchy_mutex_)
    sub_lock.unlock();

    if (!matches_entity(*state, change.entity_id)) {
      continue;
    }

    sub_lock.lock();

    // Re-check status after re-acquiring lock
    if (state->info.status != TriggerStatus::ACTIVE || !state->active.load()) {
      continue;
    }

    // Extract value via JSON Pointer
    nlohmann::json extracted_value;
    if (state->info.path.empty()) {
      extracted_value = change.value;
    } else {
      try {
        auto ptr = nlohmann::json::json_pointer(state->info.path);
        if (change.value.contains(ptr)) {
          extracted_value = change.value[ptr];
        } else {
          extracted_value = change.value;
        }
      } catch (...) {
        extracted_value = change.value;
      }
    }

    // Get the evaluator
    auto evaluator = conditions_.get(state->info.condition_type);
    if (!evaluator) {
      continue;
    }

    // Build optional previous value
    std::optional<nlohmann::json> prev_opt;
    if (state->has_previous_value) {
      prev_opt = state->previous_value;
    }

    // Evaluate condition
    bool fired = evaluator->evaluate(prev_opt, extracted_value, state->info.condition_params);

    // Update previous value
    state->previous_value = extracted_value;
    state->has_previous_value = true;

    // Persist state if configured for restore
    if (config_.on_restart_behavior == "restore" && state->info.persistent) {
      (void)store_.save_state(trigger_id, extracted_value);
    }

    if (fired) {
      // Build EventEnvelope
      nlohmann::json envelope;
      envelope["timestamp"] = to_iso8601(std::chrono::system_clock::now());
      envelope["payload"] = change.value;

      state->pending_event = std::move(envelope);

      if (!state->info.multishot) {
        state->info.status = TriggerStatus::TERMINATED;
        state->active.store(false);
        if (state->info.persistent) {
          nlohmann::json fields;
          fields["status"] = "TERMINATED";
          (void)store_.update(trigger_id, fields);
        }
      }

      sub_lock.unlock();
      state->cv.notify_all();
    }
  }
}

}  // namespace ros2_medkit_gateway
