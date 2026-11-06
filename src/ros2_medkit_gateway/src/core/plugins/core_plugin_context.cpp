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

#include "ros2_medkit_gateway/core/plugins/core_plugin_context.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/core/aggregation/peer_ids.hpp"
#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/entity_validation.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Map a SovdEntityType to the plural URL path segment used in the REST API.
/// Returns empty string for SERVER/UNKNOWN (no faults collection on those).
std::string entity_type_to_url_segment(SovdEntityType type) {
  switch (type) {
    case SovdEntityType::APP:
      return "apps";
    case SovdEntityType::COMPONENT:
      return "components";
    case SovdEntityType::AREA:
      return "areas";
    case SovdEntityType::FUNCTION:
      return "functions";
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      return "";
  }
  return "";
}

/// The peers the entity of `type` named `id` names among its contributors,
/// read off the one copy of the entity found, so they come from the same cache
/// generation, and a peer that contributed some other entity sharing the id is
/// not among them. Nullopt when no entity of `type` holds the id.
std::optional<std::vector<PeerTarget>> find_entity_peers(const ThreadSafeEntityCache & cache, SovdEntityType type,
                                                         const std::string & id) {
  switch (type) {
    case SovdEntityType::COMPONENT:
      if (auto comp = cache.get_component(id)) {
        return peer_targets_of(*comp);
      }
      break;
    case SovdEntityType::APP:
      if (auto app = cache.get_app(id)) {
        return peer_targets_of(*app);
      }
      break;
    case SovdEntityType::AREA:
      if (auto area = cache.get_area(id)) {
        return peer_targets_of(*area);
      }
      break;
    case SovdEntityType::FUNCTION:
      if (auto func = cache.get_function(id)) {
        return peer_targets_of(*func);
      }
      break;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
    default:
      break;
  }
  return std::nullopt;
}

/// "Component, App" for the types `holders` lists.
std::string join_types(const std::vector<SovdEntityType> & holders) {
  std::string out;
  for (auto type : holders) {
    out += out.empty() ? to_string(type) : ", " + to_string(type);
  }
  return out;
}

}  // namespace

CorePluginContext::CorePluginContext(GatewayServices * services, CorePluginContextWiring wiring)
  : services_(services), wiring_(std::move(wiring)) {
}

rclcpp::Node * CorePluginContext::node() const {
  return nullptr;
}

std::vector<SovdEntityType> CorePluginContext::holders_of(const std::string & id) const {
  std::vector<SovdEntityType> holders;
  if (!services_) {
    return holders;
  }
  const auto & cache = services_->entity_cache();
  for (auto type : {SovdEntityType::COMPONENT, SovdEntityType::APP, SovdEntityType::AREA, SovdEntityType::FUNCTION}) {
    if (cache.has_entity(type, id)) {
      holders.push_back(type);
    }
  }
  return holders;
}

std::optional<SovdEntityType> CorePluginContext::resolve_bare_id(const std::string & id, const char * call,
                                                                 std::vector<SovdEntityType> & holders) const {
  holders = holders_of(id);
  bool newly_ambiguous = false;
  {
    std::lock_guard<std::mutex> lock(ambiguity_mutex_);
    if (holders.size() > 1u) {
      newly_ambiguous = ambiguous_ids_logged_.insert(id).second;
    } else {
      ambiguous_ids_logged_.erase(id);
    }
  }
  if (newly_ambiguous && wiring_.warn) {
    wiring_.warn(std::string(call) + "('" + id + "'): the id is held by " + join_types(holders) +
                 "; an id-only plugin call resolves only an id one entity holds, so it is refused - use the typed " +
                 "call, which names the entity type");
  }
  if (holders.size() == 1u) {
    return holders.front();
  }
  return std::nullopt;
}

std::optional<PluginEntityInfo> CorePluginContext::get_entity(const std::string & id) const {
  std::vector<SovdEntityType> holders;
  auto type = resolve_bare_id(id, "get_entity", holders);
  if (!type) {
    return std::nullopt;
  }
  return get_entity(*type, id);
}

std::optional<PluginEntityInfo> CorePluginContext::get_entity_typed(SovdEntityType type, const std::string & id) const {
  if (!services_) {
    return std::nullopt;
  }
  const auto & cache = services_->entity_cache();
  switch (type) {
    case SovdEntityType::COMPONENT:
      if (auto comp = cache.get_component(id)) {
        return PluginEntityInfo{SovdEntityType::COMPONENT, id, comp->namespace_path, comp->fqn};
      }
      break;
    case SovdEntityType::APP:
      if (auto app = cache.get_app(id)) {
        return PluginEntityInfo{SovdEntityType::APP, id, {}, app->effective_fqn()};
      }
      break;
    case SovdEntityType::AREA:
      if (auto area = cache.get_area(id)) {
        return PluginEntityInfo{SovdEntityType::AREA, id, area->namespace_path, {}};
      }
      break;
    case SovdEntityType::FUNCTION:
      if (cache.get_function(id)) {
        return PluginEntityInfo{SovdEntityType::FUNCTION, id, {}, {}};
      }
      break;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
    default:
      break;
  }
  return std::nullopt;
}

std::vector<PluginEntityInfo> CorePluginContext::get_child_apps(const std::string & component_id) const {
  std::vector<PluginEntityInfo> result;
  if (!services_) {
    return result;
  }
  const auto & cache = services_->entity_cache();
  auto app_ids = cache.get_apps_for_component(component_id);
  for (const auto & app_id : app_ids) {
    if (auto app = cache.get_app(app_id)) {
      result.push_back(PluginEntityInfo{SovdEntityType::APP, app_id,
                                        "",  // namespace_path not needed for Apps
                                        app->effective_fqn()});
    }
  }
  return result;
}

nlohmann::json CorePluginContext::list_entity_faults(const std::string & entity_id) const {
  std::vector<SovdEntityType> holders;
  auto type = resolve_bare_id(entity_id, "list_entity_faults", holders);
  if (type) {
    return list_entity_faults(*type, entity_id);
  }
  if (holders.size() > 1u) {
    nlohmann::json types = nlohmann::json::array();
    for (auto holder : holders) {
      types.push_back(to_string(holder));
    }
    return nlohmann::json{
        {"error_code", ERR_INVALID_PARAMETER},
        {"message", "Entity id '" + entity_id + "' is held by " + join_types(holders) + "; name the entity type"},
        {"entity_types", std::move(types)}};
  }
  return nlohmann::json::array();
}

nlohmann::json CorePluginContext::list_entity_faults_typed(SovdEntityType entity_type,
                                                           const std::string & entity_id) const {
  if (!services_) {
    return nlohmann::json::array();
  }
  auto entity_peers = find_entity_peers(services_->entity_cache(), entity_type, entity_id);
  if (!entity_peers) {
    return nlohmann::json::array();
  }

  nlohmann::json local_faults = nlohmann::json::array();

  if (wiring_.fault_manager && wiring_.fault_manager->is_available()) {
    // Mirror the HTTP per-entity fault handler: fetch all faults, then scope
    // them to the entity's resolved App-FQN set (APP -> itself, COMPONENT ->
    // hosted apps, AREA -> descendant apps, FUNCTION -> host apps) via the
    // shared core helper. The fault manager returns a {"faults": [...]}
    // object; this method's contract is a bare array, so extract and filter
    // the inner array.
    auto result = wiring_.fault_manager->list_faults("");
    if (result.success && result.data.contains("faults") && result.data["faults"].is_array()) {
      const auto & cache = services_->entity_cache();
      const auto source_fqns = faults::resolve_entity_source_fqns(cache, entity_type, entity_id);
      local_faults = faults::filter_faults_by_sources(result.data["faults"], source_fqns);
    }
  }

  // Fan-out to peer gateways: fetch faults for this entity from the peers that
  // contributed it, and from no other, as the HTTP per-entity route does. A
  // local-only entity asks no peer at all. The peer path mirrors the HTTP REST
  // route: /api/v1/<type_plural>/<id>/faults, with the id the peer knows the
  // entity by. That id comes from the rename the merge recorded on the entity
  // (`original_id`), never from the spelling of the served id: a peer's own
  // entity whose id merely starts with `<peer>__` is asked for under that id.
  // Auth: the ROS 2 service path has no inbound HTTP Authorization header to
  // forward. Each peer is asked with this gateway's own credential
  // (aggregation.peer_auth_header) when one is configured; without one the
  // request carries none, and a peer that gates its /faults route contributes
  // nothing.
  auto * agg = wiring_.aggregation ? wiring_.aggregation() : nullptr;
  if (agg) {
    const std::string type_segment = entity_type_to_url_segment(entity_type);
    std::map<std::string, std::vector<std::string>> peers_by_path;
    if (!type_segment.empty()) {
      for (auto & target : *entity_peers) {
        std::string peer_path = "/api/v1/";
        peer_path += type_segment;
        peer_path += '/';
        peer_path += target.id;
        peer_path += "/faults";
        peers_by_path[peer_path].push_back(std::move(target.peer));
      }
    }
    for (const auto & [peer_path, peers] : peers_by_path) {
      auto fan_result = agg->fan_out_get(peer_path, "", &peers);
      if (fan_result.merged_items.is_array()) {
        for (const auto & item : fan_result.merged_items) {
          local_faults.push_back(item);
        }
      }
      if (fan_result.is_partial) {
        if (wiring_.warn) {
          wiring_.warn("list_entity_faults('" + entity_id + "'): partial peer fan-out, " +
                       std::to_string(fan_result.failed_peers.size()) + " peer(s) failed");
        }
      }
    }
  }

  return local_faults;
}

std::optional<PluginEntityInfo> CorePluginContext::validate_entity_for_route(const PluginRequest & req,
                                                                             PluginResponse & res,
                                                                             const std::string & entity_id) const {
  // Validate entity ID format (shared validation with HandlerContext)
  if (auto err = validate_entity_id(entity_id); !err) {
    res.send_error(400, ERR_INVALID_PARAMETER, err.error());
    return std::nullopt;
  }

  // Determine expected type from route path (segment-boundary-aware matching).
  // The entity is the route's type: an App and a Component may share an id.
  auto expected_type = extract_entity_type_from_path(req.path());
  if (expected_type != SovdEntityType::UNKNOWN) {
    if (auto entity = get_entity(expected_type, entity_id)) {
      return entity;
    }
  }

  const auto holders = holders_of(entity_id);
  if (holders.empty()) {
    res.send_error(404, ERR_ENTITY_NOT_FOUND, to_string(expected_type) + " not found: " + entity_id);
    return std::nullopt;
  }
  if (expected_type == SovdEntityType::UNKNOWN && holders.size() == 1u) {
    return get_entity(holders.front(), entity_id);
  }
  res.send_error(400, ERR_INVALID_PARAMETER,
                 "Entity '" + entity_id + "' is a " + join_types(holders) + ", not a " + to_string(expected_type));
  return std::nullopt;
}

void CorePluginContext::register_capability(SovdEntityType entity_type, const std::string & capability_name) {
  std::lock_guard<std::mutex> lock(capabilities_mutex_);
  auto & caps = type_capabilities_[entity_type];
  // Avoid duplicates
  for (const auto & c : caps) {
    if (c == capability_name) {
      return;
    }
  }
  caps.push_back(capability_name);
}

namespace {

void add_unique(std::vector<std::string> & caps, const std::string & capability_name) {
  if (std::find(caps.begin(), caps.end(), capability_name) == caps.end()) {
    caps.push_back(capability_name);
  }
}

}  // namespace

void CorePluginContext::register_entity_capability(const std::string & entity_id, const std::string & capability_name) {
  // Resolved when looked up, not now: a plugin registers from introspect(),
  // before the entity it introspects reaches the cache.
  std::lock_guard<std::mutex> lock(capabilities_mutex_);
  add_unique(untyped_entity_capabilities_[entity_id], capability_name);
}

void CorePluginContext::register_entity_capability_typed(SovdEntityType entity_type, const std::string & entity_id,
                                                         const std::string & capability_name) {
  std::lock_guard<std::mutex> lock(capabilities_mutex_);
  add_unique(entity_capabilities_[MemberRef{entity_type, entity_id}], capability_name);
}

std::vector<std::string> CorePluginContext::get_type_capabilities(SovdEntityType entity_type) const {
  std::lock_guard<std::mutex> lock(capabilities_mutex_);
  auto it = type_capabilities_.find(entity_type);
  if (it != type_capabilities_.end()) {
    return it->second;
  }
  return {};
}

std::vector<std::string> CorePluginContext::get_entity_capabilities(const std::string & entity_id) const {
  std::vector<SovdEntityType> holders;
  auto type = resolve_bare_id(entity_id, "get_entity_capabilities", holders);
  if (!type) {
    return {};
  }
  return get_entity_capabilities(*type, entity_id);
}

std::vector<std::string> CorePluginContext::get_entity_capabilities_typed(SovdEntityType entity_type,
                                                                          const std::string & entity_id) const {
  std::vector<std::string> caps;
  bool has_untyped = false;
  {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto typed = entity_capabilities_.find(MemberRef{entity_type, entity_id});
    if (typed != entity_capabilities_.end()) {
      caps = typed->second;
    }
    has_untyped = untyped_entity_capabilities_.count(entity_id) > 0u;
  }
  if (!has_untyped) {
    return caps;
  }
  // An id-only registration names the one entity holding the id; while more
  // than one entity holds it, it applies to none.
  std::vector<SovdEntityType> holders;
  auto holder = resolve_bare_id(entity_id, "register_entity_capability", holders);
  if (holder != entity_type) {
    return caps;
  }
  std::lock_guard<std::mutex> lock(capabilities_mutex_);
  auto untyped = untyped_entity_capabilities_.find(entity_id);
  if (untyped != untyped_entity_capabilities_.end()) {
    for (const auto & cap : untyped->second) {
      add_unique(caps, cap);
    }
  }
  return caps;
}

namespace {

/// Why an id-only lock call was refused: `id` is held by more than one entity.
std::string ambiguous_lock_message(const std::string & id, const std::vector<SovdEntityType> & holders) {
  return "Entity id '" + id + "' is held by " + join_types(holders) + "; name the entity type";
}

/// The types a lock can be kept by: the entity types the lock routes serve.
bool is_lockable_type(SovdEntityType type) {
  return type == SovdEntityType::AREA || type == SovdEntityType::COMPONENT || type == SovdEntityType::APP ||
         type == SovdEntityType::FUNCTION;
}

/// What the lock routes answer for an entity that does not exist. A lock is kept by the
/// entity's type and id, so none is taken or released for an entity no type names.
LockError entity_not_found() {
  return LockError{ERR_ENTITY_NOT_FOUND, "Entity not found", 404, std::nullopt};
}

}  // namespace

LockAccessResult CorePluginContext::check_lock(const std::string & entity_id, const std::string & client_id,
                                               const std::string & collection) const {
  std::vector<SovdEntityType> holders;
  auto type = resolve_bare_id(entity_id, "check_lock", holders);
  if (holders.size() > 1u) {
    return LockAccessResult{false, "", ambiguous_lock_message(entity_id, holders), "invalid-parameter"};
  }
  if (!type) {
    return LockAccessResult{true, "", "", ""};  // No entity holds the id, so nobody has locked it.
  }
  return check_lock(*type, entity_id, client_id, collection);
}

tl::expected<LockInfo, LockError> CorePluginContext::acquire_lock(const std::string & entity_id,
                                                                  const std::string & client_id,
                                                                  const std::vector<std::string> & scopes,
                                                                  int expiration_seconds) {
  std::vector<SovdEntityType> holders;
  auto type = resolve_bare_id(entity_id, "acquire_lock", holders);
  if (holders.size() > 1u) {
    return tl::make_unexpected(
        LockError{"invalid-parameter", ambiguous_lock_message(entity_id, holders), 400, std::nullopt});
  }
  if (!type) {
    return tl::make_unexpected(entity_not_found());
  }
  return acquire_lock(*type, entity_id, client_id, scopes, expiration_seconds);
}

tl::expected<void, LockError> CorePluginContext::release_lock(const std::string & entity_id,
                                                              const std::string & client_id) {
  std::vector<SovdEntityType> holders;
  auto type = resolve_bare_id(entity_id, "release_lock", holders);
  if (holders.size() > 1u) {
    return tl::make_unexpected(
        LockError{"invalid-parameter", ambiguous_lock_message(entity_id, holders), 400, std::nullopt});
  }
  if (!type) {
    return tl::make_unexpected(entity_not_found());
  }
  return release_lock(*type, entity_id, client_id);
}

LockAccessResult CorePluginContext::check_lock_typed(SovdEntityType entity_type, const std::string & entity_id,
                                                     const std::string & client_id,
                                                     const std::string & collection) const {
  auto * lock_mgr = services_ ? services_->lock_manager() : nullptr;
  if (!lock_mgr || !is_lockable_type(entity_type)) {
    return LockAccessResult{true, "", "", ""};
  }
  return lock_mgr->check_access(entity_type, entity_id, client_id, collection);
}

tl::expected<LockInfo, LockError> CorePluginContext::acquire_lock_typed(SovdEntityType entity_type,
                                                                        const std::string & entity_id,
                                                                        const std::string & client_id,
                                                                        const std::vector<std::string> & scopes,
                                                                        int expiration_seconds) {
  auto * lock_mgr = services_ ? services_->lock_manager() : nullptr;
  if (!lock_mgr) {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking is not enabled", 503, std::nullopt});
  }
  if (!is_lockable_type(entity_type)) {
    return tl::make_unexpected(entity_not_found());
  }
  return lock_mgr->acquire(entity_type, entity_id, client_id, scopes, expiration_seconds);
}

tl::expected<void, LockError> CorePluginContext::release_lock_typed(SovdEntityType entity_type,
                                                                    const std::string & entity_id,
                                                                    const std::string & client_id) {
  auto * lock_mgr = services_ ? services_->lock_manager() : nullptr;
  if (!lock_mgr) {
    return tl::make_unexpected(LockError{"lock-disabled", "Locking is not enabled", 503, std::nullopt});
  }
  if (!is_lockable_type(entity_type)) {
    return tl::make_unexpected(entity_not_found());
  }
  return lock_mgr->release(entity_type, entity_id, client_id);
}

IntrospectionInput CorePluginContext::get_entity_snapshot() const {
  IntrospectionInput input;
  if (!services_) {
    return input;
  }
  const auto & cache = services_->entity_cache();
  input.areas = cache.get_areas();
  input.components = cache.get_components();
  input.apps = cache.get_apps();
  input.functions = cache.get_functions();
  return input;
}

nlohmann::json CorePluginContext::list_all_faults() const {
  nlohmann::json response = nlohmann::json::object();

  if (wiring_.fault_manager && wiring_.fault_manager->is_available()) {
    auto result = wiring_.fault_manager->list_faults("");
    if (result.success) {
      response = result.data;
    }
  }

  // Fan-out to peer gateways: fetch all faults from /api/v1/faults on each peer.
  // Auth: see list_entity_faults - the ROS 2 service path forwards no inbound
  // Authorization header, so peers are queried with an empty credential.
  // The visited path is what lets a peer ask its own peers, so the list
  // reaches past the first hop without coming back here.
  auto * agg = wiring_.aggregation ? wiring_.aggregation() : nullptr;
  if (agg) {
    auto fan_result = agg->fan_out_get("/api/v1/faults", "", nullptr, {{kFanOutVisitedHeader, agg->instance_id()}});
    if (fan_result.merged_items.is_array()) {
      // Ensure local response has a "faults" array to append to.
      if (!response.contains("faults") || !response["faults"].is_array()) {
        response["faults"] = nlohmann::json::array();
      }
      for (const auto & item : fan_result.merged_items) {
        response["faults"].push_back(item);
      }
    }
    if (fan_result.is_partial) {
      if (wiring_.warn) {
        wiring_.warn("list_all_faults: partial peer fan-out, " + std::to_string(fan_result.failed_peers.size()) +
                     " peer(s) failed");
      }
    }
  }

  // "count" comes from the local-only query; after merging peer faults it
  // would undercount, so keep it consistent with the merged array.
  if (response.contains("count") && response.contains("faults") && response["faults"].is_array()) {
    response["count"] = response["faults"].size();
  }

  return response;
}

/// A plugin sampler is sampled with no way of saying that it narrows its
/// payload to the resource named by its second argument, so it is registered
/// as streaming its collection whole. A subscription URI naming a single item
/// of that collection is then refused rather than answered with everything.
void CorePluginContext::register_sampler(const std::string & collection,
                                         const std::function<tl::expected<nlohmann::json, std::string>(
                                             SovdEntityType, const std::string &, const std::string &)> & fn) {
  if (wiring_.sampler_registry) {
    wiring_.sampler_registry->register_sampler(collection, fn, /*is_builtin=*/false, /*honours_resource_path=*/false);
  }
}

ResourceChangeNotifier * CorePluginContext::get_resource_change_notifier() {
  return wiring_.notifier ? wiring_.notifier() : nullptr;
}

ConditionRegistry * CorePluginContext::get_condition_registry() {
  return wiring_.conditions ? wiring_.conditions() : nullptr;
}

void CorePluginContext::notify_entities_changed(const EntityChangeScope & scope) {
  if (wiring_.on_entities_changed) {
    wiring_.on_entities_changed(scope);
  }
}

std::unique_ptr<CorePluginContext> make_core_plugin_context(GatewayServices & services,
                                                            CorePluginContextWiring wiring) {
  if (!wiring.fault_manager) {
    wiring.fault_manager = services.fault_manager();
  }
  if (!wiring.sampler_registry) {
    wiring.sampler_registry = services.sampler_registry();
  }
  return std::make_unique<CorePluginContext>(&services, std::move(wiring));
}

}  // namespace ros2_medkit_gateway
