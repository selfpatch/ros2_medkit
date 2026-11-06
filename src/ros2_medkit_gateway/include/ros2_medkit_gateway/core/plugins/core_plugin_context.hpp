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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/gateway_services.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

namespace ros2_medkit_gateway {

class AggregationManager;
class ConditionRegistry;
class FaultManager;
class ResourceChangeNotifier;
class ResourceSamplerRegistry;
struct EntityChangeScope;

/// What a host gives the plugin context besides its `GatewayServices`.
struct CorePluginContextWiring {
  /// Fault source of `list_entity_faults()` and `list_all_faults()`. Null: those answer nothing from this host.
  FaultManager * fault_manager{nullptr};
  /// Where `register_sampler()` registers. Null: the call is ignored.
  ResourceSamplerRegistry * sampler_registry{nullptr};
  /// Read at each call, so the object may be built after the context. A null result gives the plugin a null.
  std::function<ResourceChangeNotifier *()> notifier;
  std::function<ConditionRegistry *()> conditions;
  /// Peer aggregation for the fault listings. Null or a null result: no peers.
  std::function<AggregationManager *()> aggregation;
  /// Runs `notify_entities_changed()`. Null: the notification is ignored.
  std::function<void(const EntityChangeScope &)> on_entities_changed;
  /// Operator-facing notices (a partial peer fan-out).
  std::function<void(const std::string &)> warn;
};

/// Plugin context over `GatewayServices`, with no ROS node.
///
/// A program that hosts the core builds it with `make_core_plugin_context()` and gives it to
/// `PluginManager::set_context()`. It is a `RosPluginContext` so that `as_ros_plugin_context()` stays valid for every
/// plugin, and its `node()` is null: a plugin that needs ROS checks for it. The ROS 2 node derives from this class and
/// overrides only `node()`.
class CorePluginContext : public RosPluginContext {
 public:
  /// `services` may be null: entity queries then answer nothing, locks answer "disabled", and capabilities and
  /// samplers still work.
  CorePluginContext(GatewayServices * services, CorePluginContextWiring wiring);

  /// Null: this host has no ROS node.
  rclcpp::Node * node() const override;

  // The typed calls of PluginContext, beside the id-only overrides below.
  using PluginContext::acquire_lock;
  using PluginContext::check_lock;
  using PluginContext::get_entity;
  using PluginContext::get_entity_capabilities;
  using PluginContext::list_entity_faults;
  using PluginContext::register_entity_capability;
  using PluginContext::release_lock;

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override;
  std::vector<PluginEntityInfo> get_child_apps(const std::string & component_id) const override;
  nlohmann::json list_entity_faults(const std::string & entity_id) const override;
  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & req, PluginResponse & res,
                                                            const std::string & entity_id) const override;
  void register_capability(SovdEntityType entity_type, const std::string & capability_name) override;
  void register_entity_capability(const std::string & entity_id, const std::string & capability_name) override;
  std::vector<std::string> get_type_capabilities(SovdEntityType entity_type) const override;
  std::vector<std::string> get_entity_capabilities(const std::string & entity_id) const override;
  LockAccessResult check_lock(const std::string & entity_id, const std::string & client_id,
                              const std::string & collection) const override;
  tl::expected<LockInfo, LockError> acquire_lock(const std::string & entity_id, const std::string & client_id,
                                                 const std::vector<std::string> & scopes,
                                                 int expiration_seconds) override;
  tl::expected<void, LockError> release_lock(const std::string & entity_id, const std::string & client_id) override;
  IntrospectionInput get_entity_snapshot() const override;
  nlohmann::json list_all_faults() const override;
  void
  register_sampler(const std::string & collection,
                   const std::function<tl::expected<nlohmann::json, std::string>(SovdEntityType, const std::string &,
                                                                                 const std::string &)> & fn) override;
  ResourceChangeNotifier * get_resource_change_notifier() override;
  ConditionRegistry * get_condition_registry() override;
  void notify_entities_changed(const EntityChangeScope & scope) override;

 protected:
  std::optional<PluginEntityInfo> get_entity_typed(SovdEntityType type, const std::string & id) const override;
  nlohmann::json list_entity_faults_typed(SovdEntityType entity_type, const std::string & entity_id) const override;
  void register_entity_capability_typed(SovdEntityType entity_type, const std::string & entity_id,
                                        const std::string & capability_name) override;
  std::vector<std::string> get_entity_capabilities_typed(SovdEntityType entity_type,
                                                         const std::string & entity_id) const override;
  LockAccessResult check_lock_typed(SovdEntityType entity_type, const std::string & entity_id,
                                    const std::string & client_id, const std::string & collection) const override;
  tl::expected<LockInfo, LockError> acquire_lock_typed(SovdEntityType entity_type, const std::string & entity_id,
                                                       const std::string & client_id,
                                                       const std::vector<std::string> & scopes,
                                                       int expiration_seconds) override;
  tl::expected<void, LockError> release_lock_typed(SovdEntityType entity_type, const std::string & entity_id,
                                                   const std::string & client_id) override;

 private:
  /// The types of the entities that hold `id` in the cache, in the order Component, App, Area, Function.
  std::vector<SovdEntityType> holders_of(const std::string & id) const;

  /// The type of the one entity that holds a bare `id`, for the id-only calls. Nullopt when none holds it, or when
  /// more than one does: that is logged once per id (naming `call`, the id and the types), and again only after the
  /// id has been held by at most one entity in between. `holders` receives the types found.
  std::optional<SovdEntityType> resolve_bare_id(const std::string & id, const char * call,
                                                std::vector<SovdEntityType> & holders) const;

  GatewayServices * services_;
  CorePluginContextWiring wiring_;

  mutable std::mutex capabilities_mutex_;
  std::unordered_map<SovdEntityType, std::vector<std::string>> type_capabilities_;
  /// By type and id (register_entity_capability(type, id, name)).
  std::unordered_map<MemberRef, std::vector<std::string>, MemberRefHash> entity_capabilities_;
  /// By id alone (register_entity_capability(id, name)): applied, when looked up, to the one entity holding the id.
  std::unordered_map<std::string, std::vector<std::string>> untyped_entity_capabilities_;

  mutable std::mutex ambiguity_mutex_;
  /// Bare ids logged as held by more than one entity, until a lookup finds them held by at most one.
  mutable std::unordered_set<std::string> ambiguous_ids_logged_;
};

/// Builds the context of a host without a ROS node. The fault manager and the sampler registry of `services` are used
/// unless `wiring` names others. `services` must outlive the context.
std::unique_ptr<CorePluginContext> make_core_plugin_context(GatewayServices & services,
                                                            CorePluginContextWiring wiring = {});

}  // namespace ros2_medkit_gateway
