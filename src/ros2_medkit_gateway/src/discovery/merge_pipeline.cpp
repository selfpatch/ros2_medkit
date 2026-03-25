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

#include "ros2_medkit_gateway/discovery/merge_pipeline.hpp"

#include "ros2_medkit_gateway/providers/introspection_provider.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

namespace ros2_medkit_gateway {
namespace discovery {

namespace {

// Policy priority: AUTH=2, ENRICH=1, FALLBACK=0
int policy_priority(MergePolicy p) {
  switch (p) {
    case MergePolicy::AUTHORITATIVE:
      return 2;
    case MergePolicy::ENRICHMENT:
      return 1;
    case MergePolicy::FALLBACK:
      return 0;
  }
  return 0;
}

enum class MergeWinner { TARGET, SOURCE, BOTH };

struct MergeResolution {
  MergeWinner scalar;
  MergeWinner collection;
  bool is_conflict{false};
};

MergeResolution resolve_policies(MergePolicy target_policy, MergePolicy source_policy) {
  int tp = policy_priority(target_policy);
  int sp = policy_priority(source_policy);

  if (tp > sp) {
    return {MergeWinner::TARGET, MergeWinner::TARGET, false};
  } else if (sp > tp) {
    return {MergeWinner::SOURCE, MergeWinner::SOURCE, false};
  } else {
    // Same level
    if (target_policy == MergePolicy::AUTHORITATIVE) {
      return {MergeWinner::TARGET, MergeWinner::TARGET, true};  // AUTH vs AUTH -> conflict
    } else if (target_policy == MergePolicy::ENRICHMENT) {
      return {MergeWinner::BOTH, MergeWinner::BOTH, false};  // mutual enrichment
    } else {
      return {MergeWinner::BOTH, MergeWinner::BOTH, false};  // FALLBACK vs FALLBACK: fill gaps
    }
  }
}

void merge_scalar(std::string & target, const std::string & source, MergeWinner winner) {
  switch (winner) {
    case MergeWinner::SOURCE:
      if (!source.empty()) {
        target = source;
      }
      break;
    case MergeWinner::BOTH:
      // First non-empty (target preferred since higher priority)
      if (target.empty() && !source.empty()) {
        target = source;
      }
      break;
    case MergeWinner::TARGET:
      break;
  }
}

void merge_bool(bool & target, bool source, MergeWinner winner) {
  switch (winner) {
    case MergeWinner::SOURCE:
      target = source;
      break;
    case MergeWinner::BOTH:
      // OR semantics: once true, stays true. This is intentional for status flags
      // like is_online (any layer reporting online = online).
      target = target || source;
      break;
    case MergeWinner::TARGET:
      break;
  }
}

void merge_collection(std::vector<std::string> & target, const std::vector<std::string> & source, MergeWinner winner) {
  switch (winner) {
    case MergeWinner::SOURCE:
      target = source;
      break;
    case MergeWinner::BOTH: {
      std::unordered_set<std::string> seen(target.begin(), target.end());
      for (const auto & s : source) {
        if (seen.insert(s).second) {
          target.push_back(s);
        }
      }
      break;
    }
    case MergeWinner::TARGET:
      break;
  }
}

template <typename T, typename KeyFn>
void merge_by_key(std::vector<T> & target, const std::vector<T> & source, KeyFn key_fn, MergeWinner winner) {
  switch (winner) {
    case MergeWinner::SOURCE:
      target = source;
      break;
    case MergeWinner::BOTH: {
      std::unordered_set<std::string> seen;
      for (const auto & t : target) {
        seen.insert(key_fn(t));
      }
      for (const auto & s : source) {
        if (seen.insert(key_fn(s)).second) {
          target.push_back(s);
        }
      }
      break;
    }
    case MergeWinner::TARGET:
      break;
  }
}

template <typename T>
void merge_optional(std::optional<T> & target, const std::optional<T> & source, MergeWinner winner) {
  switch (winner) {
    case MergeWinner::SOURCE:
      if (source.has_value()) {
        target = source;
      }
      break;
    case MergeWinner::BOTH:
      if (!target.has_value() && source.has_value()) {
        target = source;
      }
      break;
    case MergeWinner::TARGET:
      break;
  }
}

void merge_topics(ComponentTopics & target, const ComponentTopics & source, MergeWinner winner) {
  merge_collection(target.publishes, source.publishes, winner);
  merge_collection(target.subscribes, source.subscribes, winner);
}

// Per-entity-type field-group merge dispatch
template <typename Entity>
void apply_field_group_merge(Entity & target, const Entity & source, FieldGroup group, const MergeResolution & res) {
  if constexpr (std::is_same_v<Entity, Area>) {
    switch (group) {
      case FieldGroup::IDENTITY:
        merge_scalar(target.name, source.name, res.scalar);
        merge_scalar(target.translation_id, source.translation_id, res.scalar);
        merge_scalar(target.description, source.description, res.scalar);
        merge_collection(target.tags, source.tags, res.collection);
        break;
      case FieldGroup::HIERARCHY:
        merge_scalar(target.namespace_path, source.namespace_path, res.scalar);
        merge_scalar(target.parent_area_id, source.parent_area_id, res.scalar);
        break;
      case FieldGroup::METADATA:
        merge_scalar(target.source, source.source, res.scalar);
        break;
      default:
        break;
    }
  } else if constexpr (std::is_same_v<Entity, Component>) {
    switch (group) {
      case FieldGroup::IDENTITY:
        merge_scalar(target.name, source.name, res.scalar);
        merge_scalar(target.translation_id, source.translation_id, res.scalar);
        merge_scalar(target.description, source.description, res.scalar);
        merge_collection(target.tags, source.tags, res.collection);
        break;
      case FieldGroup::HIERARCHY:
        merge_scalar(target.namespace_path, source.namespace_path, res.scalar);
        merge_scalar(target.fqn, source.fqn, res.scalar);
        merge_scalar(target.area, source.area, res.scalar);
        merge_scalar(target.parent_component_id, source.parent_component_id, res.scalar);
        merge_collection(target.depends_on, source.depends_on, res.collection);
        break;
      case FieldGroup::LIVE_DATA:
        merge_topics(target.topics, source.topics, res.collection);
        merge_by_key(
            target.services, source.services,
            [](const ServiceInfo & s) {
              return s.full_path;
            },
            res.collection);
        merge_by_key(
            target.actions, source.actions,
            [](const ActionInfo & a) {
              return a.full_path;
            },
            res.collection);
        break;
      case FieldGroup::METADATA:
        merge_scalar(target.source, source.source, res.scalar);
        merge_scalar(target.variant, source.variant, res.scalar);
        break;
      default:
        break;
    }
  } else if constexpr (std::is_same_v<Entity, App>) {
    switch (group) {
      case FieldGroup::IDENTITY:
        merge_scalar(target.name, source.name, res.scalar);
        merge_scalar(target.translation_id, source.translation_id, res.scalar);
        merge_scalar(target.description, source.description, res.scalar);
        merge_collection(target.tags, source.tags, res.collection);
        break;
      case FieldGroup::HIERARCHY:
        merge_scalar(target.component_id, source.component_id, res.scalar);
        merge_collection(target.depends_on, source.depends_on, res.collection);
        break;
      case FieldGroup::LIVE_DATA:
        merge_topics(target.topics, source.topics, res.collection);
        merge_by_key(
            target.services, source.services,
            [](const ServiceInfo & s) {
              return s.full_path;
            },
            res.collection);
        merge_by_key(
            target.actions, source.actions,
            [](const ActionInfo & a) {
              return a.full_path;
            },
            res.collection);
        break;
      case FieldGroup::STATUS:
        merge_bool(target.is_online, source.is_online, res.scalar);
        merge_optional(target.bound_fqn, source.bound_fqn, res.scalar);
        break;
      case FieldGroup::METADATA:
        merge_scalar(target.source, source.source, res.scalar);
        merge_optional(target.ros_binding, source.ros_binding, res.scalar);
        // Use scalar semantics (not OR) - external is a classification, not a status flag
        if (res.scalar == MergeWinner::SOURCE) {
          target.external = source.external;
        }
        // TARGET and BOTH: keep target value (no OR semantics)
        break;
    }
  } else if constexpr (std::is_same_v<Entity, Function>) {
    switch (group) {
      case FieldGroup::IDENTITY:
        merge_scalar(target.name, source.name, res.scalar);
        merge_scalar(target.translation_id, source.translation_id, res.scalar);
        merge_scalar(target.description, source.description, res.scalar);
        merge_collection(target.tags, source.tags, res.collection);
        break;
      case FieldGroup::HIERARCHY:
        merge_collection(target.hosts, source.hosts, res.collection);
        merge_collection(target.depends_on, source.depends_on, res.collection);
        break;
      case FieldGroup::METADATA:
        merge_scalar(target.source, source.source, res.scalar);
        break;
      default:
        break;
    }
  }
}

constexpr FieldGroup ALL_FIELD_GROUPS[] = {FieldGroup::IDENTITY, FieldGroup::HIERARCHY, FieldGroup::LIVE_DATA,
                                           FieldGroup::STATUS, FieldGroup::METADATA};

}  // namespace

MergePipeline::MergePipeline(rclcpp::Logger logger) : logger_(std::move(logger)) {
}

void MergePipeline::add_layer(std::unique_ptr<DiscoveryLayer> layer) {
  layers_.push_back(std::move(layer));
}

void MergePipeline::set_linker(std::unique_ptr<RuntimeLinker> linker, const ManifestConfig & config) {
  linker_ = std::move(linker);
  manifest_config_ = config;
}

template <typename Entity>
std::vector<Entity> MergePipeline::merge_entities(std::vector<std::pair<size_t, std::vector<Entity>>> & layer_entities,
                                                  MergeReport & report) {
  // Collect all entities by ID with their layer index
  struct LayerEntity {
    size_t layer_idx;
    Entity entity;
  };
  std::unordered_map<std::string, std::vector<LayerEntity>> by_id;
  std::vector<std::string> insertion_order;

  for (auto & [layer_idx, entities] : layer_entities) {
    for (auto & entity : entities) {
      if (by_id.find(entity.id) == by_id.end()) {
        insertion_order.push_back(entity.id);
      }
      auto id = entity.id;  // copy id before move
      by_id[id].push_back({layer_idx, std::move(entity)});
    }
  }

  std::vector<Entity> result;
  result.reserve(insertion_order.size());

  for (const auto & id : insertion_order) {
    auto & entries = by_id[id];

    // Start with highest-priority layer's entity as base
    Entity merged = std::move(entries[0].entity);
    size_t owner_layer_idx = entries[0].layer_idx;
    report.entity_source[id] = layers_[owner_layer_idx]->name();

    // Track current owning layer per field group (initially all owned by first layer)
    std::array<size_t, sizeof(ALL_FIELD_GROUPS) / sizeof(ALL_FIELD_GROUPS[0])> fg_owner;
    fg_owner.fill(owner_layer_idx);

    // Merge with each subsequent (lower-priority) layer
    for (size_t i = 1; i < entries.size(); i++) {
      size_t source_layer_idx = entries[i].layer_idx;
      report.enriched_count++;

      for (size_t fg_idx = 0; fg_idx < fg_owner.size(); ++fg_idx) {
        auto fg = ALL_FIELD_GROUPS[fg_idx];
        size_t current_owner = fg_owner[fg_idx];
        auto target_policy = layers_[current_owner]->policy_for(fg);
        auto source_policy = layers_[source_layer_idx]->policy_for(fg);
        auto res = resolve_policies(target_policy, source_policy);

        if (res.is_conflict) {
          report.conflicts.push_back({id, fg, layers_[current_owner]->name(), layers_[source_layer_idx]->name()});
          report.conflict_count++;
        }

        apply_field_group_merge(merged, entries[i].entity, fg, res);

        // If source won with a strictly higher-priority policy, it becomes
        // the owner of this field group for subsequent merge comparisons.
        if (!res.is_conflict && policy_priority(source_policy) > policy_priority(target_policy)) {
          fg_owner[fg_idx] = source_layer_idx;
        }
      }
    }

    result.push_back(std::move(merged));
  }

  return result;
}

MergeResult MergePipeline::execute() {
  MergeReport report;
  for (const auto & layer : layers_) {
    report.layers.push_back(layer->name());
  }

  // Collect outputs from all layers
  std::vector<std::pair<size_t, std::vector<Area>>> area_layers;
  std::vector<std::pair<size_t, std::vector<Component>>> component_layers;
  std::vector<std::pair<size_t, std::vector<App>>> app_layers;
  std::vector<std::pair<size_t, std::vector<Function>>> function_layers;

  // RuntimeLinker needs runtime-only apps (nodes as Apps, not merged with
  // manifest apps). Manifest apps lack bound_fqn until linked.
  std::vector<App> runtime_apps;

  for (size_t i = 0; i < layers_.size(); ++i) {
    // Build discovery context from entities collected so far (for plugin layers)
    IntrospectionInput context;
    for (const auto & [idx, entities] : area_layers) {
      context.areas.insert(context.areas.end(), entities.begin(), entities.end());
    }
    for (const auto & [idx, entities] : component_layers) {
      context.components.insert(context.components.end(), entities.begin(), entities.end());
    }
    for (const auto & [idx, entities] : app_layers) {
      context.apps.insert(context.apps.end(), entities.begin(), entities.end());
    }
    for (const auto & [idx, entities] : function_layers) {
      context.functions.insert(context.functions.end(), entities.begin(), entities.end());
    }
    layers_[i]->set_discovery_context(context);

    LayerOutput output;
    try {
      output = layers_[i]->discover();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Layer '%s' threw exception during discover(): %s", layers_[i]->name().c_str(), e.what());
      continue;
    } catch (...) {
      RCLCPP_ERROR(logger_, "Layer '%s' threw unknown exception during discover()", layers_[i]->name().c_str());
      continue;
    }

    // Collect gap-fill filtering stats (virtual dispatch, no dynamic_cast)
    report.filtered_by_gap_fill += layers_[i]->filtered_count();

    // Save runtime apps for the linker. Use get_linking_apps() which returns
    // the unfiltered set - gap-fill may exclude apps from output.apps, but
    // the linker needs all runtime apps to bind manifest apps to live nodes.
    if (layers_[i]->provides_runtime_apps()) {
      runtime_apps = layers_[i]->get_linking_apps();
    }

    if (!output.areas.empty()) {
      area_layers.emplace_back(i, std::move(output.areas));
    }
    if (!output.components.empty()) {
      component_layers.emplace_back(i, std::move(output.components));
    }
    if (!output.apps.empty()) {
      app_layers.emplace_back(i, std::move(output.apps));
    }
    if (!output.functions.empty()) {
      function_layers.emplace_back(i, std::move(output.functions));
    }
    // entity_metadata is not consumed here - plugins serve their metadata
    // as SOVD vendor extension resources via register_routes() and register_capability().
  }

  MergeResult result;
  result.areas = merge_entities<Area>(area_layers, report);
  result.components = merge_entities<Component>(component_layers, report);
  result.apps = merge_entities<App>(app_layers, report);
  result.functions = merge_entities<Function>(function_layers, report);

  report.total_entities = result.areas.size() + result.components.size() + result.apps.size() + result.functions.size();

  // Cross-type ID collision detection
  std::unordered_map<std::string, std::string> global_ids;
  auto check_ids = [&](const auto & entities, const std::string & type) {
    for (const auto & e : entities) {
      auto [it, inserted] = global_ids.emplace(e.id, type);
      if (!inserted && it->second != type) {
        report.id_collision_count++;
        RCLCPP_ERROR(logger_, "ID collision: '%s' used by both %s and %s", e.id.c_str(), it->second.c_str(),
                     type.c_str());
      }
    }
  };
  check_ids(result.areas, "Area");
  check_ids(result.components, "Component");
  check_ids(result.apps, "App");
  check_ids(result.functions, "Function");

  // Post-merge linking: bind manifest apps to runtime nodes (Apps, not Components)
  if (linker_) {
    auto linking = linker_->link(result.apps, runtime_apps, manifest_config_);

    // Replace apps with linked versions (have is_online, bound_fqn set)
    result.apps = std::move(linking.linked_apps);

    // Record linking stats in report
    report.total_entities =
        result.areas.size() + result.components.size() + result.apps.size() + result.functions.size();

    linking_result_ = linker_->get_last_result();
    result.linking_result = linking_result_;

    // Suppress runtime-origin entities that duplicate manifest entities (#307)
    // In hybrid mode with a manifest, the manifest is the source of truth for entity
    // structure. Heuristic entities from runtime discovery should be suppressed when
    // their namespace is covered by manifest entities or linked apps.

    // Build set of namespaces covered by linked manifest apps
    std::set<std::string> linked_namespaces;
    for (const auto & [fqn, app_id] : linking_result_.node_to_app) {
      std::string clean_fqn = fqn;
      // Strip trailing slash if present (defensive - ROS 2 normalizes FQNs)
      if (clean_fqn.size() > 1 && clean_fqn.back() == '/') {
        clean_fqn.pop_back();
      }
      auto last_slash = clean_fqn.rfind('/');
      if (last_slash != std::string::npos && last_slash > 0) {
        linked_namespaces.insert(clean_fqn.substr(0, last_slash));
      } else if (last_slash == 0) {
        // Root namespace node (e.g., /fault_manager) - include "/" as covered
        linked_namespaces.insert("/");
      }
    }

    // Also track manifest component/area namespaces directly
    std::set<std::string> manifest_comp_ns;
    for (const auto & comp : result.components) {
      if (comp.source == "manifest" && !comp.namespace_path.empty()) {
        manifest_comp_ns.insert(comp.namespace_path);
      }
    }
    std::set<std::string> manifest_area_ns;
    for (const auto & area : result.areas) {
      if (area.source == "manifest" && !area.namespace_path.empty()) {
        manifest_area_ns.insert(area.namespace_path);
      }
    }

    // Remove heuristic apps that were merged into manifest apps (same ID after merge).
    // These are runtime duplicates of linked manifest entities. Gap-fill apps (new
    // heuristic apps in any namespace) survive - they fill manifest gaps intentionally.
    std::set<std::string> manifest_app_ids;
    for (const auto & app : result.apps) {
      if (app.source == "manifest") {
        manifest_app_ids.insert(app.id);
      }
    }
    // Also suppress heuristic apps whose original runtime ID was linked to a manifest app
    // (linker renames runtime app ID to manifest ID, so we check node_to_app values)
    std::set<std::string> linked_app_ids(manifest_app_ids);
    for (const auto & [fqn, app_id] : linking_result_.node_to_app) {
      linked_app_ids.insert(app_id);
    }
    auto app_it = std::remove_if(result.apps.begin(), result.apps.end(), [&](const App & app) {
      if (app.source != "heuristic" && app.source != "topic" && app.source != "synthetic") {
        return false;
      }
      // Keep gap-fill apps (not linked to any manifest entity)
      // Suppress only if this app's ID matches a linked manifest app
      return linked_app_ids.count(app.id) > 0;
    });
    result.apps.erase(app_it, result.apps.end());

    // Remove runtime components whose namespace is covered
    auto comp_it = std::remove_if(result.components.begin(), result.components.end(), [&](const Component & comp) {
      // Only suppress known runtime sources - preserve manifest and plugin entities
      if (comp.source != "heuristic" && comp.source != "topic" && comp.source != "synthetic") {
        return false;
      }
      return manifest_comp_ns.count(comp.namespace_path) > 0 || linked_namespaces.count(comp.namespace_path) > 0;
    });
    result.components.erase(comp_it, result.components.end());

    // Remove runtime areas whose namespace is covered
    auto area_it = std::remove_if(result.areas.begin(), result.areas.end(), [&](const Area & area) {
      // Only suppress known runtime sources - preserve manifest and plugin entities
      if (area.source != "heuristic" && area.source != "topic" && area.source != "synthetic") {
        return false;
      }
      return manifest_area_ns.count(area.namespace_path) > 0 || linked_namespaces.count(area.namespace_path) > 0;
    });
    result.areas.erase(area_it, result.areas.end());

    // Recount after suppression
    report.total_entities =
        result.areas.size() + result.components.size() + result.apps.size() + result.functions.size();
  }

  RCLCPP_INFO(logger_, "MergePipeline: %zu entities from %zu layers, %zu enriched, %zu conflicts",
              report.total_entities, report.layers.size(), report.enriched_count, report.conflict_count);
  if (report.conflict_count > 0) {
    RCLCPP_WARN(logger_,
                "MergePipeline: %zu merge conflicts (higher-priority layer wins in all cases). "
                "Details available via GET /health.",
                report.conflict_count);
  }
  for (const auto & conflict : report.conflicts) {
    RCLCPP_DEBUG(logger_, "Merge conflict: entity '%s' field_group %s - '%s' wins over '%s'",
                 conflict.entity_id.c_str(), field_group_to_string(conflict.field_group),
                 conflict.winning_layer.c_str(), conflict.losing_layer.c_str());
  }

  last_report_ = report;
  result.report = std::move(report);
  return result;
}

// Explicit template instantiations
template std::vector<Area> MergePipeline::merge_entities<Area>(std::vector<std::pair<size_t, std::vector<Area>>> &,
                                                               MergeReport &);
template std::vector<Component>
MergePipeline::merge_entities<Component>(std::vector<std::pair<size_t, std::vector<Component>>> &, MergeReport &);
template std::vector<App> MergePipeline::merge_entities<App>(std::vector<std::pair<size_t, std::vector<App>>> &,
                                                             MergeReport &);
template std::vector<Function>
MergePipeline::merge_entities<Function>(std::vector<std::pair<size_t, std::vector<Function>>> &, MergeReport &);

}  // namespace discovery
}  // namespace ros2_medkit_gateway
