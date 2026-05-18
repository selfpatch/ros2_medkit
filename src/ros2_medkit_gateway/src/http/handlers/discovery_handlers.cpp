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

#include "ros2_medkit_gateway/core/http/handlers/discovery_handlers.hpp"

#include <map>
#include <set>

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/capability_builder.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

namespace {

/// Check if a capability name is already present in the capabilities array
bool has_capability(const json & capabilities, const std::string & name) {
  for (const auto & cap : capabilities) {
    if (cap.contains("name") && cap["name"] == name) {
      return true;
    }
  }
  return false;
}

/// Append plugin-registered capabilities to a capabilities JSON array
void append_plugin_capabilities(json & capabilities, const std::string & entity_type_path,
                                const std::string & entity_id, SovdEntityType entity_type, const GatewayNode * node) {
  auto * pmgr = node->get_plugin_manager();
  if (!pmgr) {
    return;
  }

  std::string href_prefix;
  href_prefix.reserve(64);
  href_prefix.append("/api/v1/").append(entity_type_path).append("/").append(entity_id).append("/");

  // Auto-add standard capabilities based on registered providers
  if (pmgr->get_data_provider_for_entity(entity_id) && !has_capability(capabilities, "data")) {
    capabilities.push_back({{"name", "data"}, {"href", href_prefix + "data"}});
  }
  if (pmgr->get_operation_provider_for_entity(entity_id) && !has_capability(capabilities, "operations")) {
    capabilities.push_back({{"name", "operations"}, {"href", href_prefix + "operations"}});
  }
  if (pmgr->get_fault_provider_for_entity(entity_id) && !has_capability(capabilities, "faults")) {
    capabilities.push_back({{"name", "faults"}, {"href", href_prefix + "faults"}});
  }

  // Plugin-registered custom capabilities (via PluginContext)
  auto * ctx = pmgr->get_context();
  if (!ctx) {
    return;
  }

  // Type-level capabilities (registered for all entities of this type)
  for (const auto & cap_name : ctx->get_type_capabilities(entity_type)) {
    if (!has_capability(capabilities, cap_name)) {
      capabilities.push_back({{"name", cap_name}, {"href", href_prefix + cap_name}});
    }
  }

  // Entity-specific capabilities
  for (const auto & cap_name : ctx->get_entity_capabilities(entity_id)) {
    if (!has_capability(capabilities, cap_name)) {
      capabilities.push_back({{"name", cap_name}, {"href", href_prefix + cap_name}});
    }
  }
}

}  // namespace

// =============================================================================
// Area handlers
// =============================================================================

void DiscoveryHandlers::handle_list_areas(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    const auto areas = cache.get_areas();

    dto::Collection<dto::AreaListItem> response;
    for (const auto & area : areas) {
      // Subareas (with parent_area_id) are only visible via
      // GET /areas/{id}/subareas, not in the top-level list.
      if (!area.parent_area_id.empty()) {
        continue;
      }

      dto::AreaListItem item;
      item.id = area.id;
      item.name = area.name.empty() ? area.id : area.name;
      item.href = "/api/v1/areas/" + area.id;
      item.type = "area";

      if (!area.description.empty()) {
        item.description = area.description;
      }
      if (!area.tags.empty()) {
        item.tags = area.tags;
      }

      if (!area.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = area.namespace_path;
        dto::XMedkitArea ext;
        ext.ros2 = ros2;
        item.x_medkit = ext;
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_areas: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_area(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    // Validate entity and forward to peer if remote (aggregation support)
    auto entity_opt = ctx_.validate_entity_for_route(req, res, area_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }

    // Local entity - look up full object from cache for detail response
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto area_opt = cache.get_area(area_id);
    if (!area_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      area_opt = discovery->get_area(area_id);
    }

    if (!area_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Area not found", {{"area_id", area_id}});
      return;
    }

    const auto & area = *area_opt;

    dto::AreaDetail detail;
    detail.id = area.id;
    detail.name = area.name.empty() ? area.id : area.name;
    detail.type = "area";

    if (!area.description.empty()) {
      detail.description = area.description;
    }
    if (!area.tags.empty()) {
      detail.tags = area.tags;
    }

    std::string base_uri = "/api/v1/areas/" + area.id;
    detail.subareas = base_uri + "/subareas";
    detail.components = base_uri + "/components";
    detail.contains = base_uri + "/contains";
    detail.data = base_uri + "/data";
    detail.operations = base_uri + "/operations";
    detail.configurations = base_uri + "/configurations";
    detail.faults = base_uri + "/faults";
    detail.logs = base_uri + "/logs";
    detail.bulk_data = base_uri + "/bulk-data";
    detail.triggers = base_uri + "/triggers";

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::SUBAREAS, Cap::CONTAINS, Cap::DATA,      Cap::OPERATIONS, Cap::CONFIGURATIONS,
                             Cap::FAULTS,   Cap::LOGS,     Cap::BULK_DATA, Cap::TRIGGERS};
    auto area_caps = CapabilityBuilder::build_capabilities("areas", area.id, caps);
    append_plugin_capabilities(area_caps, "areas", area.id, SovdEntityType::AREA, ctx_.node());
    detail.capabilities = area_caps;

    LinksBuilder links;
    links.self("/api/v1/areas/" + area.id).collection("/api/v1/areas");
    if (!area.parent_area_id.empty()) {
      links.parent("/api/v1/areas/" + area.parent_area_id);
    }
    detail.links = links.build();

    dto::XMedkitArea x_medkit_area;
    if (!area.namespace_path.empty()) {
      dto::XMedkitRos2 ros2;
      ros2.ns = area.namespace_path;
      x_medkit_area.ros2 = ros2;
    }
    if (!area.parent_area_id.empty()) {
      x_medkit_area.parent_area_id = area.parent_area_id;
    }
    if (!area.contributors.empty()) {
      x_medkit_area.contributors = sorted_contributors(area.contributors);
    }
    detail.x_medkit = x_medkit_area;

    HandlerContext::send_dto(res, detail);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_area: %s", e.what());
  }
}

void DiscoveryHandlers::handle_area_components(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid area ID",
                                 {{"details", validation_result.error()}, {"area_id", area_id}});
      return;
    }

    const auto & cache = ctx_.node()->get_thread_safe_cache();

    if (!cache.has_area(area_id)) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Area not found", {{"area_id", area_id}});
      return;
    }

    const auto components = cache.get_components();
    dto::Collection<dto::ComponentListItem> response;
    for (const auto & component : components) {
      if (component.area == area_id) {
        dto::ComponentListItem item;
        item.id = component.id;
        item.name = component.name.empty() ? component.id : component.name;
        item.href = "/api/v1/components/" + component.id;
        item.type = "component";

        if (!component.description.empty()) {
          item.description = component.description;
        }

        dto::XMedkitComponent x_medkit_comp;
        if (!component.source.empty()) {
          x_medkit_comp.source = component.source;
        }
        if (!component.namespace_path.empty()) {
          dto::XMedkitRos2 ros2;
          ros2.ns = component.namespace_path;
          x_medkit_comp.ros2 = ros2;
        }
        item.x_medkit = x_medkit_comp;

        response.items.push_back(std::move(item));
      }
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_area_components: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_subareas(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid area ID",
                                 {{"details", validation_result.error()}, {"area_id", area_id}});
      return;
    }

    // Cache-first lookup: EntityCache has merged entities from peers
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto area_opt = cache.get_area(area_id);
    if (!area_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      area_opt = discovery->get_area(area_id);
    }

    if (!area_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Area not found", {{"area_id", area_id}});
      return;
    }

    // Use cache relationship index for subarea IDs, then look up each
    auto subarea_ids = cache.get_subareas(area_id);

    dto::Collection<dto::AreaListItem> response;
    for (const auto & sub_id : subarea_ids) {
      auto subarea_opt = cache.get_area(sub_id);
      if (!subarea_opt) {
        continue;
      }
      const auto & subarea = *subarea_opt;

      dto::AreaListItem item;
      item.id = subarea.id;
      item.name = subarea.name.empty() ? subarea.id : subarea.name;
      item.href = "/api/v1/areas/" + subarea.id;
      item.type = "area";

      if (!subarea.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = subarea.namespace_path;
        dto::XMedkitArea x_medkit_area;
        x_medkit_area.ros2 = ros2;
        item.x_medkit = x_medkit_area;
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/areas/" + area_id + "/subareas";
    links["parent"] = "/api/v1/areas/" + area_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_subareas: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_contains(const httplib::Request & req, httplib::Response & res) {
  // @verifies REQ_INTEROP_006
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid area ID",
                                 {{"details", validation_result.error()}, {"area_id", area_id}});
      return;
    }

    // Cache-first lookup: EntityCache has merged entities from peers
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto area_opt = cache.get_area(area_id);
    if (!area_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      area_opt = discovery->get_area(area_id);
    }

    if (!area_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Area not found", {{"area_id", area_id}});
      return;
    }

    // Recursively collect components from this area and all descendant subareas
    // (mirrors ManifestManager::get_components_for_area behavior)
    std::vector<std::string> area_queue = {area_id};
    std::set<std::string> visited_areas;
    std::vector<std::string> all_comp_ids;

    while (!area_queue.empty()) {
      auto current_area = area_queue.back();
      area_queue.pop_back();
      if (!visited_areas.insert(current_area).second) {
        continue;
      }

      auto comp_ids = cache.get_components_for_area(current_area);
      all_comp_ids.insert(all_comp_ids.end(), comp_ids.begin(), comp_ids.end());

      auto sub_ids = cache.get_subareas(current_area);
      area_queue.insert(area_queue.end(), sub_ids.begin(), sub_ids.end());
    }

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & comp_id : all_comp_ids) {
      auto comp_opt = cache.get_component(comp_id);
      if (!comp_opt) {
        continue;
      }
      const auto & comp = *comp_opt;

      dto::ComponentListItem item;
      item.id = comp.id;
      item.name = comp.name.empty() ? comp.id : comp.name;
      item.href = "/api/v1/components/" + comp.id;
      item.type = "component";

      dto::XMedkitComponent x_medkit_comp;
      if (!comp.source.empty()) {
        x_medkit_comp.source = comp.source;
      }
      if (!comp.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = comp.namespace_path;
        x_medkit_comp.ros2 = ros2;
      }
      item.x_medkit = x_medkit_comp;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/areas/" + area_id + "/contains";
    links["area"] = "/api/v1/areas/" + area_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_contains: %s", e.what());
  }
}

// =============================================================================
// Component handlers
// =============================================================================

void DiscoveryHandlers::handle_list_components(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    const auto components = cache.get_components();

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & component : components) {
      // Subcomponents (with parent_component_id) are only visible via
      // GET /components/{id}/subcomponents, not in the top-level list.
      if (!component.parent_component_id.empty()) {
        continue;
      }

      dto::ComponentListItem item;
      item.id = component.id;
      item.name = component.name.empty() ? component.id : component.name;
      item.href = "/api/v1/components/" + component.id;
      item.type = "component";

      if (!component.description.empty()) {
        item.description = component.description;
      }
      if (!component.tags.empty()) {
        item.tags = component.tags;
      }

      dto::XMedkitComponent x_medkit_comp;
      if (!component.source.empty()) {
        x_medkit_comp.source = component.source;
      }
      if (!component.fqn.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.node = component.fqn;
        if (!component.namespace_path.empty()) {
          ros2.ns = component.namespace_path;
        }
        x_medkit_comp.ros2 = ros2;
      } else if (!component.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = component.namespace_path;
        x_medkit_comp.ros2 = ros2;
      }
      item.x_medkit = x_medkit_comp;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_components: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_component(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    // Validate entity and forward to peer if remote (aggregation support)
    auto entity_opt = ctx_.validate_entity_for_route(req, res, component_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }

    // Local entity - look up full object from cache for detail response
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    const auto & comp = *comp_opt;

    dto::ComponentDetail detail;
    detail.id = comp.id;
    detail.name = comp.name.empty() ? comp.id : comp.name;
    detail.type = "component";

    if (!comp.description.empty()) {
      detail.description = comp.description;
    }
    if (!comp.tags.empty()) {
      detail.tags = comp.tags;
    }

    std::string base = "/api/v1/components/" + comp.id;
    detail.data = base + "/data";
    detail.operations = base + "/operations";
    detail.configurations = base + "/configurations";
    detail.faults = base + "/faults";
    detail.subcomponents = base + "/subcomponents";
    detail.hosts = base + "/hosts";
    detail.logs = base + "/logs";
    detail.bulk_data = base + "/bulk-data";
    detail.cyclic_subscriptions = base + "/cyclic-subscriptions";
    detail.triggers = base + "/triggers";

    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      detail.scripts = base + "/scripts";
    }

    if (!comp.depends_on.empty()) {
      detail.depends_on = base + "/depends-on";
    }

    if (!comp.area.empty()) {
      detail.belongs_to = "/api/v1/areas/" + comp.area;
    }

    LinksBuilder links;
    links.self(base).collection("/api/v1/components");
    if (!comp.area.empty()) {
      links.add("area", "/api/v1/areas/" + comp.area);
    }
    if (!comp.parent_component_id.empty()) {
      links.parent("/api/v1/components/" + comp.parent_component_id);
    }
    detail.links = links.build();

    dto::XMedkitComponent x_medkit_comp;
    if (!comp.source.empty()) {
      x_medkit_comp.source = comp.source;
    }
    if (!comp.fqn.empty()) {
      dto::XMedkitRos2 ros2;
      ros2.node = comp.fqn;
      if (!comp.namespace_path.empty()) {
        ros2.ns = comp.namespace_path;
      }
      x_medkit_comp.ros2 = ros2;
    } else if (!comp.namespace_path.empty()) {
      dto::XMedkitRos2 ros2;
      ros2.ns = comp.namespace_path;
      x_medkit_comp.ros2 = ros2;
    }
    if (!comp.type.empty()) {
      x_medkit_comp.type = comp.type;
    }
    if (!comp.parent_component_id.empty()) {
      x_medkit_comp.parent_component_id = comp.parent_component_id;
    }
    if (!comp.depends_on.empty()) {
      x_medkit_comp.depends_on = comp.depends_on;
    }
    if (!comp.area.empty()) {
      x_medkit_comp.area = comp.area;
    }
    if (!comp.variant.empty()) {
      x_medkit_comp.variant = comp.variant;
    }
    if (!comp.description.empty()) {
      x_medkit_comp.description = comp.description;
    }
    if (!comp.contributors.empty()) {
      x_medkit_comp.contributors = sorted_contributors(comp.contributors);
    }

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {
        Cap::DATA,  Cap::OPERATIONS, Cap::CONFIGURATIONS,       Cap::FAULTS,  Cap::LOGS, Cap::SUBCOMPONENTS,
        Cap::HOSTS, Cap::BULK_DATA,  Cap::CYCLIC_SUBSCRIPTIONS, Cap::TRIGGERS};
    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      caps.push_back(Cap::SCRIPTS);
    }
    if (ctx_.node() && ctx_.node()->get_lock_manager()) {
      caps.push_back(Cap::LOCKS);
    }
    if (!comp.depends_on.empty()) {
      caps.push_back(Cap::DEPENDS_ON);
    }
    auto comp_caps = CapabilityBuilder::build_capabilities("components", comp.id, caps);
    append_plugin_capabilities(comp_caps, "components", comp.id, SovdEntityType::COMPONENT, ctx_.node());
    // Capabilities at root level (SOVD standard) and in x-medkit (vendor extension for tools
    // that only read x-medkit). Apps don't duplicate because they have no vendor extensions block.
    detail.capabilities = comp_caps;
    x_medkit_comp.capabilities = comp_caps;
    detail.x_medkit = x_medkit_comp;

    HandlerContext::send_dto(res, detail);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_component: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_subcomponents(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    // Cache-first lookup: EntityCache has merged entities from peers
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    // Cache has no get_subcomponents(), so filter from all components
    const auto all_components = cache.get_components();

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & sub : all_components) {
      if (sub.parent_component_id != component_id) {
        continue;
      }

      dto::ComponentListItem item;
      item.id = sub.id;
      item.name = sub.name.empty() ? sub.id : sub.name;
      item.href = "/api/v1/components/" + sub.id;
      item.type = "component";

      dto::XMedkitComponent x_medkit_comp;
      if (!sub.source.empty()) {
        x_medkit_comp.source = sub.source;
      }
      if (!sub.namespace_path.empty()) {
        dto::XMedkitRos2 ros2;
        ros2.ns = sub.namespace_path;
        x_medkit_comp.ros2 = ros2;
      }
      item.x_medkit = x_medkit_comp;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/subcomponents";
    links["parent"] = "/api/v1/components/" + component_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_subcomponents: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_hosts(const httplib::Request & req, httplib::Response & res) {
  // @verifies REQ_INTEROP_007
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    // Cache-first lookup: EntityCache has merged entities from peers
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    // Use cache relationship index for app IDs, then look up each
    auto app_ids = cache.get_apps_for_component(component_id);

    dto::Collection<dto::AppListItem> response;
    for (const auto & aid : app_ids) {
      auto app_opt = cache.get_app(aid);
      if (!app_opt) {
        continue;
      }
      const auto & app = *app_opt;

      dto::AppListItem item;
      item.id = app.id;
      item.name = app.name.empty() ? app.id : app.name;
      item.href = "/api/v1/apps/" + app.id;
      item.type = "app";

      dto::XMedkitApp x_medkit_app;
      x_medkit_app.is_online = app.is_online;
      if (!app.source.empty()) {
        x_medkit_app.source = app.source;
      }
      if (app.bound_fqn) {
        dto::XMedkitRos2 ros2;
        ros2.node = *app.bound_fqn;
        x_medkit_app.ros2 = ros2;
      }
      item.x_medkit = x_medkit_app;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/hosts";
    links["component"] = "/api/v1/components/" + component_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_hosts: %s", e.what());
  }
}

void DiscoveryHandlers::handle_component_depends_on(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    // Cache-first lookup: EntityCache has merged entities from peers
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto comp_opt = cache.get_component(component_id);
    if (!comp_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      comp_opt = discovery->get_component(component_id);
    }

    if (!comp_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    const auto & comp = *comp_opt;

    dto::Collection<dto::ComponentListItem> response;
    for (const auto & dep_id : comp.depends_on) {
      dto::ComponentListItem item;
      item.id = dep_id;
      item.href = "/api/v1/components/" + dep_id;
      item.type = "component";

      auto dep_opt = cache.get_component(dep_id);
      if (dep_opt) {
        item.name = dep_opt->name.empty() ? dep_id : dep_opt->name;

        dto::XMedkitComponent x_medkit_comp;
        if (!dep_opt->source.empty()) {
          x_medkit_comp.source = dep_opt->source;
        }
        item.x_medkit = x_medkit_comp;
      } else {
        item.name = dep_id;
        dto::XMedkitComponent x_medkit_comp;
        x_medkit_comp.missing = true;
        item.x_medkit = x_medkit_comp;
        RCLCPP_WARN(HandlerContext::logger(), "Component '%s' declares dependency on unknown component '%s'",
                    component_id.c_str(), dep_id.c_str());
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/depends-on";
    links["component"] = "/api/v1/components/" + component_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_component_depends_on: %s", e.what());
  }
}

// =============================================================================
// App handlers
// =============================================================================

void DiscoveryHandlers::handle_list_apps(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    // Use ThreadSafeEntityCache for consistent discovery (avoids race with data endpoints)
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto apps = cache.get_apps();

    dto::Collection<dto::AppListItem> response;
    for (const auto & app : apps) {
      dto::AppListItem item;
      item.id = app.id;
      item.name = app.name.empty() ? app.id : app.name;
      item.href = "/api/v1/apps/" + app.id;
      item.type = "app";

      if (!app.description.empty()) {
        item.description = app.description;
      }
      if (!app.tags.empty()) {
        item.tags = app.tags;
      }

      dto::XMedkitApp x_medkit_app;
      if (!app.source.empty()) {
        x_medkit_app.source = app.source;
      }
      x_medkit_app.is_online = app.is_online;
      if (!app.component_id.empty()) {
        x_medkit_app.component_id = app.component_id;
      }
      if (app.bound_fqn) {
        dto::XMedkitRos2 ros2;
        ros2.node = *app.bound_fqn;
        x_medkit_app.ros2 = ros2;
      }
      item.x_medkit = x_medkit_app;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_apps: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_app(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    // Validate entity and forward to peer if remote (aggregation support)
    auto entity_opt = ctx_.validate_entity_for_route(req, res, app_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }

    // Local entity - look up full object from cache for detail response
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto app_opt = cache.get_app(app_id);
    if (!app_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      app_opt = discovery->get_app(app_id);
    }

    if (!app_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "App not found", {{"app_id", app_id}});
      return;
    }

    const auto & app = *app_opt;

    dto::AppDetail detail;
    detail.id = app.id;
    detail.name = app.name;
    detail.type = "app";

    if (!app.description.empty()) {
      detail.description = app.description;
    }
    if (!app.translation_id.empty()) {
      detail.translation_id = app.translation_id;
    }
    if (!app.tags.empty()) {
      detail.tags = app.tags;
    }

    std::string base_uri = "/api/v1/apps/" + app.id;
    detail.data = base_uri + "/data";
    detail.operations = base_uri + "/operations";
    detail.configurations = base_uri + "/configurations";
    detail.faults = base_uri + "/faults";
    detail.logs = base_uri + "/logs";
    detail.bulk_data = base_uri + "/bulk-data";
    detail.cyclic_subscriptions = base_uri + "/cyclic-subscriptions";
    detail.triggers = base_uri + "/triggers";

    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      detail.scripts = base_uri + "/scripts";
    }

    if (!app.component_id.empty()) {
      detail.is_located_on = "/api/v1/components/" + app.component_id;
      detail.belongs_to = base_uri + "/belongs-to";
    }

    if (!app.depends_on.empty()) {
      detail.depends_on = base_uri + "/depends-on";
    }

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS,       Cap::FAULTS,
                             Cap::LOGS, Cap::BULK_DATA,  Cap::CYCLIC_SUBSCRIPTIONS, Cap::TRIGGERS};
    // Relationship endpoints are gated the same way as the top-level URI keys
    // above so the three advertising surfaces (top-level URIs, `_links`,
    // `capabilities` array) describe the same set of available collections.
    if (!app.component_id.empty()) {
      caps.push_back(Cap::IS_LOCATED_ON);
      caps.push_back(Cap::BELONGS_TO);
    }
    if (!app.depends_on.empty()) {
      caps.push_back(Cap::DEPENDS_ON);
    }
    if (ctx_.node()->get_script_manager() && ctx_.node()->get_script_manager()->has_backend()) {
      caps.push_back(Cap::SCRIPTS);
    }
    if (ctx_.node() && ctx_.node()->get_lock_manager()) {
      caps.push_back(Cap::LOCKS);
    }
    auto app_caps = CapabilityBuilder::build_capabilities("apps", app.id, caps);
    append_plugin_capabilities(app_caps, "apps", app.id, SovdEntityType::APP, ctx_.node());
    detail.capabilities = app_caps;

    LinksBuilder links;
    links.self("/api/v1/apps/" + app.id).collection("/api/v1/apps");
    if (!app.component_id.empty()) {
      links.add("is-located-on", "/api/v1/components/" + app.component_id);
      links.add("belongs-to", base_uri + "/belongs-to");
    }
    auto links_json = links.build();

    if (!app.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : app.depends_on) {
        depends_links.push_back("/api/v1/apps/" + dep_id);
      }
      links_json["depends-on"] = depends_links;
    }
    detail.links = links_json;

    dto::XMedkitApp x_medkit_app;
    if (!app.source.empty()) {
      x_medkit_app.source = app.source;
    }
    x_medkit_app.is_online = app.is_online;
    if (app.bound_fqn) {
      dto::XMedkitRos2 ros2;
      ros2.node = *app.bound_fqn;
      x_medkit_app.ros2 = ros2;
    }
    if (!app.component_id.empty()) {
      x_medkit_app.component_id = app.component_id;
    }
    if (!app.contributors.empty()) {
      x_medkit_app.contributors = sorted_contributors(app.contributors);
    }
    detail.x_medkit = x_medkit_app;

    HandlerContext::send_dto(res, detail);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_app: %s", e.what());
  }
}

void DiscoveryHandlers::handle_app_depends_on(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    // validate_entity_for_route forwards the request to the owning peer when
    // the app is remote (aggregation setup) - validate_entity_id alone would
    // resolve locally only and return 404 for remote apps.
    auto entity_opt = ctx_.validate_entity_for_route(req, res, app_id);
    if (!entity_opt) {
      return;
    }

    // Atomic snapshot keeps the app + every dependency resolved in the same
    // cache generation; a writer refresh between per-dependency lookups could
    // otherwise yield a mix from N different generations.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto snapshot = cache.get_app_with_dependencies(app_id);
    if (!snapshot.app) {
      auto discovery = ctx_.node()->get_discovery_manager();
      snapshot.app = discovery->get_app(app_id);
      if (snapshot.app) {
        snapshot.dependencies.reserve(snapshot.app->depends_on.size());
        for (const auto & dep_id : snapshot.app->depends_on) {
          snapshot.dependencies.emplace_back(dep_id, discovery->get_app(dep_id));
        }
      }
    }

    if (!snapshot.app) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "App not found", {{"app_id", app_id}});
      return;
    }

    dto::Collection<dto::AppListItem> response;
    for (const auto & [dep_id, dep_opt] : snapshot.dependencies) {
      dto::AppListItem item;
      item.id = dep_id;
      item.href = "/api/v1/apps/" + dep_id;
      item.type = "app";

      if (dep_opt) {
        item.name = dep_opt->name.empty() ? dep_id : dep_opt->name;

        dto::XMedkitApp x_medkit_app;
        if (!dep_opt->source.empty()) {
          x_medkit_app.source = dep_opt->source;
        }
        x_medkit_app.is_online = dep_opt->is_online;
        item.x_medkit = x_medkit_app;
      } else {
        item.name = dep_id;
        dto::XMedkitApp x_medkit_app;
        x_medkit_app.missing = true;
        item.x_medkit = x_medkit_app;
        RCLCPP_WARN(HandlerContext::logger(), "App '%s' declares dependency on unknown app '%s'", app_id.c_str(),
                    dep_id.c_str());
      }

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/depends-on";
    links["app"] = "/api/v1/apps/" + app_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_app_depends_on: %s", e.what());
  }
}

void DiscoveryHandlers::handle_app_belongs_to(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    // Forward to peer if app is remote (aggregation setup); locally
    // resolved entity falls through to the cache/discovery lookup below.
    auto entity_opt = ctx_.validate_entity_for_route(req, res, app_id);
    if (!entity_opt) {
      return;
    }

    // Atomic snapshot avoids a mixed-generation view of App -> Component ->
    // Area across concurrent cache refreshes. Fall back to the discovery
    // manager (no atomic guarantee, but rare path) when the app is not in
    // the cache.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto snapshot = cache.get_app_with_links(app_id);
    if (!snapshot.app) {
      auto discovery = ctx_.node()->get_discovery_manager();
      snapshot.app = discovery->get_app(app_id);
      if (snapshot.app && !snapshot.app->component_id.empty()) {
        snapshot.component = discovery->get_component(snapshot.app->component_id);
        if (snapshot.component && !snapshot.component->area.empty()) {
          snapshot.area = discovery->get_area(snapshot.component->area);
        }
      }
    }

    if (!snapshot.app) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "App not found", {{"app_id", app_id}});
      return;
    }

    dto::Collection<dto::AreaListItem> response;
    const auto & app = *snapshot.app;

    if (!app.component_id.empty()) {
      if (!snapshot.component) {
        // Mirror handle_app_is_located_on: surface broken parent reference
        // with x-medkit.missing=true so HATEOAS clients can distinguish
        // 'no parent area' from 'manifest broken, component gone'.
        dto::AreaListItem item;
        item.id = "";
        item.name = "<unknown area>";
        item.href = "";
        item.type = "area";
        dto::XMedkitArea x_medkit_area;
        x_medkit_area.missing = true;
        x_medkit_area.unresolved_component = app.component_id;
        item.x_medkit = x_medkit_area;
        response.items.push_back(std::move(item));
        RCLCPP_WARN(HandlerContext::logger(), "App '%s' belongs-to unresolvable: parent component '%s' is unknown",
                    app_id.c_str(), app.component_id.c_str());
      } else if (!snapshot.component->area.empty()) {
        const auto & area_id_ref = snapshot.component->area;
        dto::AreaListItem item;
        item.id = area_id_ref;
        item.href = "/api/v1/areas/" + area_id_ref;
        item.type = "area";

        if (snapshot.area) {
          item.name = snapshot.area->name.empty() ? area_id_ref : snapshot.area->name;
        } else {
          item.name = area_id_ref;
          dto::XMedkitArea x_medkit_area;
          x_medkit_area.missing = true;
          item.x_medkit = x_medkit_area;
          RCLCPP_WARN(HandlerContext::logger(), "App '%s' belongs to unknown area '%s' (via component '%s')",
                      app_id.c_str(), area_id_ref.c_str(), app.component_id.c_str());
        }
        response.items.push_back(std::move(item));
      }
      // If component resolves but has no area_id assigned, items stays
      // empty - that is a legitimate manifest configuration (component
      // without parent area), not a broken reference.
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/belongs-to";
    links["app"] = "/api/v1/apps/" + app_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_app_belongs_to: %s", e.what());
  }
}

void DiscoveryHandlers::handle_app_is_located_on(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    // Forward to peer if app is remote (aggregation setup).
    auto entity_opt = ctx_.validate_entity_for_route(req, res, app_id);
    if (!entity_opt) {
      return;
    }

    // Atomic snapshot keeps app -> component consistent across a concurrent
    // cache refresh. Fall back to discovery manager when the app is not in
    // the cache.
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto snapshot = cache.get_app_with_links(app_id);
    if (!snapshot.app) {
      auto discovery = ctx_.node()->get_discovery_manager();
      snapshot.app = discovery->get_app(app_id);
      if (snapshot.app && !snapshot.app->component_id.empty()) {
        snapshot.component = discovery->get_component(snapshot.app->component_id);
      }
    }

    if (!snapshot.app) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "App not found", {{"app_id", app_id}});
      return;
    }

    dto::Collection<dto::ComponentListItem> response;
    const auto & app = *snapshot.app;

    if (!app.component_id.empty()) {
      if (snapshot.component) {
        dto::ComponentListItem item;
        item.id = snapshot.component->id;
        item.name = snapshot.component->name.empty() ? snapshot.component->id : snapshot.component->name;
        item.href = "/api/v1/components/" + snapshot.component->id;
        item.type = "component";
        response.items.push_back(std::move(item));
      } else {
        dto::ComponentListItem item;
        item.id = app.component_id;
        item.name = app.component_id;
        item.href = "/api/v1/components/" + app.component_id;
        item.type = "component";

        dto::XMedkitComponent x_medkit_comp;
        x_medkit_comp.missing = true;
        item.x_medkit = x_medkit_comp;
        response.items.push_back(std::move(item));

        RCLCPP_WARN(HandlerContext::logger(), "App '%s' references unknown component '%s'", app_id.c_str(),
                    app.component_id.c_str());
      }
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/is-located-on";
    links["app"] = "/api/v1/apps/" + app_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_app_is_located_on: %s", e.what());
  }
}

// =============================================================================
// Function handlers
// =============================================================================

void DiscoveryHandlers::handle_list_functions(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    // Use ThreadSafeEntityCache for consistent discovery (avoids race with data endpoints)
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto functions = cache.get_functions();

    dto::Collection<dto::FunctionListItem> response;
    for (const auto & func : functions) {
      dto::FunctionListItem item;
      item.id = func.id;
      item.name = func.name.empty() ? func.id : func.name;
      item.href = "/api/v1/functions/" + func.id;
      item.type = "function";

      if (!func.description.empty()) {
        item.description = func.description;
      }
      if (!func.tags.empty()) {
        item.tags = func.tags;
      }

      dto::XMedkitFunction x_medkit_func;
      if (!func.source.empty()) {
        x_medkit_func.source = func.source;
      }
      item.x_medkit = x_medkit_func;

      response.items.push_back(std::move(item));
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_functions: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_function(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    // Validate entity and forward to peer if remote (aggregation support)
    auto entity_opt = ctx_.validate_entity_for_route(req, res, function_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }

    // Local entity - look up full object from cache for detail response
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto func_opt = cache.get_function(function_id);
    if (!func_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      func_opt = discovery->get_function(function_id);
    }

    if (!func_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Function not found", {{"function_id", function_id}});
      return;
    }

    const auto & func = *func_opt;

    dto::FunctionDetail detail;
    detail.id = func.id;
    detail.name = func.name.empty() ? func.id : func.name;
    detail.type = "function";

    if (!func.description.empty()) {
      detail.description = func.description;
    }
    if (!func.translation_id.empty()) {
      detail.translation_id = func.translation_id;
    }
    if (!func.tags.empty()) {
      detail.tags = func.tags;
    }

    std::string base_uri = "/api/v1/functions/" + func.id;
    detail.hosts = base_uri + "/hosts";
    detail.data = base_uri + "/data";
    detail.operations = base_uri + "/operations";
    detail.configurations = base_uri + "/configurations";
    detail.faults = base_uri + "/faults";
    detail.logs = base_uri + "/logs";
    detail.bulk_data = base_uri + "/bulk-data";
    detail.x_medkit_graph = base_uri + "/x-medkit-graph";
    detail.cyclic_subscriptions = base_uri + "/cyclic-subscriptions";
    detail.triggers = base_uri + "/triggers";

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::HOSTS, Cap::DATA,      Cap::OPERATIONS,           Cap::CONFIGURATIONS, Cap::FAULTS,
                             Cap::LOGS,  Cap::BULK_DATA, Cap::CYCLIC_SUBSCRIPTIONS, Cap::TRIGGERS};
    auto func_caps = CapabilityBuilder::build_capabilities("functions", func.id, caps);
    append_plugin_capabilities(func_caps, "functions", func.id, SovdEntityType::FUNCTION, ctx_.node());
    detail.capabilities = func_caps;

    LinksBuilder links;
    links.self("/api/v1/functions/" + func.id).collection("/api/v1/functions");
    auto links_json = links.build();

    if (!func.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : func.depends_on) {
        depends_links.push_back("/api/v1/functions/" + dep_id);
      }
      links_json["depends-on"] = depends_links;
    }
    detail.links = links_json;

    dto::XMedkitFunction x_medkit_func;
    if (!func.source.empty()) {
      x_medkit_func.source = func.source;
    }
    if (!func.hosts.empty()) {
      x_medkit_func.hosts = func.hosts;
    }
    if (!func.description.empty()) {
      x_medkit_func.description = func.description;
    }
    if (!func.contributors.empty()) {
      x_medkit_func.contributors = sorted_contributors(func.contributors);
    }
    detail.x_medkit = x_medkit_func;

    HandlerContext::send_dto(res, detail);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_function: %s", e.what());
  }
}

void DiscoveryHandlers::handle_function_hosts(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(function_id);
    if (!validation_result) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid function ID",
                                 {{"details", validation_result.error()}, {"function_id", function_id}});
      return;
    }

    // Read from cache (has merged entities from aggregation with combined hosts)
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto func_opt = cache.get_function(function_id);
    if (!func_opt) {
      auto discovery = ctx_.node()->get_discovery_manager();
      func_opt = discovery->get_function(function_id);
    }

    if (!func_opt) {
      HandlerContext::send_error(res, 404, ERR_ENTITY_NOT_FOUND, "Function not found", {{"function_id", function_id}});
      return;
    }

    // Use the Function's hosts list directly (includes merged hosts from all peers)
    const auto & host_ids = func_opt->hosts;

    dto::Collection<dto::AppListItem> response;
    for (const auto & app_id : host_ids) {
      auto app_opt = cache.get_app(app_id);
      if (app_opt) {
        dto::AppListItem item;
        item.id = app_opt->id;
        item.name = app_opt->name.empty() ? app_opt->id : app_opt->name;
        item.href = "/api/v1/apps/" + app_opt->id;
        item.type = "app";

        dto::XMedkitApp x_medkit_app;
        x_medkit_app.is_online = app_opt->is_online;
        if (!app_opt->source.empty()) {
          x_medkit_app.source = app_opt->source;
        }
        if (app_opt->bound_fqn) {
          dto::XMedkitRos2 ros2;
          ros2.node = *app_opt->bound_fqn;
          x_medkit_app.ros2 = ros2;
        }
        item.x_medkit = x_medkit_app;

        response.items.push_back(std::move(item));
      }
    }

    dto::XMedkitCollection col_ext;
    col_ext.total_count = response.items.size();
    response.x_medkit = col_ext;

    json links;
    links["self"] = "/api/v1/functions/" + function_id + "/hosts";
    links["function"] = "/api/v1/functions/" + function_id;
    response.links = links;

    HandlerContext::send_dto(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Internal server error", {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_function_hosts: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
