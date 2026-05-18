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

#include <cstddef>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/enums.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// Area DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// AreaListItem - shape emitted per item in handle_list_areas "items" array.
//
// Wire keys:  id, name, href, type, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct AreaListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitArea> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AreaListItem> =
    std::make_tuple(field("id", &AreaListItem::id), field("name", &AreaListItem::name),
                    field("href", &AreaListItem::href), field_enum("type", &AreaListItem::type, kEntityTypeValues),
                    field("description", &AreaListItem::description), field("tags", &AreaListItem::tags),
                    field("x-medkit", &AreaListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<AreaListItem> = "AreaListItem";

// -----------------------------------------------------------------------------
// AreaDetail - shape emitted by handle_get_area.
//
// Wire keys:
//   id, name, description?, tags?,
//   subareas, components, contains, data, operations, configurations, faults,
//   logs, bulk-data, triggers,
//   capabilities (free-form JSON array of {name, href} objects),
//   _links (free-form JSON object),
//   x-medkit
// -----------------------------------------------------------------------------
struct AreaDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  // Resource collection URI fields (always present in detail response)
  std::string subareas;
  std::string components;
  std::string contains;
  std::string data;
  std::string operations;
  std::string configurations;
  std::string faults;
  std::string logs;
  std::string bulk_data;  // wire key: "bulk-data"
  std::string triggers;
  // Free-form fields
  std::optional<nlohmann::json> capabilities;  // array of {name, href}
  std::optional<nlohmann::json> links;         // wire key: "_links"
  std::optional<XMedkitArea> x_medkit;         // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AreaDetail> =
    std::make_tuple(field("id", &AreaDetail::id), field("name", &AreaDetail::name),
                    field_enum("type", &AreaDetail::type, kEntityTypeValues),
                    field("description", &AreaDetail::description), field("tags", &AreaDetail::tags),
                    field("subareas", &AreaDetail::subareas), field("components", &AreaDetail::components),
                    field("contains", &AreaDetail::contains), field("data", &AreaDetail::data),
                    field("operations", &AreaDetail::operations), field("configurations", &AreaDetail::configurations),
                    field("faults", &AreaDetail::faults), field("logs", &AreaDetail::logs),
                    field("bulk-data", &AreaDetail::bulk_data), field("triggers", &AreaDetail::triggers),
                    field("capabilities", &AreaDetail::capabilities), field("_links", &AreaDetail::links),
                    field("x-medkit", &AreaDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<AreaDetail> = "AreaDetail";

// =============================================================================
// Component DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// ComponentListItem - shape emitted per item in handle_list_components "items".
//
// Wire keys:  id, name, href, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct ComponentListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitComponent> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<ComponentListItem> =
    std::make_tuple(field("id", &ComponentListItem::id), field("name", &ComponentListItem::name),
                    field("href", &ComponentListItem::href),
                    field_enum("type", &ComponentListItem::type, kEntityTypeValues),
                    field("description", &ComponentListItem::description), field("tags", &ComponentListItem::tags),
                    field("x-medkit", &ComponentListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<ComponentListItem> = "ComponentListItem";

// -----------------------------------------------------------------------------
// ComponentDetail - shape emitted by handle_get_component.
//
// Wire keys:
//   id, name, description?, tags?,
//   data, operations, configurations, faults, subcomponents, hosts, logs,
//   bulk-data, cyclic-subscriptions, triggers,
//   scripts? (conditional on script backend), depends-on? (conditional),
//   belongs-to? (conditional on area), is-located-on? (not present here - app only),
//   capabilities (free-form JSON array),
//   _links (free-form JSON object),
//   x-medkit
// -----------------------------------------------------------------------------
struct ComponentDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  // Always-present resource collection URIs
  std::string data;
  std::string operations;
  std::string configurations;
  std::string faults;
  std::string subcomponents;
  std::string hosts;
  std::string logs;
  std::string bulk_data;             // wire key: "bulk-data"
  std::string cyclic_subscriptions;  // wire key: "cyclic-subscriptions"
  std::string triggers;
  // Conditional URI fields
  std::optional<std::string> scripts;     // present only with script backend
  std::optional<std::string> depends_on;  // wire key: "depends-on"
  std::optional<std::string> belongs_to;  // wire key: "belongs-to"
  // Free-form fields
  std::optional<nlohmann::json> capabilities;
  std::optional<nlohmann::json> links;       // wire key: "_links"
  std::optional<XMedkitComponent> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<ComponentDetail> = std::make_tuple(
    field("id", &ComponentDetail::id), field("name", &ComponentDetail::name),
    field_enum("type", &ComponentDetail::type, kEntityTypeValues), field("description", &ComponentDetail::description),
    field("tags", &ComponentDetail::tags), field("data", &ComponentDetail::data),
    field("operations", &ComponentDetail::operations), field("configurations", &ComponentDetail::configurations),
    field("faults", &ComponentDetail::faults), field("subcomponents", &ComponentDetail::subcomponents),
    field("hosts", &ComponentDetail::hosts), field("logs", &ComponentDetail::logs),
    field("bulk-data", &ComponentDetail::bulk_data),
    field("cyclic-subscriptions", &ComponentDetail::cyclic_subscriptions),
    field("triggers", &ComponentDetail::triggers), field("scripts", &ComponentDetail::scripts),
    field("depends-on", &ComponentDetail::depends_on), field("belongs-to", &ComponentDetail::belongs_to),
    field("capabilities", &ComponentDetail::capabilities), field("_links", &ComponentDetail::links),
    field("x-medkit", &ComponentDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<ComponentDetail> = "ComponentDetail";

// =============================================================================
// App DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// AppListItem - shape emitted per item in handle_list_apps "items" array.
//
// Wire keys:  id, name, href, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct AppListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitApp> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AppListItem> =
    std::make_tuple(field("id", &AppListItem::id), field("name", &AppListItem::name), field("href", &AppListItem::href),
                    field_enum("type", &AppListItem::type, kEntityTypeValues),
                    field("description", &AppListItem::description), field("tags", &AppListItem::tags),
                    field("x-medkit", &AppListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<AppListItem> = "AppListItem";

// -----------------------------------------------------------------------------
// AppDetail - shape emitted by handle_get_app.
//
// Wire keys:
//   id, name, description?, translation_id?, tags?,
//   data, operations, configurations, faults, logs, bulk-data,
//   cyclic-subscriptions, triggers,
//   scripts? (conditional on script backend),
//   is-located-on? (conditional on component_id),
//   belongs-to? (conditional on component_id),
//   depends-on? (conditional on depends_on list),
//   capabilities (free-form JSON array),
//   _links (free-form JSON object),
//   x-medkit
// -----------------------------------------------------------------------------
struct AppDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::string> translation_id;
  std::optional<std::vector<std::string>> tags;
  // Always-present resource collection URIs
  std::string data;
  std::string operations;
  std::string configurations;
  std::string faults;
  std::string logs;
  std::string bulk_data;             // wire key: "bulk-data"
  std::string cyclic_subscriptions;  // wire key: "cyclic-subscriptions"
  std::string triggers;
  // Conditional URI fields
  std::optional<std::string> scripts;        // present only with script backend
  std::optional<std::string> is_located_on;  // wire key: "is-located-on"
  std::optional<std::string> belongs_to;     // wire key: "belongs-to"
  std::optional<std::string> depends_on;     // wire key: "depends-on"
  // Free-form fields
  std::optional<nlohmann::json> capabilities;
  std::optional<nlohmann::json> links;  // wire key: "_links"
  std::optional<XMedkitApp> x_medkit;   // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<AppDetail> =
    std::make_tuple(field("id", &AppDetail::id), field("name", &AppDetail::name),
                    field_enum("type", &AppDetail::type, kEntityTypeValues),
                    field("description", &AppDetail::description), field("translation_id", &AppDetail::translation_id),
                    field("tags", &AppDetail::tags), field("data", &AppDetail::data),
                    field("operations", &AppDetail::operations), field("configurations", &AppDetail::configurations),
                    field("faults", &AppDetail::faults), field("logs", &AppDetail::logs),
                    field("bulk-data", &AppDetail::bulk_data),
                    field("cyclic-subscriptions", &AppDetail::cyclic_subscriptions),
                    field("triggers", &AppDetail::triggers), field("scripts", &AppDetail::scripts),
                    field("is-located-on", &AppDetail::is_located_on), field("belongs-to", &AppDetail::belongs_to),
                    field("depends-on", &AppDetail::depends_on), field("capabilities", &AppDetail::capabilities),
                    field("_links", &AppDetail::links), field("x-medkit", &AppDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<AppDetail> = "AppDetail";

// =============================================================================
// Function DTOs
// =============================================================================

// -----------------------------------------------------------------------------
// FunctionListItem - shape emitted per item in handle_list_functions "items".
//
// Wire keys:  id, name, href, description?, tags?, x-medkit
// -----------------------------------------------------------------------------
struct FunctionListItem {
  std::string id;
  std::string name;
  std::string href;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::vector<std::string>> tags;
  std::optional<XMedkitFunction> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FunctionListItem> =
    std::make_tuple(field("id", &FunctionListItem::id), field("name", &FunctionListItem::name),
                    field("href", &FunctionListItem::href),
                    field_enum("type", &FunctionListItem::type, kEntityTypeValues),
                    field("description", &FunctionListItem::description), field("tags", &FunctionListItem::tags),
                    field("x-medkit", &FunctionListItem::x_medkit));

template <>
inline constexpr std::string_view dto_name<FunctionListItem> = "FunctionListItem";

// -----------------------------------------------------------------------------
// FunctionDetail - shape emitted by handle_get_function.
//
// Wire keys:
//   id, name, description?, translation_id?, tags?,
//   hosts, data, operations, configurations, faults, logs, bulk-data,
//   x-medkit-graph, cyclic-subscriptions, triggers,
//   capabilities (free-form JSON array),
//   _links (free-form JSON object),
//   x-medkit
// -----------------------------------------------------------------------------
struct FunctionDetail {
  std::string id;
  std::string name;
  std::string type;  // entity type discriminator (area|component|app|function)
  std::optional<std::string> description;
  std::optional<std::string> translation_id;
  std::optional<std::vector<std::string>> tags;
  // Always-present resource collection URIs
  std::string hosts;
  std::string data;
  std::string operations;
  std::string configurations;
  std::string faults;
  std::string logs;
  std::string bulk_data;             // wire key: "bulk-data"
  std::string x_medkit_graph;        // wire key: "x-medkit-graph"
  std::string cyclic_subscriptions;  // wire key: "cyclic-subscriptions"
  std::string triggers;
  // Free-form fields
  std::optional<nlohmann::json> capabilities;
  std::optional<nlohmann::json> links;      // wire key: "_links"
  std::optional<XMedkitFunction> x_medkit;  // wire key: "x-medkit"
};

template <>
inline constexpr auto dto_fields<FunctionDetail> = std::make_tuple(
    field("id", &FunctionDetail::id), field("name", &FunctionDetail::name),
    field_enum("type", &FunctionDetail::type, kEntityTypeValues), field("description", &FunctionDetail::description),
    field("translation_id", &FunctionDetail::translation_id), field("tags", &FunctionDetail::tags),
    field("hosts", &FunctionDetail::hosts), field("data", &FunctionDetail::data),
    field("operations", &FunctionDetail::operations), field("configurations", &FunctionDetail::configurations),
    field("faults", &FunctionDetail::faults), field("logs", &FunctionDetail::logs),
    field("bulk-data", &FunctionDetail::bulk_data), field("x-medkit-graph", &FunctionDetail::x_medkit_graph),
    field("cyclic-subscriptions", &FunctionDetail::cyclic_subscriptions), field("triggers", &FunctionDetail::triggers),
    field("capabilities", &FunctionDetail::capabilities), field("_links", &FunctionDetail::links),
    field("x-medkit", &FunctionDetail::x_medkit));

template <>
inline constexpr std::string_view dto_name<FunctionDetail> = "FunctionDetail";

// =============================================================================
// Collection<T> wrapper
// =============================================================================

/// Generic collection wrapper used for every entity list response.
/// The "items" array element type T is one of the *ListItem DTOs above.
template <class T>
struct Collection {
  std::vector<T> items;
  std::optional<XMedkitCollection> x_medkit;  // wire key: "x-medkit"
};

template <class T>
inline constexpr auto dto_fields<Collection<T>> =
    std::make_tuple(field("items", &Collection<T>::items), field("x-medkit", &Collection<T>::x_medkit));

// dto_name per concrete instantiation (no runtime string concatenation):
template <>
inline constexpr std::string_view dto_name<Collection<AreaListItem>> = "AreaList";

template <>
inline constexpr std::string_view dto_name<Collection<ComponentListItem>> = "ComponentList";

template <>
inline constexpr std::string_view dto_name<Collection<AppListItem>> = "AppList";

template <>
inline constexpr std::string_view dto_name<Collection<FunctionListItem>> = "FunctionList";

}  // namespace dto
}  // namespace ros2_medkit_gateway
