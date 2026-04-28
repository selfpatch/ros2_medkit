// Copyright 2025 bburda
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

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"

#include <algorithm>
#include <stdexcept>

namespace ros2_medkit_gateway {

const std::unordered_map<UserRole, std::unordered_set<std::string>> & AuthConfig::get_role_permissions() {
  // Static permission map - built once
  // Format: "HTTP_METHOD:/path/pattern" where * is wildcard
  // @verifies REQ_INTEROP_086
  static const std::unordered_map<UserRole, std::unordered_set<std::string>> permissions = {
      {UserRole::VIEWER,
       {
           // Read-only access to all GET endpoints
           "GET:/api/v1/health",
           "GET:/api/v1/",
           "GET:/api/v1/version-info",
           // Discovery: entity collections and details
           "GET:/api/v1/areas",
           "GET:/api/v1/areas/*",
           "GET:/api/v1/components",
           "GET:/api/v1/components/*",
           "GET:/api/v1/apps",
           "GET:/api/v1/apps/*",
           "GET:/api/v1/functions",
           "GET:/api/v1/functions/*",
           // Discovery: relationship endpoints
           "GET:/api/v1/areas/*/components",
           "GET:/api/v1/areas/*/subareas",
           "GET:/api/v1/areas/*/contains",
           "GET:/api/v1/components/*/subcomponents",
           "GET:/api/v1/components/*/hosts",
           "GET:/api/v1/components/*/depends-on",
           "GET:/api/v1/apps/*/depends-on",
           "GET:/api/v1/functions/*/hosts",
           // Data: all entity types
           "GET:/api/v1/components/*/data",
           "GET:/api/v1/components/*/data/*",
           "GET:/api/v1/apps/*/data",
           "GET:/api/v1/apps/*/data/*",
           "GET:/api/v1/areas/*/data",
           "GET:/api/v1/areas/*/data/*",
           "GET:/api/v1/functions/*/data",
           "GET:/api/v1/functions/*/data/*",
           // Data categories and groups: all entity types
           "GET:/api/v1/components/*/data-categories",
           "GET:/api/v1/components/*/data-groups",
           "GET:/api/v1/apps/*/data-categories",
           "GET:/api/v1/apps/*/data-groups",
           "GET:/api/v1/areas/*/data-categories",
           "GET:/api/v1/areas/*/data-groups",
           "GET:/api/v1/functions/*/data-categories",
           "GET:/api/v1/functions/*/data-groups",
           // Operations: all entity types
           "GET:/api/v1/components/*/operations",
           "GET:/api/v1/components/*/operations/*",
           "GET:/api/v1/components/*/operations/*/executions",
           "GET:/api/v1/components/*/operations/*/executions/*",
           "GET:/api/v1/apps/*/operations",
           "GET:/api/v1/apps/*/operations/*",
           "GET:/api/v1/apps/*/operations/*/executions",
           "GET:/api/v1/apps/*/operations/*/executions/*",
           "GET:/api/v1/areas/*/operations",
           "GET:/api/v1/areas/*/operations/*",
           "GET:/api/v1/areas/*/operations/*/executions",
           "GET:/api/v1/areas/*/operations/*/executions/*",
           "GET:/api/v1/functions/*/operations",
           "GET:/api/v1/functions/*/operations/*",
           "GET:/api/v1/functions/*/operations/*/executions",
           "GET:/api/v1/functions/*/operations/*/executions/*",
           // Configurations: all entity types
           "GET:/api/v1/components/*/configurations",
           "GET:/api/v1/components/*/configurations/*",
           "GET:/api/v1/apps/*/configurations",
           "GET:/api/v1/apps/*/configurations/*",
           "GET:/api/v1/areas/*/configurations",
           "GET:/api/v1/areas/*/configurations/*",
           "GET:/api/v1/functions/*/configurations",
           "GET:/api/v1/functions/*/configurations/*",
           // Faults: per-entity (all entity types)
           "GET:/api/v1/components/*/faults",
           "GET:/api/v1/components/*/faults/*",
           "GET:/api/v1/apps/*/faults",
           "GET:/api/v1/apps/*/faults/*",
           "GET:/api/v1/areas/*/faults",
           "GET:/api/v1/areas/*/faults/*",
           "GET:/api/v1/functions/*/faults",
           "GET:/api/v1/functions/*/faults/*",
           // Faults: global
           "GET:/api/v1/faults",
           "GET:/api/v1/faults/stream",
           // Logs: all entity types
           "GET:/api/v1/components/*/logs",
           "GET:/api/v1/components/*/logs/configuration",
           "GET:/api/v1/apps/*/logs",
           "GET:/api/v1/apps/*/logs/configuration",
           "GET:/api/v1/areas/*/logs",
           "GET:/api/v1/areas/*/logs/configuration",
           "GET:/api/v1/functions/*/logs",
           "GET:/api/v1/functions/*/logs/configuration",
           // Bulk data: all entity types (read-only)
           "GET:/api/v1/components/*/bulk-data",
           "GET:/api/v1/components/*/bulk-data/*",
           "GET:/api/v1/components/*/bulk-data/*/*",
           "GET:/api/v1/apps/*/bulk-data",
           "GET:/api/v1/apps/*/bulk-data/*",
           "GET:/api/v1/apps/*/bulk-data/*/*",
           "GET:/api/v1/areas/*/bulk-data",
           "GET:/api/v1/areas/*/bulk-data/*",
           "GET:/api/v1/areas/*/bulk-data/*/*",
           "GET:/api/v1/functions/*/bulk-data",
           "GET:/api/v1/functions/*/bulk-data/*",
           "GET:/api/v1/functions/*/bulk-data/*/*",
           // Bulk data: nested entities (subareas, subcomponents)
           "GET:/api/v1/areas/*/subareas/*/bulk-data",
           "GET:/api/v1/areas/*/subareas/*/bulk-data/*",
           "GET:/api/v1/areas/*/subareas/*/bulk-data/*/*",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data/*",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data/*/*",
           // Cyclic subscriptions: apps, components, functions (read-only)
           "GET:/api/v1/components/*/cyclic-subscriptions",
           "GET:/api/v1/components/*/cyclic-subscriptions/*",
           "GET:/api/v1/components/*/cyclic-subscriptions/*/events",
           "GET:/api/v1/apps/*/cyclic-subscriptions",
           "GET:/api/v1/apps/*/cyclic-subscriptions/*",
           "GET:/api/v1/apps/*/cyclic-subscriptions/*/events",
           "GET:/api/v1/functions/*/cyclic-subscriptions",
           "GET:/api/v1/functions/*/cyclic-subscriptions/*",
           "GET:/api/v1/functions/*/cyclic-subscriptions/*/events",
           // Locks: components and apps (read-only)
           "GET:/api/v1/components/*/locks",
           "GET:/api/v1/components/*/locks/*",
           "GET:/api/v1/apps/*/locks",
           "GET:/api/v1/apps/*/locks/*",
           // Updates (read-only)
           "GET:/api/v1/updates",
           "GET:/api/v1/updates/*",
           "GET:/api/v1/updates/*/status",
           // Scripts: read-only (list, details, execution status)
           "GET:/api/v1/components/*/scripts",
           "GET:/api/v1/components/*/scripts/*",
           "GET:/api/v1/apps/*/scripts",
           "GET:/api/v1/apps/*/scripts/*",
           "GET:/api/v1/components/*/scripts/*/executions/*",
           "GET:/api/v1/apps/*/scripts/*/executions/*",
           // Docs
           "GET:/api/v1/docs",
       }},
      {UserRole::OPERATOR,
       {
           // Everything VIEWER can do, plus:
           // Read-only access (inherited from VIEWER)
           "GET:/api/v1/health",
           "GET:/api/v1/",
           "GET:/api/v1/version-info",
           // Discovery: entity collections and details
           "GET:/api/v1/areas",
           "GET:/api/v1/areas/*",
           "GET:/api/v1/components",
           "GET:/api/v1/components/*",
           "GET:/api/v1/apps",
           "GET:/api/v1/apps/*",
           "GET:/api/v1/functions",
           "GET:/api/v1/functions/*",
           // Discovery: relationship endpoints
           "GET:/api/v1/areas/*/components",
           "GET:/api/v1/areas/*/subareas",
           "GET:/api/v1/areas/*/contains",
           "GET:/api/v1/components/*/subcomponents",
           "GET:/api/v1/components/*/hosts",
           "GET:/api/v1/components/*/depends-on",
           "GET:/api/v1/apps/*/depends-on",
           "GET:/api/v1/functions/*/hosts",
           // Data: all entity types (read)
           "GET:/api/v1/components/*/data",
           "GET:/api/v1/components/*/data/*",
           "GET:/api/v1/apps/*/data",
           "GET:/api/v1/apps/*/data/*",
           "GET:/api/v1/areas/*/data",
           "GET:/api/v1/areas/*/data/*",
           "GET:/api/v1/functions/*/data",
           "GET:/api/v1/functions/*/data/*",
           // Data categories and groups: all entity types
           "GET:/api/v1/components/*/data-categories",
           "GET:/api/v1/components/*/data-groups",
           "GET:/api/v1/apps/*/data-categories",
           "GET:/api/v1/apps/*/data-groups",
           "GET:/api/v1/areas/*/data-categories",
           "GET:/api/v1/areas/*/data-groups",
           "GET:/api/v1/functions/*/data-categories",
           "GET:/api/v1/functions/*/data-groups",
           // Operations: all entity types (read)
           "GET:/api/v1/components/*/operations",
           "GET:/api/v1/components/*/operations/*",
           "GET:/api/v1/components/*/operations/*/executions",
           "GET:/api/v1/components/*/operations/*/executions/*",
           "GET:/api/v1/apps/*/operations",
           "GET:/api/v1/apps/*/operations/*",
           "GET:/api/v1/apps/*/operations/*/executions",
           "GET:/api/v1/apps/*/operations/*/executions/*",
           "GET:/api/v1/areas/*/operations",
           "GET:/api/v1/areas/*/operations/*",
           "GET:/api/v1/areas/*/operations/*/executions",
           "GET:/api/v1/areas/*/operations/*/executions/*",
           "GET:/api/v1/functions/*/operations",
           "GET:/api/v1/functions/*/operations/*",
           "GET:/api/v1/functions/*/operations/*/executions",
           "GET:/api/v1/functions/*/operations/*/executions/*",
           // Configurations: all entity types (read-only)
           "GET:/api/v1/components/*/configurations",
           "GET:/api/v1/components/*/configurations/*",
           "GET:/api/v1/apps/*/configurations",
           "GET:/api/v1/apps/*/configurations/*",
           "GET:/api/v1/areas/*/configurations",
           "GET:/api/v1/areas/*/configurations/*",
           "GET:/api/v1/functions/*/configurations",
           "GET:/api/v1/functions/*/configurations/*",
           // Faults: per-entity (all entity types, read)
           "GET:/api/v1/components/*/faults",
           "GET:/api/v1/components/*/faults/*",
           "GET:/api/v1/apps/*/faults",
           "GET:/api/v1/apps/*/faults/*",
           "GET:/api/v1/areas/*/faults",
           "GET:/api/v1/areas/*/faults/*",
           "GET:/api/v1/functions/*/faults",
           "GET:/api/v1/functions/*/faults/*",
           // Faults: global
           "GET:/api/v1/faults",
           "GET:/api/v1/faults/stream",
           // Logs: all entity types (read)
           "GET:/api/v1/components/*/logs",
           "GET:/api/v1/components/*/logs/configuration",
           "GET:/api/v1/apps/*/logs",
           "GET:/api/v1/apps/*/logs/configuration",
           "GET:/api/v1/areas/*/logs",
           "GET:/api/v1/areas/*/logs/configuration",
           "GET:/api/v1/functions/*/logs",
           "GET:/api/v1/functions/*/logs/configuration",
           // Bulk data: all entity types (read-only)
           "GET:/api/v1/components/*/bulk-data",
           "GET:/api/v1/components/*/bulk-data/*",
           "GET:/api/v1/components/*/bulk-data/*/*",
           "GET:/api/v1/apps/*/bulk-data",
           "GET:/api/v1/apps/*/bulk-data/*",
           "GET:/api/v1/apps/*/bulk-data/*/*",
           "GET:/api/v1/areas/*/bulk-data",
           "GET:/api/v1/areas/*/bulk-data/*",
           "GET:/api/v1/areas/*/bulk-data/*/*",
           "GET:/api/v1/functions/*/bulk-data",
           "GET:/api/v1/functions/*/bulk-data/*",
           "GET:/api/v1/functions/*/bulk-data/*/*",
           // Bulk data: nested entities (subareas, subcomponents)
           "GET:/api/v1/areas/*/subareas/*/bulk-data",
           "GET:/api/v1/areas/*/subareas/*/bulk-data/*",
           "GET:/api/v1/areas/*/subareas/*/bulk-data/*/*",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data/*",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data/*/*",
           // Cyclic subscriptions: apps, components, functions (read-only)
           "GET:/api/v1/components/*/cyclic-subscriptions",
           "GET:/api/v1/components/*/cyclic-subscriptions/*",
           "GET:/api/v1/components/*/cyclic-subscriptions/*/events",
           "GET:/api/v1/apps/*/cyclic-subscriptions",
           "GET:/api/v1/apps/*/cyclic-subscriptions/*",
           "GET:/api/v1/apps/*/cyclic-subscriptions/*/events",
           "GET:/api/v1/functions/*/cyclic-subscriptions",
           "GET:/api/v1/functions/*/cyclic-subscriptions/*",
           "GET:/api/v1/functions/*/cyclic-subscriptions/*/events",
           // Locks: components and apps (read-only)
           "GET:/api/v1/components/*/locks",
           "GET:/api/v1/components/*/locks/*",
           "GET:/api/v1/apps/*/locks",
           "GET:/api/v1/apps/*/locks/*",
           // Updates (read-only)
           "GET:/api/v1/updates",
           "GET:/api/v1/updates/*",
           "GET:/api/v1/updates/*/status",
           // Scripts: read (inherited from VIEWER)
           "GET:/api/v1/components/*/scripts",
           "GET:/api/v1/components/*/scripts/*",
           "GET:/api/v1/apps/*/scripts",
           "GET:/api/v1/apps/*/scripts/*",
           "GET:/api/v1/components/*/scripts/*/executions/*",
           "GET:/api/v1/apps/*/scripts/*/executions/*",
           // Docs
           "GET:/api/v1/docs",
           // --- Operator-specific write permissions ---
           // Trigger operations: all entity types (POST)
           "POST:/api/v1/components/*/operations/*/executions",
           "POST:/api/v1/apps/*/operations/*/executions",
           "POST:/api/v1/areas/*/operations/*/executions",
           "POST:/api/v1/functions/*/operations/*/executions",
           // Update operations - stop capability: all entity types (PUT)
           "PUT:/api/v1/components/*/operations/*/executions/*",
           "PUT:/api/v1/apps/*/operations/*/executions/*",
           "PUT:/api/v1/areas/*/operations/*/executions/*",
           "PUT:/api/v1/functions/*/operations/*/executions/*",
           // Cancel actions: all entity types (DELETE on executions)
           "DELETE:/api/v1/components/*/operations/*/executions/*",
           "DELETE:/api/v1/apps/*/operations/*/executions/*",
           "DELETE:/api/v1/areas/*/operations/*/executions/*",
           "DELETE:/api/v1/functions/*/operations/*/executions/*",
           // Clear faults: all entity types (DELETE on faults)
           "DELETE:/api/v1/components/*/faults/*",
           "DELETE:/api/v1/apps/*/faults/*",
           "DELETE:/api/v1/areas/*/faults/*",
           "DELETE:/api/v1/functions/*/faults/*",
           // Clear all faults: per-entity and global
           "DELETE:/api/v1/components/*/faults",
           "DELETE:/api/v1/apps/*/faults",
           "DELETE:/api/v1/areas/*/faults",
           "DELETE:/api/v1/functions/*/faults",
           "DELETE:/api/v1/faults",
           // Publish data to topics: all entity types (PUT)
           "PUT:/api/v1/components/*/data/*",
           "PUT:/api/v1/apps/*/data/*",
           "PUT:/api/v1/areas/*/data/*",
           "PUT:/api/v1/functions/*/data/*",
           // Cyclic subscriptions: create/update/delete (apps, components, functions)
           "POST:/api/v1/components/*/cyclic-subscriptions",
           "POST:/api/v1/apps/*/cyclic-subscriptions",
           "POST:/api/v1/functions/*/cyclic-subscriptions",
           "PUT:/api/v1/components/*/cyclic-subscriptions/*",
           "PUT:/api/v1/apps/*/cyclic-subscriptions/*",
           "PUT:/api/v1/functions/*/cyclic-subscriptions/*",
           "DELETE:/api/v1/components/*/cyclic-subscriptions/*",
           "DELETE:/api/v1/apps/*/cyclic-subscriptions/*",
           "DELETE:/api/v1/functions/*/cyclic-subscriptions/*",
           // Locks: acquire/extend/release (components and apps)
           "POST:/api/v1/components/*/locks",
           "POST:/api/v1/apps/*/locks",
           "PUT:/api/v1/components/*/locks/*",
           "PUT:/api/v1/apps/*/locks/*",
           "DELETE:/api/v1/components/*/locks/*",
           "DELETE:/api/v1/apps/*/locks/*",
           // Bulk data: upload/delete (apps and components only)
           "POST:/api/v1/components/*/bulk-data/*",
           "POST:/api/v1/apps/*/bulk-data/*",
           "DELETE:/api/v1/components/*/bulk-data/*/*",
           "DELETE:/api/v1/apps/*/bulk-data/*/*",
           // Scripts: start/control/delete executions
           "POST:/api/v1/components/*/scripts/*/executions",
           "POST:/api/v1/apps/*/scripts/*/executions",
           "PUT:/api/v1/components/*/scripts/*/executions/*",
           "PUT:/api/v1/apps/*/scripts/*/executions/*",
           "DELETE:/api/v1/components/*/scripts/*/executions/*",
           "DELETE:/api/v1/apps/*/scripts/*/executions/*",
       }},
      {UserRole::CONFIGURATOR,
       {
           // Everything OPERATOR can do, plus:
           // Inherited from OPERATOR - read-only access
           "GET:/api/v1/health",
           "GET:/api/v1/",
           "GET:/api/v1/version-info",
           // Discovery: entity collections and details
           "GET:/api/v1/areas",
           "GET:/api/v1/areas/*",
           "GET:/api/v1/components",
           "GET:/api/v1/components/*",
           "GET:/api/v1/apps",
           "GET:/api/v1/apps/*",
           "GET:/api/v1/functions",
           "GET:/api/v1/functions/*",
           // Discovery: relationship endpoints
           "GET:/api/v1/areas/*/components",
           "GET:/api/v1/areas/*/subareas",
           "GET:/api/v1/areas/*/contains",
           "GET:/api/v1/components/*/subcomponents",
           "GET:/api/v1/components/*/hosts",
           "GET:/api/v1/components/*/depends-on",
           "GET:/api/v1/apps/*/depends-on",
           "GET:/api/v1/functions/*/hosts",
           // Data: all entity types (read)
           "GET:/api/v1/components/*/data",
           "GET:/api/v1/components/*/data/*",
           "GET:/api/v1/apps/*/data",
           "GET:/api/v1/apps/*/data/*",
           "GET:/api/v1/areas/*/data",
           "GET:/api/v1/areas/*/data/*",
           "GET:/api/v1/functions/*/data",
           "GET:/api/v1/functions/*/data/*",
           // Data categories and groups: all entity types
           "GET:/api/v1/components/*/data-categories",
           "GET:/api/v1/components/*/data-groups",
           "GET:/api/v1/apps/*/data-categories",
           "GET:/api/v1/apps/*/data-groups",
           "GET:/api/v1/areas/*/data-categories",
           "GET:/api/v1/areas/*/data-groups",
           "GET:/api/v1/functions/*/data-categories",
           "GET:/api/v1/functions/*/data-groups",
           // Operations: all entity types (read)
           "GET:/api/v1/components/*/operations",
           "GET:/api/v1/components/*/operations/*",
           "GET:/api/v1/components/*/operations/*/executions",
           "GET:/api/v1/components/*/operations/*/executions/*",
           "GET:/api/v1/apps/*/operations",
           "GET:/api/v1/apps/*/operations/*",
           "GET:/api/v1/apps/*/operations/*/executions",
           "GET:/api/v1/apps/*/operations/*/executions/*",
           "GET:/api/v1/areas/*/operations",
           "GET:/api/v1/areas/*/operations/*",
           "GET:/api/v1/areas/*/operations/*/executions",
           "GET:/api/v1/areas/*/operations/*/executions/*",
           "GET:/api/v1/functions/*/operations",
           "GET:/api/v1/functions/*/operations/*",
           "GET:/api/v1/functions/*/operations/*/executions",
           "GET:/api/v1/functions/*/operations/*/executions/*",
           // Configurations: all entity types (read)
           "GET:/api/v1/components/*/configurations",
           "GET:/api/v1/components/*/configurations/*",
           "GET:/api/v1/apps/*/configurations",
           "GET:/api/v1/apps/*/configurations/*",
           "GET:/api/v1/areas/*/configurations",
           "GET:/api/v1/areas/*/configurations/*",
           "GET:/api/v1/functions/*/configurations",
           "GET:/api/v1/functions/*/configurations/*",
           // Faults: per-entity (all entity types, read)
           "GET:/api/v1/components/*/faults",
           "GET:/api/v1/components/*/faults/*",
           "GET:/api/v1/apps/*/faults",
           "GET:/api/v1/apps/*/faults/*",
           "GET:/api/v1/areas/*/faults",
           "GET:/api/v1/areas/*/faults/*",
           "GET:/api/v1/functions/*/faults",
           "GET:/api/v1/functions/*/faults/*",
           // Faults: global
           "GET:/api/v1/faults",
           "GET:/api/v1/faults/stream",
           // Logs: all entity types (read)
           "GET:/api/v1/components/*/logs",
           "GET:/api/v1/components/*/logs/configuration",
           "GET:/api/v1/apps/*/logs",
           "GET:/api/v1/apps/*/logs/configuration",
           "GET:/api/v1/areas/*/logs",
           "GET:/api/v1/areas/*/logs/configuration",
           "GET:/api/v1/functions/*/logs",
           "GET:/api/v1/functions/*/logs/configuration",
           // Bulk data: all entity types (read-only)
           "GET:/api/v1/components/*/bulk-data",
           "GET:/api/v1/components/*/bulk-data/*",
           "GET:/api/v1/components/*/bulk-data/*/*",
           "GET:/api/v1/apps/*/bulk-data",
           "GET:/api/v1/apps/*/bulk-data/*",
           "GET:/api/v1/apps/*/bulk-data/*/*",
           "GET:/api/v1/areas/*/bulk-data",
           "GET:/api/v1/areas/*/bulk-data/*",
           "GET:/api/v1/areas/*/bulk-data/*/*",
           "GET:/api/v1/functions/*/bulk-data",
           "GET:/api/v1/functions/*/bulk-data/*",
           "GET:/api/v1/functions/*/bulk-data/*/*",
           // Bulk data: nested entities (subareas, subcomponents)
           "GET:/api/v1/areas/*/subareas/*/bulk-data",
           "GET:/api/v1/areas/*/subareas/*/bulk-data/*",
           "GET:/api/v1/areas/*/subareas/*/bulk-data/*/*",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data/*",
           "GET:/api/v1/components/*/subcomponents/*/bulk-data/*/*",
           // Cyclic subscriptions: apps, components, functions (read-only)
           "GET:/api/v1/components/*/cyclic-subscriptions",
           "GET:/api/v1/components/*/cyclic-subscriptions/*",
           "GET:/api/v1/components/*/cyclic-subscriptions/*/events",
           "GET:/api/v1/apps/*/cyclic-subscriptions",
           "GET:/api/v1/apps/*/cyclic-subscriptions/*",
           "GET:/api/v1/apps/*/cyclic-subscriptions/*/events",
           "GET:/api/v1/functions/*/cyclic-subscriptions",
           "GET:/api/v1/functions/*/cyclic-subscriptions/*",
           "GET:/api/v1/functions/*/cyclic-subscriptions/*/events",
           // Locks: components and apps (read-only)
           "GET:/api/v1/components/*/locks",
           "GET:/api/v1/components/*/locks/*",
           "GET:/api/v1/apps/*/locks",
           "GET:/api/v1/apps/*/locks/*",
           // Updates (read-only)
           "GET:/api/v1/updates",
           "GET:/api/v1/updates/*",
           "GET:/api/v1/updates/*/status",
           // Scripts: read (inherited from VIEWER)
           "GET:/api/v1/components/*/scripts",
           "GET:/api/v1/components/*/scripts/*",
           "GET:/api/v1/apps/*/scripts",
           "GET:/api/v1/apps/*/scripts/*",
           "GET:/api/v1/components/*/scripts/*/executions/*",
           "GET:/api/v1/apps/*/scripts/*/executions/*",
           // Docs
           "GET:/api/v1/docs",
           // Inherited from OPERATOR - write permissions
           // Trigger operations: all entity types (POST)
           "POST:/api/v1/components/*/operations/*/executions",
           "POST:/api/v1/apps/*/operations/*/executions",
           "POST:/api/v1/areas/*/operations/*/executions",
           "POST:/api/v1/functions/*/operations/*/executions",
           // Update operations: all entity types (PUT)
           "PUT:/api/v1/components/*/operations/*/executions/*",
           "PUT:/api/v1/apps/*/operations/*/executions/*",
           "PUT:/api/v1/areas/*/operations/*/executions/*",
           "PUT:/api/v1/functions/*/operations/*/executions/*",
           // Cancel actions: all entity types (DELETE)
           "DELETE:/api/v1/components/*/operations/*/executions/*",
           "DELETE:/api/v1/apps/*/operations/*/executions/*",
           "DELETE:/api/v1/areas/*/operations/*/executions/*",
           "DELETE:/api/v1/functions/*/operations/*/executions/*",
           // Clear faults: all entity types
           "DELETE:/api/v1/components/*/faults/*",
           "DELETE:/api/v1/apps/*/faults/*",
           "DELETE:/api/v1/areas/*/faults/*",
           "DELETE:/api/v1/functions/*/faults/*",
           // Clear all faults: per-entity and global
           "DELETE:/api/v1/components/*/faults",
           "DELETE:/api/v1/apps/*/faults",
           "DELETE:/api/v1/areas/*/faults",
           "DELETE:/api/v1/functions/*/faults",
           "DELETE:/api/v1/faults",
           // Publish data: all entity types (PUT)
           "PUT:/api/v1/components/*/data/*",
           "PUT:/api/v1/apps/*/data/*",
           "PUT:/api/v1/areas/*/data/*",
           "PUT:/api/v1/functions/*/data/*",
           // Cyclic subscriptions: create/update/delete (apps, components, functions)
           "POST:/api/v1/components/*/cyclic-subscriptions",
           "POST:/api/v1/apps/*/cyclic-subscriptions",
           "POST:/api/v1/functions/*/cyclic-subscriptions",
           "PUT:/api/v1/components/*/cyclic-subscriptions/*",
           "PUT:/api/v1/apps/*/cyclic-subscriptions/*",
           "PUT:/api/v1/functions/*/cyclic-subscriptions/*",
           "DELETE:/api/v1/components/*/cyclic-subscriptions/*",
           "DELETE:/api/v1/apps/*/cyclic-subscriptions/*",
           "DELETE:/api/v1/functions/*/cyclic-subscriptions/*",
           // Locks: acquire/extend/release (components and apps)
           "POST:/api/v1/components/*/locks",
           "POST:/api/v1/apps/*/locks",
           "PUT:/api/v1/components/*/locks/*",
           "PUT:/api/v1/apps/*/locks/*",
           "DELETE:/api/v1/components/*/locks/*",
           "DELETE:/api/v1/apps/*/locks/*",
           // Bulk data: upload/delete (apps and components only)
           "POST:/api/v1/components/*/bulk-data/*",
           "POST:/api/v1/apps/*/bulk-data/*",
           "DELETE:/api/v1/components/*/bulk-data/*/*",
           "DELETE:/api/v1/apps/*/bulk-data/*/*",
           // Scripts: start/control/delete executions (inherited from OPERATOR)
           "POST:/api/v1/components/*/scripts/*/executions",
           "POST:/api/v1/apps/*/scripts/*/executions",
           "PUT:/api/v1/components/*/scripts/*/executions/*",
           "PUT:/api/v1/apps/*/scripts/*/executions/*",
           "DELETE:/api/v1/components/*/scripts/*/executions/*",
           "DELETE:/api/v1/apps/*/scripts/*/executions/*",
           // --- Configurator-specific write permissions ---
           // Modify configurations: all entity types (PUT)
           "PUT:/api/v1/components/*/configurations/*",
           "PUT:/api/v1/apps/*/configurations/*",
           "PUT:/api/v1/areas/*/configurations/*",
           "PUT:/api/v1/functions/*/configurations/*",
           // Reset configurations: all entity types (DELETE)
           "DELETE:/api/v1/components/*/configurations",
           "DELETE:/api/v1/components/*/configurations/*",
           "DELETE:/api/v1/apps/*/configurations",
           "DELETE:/api/v1/apps/*/configurations/*",
           "DELETE:/api/v1/areas/*/configurations",
           "DELETE:/api/v1/areas/*/configurations/*",
           "DELETE:/api/v1/functions/*/configurations",
           "DELETE:/api/v1/functions/*/configurations/*",
           // Log configuration: all entity types (PUT)
           "PUT:/api/v1/components/*/logs/configuration",
           "PUT:/api/v1/apps/*/logs/configuration",
           "PUT:/api/v1/areas/*/logs/configuration",
           "PUT:/api/v1/functions/*/logs/configuration",
           // Scripts: upload/delete scripts
           "POST:/api/v1/components/*/scripts",
           "POST:/api/v1/apps/*/scripts",
           "DELETE:/api/v1/components/*/scripts/*",
           "DELETE:/api/v1/apps/*/scripts/*",
           // Updates: register/prepare/execute/automated/delete
           "POST:/api/v1/updates",
           "PUT:/api/v1/updates/*/prepare",
           "PUT:/api/v1/updates/*/execute",
           "PUT:/api/v1/updates/*/automated",
           "DELETE:/api/v1/updates/*",
       }},
      {UserRole::ADMIN,
       {
           // Full access - all endpoints including auth
           // ** matches any number of path segments
           "GET:/api/v1/**",
           "POST:/api/v1/**",
           "PUT:/api/v1/**",
           "DELETE:/api/v1/**",
           // Auth endpoints are always accessible to admin
           "POST:/api/v1/auth/authorize",
           "POST:/api/v1/auth/token",
           "POST:/api/v1/auth/revoke",
       }}};
  return permissions;
}

AuthConfigBuilder & AuthConfigBuilder::with_enabled(bool enabled) {
  config_.enabled = enabled;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_jwt_secret(const std::string & secret) {
  config_.jwt_secret = secret;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_jwt_public_key(const std::string & public_key) {
  config_.jwt_public_key = public_key;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_algorithm(JwtAlgorithm algorithm) {
  config_.jwt_algorithm = algorithm;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_token_expiry(int seconds) {
  config_.token_expiry_seconds = seconds;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_refresh_token_expiry(int seconds) {
  config_.refresh_token_expiry_seconds = seconds;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_require_auth_for(AuthRequirement requirement) {
  config_.require_auth_for = requirement;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::with_issuer(const std::string & issuer) {
  config_.issuer = issuer;
  return *this;
}

AuthConfigBuilder & AuthConfigBuilder::add_client(const std::string & client_id, const std::string & client_secret,
                                                  UserRole role) {
  ClientCredentials creds;
  creds.client_id = client_id;
  creds.client_secret = client_secret;
  creds.role = role;
  creds.enabled = true;
  config_.clients.push_back(creds);
  return *this;
}

AuthConfig AuthConfigBuilder::build() {
  // Validate configuration
  if (config_.enabled) {
    if (config_.jwt_secret.empty()) {
      throw std::invalid_argument("JWT secret is required when authentication is enabled");
    }

    // Enforce a minimum secret length for HS256 to avoid weak symmetric keys
    if (config_.jwt_algorithm == JwtAlgorithm::HS256 && config_.jwt_secret.size() < 32U) {
      throw std::invalid_argument("JWT secret must be at least 32 characters for HS256 algorithm");
    }

    if (config_.token_expiry_seconds <= 0) {
      throw std::invalid_argument("Token expiry must be positive");
    }

    if (config_.refresh_token_expiry_seconds <= 0) {
      throw std::invalid_argument("Refresh token expiry must be positive");
    }

    if (config_.refresh_token_expiry_seconds < config_.token_expiry_seconds) {
      throw std::invalid_argument("Refresh token expiry must be greater than or equal to token expiry");
    }

    // For RS256, validate key paths exist (actual file check done at runtime)
    if (config_.jwt_algorithm == JwtAlgorithm::RS256) {
      if (config_.jwt_public_key.empty()) {
        throw std::invalid_argument("Public key path is required for RS256 algorithm");
      }
    }
  }

  return config_;
}

std::string role_to_string(UserRole role) {
  switch (role) {
    case UserRole::VIEWER:
      return "viewer";
    case UserRole::OPERATOR:
      return "operator";
    case UserRole::CONFIGURATOR:
      return "configurator";
    case UserRole::ADMIN:
      return "admin";
    default:
      return "unknown";
  }
}

UserRole string_to_role(const std::string & role_str) {
  std::string lower_role = role_str;
  std::transform(lower_role.begin(), lower_role.end(), lower_role.begin(), ::tolower);

  if (lower_role == "viewer") {
    return UserRole::VIEWER;
  }
  if (lower_role == "operator") {
    return UserRole::OPERATOR;
  }
  if (lower_role == "configurator") {
    return UserRole::CONFIGURATOR;
  }
  if (lower_role == "admin") {
    return UserRole::ADMIN;
  }
  throw std::invalid_argument("Invalid role: " + role_str);
}

std::string algorithm_to_string(JwtAlgorithm algorithm) {
  switch (algorithm) {
    case JwtAlgorithm::HS256:
      return "HS256";
    case JwtAlgorithm::RS256:
      return "RS256";
    default:
      return "unknown";
  }
}

JwtAlgorithm string_to_algorithm(const std::string & alg_str) {
  std::string upper_alg = alg_str;
  std::transform(upper_alg.begin(), upper_alg.end(), upper_alg.begin(), ::toupper);

  if (upper_alg == "HS256") {
    return JwtAlgorithm::HS256;
  }
  if (upper_alg == "RS256") {
    return JwtAlgorithm::RS256;
  }
  throw std::invalid_argument("Invalid algorithm: " + alg_str + ". Supported: HS256, RS256");
}

AuthRequirement string_to_auth_requirement(const std::string & req_str) {
  std::string lower_req = req_str;
  std::transform(lower_req.begin(), lower_req.end(), lower_req.begin(), ::tolower);

  if (lower_req == "none") {
    return AuthRequirement::NONE;
  }
  if (lower_req == "write") {
    return AuthRequirement::WRITE;
  }
  if (lower_req == "all") {
    return AuthRequirement::ALL;
  }
  throw std::invalid_argument("Invalid auth requirement: " + req_str + ". Supported: none, write, all");
}

}  // namespace ros2_medkit_gateway
