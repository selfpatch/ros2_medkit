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

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

class SchemaBuilder;

/// Builds OpenAPI 3.1.0 PathItem JSON objects for each resource type.
/// Uses SchemaBuilder for response/request schemas and adds SOVD extensions.
class PathBuilder {
 public:
  explicit PathBuilder(const SchemaBuilder & schema_builder, bool auth_enabled = false);

  // Entity collection paths (GET /areas, GET /components, etc.)
  nlohmann::json build_entity_collection(const std::string & entity_type) const;

  // Entity detail path (GET /areas/{id}, GET /apps/{id})
  nlohmann::json build_entity_detail(const std::string & entity_type) const;

  // Resource collection paths
  nlohmann::json build_data_collection(const std::string & entity_path, const std::vector<TopicData> & topics) const;
  nlohmann::json build_data_item(const std::string & entity_path, const TopicData & topic) const;
  nlohmann::json build_operations_collection(const std::string & entity_path, const AggregatedOperations & ops) const;
  nlohmann::json build_operation_item(const std::string & entity_path, const ServiceInfo & service) const;
  nlohmann::json build_operation_item(const std::string & entity_path, const ActionInfo & action) const;
  nlohmann::json build_configurations_collection(const std::string & entity_path) const;
  nlohmann::json build_faults_collection(const std::string & entity_path) const;
  nlohmann::json build_logs_collection(const std::string & entity_path) const;
  nlohmann::json build_bulk_data_collection(const std::string & entity_path) const;
  nlohmann::json build_cyclic_subscriptions_collection(const std::string & entity_path) const;

  // SSE endpoints
  nlohmann::json build_sse_endpoint(const std::string & path, const std::string & description) const;

  // Common helpers
  nlohmann::json error_responses() const;

 private:
  nlohmann::json build_path_param(const std::string & name, const std::string & description) const;
  nlohmann::json build_query_params_for_collection() const;

  const SchemaBuilder & schema_builder_;
  bool auth_enabled_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
