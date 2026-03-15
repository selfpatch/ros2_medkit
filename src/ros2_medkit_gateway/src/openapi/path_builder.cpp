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

#include "path_builder.hpp"

#include "schema_builder.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

PathBuilder::PathBuilder(const SchemaBuilder & schema_builder) : schema_builder_(schema_builder) {
}

// -----------------------------------------------------------------------------
// Entity collection paths
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_entity_collection(const std::string & entity_type) const {
  nlohmann::json path_item;

  nlohmann::json get_op;
  get_op["summary"] = "List all " + entity_type;
  get_op["description"] = "Returns the collection of " + entity_type + " entities.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::entity_list_schema();

  // Merge error responses
  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Entity detail paths
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_entity_detail(const std::string & entity_type) const {
  nlohmann::json path_item;

  // Derive singular name from entity_type for param description
  // "areas" -> "area", "components" -> "component", "apps" -> "app"
  std::string singular = entity_type;
  if (!singular.empty() && singular.back() == 's') {
    singular.pop_back();
  }

  nlohmann::json get_op;
  get_op["summary"] = "Get " + singular + " details";
  get_op["description"] = "Returns detailed information about a specific " + singular + ".";
  get_op["parameters"] = nlohmann::json::array({build_path_param(singular + "_id", "The " + singular + " identifier")});
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::entity_detail_schema();

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Data collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_data_collection(const std::string & entity_path,
                                                  const std::vector<TopicData> & /*topics*/) const {
  nlohmann::json path_item;

  nlohmann::json get_op;
  get_op["summary"] = "List data items for " + entity_path;
  get_op["description"] = "Returns all available data items (topics) for this entity.";
  get_op["parameters"] = build_query_params_for_collection();

  // Use the standard items wrapper with a generic data item schema
  nlohmann::json data_item_schema = {{"type", "object"},
                                     {"properties",
                                      {{"name", {{"type", "string"}}},
                                       {"type", {{"type", "string"}}},
                                       {"direction", {{"type", "string"}}},
                                       {"uri", {{"type", "string"}}}}}};
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::items_wrapper(data_item_schema);

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);
  path_item["x-sovd-data-category"] = "currentData";
  return path_item;
}

// -----------------------------------------------------------------------------
// Data item
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_data_item(const std::string & /*entity_path*/, const TopicData & topic) const {
  nlohmann::json path_item;

  // GET - always available
  nlohmann::json get_op;
  get_op["summary"] = "Read data: " + topic.name;
  get_op["description"] = "Read current value of topic " + topic.name + " (type: " + topic.type + ").";
  get_op["responses"]["200"]["description"] = "Current topic value";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] = schema_builder_.from_ros_msg(topic.type);

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // PUT - only if writable (subscribe or both)
  if (topic.direction == "subscribe" || topic.direction == "both") {
    nlohmann::json put_op;
    put_op["summary"] = "Write data: " + topic.name;
    put_op["description"] = "Publish a value to topic " + topic.name + ".";
    put_op["requestBody"]["required"] = true;
    put_op["requestBody"]["content"]["application/json"]["schema"] = schema_builder_.from_ros_msg(topic.type);
    put_op["responses"]["200"]["description"] = "Value written successfully";
    put_op["responses"]["200"]["content"]["application/json"]["schema"] = {
        {"type", "object"}, {"properties", {{"status", {{"type", "string"}}}}}};

    auto put_errors = error_responses();
    for (auto & [code, val] : put_errors.items()) {
      put_op["responses"][code] = val;
    }

    path_item["put"] = std::move(put_op);
  }

  // SOVD extensions
  path_item["x-sovd-data-category"] = "currentData";
  path_item["x-sovd-cyclic-subscription-supported"] = true;
  path_item["x-sovd-name"] = topic.name;

  return path_item;
}

// -----------------------------------------------------------------------------
// Operations collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_operations_collection(const std::string & entity_path,
                                                        const AggregatedOperations & /*ops*/) const {
  nlohmann::json path_item;

  // GET - list all operations
  nlohmann::json get_op;
  get_op["summary"] = "List operations for " + entity_path;
  get_op["description"] = "Returns all available operations (services and actions) for this entity.";
  get_op["parameters"] = build_query_params_for_collection();

  nlohmann::json op_item_schema = {{"type", "object"},
                                   {"properties",
                                    {{"name", {{"type", "string"}}},
                                     {"type", {{"type", "string"}}},
                                     {"kind", {{"type", "string"}, {"enum", {"service", "action"}}}},
                                     {"path", {{"type", "string"}}}}}};

  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::items_wrapper(op_item_schema);

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Operation item (service)
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_operation_item(const std::string & /*entity_path*/,
                                                 const ServiceInfo & service) const {
  nlohmann::json path_item;

  // GET - get operation details and result
  nlohmann::json get_op;
  get_op["summary"] = "Get operation: " + service.name;
  get_op["description"] = "Get details and last result of service " + service.name + " (type: " + service.type + ").";
  get_op["responses"]["200"]["description"] = "Operation details";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_srv_response(service.type);

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // POST - execute operation
  nlohmann::json post_op;
  post_op["summary"] = "Execute operation: " + service.name;
  post_op["description"] = "Execute service " + service.name + " synchronously.";
  post_op["requestBody"]["required"] = true;
  post_op["requestBody"]["content"]["application/json"]["schema"] = schema_builder_.from_ros_srv_request(service.type);
  post_op["responses"]["200"]["description"] = "Operation result";
  post_op["responses"]["200"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_srv_response(service.type);

  auto post_errors = error_responses();
  for (auto & [code, val] : post_errors.items()) {
    post_op["responses"][code] = val;
  }

  path_item["post"] = std::move(post_op);
  path_item["x-sovd-name"] = service.name;
  return path_item;
}

// -----------------------------------------------------------------------------
// Operation item (action)
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_operation_item(const std::string & /*entity_path*/, const ActionInfo & action) const {
  nlohmann::json path_item;

  // GET - get action status/result
  nlohmann::json get_op;
  get_op["summary"] = "Get action status: " + action.name;
  get_op["description"] = "Get status and result of action " + action.name + " (type: " + action.type + ").";

  // Action goal result type: "pkg/action/Name" -> "pkg/action/Name_GetResult_Response"
  get_op["responses"]["200"]["description"] = "Action status";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_msg(action.type + "_GetResult_Response");

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // POST - execute action (asynchronous)
  nlohmann::json post_op;
  post_op["summary"] = "Execute action: " + action.name;
  post_op["description"] = "Start action " + action.name + " asynchronously.";
  post_op["requestBody"]["required"] = true;
  // Action goal type: "pkg/action/Name" -> "pkg/action/Name_SendGoal_Request"
  post_op["requestBody"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_msg(action.type + "_SendGoal_Request");
  post_op["responses"]["202"]["description"] = "Action accepted";
  post_op["responses"]["202"]["content"]["application/json"]["schema"] = {
      {"type", "object"}, {"properties", {{"id", {{"type", "string"}}}, {"status", {{"type", "string"}}}}}};

  auto post_errors = error_responses();
  for (auto & [code, val] : post_errors.items()) {
    post_op["responses"][code] = val;
  }

  path_item["post"] = std::move(post_op);
  path_item["x-sovd-name"] = action.name;
  path_item["x-sovd-asynchronous-execution"] = true;
  return path_item;
}

// -----------------------------------------------------------------------------
// Configurations collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_configurations_collection(const std::string & entity_path) const {
  nlohmann::json path_item;

  // GET - list all configuration parameters
  nlohmann::json get_op;
  get_op["summary"] = "List configuration parameters for " + entity_path;
  get_op["description"] = "Returns all configuration parameters for this entity.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper(SchemaBuilder::configuration_param_schema());

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // PUT - update configuration parameter
  nlohmann::json put_op;
  put_op["summary"] = "Update configuration parameter";
  put_op["description"] = "Update a specific configuration parameter for this entity.";
  put_op["requestBody"]["required"] = true;
  put_op["requestBody"]["content"]["application/json"]["schema"] = SchemaBuilder::configuration_param_schema();
  put_op["responses"]["200"]["description"] = "Parameter updated";
  put_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::configuration_param_schema();

  auto put_errors = error_responses();
  for (auto & [code, val] : put_errors.items()) {
    put_op["responses"][code] = val;
  }

  path_item["put"] = std::move(put_op);

  // DELETE - reset configuration parameter to default
  nlohmann::json delete_op;
  delete_op["summary"] = "Reset configuration parameter";
  delete_op["description"] = "Reset a specific configuration parameter to its default value.";
  delete_op["responses"]["200"]["description"] = "Parameter reset to default";
  delete_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::configuration_param_schema();

  auto del_errors = error_responses();
  for (auto & [code, val] : del_errors.items()) {
    delete_op["responses"][code] = val;
  }

  path_item["delete"] = std::move(delete_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Faults collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_faults_collection(const std::string & entity_path) const {
  nlohmann::json path_item;

  // GET - list faults
  nlohmann::json get_op;
  get_op["summary"] = entity_path.empty() ? "List all faults" : "List faults for " + entity_path;
  get_op["description"] =
      entity_path.empty() ? "Returns all faults." : "Returns all faults associated with this entity.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::fault_list_schema();

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // PUT - update fault status (acknowledge/resolve)
  nlohmann::json put_op;
  put_op["summary"] = "Update fault status";
  put_op["description"] = "Update the status of a specific fault (e.g., acknowledge or resolve).";
  put_op["requestBody"]["required"] = true;
  put_op["requestBody"]["content"]["application/json"]["schema"] = {
      {"type", "object"},
      {"properties", {{"status", {{"type", "string"}, {"enum", {"acknowledged", "resolved"}}}}}},
      {"required", {"status"}}};
  put_op["responses"]["200"]["description"] = "Fault status updated";
  put_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::fault_schema();

  auto put_errors = error_responses();
  for (auto & [code, val] : put_errors.items()) {
    put_op["responses"][code] = val;
  }

  path_item["put"] = std::move(put_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Logs collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_logs_collection(const std::string & entity_path) const {
  nlohmann::json path_item;

  nlohmann::json get_op;
  get_op["summary"] = "List log entries for " + entity_path;
  get_op["description"] = "Returns log entries associated with this entity.";

  // Log-specific query parameters
  nlohmann::json params = build_query_params_for_collection();
  nlohmann::json level_param;
  level_param["name"] = "level";
  level_param["in"] = "query";
  level_param["required"] = false;
  level_param["description"] = "Filter by log level (e.g., DEBUG, INFO, WARN, ERROR, FATAL)";
  level_param["schema"]["type"] = "string";
  params.push_back(std::move(level_param));

  get_op["parameters"] = std::move(params);
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper(SchemaBuilder::log_entry_schema());

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Bulk data collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_bulk_data_collection(const std::string & entity_path) const {
  nlohmann::json path_item;

  nlohmann::json get_op;
  get_op["summary"] = "List bulk data resources for " + entity_path;
  get_op["description"] = "Returns available bulk data resources (snapshots, recordings) for this entity.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper({{"type", "object"},
                                    {"properties",
                                     {{"id", {{"type", "string"}}},
                                      {"name", {{"type", "string"}}},
                                      {"status", {{"type", "string"}}},
                                      {"created_at", {{"type", "string"}}},
                                      {"size_bytes", {{"type", "integer"}}}}}});

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Cyclic subscriptions collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_cyclic_subscriptions_collection(const std::string & entity_path) const {
  nlohmann::json path_item;

  // GET - list active subscriptions
  nlohmann::json get_op;
  get_op["summary"] = "List cyclic subscriptions for " + entity_path;
  get_op["description"] = "Returns all active cyclic subscriptions for this entity.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper({{"type", "object"},
                                    {"properties",
                                     {{"id", {{"type", "string"}}},
                                      {"topic", {{"type", "string"}}},
                                      {"interval_ms", {{"type", "integer"}}},
                                      {"status", {{"type", "string"}}}}}});

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // POST - create a new cyclic subscription
  nlohmann::json post_op;
  post_op["summary"] = "Create cyclic subscription";
  post_op["description"] = "Create a new cyclic subscription to stream data changes via SSE.";
  post_op["requestBody"]["required"] = true;
  post_op["requestBody"]["content"]["application/json"]["schema"] = {
      {"type", "object"},
      {"properties", {{"topic", {{"type", "string"}}}, {"interval_ms", {{"type", "integer"}, {"minimum", 1}}}}},
      {"required", {"topic"}}};
  post_op["responses"]["201"]["description"] = "Subscription created";
  post_op["responses"]["201"]["content"]["application/json"]["schema"] = {{"type", "object"},
                                                                          {"properties",
                                                                           {{"id", {{"type", "string"}}},
                                                                            {"topic", {{"type", "string"}}},
                                                                            {"interval_ms", {{"type", "integer"}}},
                                                                            {"status", {{"type", "string"}}},
                                                                            {"stream_uri", {{"type", "string"}}}}}};

  auto post_errors = error_responses();
  for (auto & [code, val] : post_errors.items()) {
    post_op["responses"][code] = val;
  }

  path_item["post"] = std::move(post_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// SSE endpoint
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_sse_endpoint(const std::string & /*path*/, const std::string & description) const {
  nlohmann::json path_item;

  nlohmann::json get_op;
  get_op["summary"] = description;
  get_op["description"] = description + " Streams events using Server-Sent Events (SSE).";
  get_op["responses"]["200"]["description"] = "SSE event stream";
  get_op["responses"]["200"]["content"]["text/event-stream"]["schema"] = {{"type", "string"}};

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Error responses
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::error_responses(bool include_auth) {
  nlohmann::json errors;

  errors["400"]["description"] = "Bad request";
  errors["400"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  errors["404"]["description"] = "Not found";
  errors["404"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  errors["500"]["description"] = "Internal server error";
  errors["500"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  if (include_auth) {
    errors["401"]["description"] = "Unauthorized - authentication required";
    errors["401"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

    errors["403"]["description"] = "Forbidden - insufficient permissions";
    errors["403"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();
  }

  return errors;
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_path_param(const std::string & name, const std::string & description) const {
  nlohmann::json param;
  param["name"] = name;
  param["in"] = "path";
  param["required"] = true;
  param["description"] = description;
  param["schema"]["type"] = "string";
  return param;
}

nlohmann::json PathBuilder::build_query_params_for_collection() const {
  nlohmann::json params = nlohmann::json::array();

  nlohmann::json limit_param;
  limit_param["name"] = "limit";
  limit_param["in"] = "query";
  limit_param["required"] = false;
  limit_param["description"] = "Maximum number of items to return";
  limit_param["schema"]["type"] = "integer";
  limit_param["schema"]["minimum"] = 1;
  params.push_back(std::move(limit_param));

  nlohmann::json offset_param;
  offset_param["name"] = "offset";
  offset_param["in"] = "query";
  offset_param["required"] = false;
  offset_param["description"] = "Number of items to skip";
  offset_param["schema"]["type"] = "integer";
  offset_param["schema"]["minimum"] = 0;
  params.push_back(std::move(offset_param));

  return params;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
