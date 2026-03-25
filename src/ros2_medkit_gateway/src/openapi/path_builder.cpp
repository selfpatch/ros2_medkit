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

PathBuilder::PathBuilder(const SchemaBuilder & schema_builder, bool auth_enabled)
  : schema_builder_(schema_builder), auth_enabled_(auth_enabled) {
}

// -----------------------------------------------------------------------------
// Entity collection paths
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_entity_collection(const std::string & entity_type) const {
  nlohmann::json path_item;

  nlohmann::json get_op;
  get_op["tags"] = nlohmann::json::array({"Discovery"});
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

nlohmann::json PathBuilder::build_entity_detail(const std::string & entity_type, bool use_template) const {
  nlohmann::json path_item;

  // Derive singular name from entity_type for param description
  // "areas" -> "area", "components" -> "component", "apps" -> "app"
  std::string singular = entity_type;
  if (!singular.empty() && singular.back() == 's') {
    singular.pop_back();
  }

  nlohmann::json get_op;
  get_op["tags"] = nlohmann::json::array({"Discovery"});
  get_op["summary"] = "Get " + singular + " details";
  get_op["description"] = "Returns detailed information about a specific " + singular + ".";
  if (use_template) {
    get_op["parameters"] =
        nlohmann::json::array({build_path_param(singular + "_id", "The " + singular + " identifier")});
  }
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
  get_op["tags"] = nlohmann::json::array({"Data"});
  get_op["summary"] = "List data items for " + entity_path;
  get_op["description"] = "Returns all available data items (topics) for this entity.";
  get_op["parameters"] = build_query_params_for_collection();

  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper(SchemaBuilder::data_item_schema());

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
  get_op["tags"] = nlohmann::json::array({"Data"});
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
    put_op["tags"] = nlohmann::json::array({"Data"});
    put_op["summary"] = "Write data: " + topic.name;
    put_op["description"] = "Publish a value to topic " + topic.name + ".";
    put_op["requestBody"]["required"] = true;
    put_op["requestBody"]["content"]["application/json"]["schema"] = schema_builder_.from_ros_msg(topic.type);
    put_op["responses"]["200"]["description"] = "Value written successfully";
    put_op["responses"]["200"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_object_schema();

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
  get_op["tags"] = nlohmann::json::array({"Operations"});
  get_op["summary"] = "List operations for " + entity_path;
  get_op["description"] = "Returns all available operations (services and actions) for this entity.";
  get_op["parameters"] = build_query_params_for_collection();

  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper(SchemaBuilder::operation_item_schema());

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
  get_op["tags"] = nlohmann::json::array({"Operations"});
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
  post_op["tags"] = nlohmann::json::array({"Operations"});
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
  get_op["tags"] = nlohmann::json::array({"Operations"});
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
  post_op["tags"] = nlohmann::json::array({"Operations"});
  post_op["summary"] = "Execute action: " + action.name;
  post_op["description"] = "Start action " + action.name + " asynchronously.";
  post_op["requestBody"]["required"] = true;
  // Action goal type: "pkg/action/Name" -> "pkg/action/Name_SendGoal_Request"
  post_op["requestBody"]["content"]["application/json"]["schema"] =
      schema_builder_.from_ros_msg(action.type + "_SendGoal_Request");
  post_op["responses"]["202"]["description"] = "Action accepted";
  post_op["responses"]["202"]["content"]["application/json"]["schema"] = SchemaBuilder::operation_execution_schema();

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
  get_op["tags"] = nlohmann::json::array({"Configuration"});
  get_op["summary"] = "List configuration parameters for " + entity_path;
  get_op["description"] = "Returns all configuration parameters for this entity.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper(SchemaBuilder::configuration_metadata_schema());

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // DELETE - delete all configuration parameters
  nlohmann::json delete_op;
  delete_op["tags"] = nlohmann::json::array({"Configuration"});
  delete_op["summary"] = "Delete all configuration parameters";
  delete_op["description"] = "Delete all configuration parameters for this entity, resetting them to defaults.";
  delete_op["responses"]["204"]["description"] = "All parameters deleted";
  delete_op["responses"]["207"]["description"] = "Partial success - some nodes failed";
  delete_op["responses"]["207"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_object_schema();

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
  get_op["tags"] = nlohmann::json::array({"Faults"});
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

  // DELETE - clear all faults for this entity
  nlohmann::json delete_op;
  delete_op["tags"] = nlohmann::json::array({"Faults"});
  delete_op["summary"] = entity_path.empty() ? "Clear all faults" : "Clear faults for " + entity_path;
  delete_op["description"] =
      entity_path.empty() ? "Clear all faults in the system." : "Clear all faults associated with this entity.";
  delete_op["responses"]["204"]["description"] = "Faults cleared successfully";

  auto del_errors = error_responses();
  for (auto & [code, val] : del_errors.items()) {
    delete_op["responses"][code] = val;
  }

  path_item["delete"] = std::move(delete_op);
  return path_item;
}

// -----------------------------------------------------------------------------
// Logs collection
// -----------------------------------------------------------------------------

nlohmann::json PathBuilder::build_logs_collection(const std::string & entity_path) const {
  nlohmann::json path_item;

  nlohmann::json get_op;
  get_op["tags"] = nlohmann::json::array({"Logs"});
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
  get_op["tags"] = nlohmann::json::array({"Bulk Data"});
  get_op["summary"] = "List bulk data resources for " + entity_path;
  get_op["description"] = "Returns available bulk data resources (snapshots, recordings) for this entity.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper(SchemaBuilder::bulk_data_descriptor_schema());

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
  get_op["tags"] = nlohmann::json::array({"Subscriptions"});
  get_op["summary"] = "List cyclic subscriptions for " + entity_path;
  get_op["description"] = "Returns all active cyclic subscriptions for this entity.";
  get_op["parameters"] = build_query_params_for_collection();
  get_op["responses"]["200"]["description"] = "Successful response";
  get_op["responses"]["200"]["content"]["application/json"]["schema"] =
      SchemaBuilder::items_wrapper(SchemaBuilder::cyclic_subscription_schema());

  auto errors = error_responses();
  for (auto & [code, val] : errors.items()) {
    get_op["responses"][code] = val;
  }

  path_item["get"] = std::move(get_op);

  // POST - create a new cyclic subscription
  nlohmann::json post_op;
  post_op["tags"] = nlohmann::json::array({"Subscriptions"});
  post_op["summary"] = "Create cyclic subscription";
  post_op["description"] = "Create a new cyclic subscription to stream data changes via SSE.";
  post_op["requestBody"]["required"] = true;
  post_op["requestBody"]["content"]["application/json"]["schema"] = SchemaBuilder::cyclic_subscription_schema();
  post_op["responses"]["201"]["description"] = "Subscription created";
  post_op["responses"]["201"]["content"]["application/json"]["schema"] = SchemaBuilder::cyclic_subscription_schema();

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
  get_op["tags"] = nlohmann::json::array({"Events"});
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

nlohmann::json PathBuilder::error_responses() const {
  nlohmann::json errors;

  errors["400"]["description"] = "Bad request";
  errors["400"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  errors["404"]["description"] = "Not found";
  errors["404"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  errors["500"]["description"] = "Internal server error";
  errors["500"]["content"]["application/json"]["schema"] = SchemaBuilder::generic_error();

  if (auth_enabled_) {
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
