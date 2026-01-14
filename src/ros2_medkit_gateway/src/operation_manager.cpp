// Copyright 2025 mfaferek93
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

#include "ros2_medkit_gateway/operation_manager.hpp"

#include <algorithm>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>

#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"
#include "ros2_medkit_serialization/service_action_types.hpp"
#include "ros2_medkit_serialization/type_cache.hpp"

namespace ros2_medkit_gateway {

/// UUID hex string length (16 bytes = 32 hex characters)
constexpr size_t kUuidHexLength = 32;

/// Default timeout for service calls in seconds
constexpr int kDefaultServiceCallTimeoutSec = 10;

OperationManager::OperationManager(rclcpp::Node * node, DiscoveryManager * discovery_manager)
  : node_(node)
  , discovery_manager_(discovery_manager)
  , rng_(std::random_device{}())
  , serializer_(std::make_shared<ros2_medkit_serialization::JsonSerializer>())
  , service_call_timeout_sec_(
        static_cast<int>(node->declare_parameter<int64_t>("service_call_timeout_sec", kDefaultServiceCallTimeoutSec))) {
  RCLCPP_INFO(node_->get_logger(), "OperationManager initialized with native serialization");
}

bool OperationManager::is_valid_message_type(const std::string & type) {
  // Valid formats:
  // - package/srv/Type (service)
  // - package/action/Type (action)
  // - package/msg/Type (message - for publishing)
  static const std::regex type_regex(R"(^[a-zA-Z_][a-zA-Z0-9_]*/(srv|action|msg)/[a-zA-Z_][a-zA-Z0-9_]*$)");
  return std::regex_match(type, type_regex);
}

bool OperationManager::is_service_type(const std::string & type) {
  return type.find("/srv/") != std::string::npos;
}

bool OperationManager::is_action_type(const std::string & type) {
  return type.find("/action/") != std::string::npos;
}

bool OperationManager::is_valid_uuid_hex(const std::string & uuid_hex) {
  if (uuid_hex.length() != kUuidHexLength) {
    return false;
  }
  return std::all_of(uuid_hex.begin(), uuid_hex.end(), [](char c) {
    return std::isxdigit(static_cast<unsigned char>(c));
  });
}

std::string OperationManager::make_client_key(const std::string & service_path, const std::string & service_type) {
  return service_path + "|" + service_type;
}

rclcpp::GenericClient::SharedPtr OperationManager::get_or_create_service_client(const std::string & service_path,
                                                                                const std::string & service_type) {
  const std::string key = make_client_key(service_path, service_type);

  // Try read lock first (fast path)
  {
    std::shared_lock<std::shared_mutex> lock(clients_mutex_);
    auto it = generic_clients_.find(key);
    if (it != generic_clients_.end()) {
      return it->second;
    }
  }

  // Need to create - take exclusive lock
  std::unique_lock<std::shared_mutex> lock(clients_mutex_);

  // Double-check (another thread might have created it)
  auto it = generic_clients_.find(key);
  if (it != generic_clients_.end()) {
    return it->second;
  }

  // Create new client
  auto client = node_->create_generic_client(service_path, service_type);
  generic_clients_[key] = client;

  RCLCPP_DEBUG(node_->get_logger(), "Created generic client for %s (%s)", service_path.c_str(), service_type.c_str());

  return client;
}

ServiceCallResult OperationManager::call_service(const std::string & service_path, const std::string & service_type,
                                                 const json & request) {
  ServiceCallResult result;

  try {
    using ros2_medkit_serialization::ServiceActionTypes;

    // Step 1: Get or create cached client
    auto client = get_or_create_service_client(service_path, service_type);

    // Step 2: Wait for service availability
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      result.success = false;
      result.error_message = "Service not available: " + service_path;
      return result;
    }

    // Step 3: Build request/response type names
    std::string request_type = ServiceActionTypes::get_service_request_type(service_type);
    std::string response_type = ServiceActionTypes::get_service_response_type(service_type);

    // Step 4: Create request message from JSON
    json request_data = request.empty() || request.is_null() ? json::object() : request;

    // Get defaults for request type and merge with provided data
    try {
      json defaults = serializer_->get_defaults(request_type);
      // Merge: request overrides defaults
      for (auto it = request_data.begin(); it != request_data.end(); ++it) {
        defaults[it.key()] = it.value();
      }
      request_data = defaults;
    } catch (const ros2_medkit_serialization::TypeNotFoundError &) {
      // If we can't get defaults, just use provided data
    }

    // Convert JSON to ROS message (deserialized form, not CDR)
    // Note: GenericClient expects void* pointing to deserialized message structure
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    RosMessage_Cpp ros_request = serializer_->from_json(request_type, request_data);

    RCLCPP_INFO(node_->get_logger(), "Calling service: %s (type: %s)", service_path.c_str(), service_type.c_str());

    // Step 5: Send request using GenericClient (expects void* to deserialized message)
    auto future_and_id = client->async_send_request(ros_request.data);

    // Step 6: Wait for response with timeout
    auto timeout = std::chrono::seconds(service_call_timeout_sec_);
    auto future_status = future_and_id.wait_for(timeout);

    if (future_status != std::future_status::ready) {
      // Clean up pending request on timeout
      client->remove_pending_request(future_and_id.request_id);
      dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);
      result.success = false;
      result.error_message =
          "Service call timed out (" + std::to_string(service_call_timeout_sec_) + "s): " + service_path;
      return result;
    }

    // Clean up request message after sending
    dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);

    // Step 7: Get response and deserialize
    auto response_ptr = future_and_id.get();

    // The response is a void* pointing to the deserialized response message data
    // We need to convert it back to JSON via the serializer
    // GenericClient returns a SharedResponse (shared_ptr<void>)
    // The data is already deserialized by rclcpp internally

    // For now, we need to serialize the response back to get JSON
    // This is a bit roundabout but works with the current GenericClient API
    // TODO: Optimize by accessing response data directly when possible

    if (response_ptr != nullptr) {
      // Create a temporary serialized message to deserialize from
      // The response_ptr points to deserialized message data
      // We need to re-serialize it to use our deserializer

      // Alternative approach: use the type info to read fields directly
      // For now, try to get the response via YAML (simpler but less efficient)

      // Actually, GenericClient gives us a deserialized message already
      // We can use dynmsg to convert it to JSON directly
      try {
        auto type_info = ros2_medkit_serialization::TypeCache::instance().get_message_type_info(response_type);
        if (type_info != nullptr) {
          result.response = serializer_->to_json(type_info, response_ptr.get());
          result.success = true;
          RCLCPP_DEBUG(node_->get_logger(), "Service call succeeded: %s", result.response.dump().c_str());
        } else {
          result.success = false;
          result.error_message = "Unknown response type: " + response_type;
        }
      } catch (const std::exception & e) {
        result.success = false;
        result.error_message = std::string("Failed to deserialize response: ") + e.what();
      }
    } else {
      result.success = false;
      result.error_message = "Service returned null response";
    }

  } catch (const ros2_medkit_serialization::TypeNotFoundError & e) {
    RCLCPP_ERROR(node_->get_logger(), "Type not found for service '%s': %s", service_path.c_str(), e.what());
    result.success = false;
    result.error_message = std::string("Unknown service type: ") + e.what();

  } catch (const ros2_medkit_serialization::SerializationError & e) {
    RCLCPP_ERROR(node_->get_logger(), "Serialization failed for service '%s': %s", service_path.c_str(), e.what());
    result.success = false;
    result.error_message = std::string("Invalid request format: ") + e.what();

  } catch (const std::exception & e) {
    std::string error_msg = e.what();
    RCLCPP_ERROR(node_->get_logger(), "Service call failed for '%s': %s", service_path.c_str(), e.what());
    result.success = false;
    result.error_message = error_msg;
  }

  return result;
}

ServiceCallResult OperationManager::call_component_service(const std::string & component_ns,
                                                           const std::string & operation_name,
                                                           const std::optional<std::string> & service_type,
                                                           const json & request) {
  ServiceCallResult result;

  // Determine service type - use provided or look up from discovery
  std::string resolved_type;
  std::string service_path;

  if (service_type.has_value() && !service_type->empty()) {
    resolved_type = *service_type;
    // Construct service path from component namespace and operation name
    service_path = component_ns;
    if (!service_path.empty() && service_path.back() != '/') {
      service_path += "/";
    }
    service_path += operation_name;
  } else {
    // Look up from discovery cache
    auto service_info = discovery_manager_->find_service(component_ns, operation_name);
    if (!service_info.has_value()) {
      result.success = false;
      result.error_message = "Service not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    resolved_type = service_info->type;
    service_path = service_info->full_path;
  }

  // Validate type format
  if (!is_valid_message_type(resolved_type)) {
    result.success = false;
    result.error_message = "Invalid service type format: " + resolved_type;
    return result;
  }

  // Verify it's a service type
  if (!is_service_type(resolved_type)) {
    result.success = false;
    result.error_message = "Type is not a service type: " + resolved_type;
    return result;
  }

  return call_service(service_path, resolved_type, request);
}

// ==================== ACTION OPERATIONS ====================

std::string action_status_to_string(ActionGoalStatus status) {
  switch (status) {
    case ActionGoalStatus::UNKNOWN:
      return "unknown";
    case ActionGoalStatus::ACCEPTED:
      return "accepted";
    case ActionGoalStatus::EXECUTING:
      return "executing";
    case ActionGoalStatus::CANCELING:
      return "canceling";
    case ActionGoalStatus::SUCCEEDED:
      return "succeeded";
    case ActionGoalStatus::CANCELED:
      return "canceled";
    case ActionGoalStatus::ABORTED:
      return "aborted";
    default:
      return "unknown";
  }
}

std::array<uint8_t, 16> OperationManager::generate_uuid() {
  std::lock_guard<std::mutex> lock(rng_mutex_);
  std::array<uint8_t, 16> uuid;
  std::uniform_int_distribution<int> dist(0, 255);
  for (auto & byte : uuid) {
    byte = static_cast<uint8_t>(dist(rng_));
  }
  // Set version 4 (random UUID)
  uuid[6] = (uuid[6] & 0x0f) | 0x40;
  // Set variant bits
  uuid[8] = (uuid[8] & 0x3f) | 0x80;
  return uuid;
}

json OperationManager::uuid_bytes_to_json_array(const std::array<uint8_t, 16> & uuid) {
  json array = json::array();
  for (auto byte : uuid) {
    array.push_back(static_cast<int>(byte));
  }
  return array;
}

json OperationManager::uuid_hex_to_json_array(const std::string & uuid_hex) {
  json array = json::array();
  for (size_t i = 0; i + 1 < uuid_hex.length() && i < kUuidHexLength; i += 2) {
    int byte_val = std::stoi(uuid_hex.substr(i, 2), nullptr, 16);
    array.push_back(byte_val);
  }
  return array;
}

OperationManager::ActionClientSet & OperationManager::get_or_create_action_clients(const std::string & action_path,
                                                                                   const std::string & action_type) {
  std::unique_lock<std::shared_mutex> lock(clients_mutex_);

  auto it = action_clients_.find(action_path);
  if (it != action_clients_.end()) {
    return it->second;
  }

  using namespace ros2_medkit_serialization;

  ActionClientSet clients;
  clients.action_type = action_type;

  // Send goal service: {action}/_action/send_goal
  std::string send_goal_service = action_path + "/_action/send_goal";
  std::string send_goal_type = ServiceActionTypes::get_action_send_goal_service_type(action_type);
  clients.send_goal_client = node_->create_generic_client(send_goal_service, send_goal_type);

  // Get result service: {action}/_action/get_result
  std::string get_result_service = action_path + "/_action/get_result";
  std::string get_result_type = ServiceActionTypes::get_action_get_result_service_type(action_type);
  clients.get_result_client = node_->create_generic_client(get_result_service, get_result_type);

  // Cancel goal service (standard type for all actions)
  std::string cancel_service = action_path + "/_action/cancel_goal";
  clients.cancel_goal_client = node_->create_generic_client(cancel_service, "action_msgs/srv/CancelGoal");

  RCLCPP_DEBUG(node_->get_logger(), "Created action clients for %s (type: %s)", action_path.c_str(),
               action_type.c_str());

  action_clients_[action_path] = std::move(clients);
  return action_clients_[action_path];
}

void OperationManager::track_goal(const std::string & goal_id, const std::string & action_path,
                                  const std::string & action_type) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  ActionGoalInfo info;
  info.goal_id = goal_id;
  info.action_path = action_path;
  info.action_type = action_type;
  info.status = ActionGoalStatus::ACCEPTED;
  info.created_at = std::chrono::system_clock::now();
  info.last_update = info.created_at;
  tracked_goals_[goal_id] = info;
}

ActionSendGoalResult OperationManager::send_action_goal(const std::string & action_path,
                                                        const std::string & action_type, const json & goal) {
  ActionSendGoalResult result;
  result.success = false;
  result.goal_accepted = false;

  try {
    using namespace ros2_medkit_serialization;

    // Step 1: Get or create action clients
    auto & clients = get_or_create_action_clients(action_path, action_type);

    // Step 2: Wait for send_goal service
    if (!clients.send_goal_client->wait_for_service(std::chrono::seconds(5))) {
      result.error_message = "Action server not available: " + action_path;
      return result;
    }

    // Step 3: Generate UUID for goal
    auto uuid_bytes = generate_uuid();

    // Step 4: Build SendGoal request
    // The request has structure: {goal_id: {uuid: [...]}, goal: {...}}
    json send_goal_request;
    send_goal_request["goal_id"]["uuid"] = uuid_bytes_to_json_array(uuid_bytes);
    send_goal_request["goal"] = goal.empty() || goal.is_null() ? json::object() : goal;

    // Step 5: Create request message from JSON and send
    std::string request_type = ServiceActionTypes::get_action_send_goal_request_type(action_type);

    RCLCPP_INFO(node_->get_logger(), "SendGoal request type: %s, JSON: %s", request_type.c_str(),
                send_goal_request.dump().c_str());

    // Convert JSON to ROS message (deserialized form, not CDR)
    // Note: GenericClient expects void* pointing to deserialized message structure
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    RosMessage_Cpp ros_request = serializer_->from_json(request_type, send_goal_request);

    RCLCPP_INFO(node_->get_logger(), "Sending action goal: %s (type: %s)", action_path.c_str(), action_type.c_str());

    // Send using GenericClient (expects void* to deserialized message)
    auto future_and_id = clients.send_goal_client->async_send_request(ros_request.data);

    // Wait for response with timeout
    auto timeout = std::chrono::seconds(service_call_timeout_sec_);
    auto future_status = future_and_id.wait_for(timeout);

    if (future_status != std::future_status::ready) {
      clients.send_goal_client->remove_pending_request(future_and_id.request_id);
      dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);
      result.error_message = "Send goal timed out";
      return result;
    }

    // Clean up request message after sending
    dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);

    // Step 6: Deserialize response
    auto response_ptr = future_and_id.get();
    if (response_ptr == nullptr) {
      result.error_message = "Send goal returned null response";
      return result;
    }

    std::string response_type = ServiceActionTypes::get_action_send_goal_response_type(action_type);
    auto type_info = ros2_medkit_serialization::TypeCache::instance().get_message_type_info(response_type);
    if (type_info == nullptr) {
      result.error_message = "Unknown response type: " + response_type;
      return result;
    }
    json response = serializer_->to_json(type_info, response_ptr.get());

    // Step 7: Extract goal_id and acceptance
    result.success = true;
    result.goal_accepted = response.value("accepted", false);

    if (result.goal_accepted) {
      result.goal_id = uuid_bytes_to_hex(uuid_bytes);
      track_goal(result.goal_id, action_path, action_type);
      subscribe_to_action_status(action_path);
      update_goal_status(result.goal_id, ActionGoalStatus::EXECUTING);
      RCLCPP_INFO(node_->get_logger(), "Action goal accepted with ID: %s", result.goal_id.c_str());
    } else {
      result.error_message = "Goal rejected by action server";
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Send action goal failed for '%s': %s", action_path.c_str(), e.what());
    result.success = false;
    result.error_message = e.what();
  }

  return result;
}

ActionSendGoalResult OperationManager::send_component_action_goal(const std::string & component_ns,
                                                                  const std::string & operation_name,
                                                                  const std::optional<std::string> & action_type,
                                                                  const json & goal) {
  ActionSendGoalResult result;

  // Determine action type - use provided or look up from discovery
  std::string resolved_type;
  std::string action_path;

  if (action_type.has_value() && !action_type->empty()) {
    resolved_type = *action_type;
    // Construct action path from component namespace and operation name
    action_path = component_ns;
    if (!action_path.empty() && action_path.back() != '/') {
      action_path += "/";
    }
    action_path += operation_name;
  } else {
    // Look up from discovery cache
    auto action_info = discovery_manager_->find_action(component_ns, operation_name);
    if (!action_info.has_value()) {
      result.success = false;
      result.error_message = "Action not found: " + operation_name + " in namespace " + component_ns;
      return result;
    }
    resolved_type = action_info->type;
    action_path = action_info->full_path;
  }

  // Validate type format
  if (!is_valid_message_type(resolved_type)) {
    result.success = false;
    result.error_message = "Invalid action type format: " + resolved_type;
    return result;
  }

  // Verify it's an action type
  if (!is_action_type(resolved_type)) {
    result.success = false;
    result.error_message = "Type is not an action type: " + resolved_type;
    return result;
  }

  return send_action_goal(action_path, resolved_type, goal);
}

ActionCancelResult OperationManager::cancel_action_goal(const std::string & action_path, const std::string & goal_id) {
  ActionCancelResult result;
  result.success = false;

  // Validate goal_id format
  if (!is_valid_uuid_hex(goal_id)) {
    result.error_message = "Invalid goal_id format: must be 32 hex characters";
    return result;
  }

  try {
    // Get the tracked goal to find action type
    auto goal_info = get_tracked_goal(goal_id);
    if (!goal_info) {
      result.error_message = "Unknown goal_id - not tracked";
      return result;
    }

    // Get or create action clients (use tracked type)
    auto & clients = get_or_create_action_clients(action_path, goal_info->action_type);

    if (!clients.cancel_goal_client->wait_for_service(std::chrono::seconds(2))) {
      result.error_message = "Cancel service not available";
      return result;
    }

    // Build cancel request: {goal_info: {goal_id: {uuid: [...]}}}
    json cancel_request;
    cancel_request["goal_info"]["goal_id"]["uuid"] = uuid_hex_to_json_array(goal_id);

    // Convert JSON to ROS message (deserialized form, not CDR)
    // Note: GenericClient expects void* pointing to deserialized message structure
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    RosMessage_Cpp ros_request = serializer_->from_json("action_msgs/srv/CancelGoal_Request", cancel_request);

    RCLCPP_INFO(node_->get_logger(), "Canceling action goal: %s (goal_id: %s)", action_path.c_str(), goal_id.c_str());

    // Send using GenericClient (expects void* to deserialized message)
    auto future_and_id = clients.cancel_goal_client->async_send_request(ros_request.data);

    auto future_status = future_and_id.wait_for(std::chrono::seconds(5));
    if (future_status != std::future_status::ready) {
      clients.cancel_goal_client->remove_pending_request(future_and_id.request_id);
      dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);
      result.error_message = "Cancel request timed out";
      return result;
    }
    dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);

    auto response_ptr = future_and_id.get();
    if (response_ptr == nullptr) {
      result.error_message = "Cancel returned null response";
      return result;
    }

    auto type_info =
        ros2_medkit_serialization::TypeCache::instance().get_message_type_info("action_msgs/srv/CancelGoal_Response");
    if (type_info == nullptr) {
      result.error_message = "Unknown response type: action_msgs/srv/CancelGoal_Response";
      return result;
    }
    json response = serializer_->to_json(type_info, response_ptr.get());

    result.success = true;
    result.return_code = static_cast<int8_t>(response.value("return_code", 0));

    if (result.return_code == 0) {
      update_goal_status(goal_id, ActionGoalStatus::CANCELING);
      RCLCPP_INFO(node_->get_logger(), "Cancel request accepted for goal: %s", goal_id.c_str());
    } else {
      // Map return codes to error messages
      switch (result.return_code) {
        case 1:
          result.error_message = "Cancel request rejected";
          break;
        case 2:
          result.error_message = "Unknown goal ID";
          break;
        case 3:
          result.error_message = "Goal already terminated";
          break;
        default:
          result.error_message = "Unknown cancel error";
          break;
      }
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Action cancel failed for '%s': %s", action_path.c_str(), e.what());
    result.error_message = e.what();
  }

  return result;
}

ActionGetResultResult OperationManager::get_action_result(const std::string & action_path,
                                                          const std::string & action_type,
                                                          const std::string & goal_id) {
  ActionGetResultResult result;
  result.success = false;
  result.status = ActionGoalStatus::UNKNOWN;

  // Validate goal_id format
  if (!is_valid_uuid_hex(goal_id)) {
    result.error_message = "Invalid goal_id format: must be 32 hex characters";
    return result;
  }

  try {
    using namespace ros2_medkit_serialization;

    // Get or create action clients
    auto & clients = get_or_create_action_clients(action_path, action_type);

    if (!clients.get_result_client->wait_for_service(std::chrono::seconds(2))) {
      result.error_message = "Get result service not available";
      return result;
    }

    // Build request: {goal_id: {uuid: [...]}}
    json get_result_request;
    get_result_request["goal_id"]["uuid"] = uuid_hex_to_json_array(goal_id);

    std::string request_type = ServiceActionTypes::get_action_get_result_request_type(action_type);

    // Convert JSON to ROS message (deserialized form, not CDR)
    // Note: GenericClient expects void* pointing to deserialized message structure
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    RosMessage_Cpp ros_request = serializer_->from_json(request_type, get_result_request);

    RCLCPP_INFO(node_->get_logger(), "Getting action result: %s (goal_id: %s)", action_path.c_str(), goal_id.c_str());

    // Send using GenericClient (expects void* to deserialized message)
    auto future_and_id = clients.get_result_client->async_send_request(ros_request.data);

    auto future_status = future_and_id.wait_for(std::chrono::seconds(service_call_timeout_sec_));
    if (future_status != std::future_status::ready) {
      clients.get_result_client->remove_pending_request(future_and_id.request_id);
      dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);
      result.error_message = "Get result timed out";
      return result;
    }
    dynmsg::cpp::ros_message_destroy_with_allocator(&ros_request, &allocator);

    auto response_ptr = future_and_id.get();
    if (response_ptr == nullptr) {
      result.error_message = "Get result returned null response";
      return result;
    }

    std::string response_type = ServiceActionTypes::get_action_get_result_response_type(action_type);
    auto type_info = ros2_medkit_serialization::TypeCache::instance().get_message_type_info(response_type);
    if (type_info == nullptr) {
      result.error_message = "Unknown response type: " + response_type;
      return result;
    }
    json response = serializer_->to_json(type_info, response_ptr.get());

    result.success = true;

    // Extract status from response
    if (response.contains("status")) {
      int status_val = response["status"].get<int>();
      result.status = static_cast<ActionGoalStatus>(status_val);
    }

    // Extract result from response
    if (response.contains("result")) {
      result.result = response["result"];
    } else {
      result.result = response;
    }

    // Update local tracking
    update_goal_status(goal_id, result.status);

    RCLCPP_INFO(node_->get_logger(), "Got action result for %s: status=%s", goal_id.c_str(),
                action_status_to_string(result.status).c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Get action result failed for '%s': %s", action_path.c_str(), e.what());
    result.error_message = e.what();
  }

  return result;
}

std::optional<ActionGoalInfo> OperationManager::get_tracked_goal(const std::string & goal_id) const {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::vector<ActionGoalInfo> OperationManager::list_tracked_goals() const {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  std::vector<ActionGoalInfo> goals;
  goals.reserve(tracked_goals_.size());
  for (const auto & [id, info] : tracked_goals_) {
    goals.push_back(info);
  }
  return goals;
}

std::vector<ActionGoalInfo> OperationManager::get_goals_for_action(const std::string & action_path) const {
  std::vector<ActionGoalInfo> goals;

  // Copy data under lock
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    for (const auto & [id, info] : tracked_goals_) {
      if (info.action_path == action_path) {
        goals.push_back(info);
      }
    }
  }  // Release lock before sorting

  // Sort by created_at, newest first (most recently created goal)
  std::sort(goals.begin(), goals.end(), [](const ActionGoalInfo & a, const ActionGoalInfo & b) {
    return a.created_at > b.created_at;
  });
  return goals;
}

std::optional<ActionGoalInfo> OperationManager::get_latest_goal_for_action(const std::string & action_path) const {
  auto goals = get_goals_for_action(action_path);
  if (goals.empty()) {
    return std::nullopt;
  }
  return goals.front();  // Already sorted newest first
}

void OperationManager::update_goal_status(const std::string & goal_id, ActionGoalStatus status) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    it->second.status = status;
    it->second.last_update = std::chrono::system_clock::now();
  }
}

void OperationManager::update_goal_feedback(const std::string & goal_id, const json & feedback) {
  std::lock_guard<std::mutex> lock(goals_mutex_);
  auto it = tracked_goals_.find(goal_id);
  if (it != tracked_goals_.end()) {
    it->second.last_feedback = feedback;
    it->second.last_update = std::chrono::system_clock::now();
  }
}

void OperationManager::cleanup_old_goals(std::chrono::seconds max_age) {
  std::set<std::string> actions_to_check;

  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    auto now = std::chrono::system_clock::now();

    for (auto it = tracked_goals_.begin(); it != tracked_goals_.end();) {
      // Only remove completed goals (succeeded, canceled, aborted)
      if (it->second.status == ActionGoalStatus::SUCCEEDED || it->second.status == ActionGoalStatus::CANCELED ||
          it->second.status == ActionGoalStatus::ABORTED) {
        auto age = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_update);
        if (age > max_age) {
          actions_to_check.insert(it->second.action_path);
          it = tracked_goals_.erase(it);
          continue;
        }
      }
      ++it;
    }
  }  // Release goals_mutex_ before checking subscriptions

  // Check if any action paths need to be unsubscribed
  for (const auto & action_path : actions_to_check) {
    auto remaining_goals = get_goals_for_action(action_path);
    if (remaining_goals.empty()) {
      unsubscribe_from_action_status(action_path);
    }
  }
}

// ==================== NATIVE STATUS SUBSCRIPTION ====================

std::string OperationManager::uuid_bytes_to_hex(const std::array<uint8_t, 16> & uuid) const {
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto & byte : uuid) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

void OperationManager::subscribe_to_action_status(const std::string & action_path) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  // Check if already subscribed
  if (status_subscriptions_.count(action_path) > 0) {
    return;
  }

  // Create subscription to status topic
  std::string status_topic = action_path + "/_action/status";

  auto callback = [this, action_path](const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
    on_action_status(action_path, msg);
  };

  auto subscription = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      status_topic, rclcpp::QoS(10).best_effort(), callback);

  status_subscriptions_[action_path] = subscription;
  RCLCPP_INFO(node_->get_logger(), "Subscribed to action status: %s", status_topic.c_str());
}

void OperationManager::unsubscribe_from_action_status(const std::string & action_path) {
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  auto it = status_subscriptions_.find(action_path);
  if (it != status_subscriptions_.end()) {
    status_subscriptions_.erase(it);
    RCLCPP_INFO(node_->get_logger(), "Unsubscribed from action status: %s/_action/status", action_path.c_str());
  }
}

void OperationManager::on_action_status(const std::string & action_path,
                                        const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
  std::lock_guard<std::mutex> lock(goals_mutex_);

  for (const auto & status : msg->status_list) {
    // Convert UUID bytes to hex string
    std::string goal_id = uuid_bytes_to_hex(status.goal_info.goal_id.uuid);

    // Find if we're tracking this goal
    auto it = tracked_goals_.find(goal_id);
    if (it != tracked_goals_.end() && it->second.action_path == action_path) {
      // Map status code to our enum
      ActionGoalStatus new_status;
      switch (status.status) {
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
          new_status = ActionGoalStatus::ACCEPTED;
          break;
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
          new_status = ActionGoalStatus::EXECUTING;
          break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
          new_status = ActionGoalStatus::CANCELING;
          break;
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
          new_status = ActionGoalStatus::SUCCEEDED;
          break;
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
          new_status = ActionGoalStatus::CANCELED;
          break;
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
          new_status = ActionGoalStatus::ABORTED;
          break;
        default:
          new_status = ActionGoalStatus::UNKNOWN;
          break;
      }

      // Only update if status changed
      if (it->second.status != new_status) {
        RCLCPP_INFO(node_->get_logger(), "Goal %s status update: %s -> %s", goal_id.c_str(),
                    action_status_to_string(it->second.status).c_str(), action_status_to_string(new_status).c_str());
        it->second.status = new_status;
        it->second.last_update = std::chrono::system_clock::now();
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
