// Copyright 2025 bburda, mfaferek93
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

namespace ros2_medkit_gateway {

/**
 * @brief Fluent builder for x-medkit extension JSON object.
 *
 * The x-medkit extension provides a clean separation between SOVD-compliant
 * fields and ros2_medkit-specific extensions in API responses.
 *
 * Example usage:
 * @code
 * XMedkit ext;
 * ext.ros2_node("/sensors/temp_sensor")
 *    .ros2_type("sensor_msgs/msg/Temperature")
 *    .source("heuristic")
 *    .is_online(true);
 *
 * json response;
 * response["id"] = "temp_sensor";
 * response["name"] = "Temperature Sensor";
 * if (!ext.empty()) {
 *   response["x-medkit"] = ext.build();
 * }
 * @endcode
 *
 * Output structure:
 * @code{.json}
 * {
 *   "id": "temp_sensor",
 *   "name": "Temperature Sensor",
 *   "x-medkit": {
 *     "ros2": {
 *       "node": "/sensors/temp_sensor",
 *       "type": "sensor_msgs/msg/Temperature"
 *     },
 *     "source": "heuristic",
 *     "is_online": true
 *   }
 * }
 * @endcode
 */
class XMedkit {
 public:
  XMedkit() = default;

  // ==================== ROS2 metadata ====================

  /**
   * @brief Set the ROS2 node name.
   * @param node_name Fully qualified node name (e.g., "/namespace/node_name")
   */
  XMedkit & ros2_node(const std::string & node_name);

  /**
   * @brief Set the ROS2 namespace.
   * @param ns Namespace (e.g., "/sensors")
   */
  XMedkit & ros2_namespace(const std::string & ns);

  /**
   * @brief Set the ROS2 message/service/action type.
   * @param type Type string (e.g., "std_msgs/msg/String")
   */
  XMedkit & ros2_type(const std::string & type);

  /**
   * @brief Set the ROS2 topic name.
   * @param topic Topic path (e.g., "/sensors/temperature")
   */
  XMedkit & ros2_topic(const std::string & topic);

  /**
   * @brief Set the ROS2 service name.
   * @param service Service path (e.g., "/calibrate")
   */
  XMedkit & ros2_service(const std::string & service);

  /**
   * @brief Set the ROS2 action name.
   * @param action Action path (e.g., "/navigate")
   */
  XMedkit & ros2_action(const std::string & action);

  /**
   * @brief Set the ROS2 interface kind.
   * @param kind Interface kind ("topic", "service", "action")
   */
  XMedkit & ros2_kind(const std::string & kind);

  // ==================== Discovery metadata ====================

  /**
   * @brief Set the entity discovery source.
   * @param source Discovery source ("heuristic", "static", "runtime")
   */
  XMedkit & source(const std::string & source);

  /**
   * @brief Set the entity online status.
   * @param online True if the entity is currently online/available
   */
  XMedkit & is_online(bool online);

  /**
   * @brief Set the parent component ID.
   * @param id Component identifier
   */
  XMedkit & component_id(const std::string & id);

  /**
   * @brief Set a generic entity ID reference.
   * @param id Entity identifier
   */
  XMedkit & entity_id(const std::string & id);

  // ==================== Type introspection ====================

  /**
   * @brief Set type introspection information.
   * @param info JSON object with type metadata
   */
  XMedkit & type_info(const nlohmann::json & info);

  /**
   * @brief Set ROS2 IDL type schema.
   *
   * Note: This is distinct from SOVD's OpenAPI schema. The type_schema contains
   * ROS2 message/service/action structure derived from IDL definitions.
   *
   * @param schema JSON object with IDL-derived type structure
   */
  XMedkit & type_schema(const nlohmann::json & schema);

  // ==================== Execution tracking ====================

  /**
   * @brief Set the ROS2 action goal ID.
   * @param id Goal UUID string
   */
  XMedkit & goal_id(const std::string & id);

  /**
   * @brief Set the ROS2 action goal status.
   * @param status Status string ("pending", "executing", "succeeded", "canceled", "aborted")
   */
  XMedkit & goal_status(const std::string & status);

  /**
   * @brief Set the last received action feedback.
   * @param feedback JSON object with feedback data
   */
  XMedkit & last_feedback(const nlohmann::json & feedback);

  // ==================== Generic methods ====================

  /**
   * @brief Add a custom field to the x-medkit object (top level).
   * @param key Field name
   * @param value Field value
   */
  XMedkit & add(const std::string & key, const nlohmann::json & value);

  /**
   * @brief Add a custom field to the ros2 sub-object.
   * @param key Field name
   * @param value Field value
   */
  XMedkit & add_ros2(const std::string & key, const nlohmann::json & value);

  /**
   * @brief Build the final x-medkit JSON object.
   * @return JSON object containing all set fields
   */
  nlohmann::json build() const;

  /**
   * @brief Check if any fields have been set.
   * @return True if no fields have been set
   */
  bool empty() const;

 private:
  nlohmann::json ros2_;   ///< ROS2-specific metadata
  nlohmann::json other_;  ///< Other extension fields
};

}  // namespace ros2_medkit_gateway
