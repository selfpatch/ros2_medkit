// Copyright 2026 Selfpatch
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

#ifndef ROS2_MEDKIT_SERIALIZATION__JSON_SERIALIZER_HPP_
#define ROS2_MEDKIT_SERIALIZATION__JSON_SERIALIZER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/serialized_message.hpp>
#include <yaml-cpp/yaml.h>

#include "ros2_medkit_serialization/serialization_error.hpp"
#include "ros2_medkit_serialization/type_cache.hpp"

namespace ros2_medkit_serialization {

/// JSON serializer for ROS 2 messages using dynmsg.
///
/// This class provides the main API for converting between JSON and ROS 2
/// messages at runtime. It uses dynmsg's YAML capabilities internally and
/// converts between JSON and YAML formats.
///
/// @note This class is stateless and thread-safe.
class JsonSerializer {
 public:
  JsonSerializer() = default;
  ~JsonSerializer() = default;

  // Non-copyable but movable
  JsonSerializer(const JsonSerializer &) = delete;
  JsonSerializer & operator=(const JsonSerializer &) = delete;
  JsonSerializer(JsonSerializer &&) = default;
  JsonSerializer & operator=(JsonSerializer &&) = default;

  /// Convert a ROS 2 message to JSON
  ///
  /// @param type_info Type introspection info
  /// @param message_data Pointer to the message data
  /// @return JSON representation of the message
  /// @throws JsonConversionError if conversion fails
  nlohmann::json to_json(const TypeInfo_Cpp * type_info, const void * message_data) const;

  /// Convert a ROS 2 message to JSON using type string
  ///
  /// @param type_string Full type string (e.g., "std_msgs/msg/String")
  /// @param message_data Pointer to the message data
  /// @return JSON representation of the message
  /// @throws TypeNotFoundError if type cannot be loaded
  /// @throws JsonConversionError if conversion fails
  nlohmann::json to_json(const std::string & type_string, const void * message_data) const;

  /// Convert JSON to a ROS 2 message
  ///
  /// @param type_info Type introspection info
  /// @param json JSON data to convert
  /// @return RosMessage_Cpp containing allocated message data
  /// @throws JsonConversionError if conversion fails
  /// @note Caller is responsible for calling ros_message_destroy_with_allocator
  RosMessage_Cpp from_json(const TypeInfo_Cpp * type_info, const nlohmann::json & json) const;

  /// Convert JSON to a ROS 2 message using type string
  ///
  /// @param type_string Full type string (e.g., "std_msgs/msg/String")
  /// @param json JSON data to convert
  /// @return RosMessage_Cpp containing allocated message data
  /// @throws TypeNotFoundError if type cannot be loaded
  /// @throws JsonConversionError if conversion fails
  /// @note Caller is responsible for calling ros_message_destroy_with_allocator
  RosMessage_Cpp from_json(const std::string & type_string, const nlohmann::json & json) const;

  /// Populate an existing message from JSON
  ///
  /// @param type_info Type introspection info
  /// @param json JSON data to convert
  /// @param message_data Pointer to pre-allocated message to populate
  /// @throws JsonConversionError if conversion fails
  void from_json_to_message(const TypeInfo_Cpp * type_info, const nlohmann::json & json, void * message_data) const;

  /// Generate a JSON schema for a ROS 2 type
  ///
  /// @param type_info Type introspection info
  /// @return JSON schema object
  nlohmann::json get_schema(const TypeInfo_Cpp * type_info) const;

  /// Generate a JSON schema using type string
  ///
  /// @param type_string Full type string
  /// @return JSON schema object
  /// @throws TypeNotFoundError if type cannot be loaded
  nlohmann::json get_schema(const std::string & type_string) const;

  /// Get default values for a ROS 2 type as JSON
  ///
  /// @param type_info Type introspection info
  /// @return JSON object with default values
  nlohmann::json get_defaults(const TypeInfo_Cpp * type_info) const;

  /// Get default values using type string
  ///
  /// @param type_string Full type string
  /// @return JSON object with default values
  /// @throws TypeNotFoundError if type cannot be loaded
  nlohmann::json get_defaults(const std::string & type_string) const;

  // Serialization for GenericClient/GenericSubscription

  /// Serialize JSON to CDR format for use with GenericClient
  ///
  /// @param type_string Full type string (e.g., "std_srvs/srv/SetBool_Request")
  /// @param json JSON data to serialize
  /// @return SerializedMessage containing CDR data
  /// @throws TypeNotFoundError if type cannot be loaded
  /// @throws SerializationError if serialization fails
  rclcpp::SerializedMessage serialize(const std::string & type_string, const nlohmann::json & json) const;

  /// Deserialize CDR data to JSON
  ///
  /// @param type_string Full type string
  /// @param serialized_msg SerializedMessage containing CDR data
  /// @return JSON representation of the message
  /// @throws TypeNotFoundError if type cannot be loaded
  /// @throws SerializationError if deserialization fails
  nlohmann::json deserialize(const std::string & type_string, const rclcpp::SerializedMessage & serialized_msg) const;

  // YAML ↔ JSON conversion utilities

  /// Convert YAML node to JSON
  ///
  /// @param yaml YAML node to convert
  /// @return JSON representation
  static nlohmann::json yaml_to_json(const YAML::Node & yaml);

  /// Convert JSON to YAML node
  ///
  /// @param json JSON to convert
  /// @return YAML node representation
  static YAML::Node json_to_yaml(const nlohmann::json & json);

 private:
  /// Get type info from cache, throwing if not found
  const TypeInfo_Cpp * get_type_info_or_throw(const std::string & type_string) const;
};

}  // namespace ros2_medkit_serialization

#endif  // ROS2_MEDKIT_SERIALIZATION__JSON_SERIALIZER_HPP_
