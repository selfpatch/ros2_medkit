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

#include "ros2_medkit_serialization/json_serializer.hpp"

#include <sstream>

#include "dynmsg/message_reading.hpp"
#include "dynmsg/msg_parser.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace ros2_medkit_serialization {

nlohmann::json JsonSerializer::to_json(const TypeInfo_Cpp * type_info, const void * message_data) const {
  if (type_info == nullptr || message_data == nullptr) {
    throw JsonConversionError("Null type_info or message_data");
  }

  // Create a RosMessage_Cpp structure for dynmsg
  RosMessage_Cpp ros_msg;
  ros_msg.type_info = type_info;
  ros_msg.data = const_cast<uint8_t *>(static_cast<const uint8_t *>(message_data));

  // Use dynmsg to convert to YAML
  YAML::Node yaml_node = dynmsg::cpp::message_to_yaml(ros_msg);

  // Convert YAML to JSON
  return yaml_to_json(yaml_node);
}

nlohmann::json JsonSerializer::to_json(const std::string & type_string, const void * message_data) const {
  const TypeInfo_Cpp * type_info = get_type_info_or_throw(type_string);
  return to_json(type_info, message_data);
}

RosMessage_Cpp JsonSerializer::from_json(const TypeInfo_Cpp * type_info, const nlohmann::json & json) const {
  if (type_info == nullptr) {
    throw JsonConversionError("Null type_info");
  }

  // Convert JSON to YAML
  YAML::Node yaml_node = json_to_yaml(json);

  // Convert YAML to string for dynmsg
  std::stringstream ss;
  ss << yaml_node;
  std::string yaml_str = ss.str();

  // Use dynmsg to parse YAML into ROS message
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  return dynmsg::cpp::yaml_and_typeinfo_to_rosmsg(type_info, yaml_str, &allocator);
}

RosMessage_Cpp JsonSerializer::from_json(const std::string & type_string, const nlohmann::json & json) const {
  const TypeInfo_Cpp * type_info = get_type_info_or_throw(type_string);
  return from_json(type_info, json);
}

void JsonSerializer::from_json_to_message(const TypeInfo_Cpp * type_info, const nlohmann::json & json,
                                          void * message_data) const {
  if (type_info == nullptr || message_data == nullptr) {
    throw JsonConversionError("Null type_info or message_data");
  }

  // Convert JSON to YAML
  YAML::Node yaml_node = json_to_yaml(json);

  // Convert YAML to string for dynmsg
  std::stringstream ss;
  ss << yaml_node;
  std::string yaml_str = ss.str();

  // Use dynmsg to populate existing message
  dynmsg::cpp::yaml_and_typeinfo_to_rosmsg(type_info, yaml_str, message_data);
}

nlohmann::json JsonSerializer::get_schema(const TypeInfo_Cpp * type_info) const {
  if (type_info == nullptr) {
    throw JsonConversionError("Null type_info");
  }

  nlohmann::json schema;
  schema["type"] = "object";
  schema["properties"] = nlohmann::json::object();

  // Iterate over message members to build schema
  for (uint32_t i = 0; i < type_info->member_count_; ++i) {
    const auto & member = type_info->members_[i];
    nlohmann::json prop;

    // Map ROS types to JSON schema types
    switch (member.type_id_) {
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
        prop["type"] = "boolean";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        prop["type"] = "integer";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
        prop["type"] = "number";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        prop["type"] = "string";
        break;
      case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE:
        // Nested message - recursively get schema
        if (member.members_ != nullptr) {
          prop = get_schema(static_cast<const TypeInfo_Cpp *>(member.members_->data));
        } else {
          prop["type"] = "object";
        }
        break;
      default:
        prop["type"] = "string";  // Fallback
        break;
    }

    // Handle arrays/sequences
    if (member.is_array_) {
      nlohmann::json array_schema;
      array_schema["type"] = "array";
      array_schema["items"] = prop;
      if (member.array_size_ > 0 && !member.is_upper_bound_) {
        // Fixed-size array
        array_schema["minItems"] = member.array_size_;
        array_schema["maxItems"] = member.array_size_;
      }
      prop = array_schema;
    }

    schema["properties"][member.name_] = prop;
  }

  return schema;
}

nlohmann::json JsonSerializer::get_schema(const std::string & type_string) const {
  const TypeInfo_Cpp * type_info = get_type_info_or_throw(type_string);
  return get_schema(type_info);
}

nlohmann::json JsonSerializer::get_defaults(const TypeInfo_Cpp * type_info) const {
  if (type_info == nullptr) {
    throw JsonConversionError("Null type_info");
  }

  // Allocate a default-initialized message
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  RosMessage_Cpp ros_msg;
  dynmsg::cpp::ros_message_with_typeinfo_init(type_info, &ros_msg, &allocator);

  // Convert the default message to JSON
  nlohmann::json result = to_json(type_info, ros_msg.data);

  // Clean up
  dynmsg::cpp::ros_message_destroy_with_allocator(&ros_msg, &allocator);

  return result;
}

nlohmann::json JsonSerializer::get_defaults(const std::string & type_string) const {
  const TypeInfo_Cpp * type_info = get_type_info_or_throw(type_string);
  return get_defaults(type_info);
}

nlohmann::json JsonSerializer::yaml_to_json(const YAML::Node & yaml) {
  switch (yaml.Type()) {
    case YAML::NodeType::Null:
      return nullptr;

    case YAML::NodeType::Scalar: {
      // Try to parse as different types
      // First try boolean
      try {
        bool b = yaml.as<bool>();
        // YAML::as<bool> succeeds for "true"/"false"/"yes"/"no" etc.
        std::string str = yaml.as<std::string>();
        if (str == "true" || str == "false" || str == "yes" || str == "no" || str == "True" || str == "False" ||
            str == "Yes" || str == "No") {
          return b;
        }
      } catch (...) {
      }

      // Try integer
      try {
        int64_t i = yaml.as<int64_t>();
        std::string str = yaml.as<std::string>();
        // Check if it looks like an integer
        if (!str.empty() && (std::isdigit(str[0]) || str[0] == '-')) {
          try {
            size_t pos;
            std::stoll(str, &pos);
            if (pos == str.length()) {
              return i;
            }
          } catch (...) {
          }
        }
      } catch (...) {
      }

      // Try floating point
      try {
        double d = yaml.as<double>();
        std::string str = yaml.as<std::string>();
        // Check if it looks like a number
        if (!str.empty() && (std::isdigit(str[0]) || str[0] == '-' || str[0] == '.')) {
          try {
            size_t pos;
            std::stod(str, &pos);
            if (pos == str.length()) {
              return d;
            }
          } catch (...) {
          }
        }
      } catch (...) {
      }

      // Fall back to string
      return yaml.as<std::string>();
    }

    case YAML::NodeType::Sequence: {
      nlohmann::json arr = nlohmann::json::array();
      for (const auto & elem : yaml) {
        arr.push_back(yaml_to_json(elem));
      }
      return arr;
    }

    case YAML::NodeType::Map: {
      nlohmann::json obj = nlohmann::json::object();
      for (const auto & pair : yaml) {
        obj[pair.first.as<std::string>()] = yaml_to_json(pair.second);
      }
      return obj;
    }

    default:
      return nullptr;
  }
}

YAML::Node JsonSerializer::json_to_yaml(const nlohmann::json & json) {
  YAML::Node yaml;

  if (json.is_null()) {
    return yaml;  // Null node
  } else if (json.is_boolean()) {
    yaml = json.get<bool>();
  } else if (json.is_number_integer()) {
    yaml = json.get<int64_t>();
  } else if (json.is_number_unsigned()) {
    yaml = json.get<uint64_t>();
  } else if (json.is_number_float()) {
    yaml = json.get<double>();
  } else if (json.is_string()) {
    yaml = json.get<std::string>();
  } else if (json.is_array()) {
    for (const auto & elem : json) {
      yaml.push_back(json_to_yaml(elem));
    }
  } else if (json.is_object()) {
    for (auto it = json.begin(); it != json.end(); ++it) {
      yaml[it.key()] = json_to_yaml(it.value());
    }
  }

  return yaml;
}

const TypeInfo_Cpp * JsonSerializer::get_type_info_or_throw(const std::string & type_string) const {
  const TypeInfo_Cpp * type_info = TypeCache::instance().get_message_type_info(type_string);

  if (type_info == nullptr) {
    throw TypeNotFoundError(type_string);
  }

  return type_info;
}

}  // namespace ros2_medkit_serialization
