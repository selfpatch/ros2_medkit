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

#include "ros2_medkit_gateway/http/x_medkit.hpp"

namespace ros2_medkit_gateway {

// ==================== ROS2 metadata ====================

XMedkit & XMedkit::ros2_node(const std::string & node_name) {
  ros2_["node"] = node_name;
  return *this;
}

XMedkit & XMedkit::ros2_namespace(const std::string & ns) {
  ros2_["namespace"] = ns;
  return *this;
}

XMedkit & XMedkit::ros2_type(const std::string & type) {
  ros2_["type"] = type;
  return *this;
}

XMedkit & XMedkit::ros2_topic(const std::string & topic) {
  ros2_["topic"] = topic;
  return *this;
}

XMedkit & XMedkit::ros2_service(const std::string & service) {
  ros2_["service"] = service;
  return *this;
}

XMedkit & XMedkit::ros2_action(const std::string & action) {
  ros2_["action"] = action;
  return *this;
}

XMedkit & XMedkit::ros2_kind(const std::string & kind) {
  ros2_["kind"] = kind;
  return *this;
}

// ==================== Discovery metadata ====================

XMedkit & XMedkit::source(const std::string & source) {
  other_["source"] = source;
  return *this;
}

XMedkit & XMedkit::is_online(bool online) {
  other_["is_online"] = online;
  return *this;
}

XMedkit & XMedkit::component_id(const std::string & id) {
  other_["component_id"] = id;
  return *this;
}

XMedkit & XMedkit::entity_id(const std::string & id) {
  other_["entity_id"] = id;
  return *this;
}

// ==================== Type introspection ====================

XMedkit & XMedkit::type_info(const nlohmann::json & info) {
  other_["type_info"] = info;
  return *this;
}

XMedkit & XMedkit::type_schema(const nlohmann::json & schema) {
  other_["type_schema"] = schema;
  return *this;
}

// ==================== Execution tracking ====================

XMedkit & XMedkit::goal_id(const std::string & id) {
  other_["goal_id"] = id;
  return *this;
}

XMedkit & XMedkit::goal_status(const std::string & status) {
  other_["goal_status"] = status;
  return *this;
}

XMedkit & XMedkit::last_feedback(const nlohmann::json & feedback) {
  other_["last_feedback"] = feedback;
  return *this;
}

// ==================== Generic methods ====================

XMedkit & XMedkit::add(const std::string & key, const nlohmann::json & value) {
  other_[key] = value;
  return *this;
}

XMedkit & XMedkit::add_ros2(const std::string & key, const nlohmann::json & value) {
  ros2_[key] = value;
  return *this;
}

nlohmann::json XMedkit::build() const {
  nlohmann::json result;

  if (!ros2_.empty()) {
    result["ros2"] = ros2_;
  }

  for (const auto & [key, value] : other_.items()) {
    result[key] = value;
  }

  return result;
}

bool XMedkit::empty() const {
  return ros2_.empty() && other_.empty();
}

}  // namespace ros2_medkit_gateway
