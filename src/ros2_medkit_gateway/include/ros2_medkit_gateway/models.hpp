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

#pragma once

#include <chrono>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief QoS profile information for a topic endpoint
 */
struct QosProfile {
  std::string reliability;  ///< "reliable", "best_effort", "system_default", "unknown"
  std::string durability;   ///< "volatile", "transient_local", "system_default", "unknown"
  std::string history;      ///< "keep_last", "keep_all", "system_default", "unknown"
  size_t depth{0};          ///< History depth (for keep_last)
  std::string liveliness;   ///< "automatic", "manual_by_topic", "system_default", "unknown"

  json to_json() const {
    return {{"reliability", reliability},
            {"durability", durability},
            {"history", history},
            {"depth", depth},
            {"liveliness", liveliness}};
  }
};

/**
 * @brief Information about an endpoint (publisher or subscriber) on a topic
 */
struct TopicEndpoint {
  std::string node_name;       ///< Name of the node (e.g., "controller_server")
  std::string node_namespace;  ///< Namespace of the node (e.g., "/navigation")
  std::string topic_type;      ///< Message type (e.g., "geometry_msgs/msg/Twist")
  QosProfile qos;              ///< QoS profile of this endpoint

  /// Get fully qualified node name
  std::string fqn() const {
    if (node_namespace == "/" || node_namespace.empty()) {
      return "/" + node_name;
    }
    return node_namespace + "/" + node_name;
  }

  json to_json() const {
    return {{"node_name", node_name}, {"node_namespace", node_namespace}, {"fqn", fqn()}, {"qos", qos.to_json()}};
  }
};

/**
 * @brief Topic with its publishers and subscribers
 */
struct TopicConnection {
  std::string topic_name;  ///< Full topic path (e.g., "/cmd_vel")
  std::string topic_type;  ///< Message type
  std::vector<TopicEndpoint> publishers;
  std::vector<TopicEndpoint> subscribers;

  json to_json() const {
    json pub_json = json::array();
    for (const auto & p : publishers) {
      pub_json.push_back(p.to_json());
    }
    json sub_json = json::array();
    for (const auto & s : subscribers) {
      sub_json.push_back(s.to_json());
    }
    return {{"topic", topic_name}, {"type", topic_type}, {"publishers", pub_json}, {"subscribers", sub_json}};
  }
};

/**
 * @brief Topics associated with a component (node)
 */
struct ComponentTopics {
  std::vector<std::string> publishes;   ///< Topics this component publishes to
  std::vector<std::string> subscribes;  ///< Topics this component subscribes to

  json to_json() const {
    return {{"publishes", publishes}, {"subscribes", subscribes}};
  }
};

struct Area {
  std::string id;
  std::string namespace_path;
  std::string type = "Area";

  json to_json() const {
    return {{"id", id}, {"namespace", namespace_path}, {"type", type}};
  }
};

/// Information about a ROS2 service discovered in the system
struct ServiceInfo {
  std::string name;               // Service name (e.g., "calibrate")
  std::string full_path;          // Full service path (e.g., "/powertrain/engine/calibrate")
  std::string type;               // Service type (e.g., "std_srvs/srv/Trigger")
  std::optional<json> type_info;  // Schema info with request/response schemas

  json to_json() const {
    json j = {{"name", name}, {"path", full_path}, {"type", type}, {"kind", "service"}};
    if (type_info.has_value()) {
      j["type_info"] = type_info.value();
    }
    return j;
  }
};

/// Information about a ROS2 action discovered in the system
struct ActionInfo {
  std::string name;               // Action name (e.g., "navigate_to_pose")
  std::string full_path;          // Full action path (e.g., "/navigation/navigate_to_pose")
  std::string type;               // Action type (e.g., "nav2_msgs/action/NavigateToPose")
  std::optional<json> type_info;  // Schema info with goal/result/feedback schemas

  json to_json() const {
    json j = {{"name", name}, {"path", full_path}, {"type", type}, {"kind", "action"}};
    if (type_info.has_value()) {
      j["type_info"] = type_info.value();
    }
    return j;
  }
};

struct Component {
  std::string id;
  std::string namespace_path;
  std::string fqn;
  std::string type = "Component";
  std::string area;
  std::string source = "node";  ///< Discovery source: "node" or "topic"
  std::vector<ServiceInfo> services;
  std::vector<ActionInfo> actions;
  ComponentTopics topics;  ///< Topics this component publishes/subscribes

  json to_json() const {
    json j = {{"id", id},         {"namespace", namespace_path}, {"fqn", fqn}, {"type", type}, {"area", area},
              {"source", source}, {"topics", topics.to_json()}};

    // Add operations array combining services and actions
    json operations = json::array();
    for (const auto & svc : services) {
      operations.push_back(svc.to_json());
    }
    for (const auto & act : actions) {
      operations.push_back(act.to_json());
    }
    if (!operations.empty()) {
      j["operations"] = operations;
    }

    return j;
  }
};

struct EntityCache {
  std::vector<Area> areas;
  std::vector<Component> components;
  std::chrono::system_clock::time_point last_update;
};

}  // namespace ros2_medkit_gateway
