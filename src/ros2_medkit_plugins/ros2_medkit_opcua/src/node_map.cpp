// Copyright 2026 mfaferek93
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

#include "ros2_medkit_opcua/node_map.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace ros2_medkit_gateway {

opcua::NodeId NodeMap::parse_node_id(const std::string & str) {
  // Parse strings like "ns=1;s=TankLevel" or "ns=1;i=1001"
  uint16_t ns = 0;
  std::string identifier;
  bool is_numeric = false;

  auto ns_pos = str.find("ns=");
  auto s_pos = str.find(";s=");
  auto i_pos = str.find(";i=");

  if (ns_pos != std::string::npos) {
    auto end = str.find(';', ns_pos);
    if (end != std::string::npos) {
      ns = static_cast<uint16_t>(std::stoi(str.substr(ns_pos + 3, end - ns_pos - 3)));
    }
  }

  if (s_pos != std::string::npos) {
    identifier = str.substr(s_pos + 3);
    is_numeric = false;
  } else if (i_pos != std::string::npos) {
    identifier = str.substr(i_pos + 3);
    is_numeric = true;
  }

  if (is_numeric) {
    return {ns, static_cast<uint32_t>(std::stoul(identifier))};
  }
  return {ns, identifier};
}

bool NodeMap::load(const std::string & yaml_path) {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path);

    // Top-level metadata
    if (root["area_id"]) {
      area_id_ = root["area_id"].as<std::string>();
    }
    if (root["area_name"]) {
      area_name_ = root["area_name"].as<std::string>();
    }
    if (root["component_id"]) {
      component_id_ = root["component_id"].as<std::string>();
    }
    if (root["component_name"]) {
      component_name_ = root["component_name"].as<std::string>();
    }
    if (root["auto_browse"]) {
      auto_browse_ = root["auto_browse"].as<bool>();
    }

    // Node entries
    auto nodes = root["nodes"];
    if (!nodes || !nodes.IsSequence()) {
      // Clear any stale state from previous load
      entries_.clear();
      entity_index_.clear();
      node_id_index_.clear();
      entity_defs_.clear();
      return false;
    }

    entries_.clear();
    entity_index_.clear();
    node_id_index_.clear();

    for (size_t i = 0; i < nodes.size(); ++i) {
      const auto & n = nodes[i];
      NodeMapEntry entry;

      entry.node_id_str = n["node_id"].as<std::string>();
      entry.node_id = parse_node_id(entry.node_id_str);
      entry.entity_id = n["entity_id"].as<std::string>();
      entry.data_name = n["data_name"].as<std::string>();
      entry.display_name = n["display_name"].as<std::string>(entry.data_name);
      entry.unit = n["unit"].as<std::string>("");
      entry.data_type = n["data_type"].as<std::string>("float");
      entry.writable = n["writable"].as<bool>(false);
      if (n["min_value"]) {
        entry.min_value = n["min_value"].as<double>();
      }
      if (n["max_value"]) {
        entry.max_value = n["max_value"].as<double>();
      }

      // ROS 2 topic for value bridging (auto-generate if not specified).
      // ROS 2 topic segments: must match [a-zA-Z_][a-zA-Z0-9_]*, no empty segments.
      auto sanitize_topic_segment = [](const std::string & s) -> std::string {
        if (s.empty()) {
          return "_unnamed";
        }
        std::string out = s;
        for (auto & c : out) {
          if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            c = '_';
          }
        }
        // ROS 2 topic segments cannot start with a digit
        if (std::isdigit(static_cast<unsigned char>(out[0]))) {
          out.insert(out.begin(), '_');
        }
        return out;
      };
      auto is_valid_ros2_topic = [](const std::string & t) -> bool {
        if (t.empty() || t[0] != '/') {
          return false;
        }
        for (char c : t) {
          if (c == '/') {
            continue;
          }
          if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
            return false;
          }
        }
        return t.find("//") == std::string::npos;
      };
      if (n["ros2_topic"]) {
        entry.ros2_topic = n["ros2_topic"].as<std::string>();
        if (!is_valid_ros2_topic(entry.ros2_topic)) {
          std::cerr << "NodeMap: invalid custom ros2_topic '" << entry.ros2_topic << "' for " << entry.node_id_str
                    << ", auto-generating" << std::endl;
          entry.ros2_topic.clear();
        }
      }
      if (entry.ros2_topic.empty()) {
        entry.ros2_topic =
            "/plc/" + sanitize_topic_segment(entry.entity_id) + "/" + sanitize_topic_segment(entry.data_name);
      }

      if (n["alarm"]) {
        AlarmConfig alarm;
        alarm.fault_code = n["alarm"]["fault_code"].as<std::string>();
        alarm.severity = n["alarm"]["severity"].as<std::string>("ERROR");
        alarm.message = n["alarm"]["message"].as<std::string>(alarm.fault_code);
        alarm.threshold = n["alarm"]["threshold"].as<double>(0.0);
        alarm.above_threshold = n["alarm"]["above_threshold"].as<bool>(true);
        entry.alarm = std::move(alarm);
      }

      size_t idx = entries_.size();
      node_id_index_[entry.node_id_str] = idx;
      entity_index_[entry.entity_id].push_back(idx);
      entries_.push_back(std::move(entry));
    }

    build_entity_defs();
    return true;

  } catch (const std::exception & e) {
    std::cerr << "NodeMap::load failed: " << e.what() << std::endl;
    return false;
  }
}

std::vector<const NodeMapEntry *> NodeMap::entries_for_entity(const std::string & entity_id) const {
  std::vector<const NodeMapEntry *> result;
  auto it = entity_index_.find(entity_id);
  if (it != entity_index_.end()) {
    for (auto idx : it->second) {
      result.push_back(&entries_[idx]);
    }
  }
  return result;
}

std::vector<const NodeMapEntry *> NodeMap::writable_entries_for_entity(const std::string & entity_id) const {
  std::vector<const NodeMapEntry *> result;
  auto it = entity_index_.find(entity_id);
  if (it != entity_index_.end()) {
    for (auto idx : it->second) {
      if (entries_[idx].writable) {
        result.push_back(&entries_[idx]);
      }
    }
  }
  return result;
}

const NodeMapEntry * NodeMap::find_by_data_name(const std::string & entity_id, const std::string & data_name) const {
  auto it = entity_index_.find(entity_id);
  if (it != entity_index_.end()) {
    for (auto idx : it->second) {
      if (entries_[idx].data_name == data_name) {
        return &entries_[idx];
      }
    }
  }
  return nullptr;
}

const NodeMapEntry * NodeMap::find_by_node_id(const std::string & node_id_str) const {
  auto it = node_id_index_.find(node_id_str);
  if (it != node_id_index_.end()) {
    return &entries_[it->second];
  }
  return nullptr;
}

std::vector<const NodeMapEntry *> NodeMap::alarm_entries() const {
  std::vector<const NodeMapEntry *> result;
  for (const auto & entry : entries_) {
    if (entry.alarm.has_value()) {
      result.push_back(&entry);
    }
  }
  return result;
}

void NodeMap::build_entity_defs() {
  entity_defs_.clear();

  // Group entries by entity_id
  std::unordered_map<std::string, PlcEntityDef> defs;

  for (const auto & entry : entries_) {
    auto & def = defs[entry.entity_id];
    if (def.id.empty()) {
      def.id = entry.entity_id;
      def.component_id = component_id_;
    }
    def.data_names.push_back(entry.data_name);
    if (entry.writable) {
      def.writable_names.push_back(entry.data_name);
    }
    if (entry.alarm.has_value()) {
      def.has_faults = true;
    }
  }

  // Build human-readable names from IDs
  for (auto & [id, def] : defs) {
    // Convert snake_case to Title Case
    std::string name;
    bool capitalize = true;
    for (char c : id) {
      if (c == '_') {
        name += ' ';
        capitalize = true;
      } else if (capitalize) {
        name += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        capitalize = false;
      } else {
        name += c;
      }
    }
    def.name = name;
    entity_defs_.push_back(std::move(def));
  }
}

}  // namespace ros2_medkit_gateway
