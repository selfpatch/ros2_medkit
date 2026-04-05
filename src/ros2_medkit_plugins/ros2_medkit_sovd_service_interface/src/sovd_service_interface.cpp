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

#include "sovd_service_interface.hpp"

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"

namespace ros2_medkit_gateway {

namespace {

SovdEntityType entity_type_from_string(const std::string & type) {
  if (type == "app") {
    return SovdEntityType::APP;
  }
  if (type == "component") {
    return SovdEntityType::COMPONENT;
  }
  if (type == "area") {
    return SovdEntityType::AREA;
  }
  if (type == "function") {
    return SovdEntityType::FUNCTION;
  }
  return SovdEntityType::UNKNOWN;
}

}  // namespace

std::string SovdServiceInterface::name() const {
  return "sovd_service_interface";
}

void SovdServiceInterface::configure(const nlohmann::json & config) {
  if (config.contains("service_prefix") && config["service_prefix"].is_string()) {
    service_prefix_ = config["service_prefix"].get<std::string>();
  }
  log_info("Configured with service prefix: " + service_prefix_);
}

void SovdServiceInterface::set_context(PluginContext & context) {
  context_ = &context;

  auto * node = context.node();
  if (!node) {
    log_error("No ROS 2 node available - cannot create service servers");
    return;
  }

  list_entities_srv_ = node->create_service<ros2_medkit_msgs::srv::ListEntities>(
      service_prefix_ + "/list_entities",
      [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListEntities::Request> req,
             std::shared_ptr<ros2_medkit_msgs::srv::ListEntities::Response> res) {
        handle_list_entities(req, res);
      });

  list_faults_srv_ = node->create_service<ros2_medkit_msgs::srv::ListFaultsForEntity>(
      service_prefix_ + "/list_entity_faults",
      [this](const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> req,
             std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> res) {
        handle_list_entity_faults(req, res);
      });

  get_data_srv_ = node->create_service<ros2_medkit_msgs::srv::GetEntityData>(
      service_prefix_ + "/get_entity_data",
      [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetEntityData::Request> req,
             std::shared_ptr<ros2_medkit_msgs::srv::GetEntityData::Response> res) {
        handle_get_entity_data(req, res);
      });

  get_capabilities_srv_ = node->create_service<ros2_medkit_msgs::srv::GetCapabilities>(
      service_prefix_ + "/get_capabilities",
      [this](const std::shared_ptr<ros2_medkit_msgs::srv::GetCapabilities::Request> req,
             std::shared_ptr<ros2_medkit_msgs::srv::GetCapabilities::Response> res) {
        handle_get_capabilities(req, res);
      });

  log_info("Service servers created: list_entities, list_entity_faults, get_entity_data, get_capabilities");
}

void SovdServiceInterface::shutdown() {
  list_entities_srv_.reset();
  list_faults_srv_.reset();
  get_data_srv_.reset();
  get_capabilities_srv_.reset();
  log_info("Service servers shut down");
}

void SovdServiceInterface::handle_list_entities(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListEntities::Request> request,
    std::shared_ptr<ros2_medkit_msgs::srv::ListEntities::Response> response) {
  try {
    auto snapshot = context_->get_entity_snapshot();
    const auto & type_filter = request->entity_type;
    const auto & parent_filter = request->parent_id;

    auto add_entity = [&](const std::string & id, const std::string & entity_name, const std::string & entity_type,
                          const std::string & parent_id, const std::string & fqn) {
      if (!type_filter.empty() && entity_type != type_filter) {
        return;
      }
      if (!parent_filter.empty() && parent_id != parent_filter) {
        return;
      }

      ros2_medkit_msgs::msg::EntityInfo info;
      info.id = id;
      info.name = entity_name;
      info.entity_type = entity_type;
      info.parent_id = parent_id;
      info.fqn = fqn;
      info.capabilities = context_->get_entity_capabilities(id);
      if (info.capabilities.empty()) {
        info.capabilities = context_->get_type_capabilities(entity_type_from_string(entity_type));
      }
      response->entities.push_back(std::move(info));
    };

    for (const auto & area : snapshot.areas) {
      add_entity(area.id, area.name, "area", area.parent_area_id, area.namespace_path);
    }
    for (const auto & comp : snapshot.components) {
      add_entity(comp.id, comp.name, "component", comp.area, comp.fqn);
    }
    for (const auto & app : snapshot.apps) {
      add_entity(app.id, app.name, "app", app.component_id, app.effective_fqn());
    }
    for (const auto & func : snapshot.functions) {
      // Functions are namespace groupings - they don't have a ROS 2 FQN
      add_entity(func.id, func.name, "function", "", "");
    }

    response->success = true;
  } catch (const std::exception & e) {
    response->success = false;
    response->error_message = std::string("Internal error: ") + e.what();
    log_error("handle_list_entities failed: " + std::string(e.what()));
  }
}

void SovdServiceInterface::handle_list_entity_faults(
    const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> request,
    std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> response) {
  try {
    auto entity = context_->get_entity(request->entity_id);
    if (!entity) {
      response->success = false;
      response->error_message = "Entity not found: " + request->entity_id;
      return;
    }

    auto faults_json = context_->list_entity_faults(request->entity_id);

    // Convert JSON faults to Fault messages using value() for type safety
    if (faults_json.is_array()) {
      for (const auto & fault_json : faults_json) {
        if (!fault_json.is_object()) {
          continue;
        }
        ros2_medkit_msgs::msg::Fault fault;
        fault.fault_code = fault_json.value("fault_code", std::string{});
        fault.severity = fault_json.value("severity", static_cast<uint8_t>(0));
        fault.description = fault_json.value("description", std::string{});
        fault.status = fault_json.value("status", std::string{});
        fault.occurrence_count = fault_json.value("occurrence_count", static_cast<uint32_t>(0));
        if (fault_json.contains("first_occurred") && fault_json["first_occurred"].is_number()) {
          double ts = fault_json["first_occurred"].get<double>();
          fault.first_occurred.sec = static_cast<int32_t>(ts);
          fault.first_occurred.nanosec = static_cast<uint32_t>((ts - static_cast<int32_t>(ts)) * 1e9);
        }
        if (fault_json.contains("last_occurred") && fault_json["last_occurred"].is_number()) {
          double ts = fault_json["last_occurred"].get<double>();
          fault.last_occurred.sec = static_cast<int32_t>(ts);
          fault.last_occurred.nanosec = static_cast<uint32_t>((ts - static_cast<int32_t>(ts)) * 1e9);
        }
        if (fault_json.contains("reporting_sources") && fault_json["reporting_sources"].is_array()) {
          for (const auto & src : fault_json["reporting_sources"]) {
            if (src.is_string()) {
              fault.reporting_sources.push_back(src.get<std::string>());
            }
          }
        }
        response->faults.push_back(std::move(fault));
      }
    }

    response->success = true;
  } catch (const std::exception & e) {
    response->success = false;
    response->error_message = std::string("Internal error: ") + e.what();
    log_error("handle_list_entity_faults failed: " + std::string(e.what()));
  }
}

void SovdServiceInterface::handle_get_entity_data(
    const std::shared_ptr<ros2_medkit_msgs::srv::GetEntityData::Request> request,
    std::shared_ptr<ros2_medkit_msgs::srv::GetEntityData::Response> response) {
  try {
    auto entity = context_->get_entity(request->entity_id);
    if (!entity) {
      response->success = false;
      response->error_message = "Entity not found: " + request->entity_id;
      return;
    }

    // TODO: Implement data retrieval when PluginContext gains data access.
    // Currently PluginContext exposes entity metadata and faults but not live
    // topic data. Use HTTP REST API (/api/v1/apps/{id}/data) as alternative.
    response->data_json = "{}";
    response->success = false;
    response->error_message = "GetEntityData not yet implemented - use HTTP REST API for topic data";
  } catch (const std::exception & e) {
    response->success = false;
    response->error_message = std::string("Internal error: ") + e.what();
    log_error("handle_get_entity_data failed: " + std::string(e.what()));
  }
}

void SovdServiceInterface::handle_get_capabilities(
    const std::shared_ptr<ros2_medkit_msgs::srv::GetCapabilities::Request> request,
    std::shared_ptr<ros2_medkit_msgs::srv::GetCapabilities::Response> response) {
  try {
    if (request->entity_id.empty()) {
      // Server-level capabilities
      response->capabilities = {"apps", "components", "areas", "functions", "faults", "health"};
      response->resource_types = {"apps", "components", "areas", "functions"};
      response->success = true;
      return;
    }

    auto entity = context_->get_entity(request->entity_id);
    if (!entity) {
      response->success = false;
      response->error_message = "Entity not found: " + request->entity_id;
      return;
    }

    // Get entity-specific capabilities first, fall back to type-level
    auto caps = context_->get_entity_capabilities(request->entity_id);
    if (caps.empty()) {
      caps = context_->get_type_capabilities(entity->type);
    }
    response->capabilities = std::move(caps);
    response->success = true;
  } catch (const std::exception & e) {
    response->success = false;
    response->error_message = std::string("Internal error: ") + e.what();
    log_error("handle_get_capabilities failed: " + std::string(e.what()));
  }
}

}  // namespace ros2_medkit_gateway
