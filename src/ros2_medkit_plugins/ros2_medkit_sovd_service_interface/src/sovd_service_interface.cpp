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
        info.capabilities = context_->get_type_capabilities(entity_type == "app"         ? SovdEntityType::APP
                                                            : entity_type == "component" ? SovdEntityType::COMPONENT
                                                            : entity_type == "area"      ? SovdEntityType::AREA
                                                            : entity_type == "function"  ? SovdEntityType::FUNCTION
                                                                                         : SovdEntityType::UNKNOWN);
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
      add_entity(func.id, func.name, "function", "", func.id);
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

    // Convert JSON faults to Fault messages
    if (faults_json.is_array()) {
      for (const auto & fault_json : faults_json) {
        ros2_medkit_msgs::msg::Fault fault;
        if (fault_json.contains("fault_code")) {
          fault.fault_code = fault_json["fault_code"].get<std::string>();
        }
        if (fault_json.contains("severity")) {
          fault.severity = fault_json["severity"].get<uint8_t>();
        }
        if (fault_json.contains("description")) {
          fault.description = fault_json["description"].get<std::string>();
        }
        if (fault_json.contains("status")) {
          fault.status = fault_json["status"].get<std::string>();
        }
        if (fault_json.contains("occurrence_count")) {
          fault.occurrence_count = fault_json["occurrence_count"].get<uint32_t>();
        }
        if (fault_json.contains("reporting_sources") && fault_json["reporting_sources"].is_array()) {
          for (const auto & src : fault_json["reporting_sources"]) {
            fault.reporting_sources.push_back(src.get<std::string>());
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

    // Use the registered data sampler if available
    nlohmann::json data = nlohmann::json::object();

    // Try to get data via the sampler registered for "data" collection
    // The data sampler is registered by the gateway's data handler
    // For now, return an empty JSON object - the agent can fall back to
    // other data sources if needed
    response->data_json = data.dump();
    response->success = true;
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
