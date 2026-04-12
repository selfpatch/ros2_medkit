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

#include "ros2_medkit_opcua/opcua_plugin.hpp"

#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>
#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include <ros2_medkit_gateway/http/error_codes.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>

namespace ros2_medkit_gateway {

namespace {
bool is_valid_path_segment(const std::string & s) {
  if (s.empty() || s.size() > 256) {
    return false;
  }
  return std::all_of(s.begin(), s.end(), [](unsigned char c) {
    return std::isalnum(c) || c == '_' || c == '-';
  });
}
}  // namespace

struct OpcuaPlugin::FaultClients {
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report;
  rclcpp::Client<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear;
};

OpcuaPlugin::OpcuaPlugin()
  : client_(std::make_unique<OpcuaClient>()), fault_clients_(std::make_unique<FaultClients>()) {
}

OpcuaPlugin::~OpcuaPlugin() {
  shutdown();
}

void OpcuaPlugin::configure(const nlohmann::json & config) {
  if (config.contains("endpoint_url")) {
    client_config_.endpoint_url = config["endpoint_url"].get<std::string>();
  }
  // NOTE: OPC-UA security (username/password, certificates) not yet implemented.
  // Connect uses Anonymous auth with SecurityPolicy=None.

  if (config.contains("node_map_path")) {
    node_map_path_ = config["node_map_path"].get<std::string>();
  }

  if (config.contains("prefer_subscriptions")) {
    poller_config_.prefer_subscriptions = config["prefer_subscriptions"].get<bool>();
  }
  if (config.contains("subscription_interval_ms")) {
    poller_config_.subscription_interval_ms = config["subscription_interval_ms"].get<double>();
  }
  if (config.contains("poll_interval_ms")) {
    auto ms = std::clamp(config["poll_interval_ms"].get<int>(), 100, 60000);
    poller_config_.poll_interval = std::chrono::milliseconds(ms);
  }

  // Environment variables override YAML config (for Docker)
  if (auto * env = std::getenv("OPCUA_ENDPOINT_URL")) {
    client_config_.endpoint_url = env;
  }
  if (auto * env = std::getenv("OPCUA_NODE_MAP_PATH")) {
    node_map_path_ = env;
  }

  if (!node_map_path_.empty()) {
    if (!node_map_.load(node_map_path_)) {
      log_error("Failed to load node map from: " + node_map_path_);
    } else {
      log_info("Loaded node map with " + std::to_string(node_map_.entries().size()) +
               " entries from: " + node_map_path_);
    }
  }
}

void OpcuaPlugin::set_context(PluginContext & context) {
  ctx_ = &context;

  // NOTE: capabilities (x-plc-data, x-plc-operations, x-plc-status) are
  // registered per entity in introspect() rather than type-level here, so
  // that only PLC-backed entities advertise them. Type-level registration
  // would leak the capabilities onto every ROS 2 app/component in the SOVD
  // entity tree, misleading clients into calling endpoints that return 404.

  auto * node = ctx_->node();
  if (node) {
    fault_clients_->report = node->create_client<ros2_medkit_msgs::srv::ReportFault>("/fault_manager/report_fault");
    fault_clients_->clear = node->create_client<ros2_medkit_msgs::srv::ClearFault>("/fault_manager/clear_fault");

    // Create ROS 2 publishers for numeric PLC values only (skip string-typed entries)
    for (const auto & entry : node_map_.entries()) {
      if (entry.data_type == "string") {
        log_info("PLC bridge: skipping non-numeric " + entry.node_id_str + " (data_type=" + entry.data_type + ")");
        continue;
      }
      publishers_[entry.node_id_str] =
          node->create_publisher<std_msgs::msg::Float32>(entry.ros2_topic, rclcpp::SensorDataQoS());
      log_info("PLC bridge: " + entry.node_id_str + " -> " + entry.ros2_topic + " (Float32)");
    }
  }

  log_warn(
      "OPC-UA security: Anonymous auth, SecurityPolicy=None. "
      "Not suitable for untrusted networks.");

  if (client_->connect(client_config_)) {
    log_info("Connected to OPC-UA server: " + client_config_.endpoint_url);
  } else {
    log_warn("Failed to connect to OPC-UA server: " + client_config_.endpoint_url);
  }

  poller_ = std::make_unique<OpcuaPoller>(*client_, node_map_);
  poller_->set_alarm_callback([this](const std::string & code, const AlarmConfig & cfg, bool active) {
    on_alarm_change(code, cfg, active);
  });
  poller_->set_poll_callback([this](const PollSnapshot & snap) {
    publish_values(snap);
  });
  poller_->start(poller_config_);
  log_info("OPC-UA poller started (mode: " + std::string(poller_->using_subscriptions() ? "subscription" : "poll") +
           ")");
}

std::vector<GatewayPlugin::PluginRoute> OpcuaPlugin::get_routes() {
  return {
      {"GET", R"(apps/([^/]+)/x-plc-data)",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_data(req, res);
       }},
      {"GET", R"(apps/([^/]+)/x-plc-data/([^/]+))",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_data_single(req, res);
       }},
      {"POST", R"(apps/([^/]+)/x-plc-operations/([^/]+))",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_operations(req, res);
       }},
      {"GET", R"(components/([^/]+)/x-plc-status)",
       [this](const PluginRequest & req, PluginResponse & res) {
         handle_plc_status(req, res);
       }},
  };
}

void OpcuaPlugin::shutdown() {
  if (shutdown_requested_.exchange(true)) {
    return;
  }
  log_info("Shutting down OPC-UA plugin...");
  if (poller_) {
    poller_->stop();
  }
  client_->disconnect();
  log_info("OPC-UA plugin shutdown complete");
}

// -- IntrospectionProvider --

IntrospectionResult OpcuaPlugin::introspect(const IntrospectionInput & /*input*/) {
  log_info("introspect() called - generating PLC entities");
  IntrospectionResult result;

  Area area;
  area.id = node_map_.area_id();
  area.name = node_map_.area_name();
  area.namespace_path = "/" + node_map_.area_id();
  area.source = "plugin";
  area.description = "PLC systems connected via OPC-UA";
  result.new_entities.areas.push_back(std::move(area));

  Component comp;
  comp.id = node_map_.component_id();
  comp.name = node_map_.component_name();
  comp.namespace_path = "/" + node_map_.area_id();
  comp.fqn = "/" + node_map_.area_id() + "/" + node_map_.component_id();
  comp.area = node_map_.area_id();
  comp.source = "plugin";
  comp.description = "PLC runtime connected at " + client_config_.endpoint_url;
  result.new_entities.components.push_back(std::move(comp));

  // Register x-plc-status on the PLC runtime component only, so non-PLC
  // components in the shared SOVD entity tree do not advertise a
  // capability they cannot serve.
  if (ctx_) {
    ctx_->register_entity_capability(node_map_.component_id(), "x-plc-status");
  }

  for (const auto & def : node_map_.entity_defs()) {
    App app;
    app.id = def.id;
    app.name = def.name;
    app.component_id = def.component_id;
    app.external = true;
    app.is_online = client_->is_connected();
    app.source = "plugin";
    app.description = "PLC application with " + std::to_string(def.data_names.size()) + " data points";
    result.new_entities.apps.push_back(std::move(app));

    // Register capabilities per entity (never type-level): only PLC-backed
    // apps advertise x-plc-data; apps with writable nodes also get
    // x-plc-operations.
    if (ctx_) {
      ctx_->register_entity_capability(def.id, "x-plc-data");
      if (!def.writable_names.empty()) {
        ctx_->register_entity_capability(def.id, "x-plc-operations");
      }
    }
  }

  return result;
}

// -- Route handlers --

void OpcuaPlugin::handle_plc_data(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto entity_id = req.path_param(1);
  auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto entries = node_map_.entries_for_entity(entity_id);
  if (entries.empty()) {
    res.send_error(404, ERR_RESOURCE_NOT_FOUND, "No PLC data mapped for entity: " + entity_id);
    return;
  }

  res.send_json(build_data_response(entity_id));
}

void OpcuaPlugin::handle_plc_data_single(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto entity_id = req.path_param(1);
  auto data_name = req.path_param(2);

  if (!is_valid_path_segment(data_name)) {
    res.send_error(400, ERR_INVALID_PARAMETER, "Invalid data name");
    return;
  }

  auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto * entry = node_map_.find_by_data_name(entity_id, data_name);
  if (!entry) {
    res.send_error(404, ERR_RESOURCE_NOT_FOUND, "Data point not found: " + data_name + " in entity: " + entity_id);
    return;
  }

  auto snap = poller_->snapshot();
  auto it = snap.values.find(entry->node_id_str);

  nlohmann::json j;
  j["name"] = entry->data_name;
  j["display_name"] = entry->display_name;
  j["node_id"] = entry->node_id_str;

  if (it != snap.values.end()) {
    std::visit(
        [&j](auto && v) {
          j["value"] = v;
        },
        it->second);
  } else {
    j["value"] = nullptr;
  }

  if (!entry->unit.empty()) {
    j["unit"] = entry->unit;
  }
  j["data_type"] = entry->data_type;
  j["writable"] = entry->writable;

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  j["timestamp"] = ts;

  res.send_json(j);
}

void OpcuaPlugin::handle_plc_operations(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto entity_id = req.path_param(1);
  auto op_name = req.path_param(2);

  if (!is_valid_path_segment(op_name)) {
    res.send_error(400, ERR_INVALID_PARAMETER, "Invalid operation name");
    return;
  }

  auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  std::string data_name;
  if (op_name.substr(0, 4) == "set_") {
    data_name = op_name.substr(4);
  } else {
    data_name = op_name;
  }

  auto * entry = node_map_.find_by_data_name(entity_id, data_name);
  if (!entry) {
    res.send_error(404, ERR_RESOURCE_NOT_FOUND, "Operation not found: " + op_name + " in entity: " + entity_id);
    return;
  }

  if (!entry->writable) {
    res.send_error(400, ERR_INVALID_REQUEST, "Data point is read-only: " + data_name);
    return;
  }

  nlohmann::json body;
  try {
    body = nlohmann::json::parse(req.body());
  } catch (const nlohmann::json::parse_error &) {
    res.send_error(400, ERR_INVALID_REQUEST, "Invalid JSON body");
    return;
  }

  if (!body.contains("value")) {
    res.send_error(400, ERR_INVALID_REQUEST, "Missing 'value' field");
    return;
  }

  OpcuaValue write_val;
  try {
    if (entry->data_type == "bool") {
      write_val = body["value"].get<bool>();
    } else if (entry->data_type == "int") {
      write_val = body["value"].get<int32_t>();
    } else if (entry->data_type == "string") {
      write_val = body["value"].get<std::string>();
    } else {
      write_val = body["value"].get<double>();
    }
  } catch (const nlohmann::json::type_error &) {
    res.send_error(400, ERR_INVALID_REQUEST, "Value type mismatch for data_type: " + entry->data_type);
    return;
  }

  // Range validation for safety
  if (entry->has_range()) {
    double v = std::visit(
        [](auto && x) -> double {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_arithmetic_v<T>) {
            return static_cast<double>(x);
          }
          return 0.0;
        },
        write_val);
    if (v < *entry->min_value || v > *entry->max_value) {
      res.send_error(400, ERR_INVALID_REQUEST,
                     "Value " + std::to_string(v) + " out of range [" + std::to_string(*entry->min_value) + ", " +
                         std::to_string(*entry->max_value) + "]");
      return;
    }
  }

  auto write_result = client_->write_value(entry->node_id, write_val, entry->data_type);
  if (!write_result) {
    int status = (write_result.error().code == OpcuaClient::WriteError::NotConnected ||
                  write_result.error().code == OpcuaClient::WriteError::TransportError)
                     ? 502
                     : 400;
    res.send_error(status, ERR_SERVICE_UNAVAILABLE, write_result.error().message);
    return;
  }

  nlohmann::json response;
  response["status"] = "ok";
  response["operation"] = op_name;
  response["node_id"] = entry->node_id_str;
  std::visit(
      [&response](auto && v) {
        response["value_written"] = v;
      },
      write_val);

  res.send_json(response);
}

void OpcuaPlugin::handle_plc_status(const PluginRequest & req, PluginResponse & res) {
  if (!ctx_ || !poller_ || shutdown_requested_.load()) {
    res.send_error(503, ERR_SERVICE_UNAVAILABLE, "OPC-UA plugin not initialized");
    return;
  }
  auto component_id = req.path_param(1);
  auto entity = ctx_->validate_entity_for_route(req, res, component_id);
  if (!entity) {
    return;
  }

  auto snap = poller_->snapshot();

  nlohmann::json j;
  j["component_id"] = component_id;
  j["connected"] = snap.connected;
  j["endpoint_url"] = client_->endpoint_url();
  j["server_description"] = client_->server_description();
  j["mode"] = poller_->using_subscriptions() ? "subscription" : "poll";
  j["poll_count"] = snap.poll_count;
  j["error_count"] = snap.error_count;
  j["node_count"] = node_map_.entries().size();
  j["entity_count"] = node_map_.entity_defs().size();

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  j["last_update"] = ts;

  nlohmann::json alarms = nlohmann::json::array();
  for (const auto & [code, active] : snap.alarms) {
    if (active) {
      alarms.push_back(code);
    }
  }
  j["active_alarms"] = alarms;

  res.send_json(j);
}

// -- Fault bridge --

void OpcuaPlugin::on_alarm_change(const std::string & fault_code, const AlarmConfig & config, bool active) {
  if (shutdown_requested_.load()) {
    return;
  }
  std::string entity_id;
  for (const auto & entry : node_map_.entries()) {
    if (entry.alarm.has_value() && entry.alarm->fault_code == fault_code) {
      entity_id = entry.entity_id;
      break;
    }
  }

  if (entity_id.empty()) {
    log_warn("Alarm " + fault_code + " has no entity mapping");
    return;
  }

  if (active) {
    log_info("Alarm activated: " + fault_code + " on " + entity_id);
    send_report_fault(entity_id, fault_code, config.severity, config.message);
  } else {
    log_info("Alarm cleared: " + fault_code + " on " + entity_id);
    send_clear_fault(fault_code);
  }
}

void OpcuaPlugin::send_report_fault(const std::string & entity_id, const std::string & fault_code,
                                    const std::string & severity_str, const std::string & message) {
  if (!fault_clients_->report) {
    log_warn("ReportFault service client not available");
    return;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
  request->fault_code = fault_code;
  request->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
  request->description = message;
  request->source_id = entity_id;

  // Map severity string to uint8 (case-insensitive)
  std::string sev_lower = severity_str;
  std::transform(sev_lower.begin(), sev_lower.end(), sev_lower.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  if (sev_lower == "critical") {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;
  } else if (sev_lower == "error") {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
  } else if (sev_lower == "warning") {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;
  } else {
    request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_INFO;
  }

  fault_clients_->report->async_send_request(request);
}

void OpcuaPlugin::send_clear_fault(const std::string & fault_code) {
  if (!fault_clients_->clear) {
    log_warn("ClearFault service client not available");
    return;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ClearFault::Request>();
  request->fault_code = fault_code;

  fault_clients_->clear->async_send_request(request);
}

void OpcuaPlugin::publish_values(const PollSnapshot & snap) {
  if (shutdown_requested_.load()) {
    return;
  }
  for (const auto & [node_id, value] : snap.values) {
    auto pub_it = publishers_.find(node_id);
    if (pub_it == publishers_.end()) {
      continue;
    }

    std_msgs::msg::Float32 msg;
    bool is_numeric = std::visit(
        [&msg](auto && v) -> bool {
          using T = std::decay_t<decltype(v)>;
          if constexpr (std::is_arithmetic_v<T>) {
            msg.data = static_cast<float>(v);
            return true;
          }
          return false;
        },
        value);

    if (!is_numeric) {
      if (warned_non_numeric_.insert(node_id).second) {
        log_warn("Skipping non-numeric value for ROS 2 bridging: " + node_id);
      }
      continue;
    }

    if (!std::isfinite(msg.data)) {
      continue;
    }

    pub_it->second->publish(msg);
  }
}

nlohmann::json OpcuaPlugin::build_data_response(const std::string & entity_id) const {
  auto entries = node_map_.entries_for_entity(entity_id);
  auto snap = poller_->snapshot();

  nlohmann::json items = nlohmann::json::array();

  for (const auto * entry : entries) {
    nlohmann::json item;
    item["name"] = entry->data_name;
    item["display_name"] = entry->display_name;
    item["node_id"] = entry->node_id_str;

    auto it = snap.values.find(entry->node_id_str);
    if (it != snap.values.end()) {
      std::visit(
          [&item](auto && v) {
            item["value"] = v;
          },
          it->second);
    } else {
      item["value"] = nullptr;
    }

    if (!entry->unit.empty()) {
      item["unit"] = entry->unit;
    }
    item["data_type"] = entry->data_type;
    item["writable"] = entry->writable;

    items.push_back(std::move(item));
  }

  nlohmann::json response;
  response["entity_id"] = entity_id;
  response["items"] = items;
  response["connected"] = snap.connected;

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  response["timestamp"] = ts;

  return response;
}

// -- DataProvider interface --

tl::expected<nlohmann::json, DataProviderErrorInfo> OpcuaPlugin::list_data(const std::string & entity_id) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "plugin not initialized", 503});
  }

  auto entries = node_map_.entries_for_entity(entity_id);
  if (entries.empty()) {
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::EntityNotFound, "No PLC data for entity: " + entity_id, 404});
  }

  nlohmann::json items = nlohmann::json::array();
  auto snap = poller_->snapshot();

  for (const auto * entry : entries) {
    nlohmann::json item;
    item["id"] = entry->data_name;
    item["name"] = entry->display_name;
    item["category"] = "currentData";

    auto it = snap.values.find(entry->node_id_str);
    if (it != snap.values.end()) {
      std::visit(
          [&item](auto && v) {
            item["value"] = v;
          },
          it->second);
    } else {
      item["value"] = nullptr;
    }

    if (!entry->unit.empty()) {
      item["unit"] = entry->unit;
    }
    item["data_type"] = entry->data_type;
    item["writable"] = entry->writable;
    items.push_back(std::move(item));
  }

  return tl::expected<nlohmann::json, DataProviderErrorInfo>{nlohmann::json{{"items", items}}};
}

tl::expected<nlohmann::json, DataProviderErrorInfo> OpcuaPlugin::read_data(const std::string & entity_id,
                                                                           const std::string & resource_name) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "plugin not initialized", 503});
  }

  auto * entry = node_map_.find_by_data_name(entity_id, resource_name);
  if (!entry) {
    return tl::make_unexpected(DataProviderErrorInfo{
        DataProviderError::ResourceNotFound, "Data point not found: " + resource_name + " in " + entity_id, 404});
  }

  auto snap = poller_->snapshot();
  auto it = snap.values.find(entry->node_id_str);

  nlohmann::json result;
  result["id"] = entry->data_name;
  result["name"] = entry->display_name;
  result["node_id"] = entry->node_id_str;

  if (it != snap.values.end()) {
    std::visit(
        [&result](auto && v) {
          result["value"] = v;
        },
        it->second);
  } else {
    result["value"] = nullptr;
  }

  if (!entry->unit.empty()) {
    result["unit"] = entry->unit;
  }
  result["data_type"] = entry->data_type;
  result["writable"] = entry->writable;

  auto ts = std::chrono::system_clock::to_time_t(snap.timestamp);
  result["timestamp"] = ts;

  return tl::expected<nlohmann::json, DataProviderErrorInfo>{result};
}

tl::expected<nlohmann::json, DataProviderErrorInfo> OpcuaPlugin::write_data(const std::string & entity_id,
                                                                            const std::string & resource_name,
                                                                            const nlohmann::json & value) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "plugin not initialized", 503});
  }

  auto * entry = node_map_.find_by_data_name(entity_id, resource_name);
  if (!entry) {
    return tl::make_unexpected(DataProviderErrorInfo{
        DataProviderError::ResourceNotFound, "Data point not found: " + resource_name + " in " + entity_id, 404});
  }
  if (!entry->writable) {
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::ReadOnly, "Data point is read-only: " + resource_name, 400});
  }

  if (!value.contains("value")) {
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::InvalidValue, "Missing 'value' field in write body", 400});
  }

  OpcuaValue write_val;
  try {
    if (entry->data_type == "bool") {
      write_val = value["value"].get<bool>();
    } else if (entry->data_type == "int") {
      write_val = value["value"].get<int32_t>();
    } else if (entry->data_type == "string") {
      write_val = value["value"].get<std::string>();
    } else {
      write_val = value["value"].get<double>();
    }
  } catch (const nlohmann::json::type_error &) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::InvalidValue,
                                                     "Value type mismatch for data_type: " + entry->data_type, 400});
  }

  if (entry->has_range()) {
    double v = std::visit(
        [](auto && x) -> double {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_arithmetic_v<T>) {
            return static_cast<double>(x);
          }
          return 0.0;
        },
        write_val);
    if (v < *entry->min_value || v > *entry->max_value) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::InvalidValue,
                                                       "Value " + std::to_string(v) + " out of range [" +
                                                           std::to_string(*entry->min_value) + ", " +
                                                           std::to_string(*entry->max_value) + "]",
                                                       400});
    }
  }

  auto write_result = client_->write_value(entry->node_id, write_val, entry->data_type);
  if (!write_result) {
    auto wcode = write_result.error().code;
    if (wcode == OpcuaClient::WriteError::TypeMismatch) {
      return tl::make_unexpected(
          DataProviderErrorInfo{DataProviderError::InvalidValue, write_result.error().message, 400});
    }
    if (wcode == OpcuaClient::WriteError::AccessDenied) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, write_result.error().message, 403});
    }
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::TransportError, write_result.error().message, 502});
  }

  nlohmann::json result;
  result["id"] = resource_name;
  result["status"] = "ok";
  std::visit(
      [&result](auto && v) {
        result["value_written"] = v;
      },
      write_val);
  return tl::expected<nlohmann::json, DataProviderErrorInfo>{result};
}

// -- OperationProvider interface --

tl::expected<nlohmann::json, OperationProviderErrorInfo> OpcuaPlugin::list_operations(const std::string & entity_id) {
  nlohmann::json items = nlohmann::json::array();

  for (const auto & def : node_map_.entity_defs()) {
    if (def.id != entity_id) {
      continue;
    }
    for (const auto & writable_name : def.writable_names) {
      auto * entry = node_map_.find_by_data_name(entity_id, writable_name);
      nlohmann::json item;
      item["id"] = "set_" + writable_name;
      item["name"] = entry ? ("Set " + entry->display_name) : ("Set " + writable_name);
      item["proximity_proof_required"] = false;
      item["asynchronous_execution"] = false;
      items.push_back(std::move(item));
    }
    return tl::expected<nlohmann::json, OperationProviderErrorInfo>{nlohmann::json{{"items", items}}};
  }

  return tl::make_unexpected(
      OperationProviderErrorInfo{OperationProviderError::EntityNotFound, "Entity not found: " + entity_id, 404});
}

tl::expected<nlohmann::json, OperationProviderErrorInfo>
OpcuaPlugin::execute_operation(const std::string & entity_id, const std::string & operation_name,
                               const nlohmann::json & parameters) {
  if (!ctx_ || !poller_) {
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::Internal, "plugin not initialized", 503});
  }

  std::string data_name;
  if (operation_name.substr(0, 4) == "set_") {
    data_name = operation_name.substr(4);
  } else {
    data_name = operation_name;
  }

  auto * entry = node_map_.find_by_data_name(entity_id, data_name);
  if (!entry) {
    return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::OperationNotFound,
                                                          "Operation not found: " + operation_name, 404});
  }
  if (!entry->writable) {
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::Rejected, "Data point is read-only: " + data_name, 400});
  }
  if (!parameters.contains("value")) {
    return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                                          "Missing 'value' field in parameters", 400});
  }

  OpcuaValue write_val;
  try {
    if (entry->data_type == "bool") {
      write_val = parameters["value"].get<bool>();
    } else if (entry->data_type == "int") {
      write_val = parameters["value"].get<int32_t>();
    } else if (entry->data_type == "string") {
      write_val = parameters["value"].get<std::string>();
    } else {
      write_val = parameters["value"].get<double>();
    }
  } catch (const nlohmann::json::type_error &) {
    return tl::make_unexpected(OperationProviderErrorInfo{
        OperationProviderError::InvalidParameters, "Value type mismatch for data_type: " + entry->data_type, 400});
  }

  if (entry->has_range()) {
    double v = std::visit(
        [](auto && x) -> double {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_arithmetic_v<T>) {
            return static_cast<double>(x);
          }
          return 0.0;
        },
        write_val);
    if (v < *entry->min_value || v > *entry->max_value) {
      return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                                            "Value " + std::to_string(v) + " out of range [" +
                                                                std::to_string(*entry->min_value) + ", " +
                                                                std::to_string(*entry->max_value) + "]",
                                                            400});
    }
  }

  auto write_result = client_->write_value(entry->node_id, write_val, entry->data_type);
  if (!write_result) {
    auto wcode = write_result.error().code;
    if (wcode == OpcuaClient::WriteError::TypeMismatch) {
      return tl::make_unexpected(
          OperationProviderErrorInfo{OperationProviderError::InvalidParameters, write_result.error().message, 400});
    }
    if (wcode == OpcuaClient::WriteError::AccessDenied) {
      return tl::make_unexpected(
          OperationProviderErrorInfo{OperationProviderError::Rejected, write_result.error().message, 403});
    }
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::TransportError, write_result.error().message, 502});
  }

  nlohmann::json result;
  result["status"] = "ok";
  result["operation"] = operation_name;
  result["node_id"] = entry->node_id_str;
  std::visit(
      [&result](auto && v) {
        result["value_written"] = v;
      },
      write_val);
  return tl::expected<nlohmann::json, OperationProviderErrorInfo>{result};
}

// -- FaultProvider interface --

tl::expected<nlohmann::json, FaultProviderErrorInfo> OpcuaPlugin::list_faults(const std::string & entity_id) {
  if (!ctx_) {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::Internal, "plugin not initialized", 503});
  }

  auto faults = ctx_->list_entity_faults(entity_id);
  if (faults.is_null() || faults.empty()) {
    return tl::expected<nlohmann::json, FaultProviderErrorInfo>{nlohmann::json{{"items", nlohmann::json::array()}}};
  }

  if (faults.contains("faults") && faults["faults"].is_array()) {
    nlohmann::json items = nlohmann::json::array();
    for (const auto & f : faults["faults"]) {
      nlohmann::json item;
      item["code"] = f.value("fault_code", "");
      item["severity"] = f.value("severity", 0);
      item["description"] = f.value("description", "");
      item["status"] = f.value("status", "");
      item["source_id"] = f.value("source_id", "");
      items.push_back(std::move(item));
    }
    return tl::expected<nlohmann::json, FaultProviderErrorInfo>{nlohmann::json{{"items", items}}};
  }

  return tl::expected<nlohmann::json, FaultProviderErrorInfo>{nlohmann::json{{"items", nlohmann::json::array()}}};
}

tl::expected<nlohmann::json, FaultProviderErrorInfo> OpcuaPlugin::get_fault(const std::string & entity_id,
                                                                            const std::string & fault_code) {
  if (!ctx_) {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::Internal, "plugin not initialized", 503});
  }

  auto faults = ctx_->list_entity_faults(entity_id);
  if (faults.contains("faults") && faults["faults"].is_array()) {
    for (const auto & f : faults["faults"]) {
      if (f.value("fault_code", "") == fault_code) {
        return tl::expected<nlohmann::json, FaultProviderErrorInfo>{f};
      }
    }
  }

  return tl::make_unexpected(
      FaultProviderErrorInfo{FaultProviderError::FaultNotFound, "Fault not found: " + fault_code, 404});
}

tl::expected<nlohmann::json, FaultProviderErrorInfo> OpcuaPlugin::clear_fault(const std::string & entity_id,
                                                                              const std::string & fault_code) {
  if (!ctx_ || !fault_clients_) {
    return tl::make_unexpected(FaultProviderErrorInfo{FaultProviderError::Internal, "plugin not initialized", 503});
  }

  send_clear_fault(fault_code);
  return tl::expected<nlohmann::json, FaultProviderErrorInfo>{
      nlohmann::json{{"status", "cleared"}, {"fault_code", fault_code}, {"entity_id", entity_id}}};
}

}  // namespace ros2_medkit_gateway
