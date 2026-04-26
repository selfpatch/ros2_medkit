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
#include <rcutils/logging.h>
#include <ros2_medkit_msgs/msg/fault.hpp>
#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include <ros2_medkit_gateway/http/error_codes.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace ros2_medkit_gateway {

namespace {

// Named logger so per-operation traces respect ROS log level filtering
// (bburda review on PR #387). Quiet at INFO; ``--log-level
// opcua.plugin:=debug`` re-enables for diagnostics.
inline rclcpp::Logger opcua_plugin_logger() {
  static auto logger = rclcpp::get_logger("opcua.plugin");
  return logger;
}

inline bool plugin_debug_enabled() {
  // rcutils API for Humble compatibility; Jazzy+ adds rclcpp::Logger::
  // get_effective_level but Humble does not.
  return rcutils_logging_logger_is_enabled_for("opcua.plugin", RCUTILS_LOG_SEVERITY_DEBUG);
}

/// Parse a JSON "value" field, coerce to the node's declared data_type, and
/// validate against the optional min/max range. Shared by handle_plc_operations,
/// DataProvider::write_data, and OperationProvider::execute_operation to keep
/// the three write paths in sync.
tl::expected<OpcuaValue, std::string> parse_coerce_validate(const nlohmann::json & json_value,
                                                            const NodeMapEntry & entry) {
  OpcuaValue val;
  try {
    if (entry.data_type == "bool") {
      val = json_value.get<bool>();
    } else if (entry.data_type == "int") {
      val = json_value.get<int32_t>();
    } else if (entry.data_type == "string") {
      val = json_value.get<std::string>();
    } else {
      val = json_value.get<double>();
    }
  } catch (const nlohmann::json::type_error &) {
    return tl::make_unexpected("Value type mismatch for data_type: " + entry.data_type);
  }

  if (entry.has_range()) {
    double v = std::visit(
        [](auto && x) -> double {
          using T = std::decay_t<decltype(x)>;
          if constexpr (std::is_arithmetic_v<T>) {
            return static_cast<double>(x);
          }
          return 0.0;
        },
        val);
    if (v < *entry.min_value || v > *entry.max_value) {
      return tl::make_unexpected("Value " + std::to_string(v) + " out of range [" + std::to_string(*entry.min_value) +
                                 ", " + std::to_string(*entry.max_value) + "]");
    }
  }

  return val;
}

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
  poller_->set_event_alarm_callback([this](const AlarmEventDelivery & delivery) {
    on_event_alarm(delivery);
  });
  poller_->set_poll_callback([this](const PollSnapshot & snap) {
    publish_values(snap);
  });
  // Plumb operator-visible warnings out of the poll thread into the ROS log.
  // Without this, ``ConditionRefresh rejected`` etc. only land in the
  // container's stderr, invisible to anyone watching /rosout or rclpp logs.
  poller_config_.log_warn = [this](const std::string & msg) {
    log_warn(msg);
  };
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

  auto parsed = parse_coerce_validate(body["value"], *entry);
  if (!parsed) {
    res.send_error(400, ERR_INVALID_REQUEST, parsed.error());
    return;
  }

  auto write_result = client_->write_value(entry->node_id, *parsed, entry->data_type);
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
      *parsed);

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

void OpcuaPlugin::on_event_alarm(const AlarmEventDelivery & delivery) {
  if (shutdown_requested_.load()) {
    return;
  }

  // Map raw OPC-UA Severity (1-1000) to SOVD severity bucket.
  // Selfpatch convention documented in design/index.rst; not from IEC 62682.
  // Severity_override on the AlarmEventConfig wins when set.
  auto severity_str = [&]() {
    auto * cfg = node_map_.find_event_alarm(delivery.entity_id, delivery.fault_code);
    if (cfg && !cfg->severity_override.empty()) {
      return cfg->severity_override;
    }
    if (delivery.severity >= 801) {
      return std::string("CRITICAL");
    }
    if (delivery.severity >= 501) {
      return std::string("ERROR");
    }
    if (delivery.severity >= 201) {
      return std::string("WARNING");
    }
    return std::string("INFO");
  }();

  switch (delivery.action) {
    case AlarmAction::ReportConfirmed:
      log_info("AlarmCondition CONFIRMED: " + delivery.fault_code + " on " + delivery.entity_id +
               " (severity=" + std::to_string(delivery.severity) + ")");
      send_report_fault(delivery.entity_id, delivery.fault_code, severity_str, delivery.message);
      break;
    case AlarmAction::ReportHealed:
      // Intentional no-op (Copilot review on PR #387).
      //
      // OPC-UA AlarmConditionType HEALED means "alarm physically cleared
      // (ActiveState=false) but operator workflow incomplete (ack and/or
      // confirm pending)". Per Part 9 §5.7 the Cleared transition is
      // operator-driven, not statistical.
      //
      // ros2_medkit_msgs/srv/ReportFault has only FAILED/PASSED verbs and
      // fault_manager treats PASSED through a debounce engine. Sending
      // EVENT_PASSED on every latch would let fault_manager auto-clear
      // the fault via healing_threshold debounce, defeating the spec
      // contract that requires explicit operator Acknowledge + Confirm.
      // Conversely, healing_enabled=false would silently lose the HEALED
      // signal entirely.
      //
      // Until we add STATUS_LATCHED (or a similar lifecycle-distinguishing
      // status) to ros2_medkit_msgs/msg/Fault we keep status=CONFIRMED
      // until the next ClearFault fires. The operator-side gap (cannot
      // see "physically cleared, awaiting confirm" in the UI) is tracked
      // separately; see PR #387 review thread.
      log_info("AlarmCondition HEALED (latched, awaiting ack/confirm): " + delivery.fault_code);
      break;
    case AlarmAction::ClearFault:
      log_info("AlarmCondition CLEARED: " + delivery.fault_code);
      send_clear_fault(delivery.fault_code);
      break;
    case AlarmAction::NoOp:
      break;
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

  auto parsed = parse_coerce_validate(value["value"], *entry);
  if (!parsed) {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::InvalidValue, parsed.error(), 400});
  }

  auto write_result = client_->write_value(entry->node_id, *parsed, entry->data_type);
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
      *parsed);
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

    // Issue #386: emit acknowledge_fault / confirm_fault when the entity
    // has at least one native AlarmConditionType event subscription. The
    // fault_code parameter (passed in operation execution body) discriminates
    // which condition the operator is acting on.
    bool has_event_alarms = std::any_of(node_map_.event_alarms().begin(), node_map_.event_alarms().end(),
                                        [&entity_id](const AlarmEventConfig & cfg) {
                                          return cfg.entity_id == entity_id;
                                        });
    if (has_event_alarms) {
      nlohmann::json ack;
      ack["id"] = "acknowledge_fault";
      ack["name"] = "Acknowledge an alarm";
      ack["proximity_proof_required"] = false;
      ack["asynchronous_execution"] = false;
      items.push_back(std::move(ack));

      nlohmann::json confirm;
      confirm["id"] = "confirm_fault";
      confirm["name"] = "Confirm an alarm condition is addressed";
      confirm["proximity_proof_required"] = false;
      confirm["asynchronous_execution"] = false;
      items.push_back(std::move(confirm));
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

  // Issue #386: acknowledge_fault and confirm_fault dispatch to OPC-UA
  // Method calls on the live ConditionId. AcknowledgeableConditionType
  // declares Acknowledge as method i=9111 and Confirm as method i=9113;
  // the objectId argument is the ConditionId NodeId of the live instance.
  if (operation_name == "acknowledge_fault" || operation_name == "confirm_fault") {
    if (!parameters.contains("fault_code") || !parameters["fault_code"].is_string()) {
      return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                                            "Missing required string parameter: fault_code", 400});
    }
    std::string fault_code = parameters["fault_code"].get<std::string>();
    std::string comment = parameters.value("comment", std::string{});

    // Input validation (bburda review on PR #387). ``fault_code`` lands in
    // log lines and HTTP error bodies via string concatenation; without
    // length / charset bounds an authenticated operator can inject
    // newlines, control chars, or multi-MB blobs into gateway logs.
    // ``comment`` is forwarded verbatim to the PLC as ``LocalizedText`` -
    // unbounded length lets the same role spam the PLC's condition log.
    // 256 chars matches the existing entity-id bound (``is_valid_path_segment``)
    // and is generous enough for typical ``PLC_OVERPRESSURE_SENSOR_3`` style
    // identifiers + a one-sentence operator comment.
    if (!is_valid_path_segment(fault_code)) {
      return tl::make_unexpected(OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                                            "fault_code must be alphanumeric+_- and 1-256 chars (got " +
                                                                std::to_string(fault_code.size()) + " chars)",
                                                            400});
    }
    if (comment.size() > 256) {
      return tl::make_unexpected(
          OperationProviderErrorInfo{OperationProviderError::InvalidParameters,
                                     "comment exceeds 256 chars (got " + std::to_string(comment.size()) + ")", 400});
    }

    auto runtime = poller_->lookup_condition(entity_id, fault_code);
    if (!runtime) {
      return tl::make_unexpected(OperationProviderErrorInfo{
          OperationProviderError::OperationNotFound,
          "No live condition for fault_code=" + fault_code + " on entity=" + entity_id, 404});
    }

    constexpr uint32_t kAcknowledgeMethodId = 9111;
    constexpr uint32_t kConfirmMethodId = 9113;
    opcua::NodeId method_id(0, operation_name == "acknowledge_fault" ? kAcknowledgeMethodId : kConfirmMethodId);

    std::vector<opcua::Variant> args;
    args.push_back(opcua::Variant::fromScalar(runtime->latest_event_id));
    args.push_back(opcua::Variant::fromScalar(opcua::LocalizedText("", comment)));

    if (plugin_debug_enabled()) {
      std::ostringstream hex_oss;
      const auto * bytes = runtime->latest_event_id.data();
      for (size_t i = 0; i < std::min<size_t>(runtime->latest_event_id.length(), 16); ++i) {
        char buf[3];
        std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned>(bytes[i]) & 0xffu);
        hex_oss << buf;
      }
      RCLCPP_DEBUG_STREAM(opcua_plugin_logger(), operation_name << " EventId len=" << runtime->latest_event_id.length()
                                                                << " hex=" << hex_oss.str()
                                                                << " conditionId=" << runtime->condition_id.toString());
    }

    auto result = client_->call_method(runtime->condition_id, method_id, args);
    if (!result.has_value()) {
      auto code = result.error().code;
      int http = 502;
      auto err = OperationProviderError::TransportError;
      if (code == OpcuaClient::MethodError::NotConnected) {
        http = 503;
        err = OperationProviderError::Internal;
      } else if (code == OpcuaClient::MethodError::MethodNotFound) {
        http = 404;
        err = OperationProviderError::OperationNotFound;
      } else if (code == OpcuaClient::MethodError::InvalidArgument) {
        http = 400;
        err = OperationProviderError::InvalidParameters;
      } else if (code == OpcuaClient::MethodError::MethodTimeout) {
        http = 504;
        err = OperationProviderError::TransportError;
      }
      return tl::make_unexpected(OperationProviderErrorInfo{err, result.error().message, http});
    }

    nlohmann::json out;
    out["status"] = "ok";
    out["operation"] = operation_name;
    out["fault_code"] = fault_code;
    out["entity_id"] = entity_id;
    out["condition_id"] = runtime->condition_id.toString();
    return tl::expected<nlohmann::json, OperationProviderErrorInfo>{out};
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

  auto parsed = parse_coerce_validate(parameters["value"], *entry);
  if (!parsed) {
    return tl::make_unexpected(
        OperationProviderErrorInfo{OperationProviderError::InvalidParameters, parsed.error(), 400});
  }

  auto write_result = client_->write_value(entry->node_id, *parsed, entry->data_type);
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
      *parsed);
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
