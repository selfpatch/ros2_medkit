// Copyright 2026 selfpatch GmbH
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

#include "ros2_medkit_param_beacon/param_beacon_plugin.hpp"

#include <chrono>
#include <string>
#include <utility>
#include <vector>

using ros2_medkit_beacon::BeaconEntityMapper;
using ros2_medkit_beacon::BeaconHint;
using ros2_medkit_beacon::BeaconHintStore;
using ros2_medkit_beacon::validate_beacon_hint;
using ros2_medkit_beacon::ValidationLimits;
using ros2_medkit_gateway::GatewayPlugin;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_gateway::IntrospectionProvider;
using ros2_medkit_gateway::IntrospectionResult;
using ros2_medkit_gateway::PLUGIN_API_VERSION;
using ros2_medkit_gateway::PluginContext;
using ros2_medkit_gateway::SovdEntityType;

std::string ParameterBeaconPlugin::name() const {
  return "parameter_beacon";
}

void ParameterBeaconPlugin::configure(const nlohmann::json & config) {
  parameter_prefix_ = config.value("parameter_prefix", std::string("ros2_medkit.discovery"));
  poll_interval_ = std::chrono::duration<double>(config.value("poll_interval_sec", 5.0));
  poll_budget_sec_ = config.value("poll_budget_sec", 10.0);
  param_timeout_sec_ = config.value("param_timeout_sec", 2.0);

  BeaconHintStore::Config store_config;
  store_config.beacon_ttl_sec = config.value("beacon_ttl_sec", 15.0);
  store_config.beacon_expiry_sec = config.value("beacon_expiry_sec", 300.0);
  store_config.check_process_alive = config.value("check_process_alive", true);
  store_config.max_hints = config.value("max_hints", static_cast<size_t>(10000));

  // Config validation
  if (store_config.beacon_ttl_sec <= poll_interval_.count()) {
    log_warn(
        "beacon_ttl_sec <= poll_interval_sec: hints will stale between polls. Auto-setting ttl = 3 "
        "* poll_interval");
    store_config.beacon_ttl_sec = poll_interval_.count() * 3.0;
  }
  if (store_config.beacon_expiry_sec <= store_config.beacon_ttl_sec) {
    log_warn("beacon_expiry_sec <= beacon_ttl_sec. Auto-setting expiry = 10 * ttl");
    store_config.beacon_expiry_sec = store_config.beacon_ttl_sec * 10.0;
  }
  if (poll_budget_sec_ <= param_timeout_sec_) {
    log_warn("poll_budget_sec <= param_timeout_sec: at most one node polled per cycle");
  }

  store_ = std::make_unique<BeaconHintStore>(store_config);

  BeaconEntityMapper::Config mapper_config;
  mapper_config.allow_new_entities = config.value("allow_new_entities", false);
  mapper_ = BeaconEntityMapper(mapper_config);
}

void ParameterBeaconPlugin::set_context(PluginContext & context) {
  ctx_ = &context;

  if (!store_) {
    store_ = std::make_unique<BeaconHintStore>();
  }

  // Create dedicated node for parameter client operations
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.use_global_arguments(false);
  param_node_ = std::make_shared<rclcpp::Node>("_param_beacon_node", options);

  // Set default client factory if not injected (tests inject mock factory)
  if (!client_factory_) {
    auto node = param_node_;
    client_factory_ = [node](const std::string & target) {
      return std::make_shared<ros2_medkit_param_beacon::RealParameterClient>(node, target);
    };
  }

  // Register vendor extension capability
  ctx_->register_capability(SovdEntityType::APP, "x-medkit-param-beacon");
  ctx_->register_capability(SovdEntityType::COMPONENT, "x-medkit-param-beacon");

  // Start polling thread
  shutdown_requested_ = false;
  poll_thread_ = std::thread([this]() {
    poll_loop();
  });

  RCLCPP_INFO(ctx_->node()->get_logger(), "ParameterBeaconPlugin: polling '%s.*' every %.1fs",
              parameter_prefix_.c_str(), poll_interval_.count());
}

void ParameterBeaconPlugin::shutdown() {
  {
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    shutdown_requested_ = true;
  }
  shutdown_cv_.notify_one();
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
  param_node_.reset();
  std::lock_guard<std::mutex> lock(clients_mutex_);
  clients_.clear();
  backoff_counts_.clear();
  skip_remaining_.clear();
}

void ParameterBeaconPlugin::register_routes(httplib::Server & server, const std::string & api_prefix) {
  for (const auto & entity_type : {"apps", "components"}) {
    auto pattern = api_prefix + "/" + entity_type + R"(/([^/]+)/x-medkit-param-beacon)";
    server.Get(pattern.c_str(), [this](const httplib::Request & req, httplib::Response & res) {
      auto entity_id = req.matches[1].str();
      auto stored = store_->get(entity_id);
      if (!stored) {
        res.status = 404;
        nlohmann::json err = {{"error", "No beacon data for entity"}, {"entity_id", entity_id}};
        res.set_content(err.dump(), "application/json");
        return;
      }
      nlohmann::json data;
      data["entity_id"] = entity_id;
      data["status"] = stored->status == BeaconHintStore::HintStatus::ACTIVE ? "active" : "stale";
      auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - stored->last_seen).count();
      data["age_sec"] = age;
      data["stable_id"] = stored->hint.stable_id;
      data["display_name"] = stored->hint.display_name;
      data["transport_type"] = stored->hint.transport_type;
      data["negotiated_format"] = stored->hint.negotiated_format;
      data["process_id"] = stored->hint.process_id;
      data["process_name"] = stored->hint.process_name;
      data["hostname"] = stored->hint.hostname;
      data["component_id"] = stored->hint.component_id;
      data["function_ids"] = stored->hint.function_ids;
      data["depends_on"] = stored->hint.depends_on;
      data["metadata"] = stored->hint.metadata;
      PluginContext::send_json(res, data);
    });
  }
}

IntrospectionResult ParameterBeaconPlugin::introspect(const IntrospectionInput & input) {
  // Cache node list for poll thread
  {
    std::unique_lock<std::shared_mutex> lock(nodes_mutex_);
    poll_targets_.clear();
    for (const auto & app : input.apps) {
      if (app.is_online && app.bound_fqn.has_value()) {
        poll_targets_.push_back(*app.bound_fqn);
      }
    }
  }

  auto snapshot = store_->evict_and_snapshot();
  return mapper_.map(snapshot, input);
}

// --- Polling logic ---

void ParameterBeaconPlugin::poll_loop() {
  while (!shutdown_requested_.load()) {
    poll_cycle();
    std::unique_lock<std::mutex> lock(shutdown_mutex_);
    shutdown_cv_.wait_for(lock, poll_interval_, [this] {
      return shutdown_requested_.load();
    });
  }
}

void ParameterBeaconPlugin::poll_cycle() {
  // Get targets: prefer introspection-provided list, fall back to ROS graph discovery
  std::vector<std::string> targets;
  {
    std::shared_lock<std::shared_mutex> lock(nodes_mutex_);
    targets = poll_targets_;
  }

  if (targets.empty() && param_node_) {
    // No introspection targets available (e.g., not in hybrid mode).
    // Discover nodes directly from the ROS 2 graph.
    auto names_and_ns = param_node_->get_node_graph_interface()->get_node_names_and_namespaces();
    for (const auto & [name, ns] : names_and_ns) {
      // Skip internal nodes (leading underscore) and the gateway
      if (name.empty() || name[0] == '_' || name == "ros2_medkit_gateway") {
        continue;
      }
      auto fqn = (ns == "/" ? "/" : ns + "/") + name;
      targets.push_back(fqn);
    }
  }

  if (targets.empty()) {
    return;
  }

  // Evict stale clients
  evict_stale_clients();

  auto cycle_start = std::chrono::steady_clock::now();
  auto n = targets.size();
  size_t polled = 0;

  for (size_t i = 0; i < n; ++i) {
    // Budget check
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - cycle_start).count();
    if (elapsed >= poll_budget_sec_) {
      break;
    }

    auto idx = (start_offset_ + i) % n;
    const auto & fqn = targets[idx];

    // Check backoff
    auto skip_it = skip_remaining_.find(fqn);
    if (skip_it != skip_remaining_.end() && skip_it->second > 0) {
      --skip_it->second;
      continue;
    }

    poll_node(fqn);
    ++polled;
  }

  start_offset_ = (start_offset_ + 1) % n;
}

void ParameterBeaconPlugin::poll_node(const std::string & fqn) {
  try {
    auto client = get_or_create_client(fqn);

    std::lock_guard<std::mutex> ops_lock(param_ops_mutex_);

    if (!client->wait_for_service(std::chrono::duration<double>(param_timeout_sec_))) {
      // Timeout - apply backoff
      auto & count = backoff_counts_[fqn];
      ++count;
      int skip = std::min(1 << std::min(count - 1, 3), 8);
      skip_remaining_[fqn] = skip;
      return;
    }

    // List parameters under prefix
    auto list_result = client->list_parameters({parameter_prefix_}, 0);
    if (list_result.names.empty()) {
      return;  // No beacon parameters declared
    }

    // Fetch parameter values
    auto params = client->get_parameters(list_result.names);

    // Convert to BeaconHint
    auto hint = parse_parameters(fqn, params);
    if (hint.entity_id.empty()) {
      return;  // No entity_id parameter - skip
    }

    // Validate
    auto result = validate_beacon_hint(hint, limits_);
    if (!result.valid) {
      return;
    }

    // Store
    store_->update(hint);

    // Reset backoff on success
    backoff_counts_.erase(fqn);
    skip_remaining_.erase(fqn);

  } catch (const std::exception & e) {
    // Node disappeared or service error - apply backoff
    auto & count = backoff_counts_[fqn];
    ++count;
    int skip = std::min(1 << std::min(count - 1, 3), 8);
    skip_remaining_[fqn] = skip;
  }
}

BeaconHint ParameterBeaconPlugin::parse_parameters(const std::string & /*fqn*/,
                                                   const std::vector<rclcpp::Parameter> & params) {
  BeaconHint hint;
  hint.received_at = std::chrono::steady_clock::now();

  std::string metadata_prefix = parameter_prefix_ + ".metadata.";

  for (const auto & param : params) {
    auto param_name = param.get_name();

    // Strip prefix to get the field name
    if (param_name.rfind(parameter_prefix_ + ".", 0) != 0) {
      continue;
    }
    auto field = param_name.substr(parameter_prefix_.size() + 1);

    // Handle metadata sub-parameters
    if (param_name.rfind(metadata_prefix, 0) == 0) {
      auto key = param_name.substr(metadata_prefix.size());
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        hint.metadata[key] = param.as_string();
      }
      continue;
    }

    // Map known fields
    if (field == "entity_id" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.entity_id = param.as_string();
    } else if (field == "stable_id" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.stable_id = param.as_string();
    } else if (field == "display_name" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.display_name = param.as_string();
    } else if (field == "component_id" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.component_id = param.as_string();
    } else if (field == "transport_type" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.transport_type = param.as_string();
    } else if (field == "negotiated_format" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.negotiated_format = param.as_string();
    } else if (field == "process_name" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.process_name = param.as_string();
    } else if (field == "hostname" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      hint.hostname = param.as_string();
    } else if (field == "process_id" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      auto val = param.as_int();
      if (val >= 0 && val <= static_cast<int64_t>(UINT32_MAX)) {
        hint.process_id = static_cast<uint32_t>(val);
      }
    } else if (field == "function_ids" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      hint.function_ids = param.as_string_array();
    } else if (field == "depends_on" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      hint.depends_on = param.as_string_array();
    }
    // Unknown fields or type mismatches: silently skip
  }

  return hint;
}

std::shared_ptr<ros2_medkit_param_beacon::ParameterClientInterface>
ParameterBeaconPlugin::get_or_create_client(const std::string & fqn) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(fqn);
  if (it != clients_.end()) {
    return it->second;
  }
  auto client = client_factory_(fqn);
  clients_[fqn] = client;
  return client;
}

void ParameterBeaconPlugin::evict_stale_clients() {
  std::shared_lock<std::shared_mutex> nodes_lock(nodes_mutex_);
  std::lock_guard<std::mutex> clients_lock(clients_mutex_);

  for (auto it = clients_.begin(); it != clients_.end();) {
    bool found = false;
    for (const auto & target : poll_targets_) {
      if (target == it->first) {
        found = true;
        break;
      }
    }
    if (!found) {
      backoff_counts_.erase(it->first);
      skip_remaining_.erase(it->first);
      it = clients_.erase(it);
    } else {
      ++it;
    }
  }
}

// --- Plugin exports ---

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new ParameterBeaconPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin) {
  return static_cast<ParameterBeaconPlugin *>(plugin);
}
