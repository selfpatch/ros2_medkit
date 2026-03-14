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

#include "ros2_medkit_topic_beacon/topic_beacon_plugin.hpp"

#include <algorithm>
#include <chrono>

#include "ros2_medkit_beacon_common/beacon_response_builder.hpp"

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

std::string TopicBeaconPlugin::name() const {
  return "topic_beacon";
}

void TopicBeaconPlugin::configure(const nlohmann::json & config) {
  topic_ = config.value("topic", std::string("/ros2_medkit/discovery"));

  BeaconHintStore::Config store_config;
  auto beacon_ttl = config.value("beacon_ttl_sec", 10.0);
  auto beacon_expiry = config.value("beacon_expiry_sec", 300.0);
  auto max_hints = config.value("max_hints", static_cast<size_t>(10000));
  auto max_mps = config.value("max_messages_per_second", 100.0);

  // Clamp to safe minimums
  if (beacon_ttl < 0.1) {
    log_warn("beacon_ttl_sec clamped from " + std::to_string(beacon_ttl) + " to 0.1");
    beacon_ttl = 0.1;
  }
  if (beacon_expiry < 1.0) {
    log_warn("beacon_expiry_sec clamped from " + std::to_string(beacon_expiry) + " to 1.0");
    beacon_expiry = 1.0;
  }
  if (max_hints < 1) {
    log_warn("max_hints clamped from 0 to 1");
    max_hints = 1;
  }
  if (max_mps < 1.0) {
    log_warn("max_messages_per_second clamped from " + std::to_string(max_mps) + " to 1.0");
    max_mps = 1.0;
  }

  store_config.beacon_ttl_sec = beacon_ttl;
  store_config.beacon_expiry_sec = beacon_expiry;
  store_config.max_hints = max_hints;
  store_ = std::make_unique<BeaconHintStore>(store_config);

  BeaconEntityMapper::Config mapper_config;
  mapper_config.allow_new_entities = config.value("allow_new_entities", false);
  mapper_ = BeaconEntityMapper(mapper_config);

  rate_limiter_.reset(max_mps);

  // Note: field group policy overrides are handled by PluginLayer via
  // apply_layer_policy_overrides() in discovery_manager.cpp, not here.
  // Plugin config only needs to provide values in gateway_params.yaml
  // under plugins.topic_beacon.policies.{identity,hierarchy,metadata}.
}

void TopicBeaconPlugin::set_context(PluginContext & context) {
  ctx_ = &context;
  auto node = ctx_->node();

  // Ensure store_ exists even if configure() was not called
  if (!store_) {
    store_ = std::make_unique<BeaconHintStore>();
  }

  // Create subscription on configured topic
  subscription_ = node->create_subscription<ros2_medkit_msgs::msg::MedkitDiscoveryHint>(
      topic_, rclcpp::QoS(100).reliable(), [this](const ros2_medkit_msgs::msg::MedkitDiscoveryHint::SharedPtr msg) {
        on_beacon(msg);
      });

  // Register vendor extension capability
  ctx_->register_capability(SovdEntityType::APP, "x-medkit-topic-beacon");
  ctx_->register_capability(SovdEntityType::COMPONENT, "x-medkit-topic-beacon");

  RCLCPP_INFO(node->get_logger(), "TopicBeaconPlugin: subscribed to '%s'", topic_.c_str());
}

void TopicBeaconPlugin::shutdown() {
  subscription_.reset();
}

void TopicBeaconPlugin::register_routes(httplib::Server & server, const std::string & api_prefix) {
  // Register beacon metadata endpoint for apps and components
  for (const auto & entity_type : {"apps", "components"}) {
    auto pattern = api_prefix + "/" + entity_type + R"(/([^/]+)/x-medkit-topic-beacon)";
    server.Get(pattern.c_str(), [this](const httplib::Request & req, httplib::Response & res) {
      auto entity_id = req.matches[1].str();

      auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
      if (!entity) {
        return;
      }

      auto stored = store_->get(entity_id);
      if (!stored) {
        PluginContext::send_error(res, 404, "x-medkit-beacon-not-found", "No beacon data for entity");
        return;
      }

      PluginContext::send_json(res, ros2_medkit_beacon::build_beacon_response(entity_id, *stored));
    });
  }
}

IntrospectionResult TopicBeaconPlugin::introspect(const IntrospectionInput & input) {
  auto snapshot = store_->evict_and_snapshot();
  return mapper_.map(snapshot, input);
}

void TopicBeaconPlugin::on_beacon(const ros2_medkit_msgs::msg::MedkitDiscoveryHint::SharedPtr & msg) {
  // Rate limiting
  if (!rate_limiter_.try_consume()) {
    return;  // drop silently
  }

  // Convert ROS message to internal BeaconHint
  BeaconHint hint;
  hint.entity_id = msg->entity_id;
  hint.stable_id = msg->stable_id;
  hint.display_name = msg->display_name;
  hint.function_ids = msg->function_ids;
  hint.depends_on = msg->depends_on;
  hint.component_id = msg->component_id;
  hint.transport_type = msg->transport_type;
  hint.negotiated_format = msg->negotiated_format;
  hint.process_id = msg->process_id;
  hint.process_name = msg->process_name;
  hint.hostname = msg->hostname;
  hint.received_at = std::chrono::steady_clock::now();

  // Convert KeyValue[] to map
  for (const auto & kv : msg->metadata) {
    hint.metadata[kv.key] = kv.value;
  }

  // Validate
  auto validation = validate_beacon_hint(hint, limits_);
  if (!validation.valid) {
    return;  // invalid hint
  }

  // Store
  store_->update(hint);
}

// --- Plugin exports ---

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new TopicBeaconPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin) {
  return static_cast<TopicBeaconPlugin *>(plugin);
}
