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

#pragma once

#include <httplib.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/medkit_discovery_hint.hpp>

#include "ros2_medkit_beacon_common/beacon_entity_mapper.hpp"
#include "ros2_medkit_beacon_common/beacon_hint_store.hpp"
#include "ros2_medkit_beacon_common/beacon_types.hpp"
#include "ros2_medkit_beacon_common/beacon_validator.hpp"
#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"

// Simple token bucket for rate limiting.
// Thread safety: on_beacon() is called from the DDS callback thread,
// so try_consume() is guarded by a mutex for multi-threaded executor safety.
class TokenBucket {
 public:
  TokenBucket() : TokenBucket(100.0) {
  }

  explicit TokenBucket(double max_per_second)
    : max_tokens_(max_per_second), tokens_(max_per_second), last_refill_(std::chrono::steady_clock::now()) {
  }

  bool try_consume() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - last_refill_).count();
    tokens_ = std::min(max_tokens_, tokens_ + elapsed * max_tokens_);
    last_refill_ = now;
    if (tokens_ >= 1.0) {
      tokens_ -= 1.0;
      return true;
    }
    return false;
  }

  /// Reinitialize the bucket with a new rate (avoids move/copy of mutex).
  void reset(double max_per_second) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_tokens_ = max_per_second;
    tokens_ = max_per_second;
    last_refill_ = std::chrono::steady_clock::now();
  }

 private:
  mutable std::mutex mutex_;
  double max_tokens_;
  double tokens_;
  std::chrono::steady_clock::time_point last_refill_;
};

class TopicBeaconPlugin : public ros2_medkit_gateway::GatewayPlugin, public ros2_medkit_gateway::IntrospectionProvider {
 public:
  std::string name() const override;
  void configure(const nlohmann::json & config) override;
  void set_context(ros2_medkit_gateway::PluginContext & context) override;
  void shutdown() override;
  void register_routes(httplib::Server & server, const std::string & api_prefix) override;
  std::vector<ros2_medkit_gateway::GatewayPlugin::RouteDescription> get_route_descriptions() const override;
  ros2_medkit_gateway::IntrospectionResult introspect(const ros2_medkit_gateway::IntrospectionInput & input) override;

  // Expose for testing
  ros2_medkit_beacon::BeaconHintStore & store() {
    return *store_;
  }
  const rclcpp::Subscription<ros2_medkit_msgs::msg::MedkitDiscoveryHint>::SharedPtr & subscription() const {
    return subscription_;
  }

 private:
  void on_beacon(const ros2_medkit_msgs::msg::MedkitDiscoveryHint::SharedPtr & msg);

  std::string topic_{"/ros2_medkit/discovery"};
  ros2_medkit_gateway::PluginContext * ctx_{nullptr};
  rclcpp::Subscription<ros2_medkit_msgs::msg::MedkitDiscoveryHint>::SharedPtr subscription_;
  std::unique_ptr<ros2_medkit_beacon::BeaconHintStore> store_;
  ros2_medkit_beacon::BeaconEntityMapper mapper_;
  ros2_medkit_beacon::ValidationLimits limits_;
  TokenBucket rate_limiter_{100.0};
  bool capacity_warned_{false};
  std::unordered_set<std::string> logged_skipped_entities_;
};
