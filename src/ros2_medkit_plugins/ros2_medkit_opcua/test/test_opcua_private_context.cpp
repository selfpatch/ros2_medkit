// Copyright 2026 bburda
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

// A host can run the gateway on a private context and never initialise the default one.
// This binary must not call rclcpp::init().

#include "ros2_medkit_opcua/opcua_plugin.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/srv/clear_fault.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

// -- Stub PluginRequest/PluginResponse (same pattern as graph_provider test) --

namespace ros2_medkit_gateway {

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}
std::string PluginRequest::path_param(size_t /*index*/) const {
  return {};
}
std::string PluginRequest::header(const std::string & /*name*/) const {
  return {};
}
const std::string & PluginRequest::path() const {
  static const std::string empty;
  return empty;
}
const std::string & PluginRequest::body() const {
  static const std::string empty;
  return empty;
}
std::string PluginRequest::query_param(const std::string & /*name*/) const {
  return {};
}

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}
void PluginResponse::send_json(const nlohmann::json & /*data*/) {
}
void PluginResponse::send_error(int /*status*/, const std::string & /*error_code*/, const std::string & /*message*/,
                                const nlohmann::json & /*parameters*/) {
}

// -- FakePluginContext --

class FakePluginContext : public RosPluginContext {
 public:
  std::unordered_map<std::string, PluginEntityInfo> entities;
  nlohmann::json all_faults = nlohmann::json::object();

  rclcpp::Node * node() const override {
    return nullptr;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    auto it = entities.find(id);
    if (it != entities.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & /*entity_id*/) const override {
    return {};
  }

  nlohmann::json list_entity_faults(const std::string & entity_id) const override {
    // Contract: a bare JSON array of fault objects scoped to the entity.
    nlohmann::json result = nlohmann::json::array();
    if (all_faults.contains("faults")) {
      for (const auto & f : all_faults["faults"]) {
        bool in_scope = f.value("source_id", "") == entity_id;
        if (!in_scope && f.contains("reporting_sources") && f["reporting_sources"].is_array()) {
          for (const auto & src : f["reporting_sources"]) {
            in_scope = in_scope || (src.is_string() && src.get<std::string>() == entity_id);
          }
        }
        if (in_scope) {
          result.push_back(f);
        }
      }
    }
    return result;
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const PluginRequest & /*req*/, PluginResponse & /*res*/,
                                                            const std::string & entity_id) const override {
    return get_entity(entity_id);
  }

  // Recorded so tests can assert exactly which entities get which vendor
  // capability (e.g. x-plc-data must not be registered for a def with no
  // data points - see the auto_alarms fallback entity below).
  std::vector<std::pair<std::string, std::string>> registered_entity_capabilities;

  void register_capability(SovdEntityType /*type*/, const std::string & /*capability*/) override {
  }
  void register_entity_capability(const std::string & entity_id, const std::string & capability) override {
    registered_entity_capabilities.emplace_back(entity_id, capability);
  }
  std::vector<std::string> get_type_capabilities(SovdEntityType /*type*/) const override {
    return {};
  }
  std::vector<std::string> get_entity_capabilities(const std::string & /*entity_id*/) const override {
    return {};
  }
  LockAccessResult check_lock(const std::string & /*entity_id*/, const std::string & /*collection*/,
                              const std::string & /*client_id*/) const override {
    return {true, "", "", ""};
  }
  tl::expected<LockInfo, LockError> acquire_lock(const std::string & /*entity_id*/, const std::string & /*collection*/,
                                                 const std::vector<std::string> & /*scopes*/,
                                                 int /*expiration_s*/) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  tl::expected<void, LockError> release_lock(const std::string & /*entity_id*/,
                                             const std::string & /*lock_id*/) override {
    return tl::make_unexpected(LockError{"not supported", "", 409, std::nullopt});
  }
  IntrospectionInput get_entity_snapshot() const override {
    return {};
  }
  nlohmann::json list_all_faults() const override {
    return all_faults;
  }
  void register_sampler(
      const std::string & /*topic*/,
      const std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> &
      /*sampler*/) override {
  }
  ResourceChangeNotifier * get_resource_change_notifier() override {
    return nullptr;
  }
  ConditionRegistry * get_condition_registry() override {
    return nullptr;
  }
};

namespace {

class PrivateNodePluginContext : public FakePluginContext {
 public:
  explicit PrivateNodePluginContext(rclcpp::Node * node) : node_(node) {
  }
  rclcpp::Node * node() const override {
    return node_;
  }

 private:
  rclcpp::Node * node_;
};

}  // namespace

TEST(OpcuaPrivateContext, PluginRunsOnTheGatewayContext) {
  ASSERT_FALSE(rclcpp::ok()) << "the default context must stay uninitialised";
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto gateway_node = std::make_shared<rclcpp::Node>("private_context_gateway", rclcpp::NodeOptions().context(context));

  {
    ros2_medkit_gateway::OpcuaPlugin plugin;
    nlohmann::json config;
    // Nothing listens there, so the poll thread only retries.
    config["endpoint_url"] = "opc.tcp://127.0.0.1:1";
    config["poll_interval_ms"] = 100;
    plugin.configure(config);
    PrivateNodePluginContext ctx(gateway_node.get());
    // The gateway's PluginManager disables a plugin whose set_context() throws.
    EXPECT_NO_THROW(plugin.set_context(ctx));
    EXPECT_NO_THROW(plugin.shutdown());
  }

  gateway_node.reset();
  context->shutdown("test done");
  EXPECT_FALSE(rclcpp::ok()) << "something initialised the default context";
}

}  // namespace ros2_medkit_gateway
