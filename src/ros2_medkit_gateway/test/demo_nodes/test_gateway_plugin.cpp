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

#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/providers/update_provider.hpp"

#include <httplib.h>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using namespace ros2_medkit_gateway;

/**
 * @brief Test gateway plugin implementing all provider interfaces.
 *
 * Used by test_plugin_loader to verify dlopen/dlsym-based loading,
 * API version checking, and dynamic_cast to provider interfaces.
 */
class TestGatewayPlugin : public GatewayPlugin, public UpdateProvider, public IntrospectionProvider {
 public:
  // --- GatewayPlugin ---
  std::string name() const override {
    return "test_plugin";
  }

  void configure(const nlohmann::json & /*config*/) override {
  }

  void register_routes(httplib::Server & server, const std::string & api_prefix) override {
    server.Get((api_prefix + "/x-test/ping").c_str(), [](const httplib::Request &, httplib::Response & res) {
      res.set_content("pong", "text/plain");
    });
  }

  // --- UpdateProvider ---
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter &) override {
    return std::vector<std::string>{};
  }

  tl::expected<nlohmann::json, UpdateBackendErrorInfo> get_update(const std::string & id) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found: " + id});
  }

  tl::expected<void, UpdateBackendErrorInfo> register_update(const nlohmann::json &) override {
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string &) override {
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string &, UpdateProgressReporter &) override {
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string &, UpdateProgressReporter &) override {
    return {};
  }

  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string &) override {
    return false;
  }

  // --- IntrospectionProvider ---
  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }
};

extern "C" int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GatewayPlugin * create_plugin() {
  return new TestGatewayPlugin();
}
