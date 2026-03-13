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

using namespace ros2_medkit_gateway;  // NOLINT(build/namespaces)

class ContainerPlugin : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "container_introspection";
  }
  void configure(const nlohmann::json &) override {
  }
  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }
};

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}
extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new ContainerPlugin();
}
extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * p) {
  return static_cast<ContainerPlugin *>(p);
}
