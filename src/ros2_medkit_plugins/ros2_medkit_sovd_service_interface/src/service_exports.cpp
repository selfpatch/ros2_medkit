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

#include "ros2_medkit_gateway/plugins/plugin_types.hpp"
#include "sovd_service_interface.hpp"

using namespace ros2_medkit_gateway;

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;  // Must return 4 (exact match required)
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new SovdServiceInterface();
}
