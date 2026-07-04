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

#include "ros2_medkit_opcua/device_identity.hpp"

namespace ros2_medkit_gateway {

namespace {
constexpr const char * kOpcuaSource = "opcua";
}  // namespace

AssetIdentity opcua_device_info_to_identity(const OpcuaClient::DeviceInfo & info, const std::string & endpoint_url) {
  AssetIdentity identity;
  auto set_field = [&](std::string AssetIdentity::*member, const std::string & value, const char * prov_key) {
    if (!value.empty()) {
      identity.*member = value;
      identity.provenance[prov_key] = kOpcuaSource;
    }
  };

  // DI nameplate (per-device) wins over the server-level BuildInfo.
  const std::string & manufacturer = !info.di_manufacturer.empty() ? info.di_manufacturer : info.manufacturer_name;
  const std::string & model = !info.di_model.empty() ? info.di_model : info.product_name;
  const std::string & software_version =
      !info.di_software_revision.empty() ? info.di_software_revision : info.software_version;

  set_field(&AssetIdentity::manufacturer, manufacturer, "manufacturer");
  set_field(&AssetIdentity::model, model, "model");
  set_field(&AssetIdentity::order_code, info.di_order_number, "order_code");
  set_field(&AssetIdentity::serial_number, info.di_serial_number, "serial_number");
  set_field(&AssetIdentity::hardware_revision, info.di_hardware_revision, "hardware_revision");
  set_field(&AssetIdentity::software_version, software_version, "software_version");
  set_field(&AssetIdentity::network_endpoint, endpoint_url, "network_endpoint");

  if (!info.build_number.empty()) {
    identity.extra["buildNumber"] = info.build_number;
    identity.provenance["extra.buildNumber"] = kOpcuaSource;
  }

  return identity;
}

bool opcua_identity_trusted(const OpcuaClientConfig & config) {
  return OpcuaClient::requires_secure_channel(config) && config.reject_untrusted;
}

}  // namespace ros2_medkit_gateway
