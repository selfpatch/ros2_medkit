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

// Unit tests for the OPC-UA device-info -> AssetIdentity mapping (INV2). Pure
// function, no server: covers the BuildInfo path, the DI-nameplate-wins
// precedence, extras, endpoint, and per-field "opcua" provenance stamping.

#include "ros2_medkit_opcua/device_identity.hpp"

#include <gtest/gtest.h>

namespace ros2_medkit_gateway {

namespace {
OpcuaClient::DeviceInfo make_build_info() {
  OpcuaClient::DeviceInfo info;
  info.manufacturer_name = "open62541";
  info.product_name = "Test Alarm PLC";
  info.software_version = "1.2.3";
  info.build_number = "build-4567";
  return info;
}
}  // namespace

TEST(DeviceIdentityMap, EmptyInfoYieldsEmptyIdentity) {
  OpcuaClient::DeviceInfo info;
  auto id = opcua_device_info_to_identity(info, "");
  EXPECT_TRUE(id.empty());
  EXPECT_TRUE(id.provenance.empty());
}

TEST(DeviceIdentityMap, BuildInfoMapsToTypedFields) {
  auto id = opcua_device_info_to_identity(make_build_info(), "opc.tcp://plc:4840");

  EXPECT_EQ(id.manufacturer, "open62541");
  EXPECT_EQ(id.model, "Test Alarm PLC");  // product name maps to model
  EXPECT_EQ(id.software_version, "1.2.3");
  EXPECT_EQ(id.network_endpoint, "opc.tcp://plc:4840");
  // BuildNumber has no typed field -> carried as an extra.
  ASSERT_TRUE(id.extra.count("buildNumber"));
  EXPECT_EQ(id.extra.at("buildNumber"), "build-4567");
  // Serial / hardware revision only come from DI, absent here.
  EXPECT_TRUE(id.serial_number.empty());
  EXPECT_TRUE(id.hardware_revision.empty());
}

TEST(DeviceIdentityMap, ProvenanceStampedOpcuaPerField) {
  auto id = opcua_device_info_to_identity(make_build_info(), "opc.tcp://plc:4840");

  EXPECT_EQ(id.provenance.at("manufacturer"), "opcua");
  EXPECT_EQ(id.provenance.at("model"), "opcua");
  EXPECT_EQ(id.provenance.at("software_version"), "opcua");
  EXPECT_EQ(id.provenance.at("network_endpoint"), "opcua");
  EXPECT_EQ(id.provenance.at("extra.buildNumber"), "opcua");
}

TEST(DeviceIdentityMap, DiNameplateWinsOverBuildInfo) {
  OpcuaClient::DeviceInfo info = make_build_info();
  info.di_manufacturer = "SelfPatch Devices";
  info.di_model = "SPX-1000";
  info.di_serial_number = "SN-0001";
  info.di_hardware_revision = "HW-A2";
  info.di_software_revision = "SW-3.4.5";

  auto id = opcua_device_info_to_identity(info, "opc.tcp://plc:4840");

  // DI values (device-specific) override the server-level BuildInfo.
  EXPECT_EQ(id.manufacturer, "SelfPatch Devices");
  EXPECT_EQ(id.model, "SPX-1000");
  EXPECT_EQ(id.software_version, "SW-3.4.5");
  // DI-only fields.
  EXPECT_EQ(id.serial_number, "SN-0001");
  EXPECT_EQ(id.hardware_revision, "HW-A2");
  // BuildNumber still carried from BuildInfo.
  EXPECT_EQ(id.extra.at("buildNumber"), "build-4567");
}

TEST(DeviceIdentityMap, EmptyEndpointSkipsNetworkEndpoint) {
  auto id = opcua_device_info_to_identity(make_build_info(), "");
  EXPECT_TRUE(id.network_endpoint.empty());
  EXPECT_EQ(id.provenance.count("network_endpoint"), 0u);
}

TEST(DeviceIdentityMap, PartialDiFallsBackToBuildInfoPerField) {
  // Only DI serial + hardware are present; manufacturer / model / software fall
  // back to BuildInfo since the DI equivalents are empty.
  OpcuaClient::DeviceInfo info = make_build_info();
  info.di_serial_number = "SN-42";
  info.di_hardware_revision = "HW-1";

  auto id = opcua_device_info_to_identity(info, "");
  EXPECT_EQ(id.manufacturer, "open62541");  // BuildInfo fallback
  EXPECT_EQ(id.model, "Test Alarm PLC");    // BuildInfo fallback
  EXPECT_EQ(id.software_version, "1.2.3");  // BuildInfo fallback
  EXPECT_EQ(id.serial_number, "SN-42");     // DI
  EXPECT_EQ(id.hardware_revision, "HW-1");  // DI
}

}  // namespace ros2_medkit_gateway
