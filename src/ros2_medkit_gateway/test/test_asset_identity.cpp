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

#include "ros2_medkit_gateway/core/discovery/identity_merge.hpp"
#include "ros2_medkit_gateway/core/discovery/models/asset_identity.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"

#include <gtest/gtest.h>

using ros2_medkit_gateway::AssetIdentity;
using ros2_medkit_gateway::Component;
using namespace ros2_medkit_gateway::discovery;

// ---------------------------------------------------------------------------
// AssetIdentity model
// ---------------------------------------------------------------------------

TEST(AssetIdentityModel, DefaultIsEmpty) {
  AssetIdentity id;
  EXPECT_TRUE(id.empty());
  EXPECT_TRUE(id.to_json().empty());  // empty JSON object
}

TEST(AssetIdentityModel, ProvenanceAloneDoesNotMakeItNonEmpty) {
  AssetIdentity id;
  id.provenance["serial_number"] = "opcua";
  EXPECT_TRUE(id.empty());
}

TEST(AssetIdentityModel, ToJsonOnlyEmitsNonEmptyFields) {
  AssetIdentity id;
  id.manufacturer = "Siemens";
  id.serial_number = "SN-42";
  id.extra["slot"] = "3";
  id.provenance["manufacturer"] = "manifest";

  auto j = id.to_json();
  EXPECT_EQ(j["manufacturer"], "Siemens");
  EXPECT_EQ(j["serialNumber"], "SN-42");
  EXPECT_EQ(j["extra"]["slot"], "3");
  EXPECT_EQ(j["_provenance"]["manufacturer"], "manifest");
  EXPECT_FALSE(j.contains("model"));
  EXPECT_FALSE(j.contains("firmwareVersion"));
}

// Wire-level contract: field keys are camelCase (serialNumber) while their
// provenance keys stay snake_case (serial_number) under _provenance. Pins the
// documented mapping so a consumer can rely on the field -> provenance-key
// translation and it cannot silently drift.
TEST(AssetIdentityModel, ProvenanceKeysStaySnakeCaseWhileFieldKeysAreCamelCase) {
  AssetIdentity id;
  id.serial_number = "SN-42";
  id.hardware_revision = "A2";
  id.firmware_version = "2.9.4";
  id.software_version = "1.0";
  id.network_endpoint = "opc.tcp://192.168.1.10:4840";
  id.extra["asset_tag"] = "TAG-7";
  id.provenance["serial_number"] = "opcua";
  id.provenance["hardware_revision"] = "opcua";
  id.provenance["firmware_version"] = "opcua";
  id.provenance["software_version"] = "manifest";
  id.provenance["network_endpoint"] = "opcua";
  id.provenance["extra.asset_tag"] = "manifest";

  auto j = id.to_json();

  // Fields serialize under camelCase keys ...
  EXPECT_TRUE(j.contains("serialNumber"));
  EXPECT_TRUE(j.contains("hardwareRevision"));
  EXPECT_TRUE(j.contains("firmwareVersion"));
  EXPECT_TRUE(j.contains("softwareVersion"));
  EXPECT_TRUE(j.contains("networkEndpoint"));
  // ... but the camelCase names must NOT leak into _provenance.
  EXPECT_FALSE(j["_provenance"].contains("serialNumber"));
  EXPECT_FALSE(j["_provenance"].contains("hardwareRevision"));

  // Provenance is keyed by the snake_case field name / "extra." prefix.
  EXPECT_EQ(j["_provenance"]["serial_number"], "opcua");
  EXPECT_EQ(j["_provenance"]["hardware_revision"], "opcua");
  EXPECT_EQ(j["_provenance"]["firmware_version"], "opcua");
  EXPECT_EQ(j["_provenance"]["software_version"], "manifest");
  EXPECT_EQ(j["_provenance"]["network_endpoint"], "opcua");
  EXPECT_EQ(j["_provenance"]["extra.asset_tag"], "manifest");

  // The pairing survives a round-trip through from_json.
  AssetIdentity parsed = AssetIdentity::from_json(j);
  EXPECT_EQ(parsed.serial_number, "SN-42");
  EXPECT_EQ(parsed.provenance.at("serial_number"), "opcua");
  EXPECT_EQ(parsed, id);
}

TEST(AssetIdentityModel, JsonRoundTrip) {
  AssetIdentity id;
  id.manufacturer = "Siemens";
  id.model = "6ES7";
  id.serial_number = "SN-42";
  id.hardware_revision = "A2";
  id.firmware_version = "2.9.4";
  id.software_version = "1.0";
  id.network_endpoint = "opc.tcp://192.168.1.10:4840";
  id.role = "plc";
  id.extra["mac"] = "00:11:22:33:44:55";
  id.provenance["serial_number"] = "opcua";

  AssetIdentity parsed = AssetIdentity::from_json(id.to_json());
  EXPECT_EQ(parsed, id);
}

TEST(AssetIdentityModel, FromJsonTolerantOfNonObject) {
  EXPECT_TRUE(AssetIdentity::from_json(nlohmann::json(nullptr)).empty());
  EXPECT_TRUE(AssetIdentity::from_json(nlohmann::json("oops")).empty());
}

// ---------------------------------------------------------------------------
// Backward compatibility of the Component DTO / JSON
// ---------------------------------------------------------------------------

TEST(ComponentIdentityCompat, NoIdentityKeyWhenEmpty) {
  Component c;
  c.id = "motor_controller";
  c.name = "motor_controller";
  c.namespace_path = "/powertrain";
  c.fqn = "/powertrain/motor_controller";

  auto j = c.to_json();
  ASSERT_TRUE(j.contains("x-medkit"));
  // Existing consumers must not see a new "identity" key for an identity-less component.
  EXPECT_FALSE(j["x-medkit"].contains("identity"));
  // Existing fields untouched.
  EXPECT_EQ(j["x-medkit"]["fqn"], "/powertrain/motor_controller");
}

TEST(ComponentIdentityCompat, IdentityEmittedUnderXMedkitWhenPresent) {
  Component c;
  c.id = "plc_1";
  c.identity.manufacturer = "Siemens";
  c.identity.serial_number = "SN-42";
  c.identity.provenance["manufacturer"] = "manifest";

  auto j = c.to_json();
  ASSERT_TRUE(j["x-medkit"].contains("identity"));
  EXPECT_EQ(j["x-medkit"]["identity"]["manufacturer"], "Siemens");
  EXPECT_EQ(j["x-medkit"]["identity"]["serialNumber"], "SN-42");
  EXPECT_EQ(j["x-medkit"]["identity"]["_provenance"]["manufacturer"], "manifest");
}

// ---------------------------------------------------------------------------
// merge_identity: precedence + per-field provenance + extensible map
// ---------------------------------------------------------------------------

TEST(MergeIdentity, MergesTwoSourcesWithProvenancePerField) {
  IdentityMergeConfig cfg;  // default: opcua outranks manifest

  // Base: manifest knows manufacturer + role.
  AssetIdentity merged;
  merged.manufacturer = "Siemens";
  merged.role = "plc";
  stamp_identity_provenance(merged, "manifest");

  // Incoming: opcua nameplate read fills serial + firmware.
  AssetIdentity opcua;
  opcua.serial_number = "SN-42";
  opcua.firmware_version = "2.9.4";
  merge_identity(merged, opcua, "opcua", cfg);

  EXPECT_EQ(merged.manufacturer, "Siemens");
  EXPECT_EQ(merged.role, "plc");
  EXPECT_EQ(merged.serial_number, "SN-42");
  EXPECT_EQ(merged.firmware_version, "2.9.4");

  // Provenance records which source set each field.
  EXPECT_EQ(merged.provenance.at("manufacturer"), "manifest");
  EXPECT_EQ(merged.provenance.at("role"), "manifest");
  EXPECT_EQ(merged.provenance.at("serial_number"), "opcua");
  EXPECT_EQ(merged.provenance.at("firmware_version"), "opcua");
}

TEST(MergeIdentity, HigherAuthoritySourceOverridesLowerForSameField) {
  IdentityMergeConfig cfg;

  AssetIdentity merged;
  merged.manufacturer = "GuessedVendor";
  stamp_identity_provenance(merged, "manifest");

  // opcua outranks manifest -> overrides the manufacturer.
  AssetIdentity opcua;
  opcua.manufacturer = "Siemens AG";
  merge_identity(merged, opcua, "opcua", cfg);

  EXPECT_EQ(merged.manufacturer, "Siemens AG");
  EXPECT_EQ(merged.provenance.at("manufacturer"), "opcua");
}

TEST(MergeIdentity, LowerAuthoritySourceDoesNotOverrideExistingField) {
  IdentityMergeConfig cfg;

  AssetIdentity merged;
  merged.manufacturer = "Siemens AG";
  stamp_identity_provenance(merged, "opcua");

  // manifest is lower authority -> must not override opcua's value, but still fills gaps.
  AssetIdentity manifest;
  manifest.manufacturer = "Manifest Vendor";
  manifest.role = "plc";
  merge_identity(merged, manifest, "manifest", cfg);

  EXPECT_EQ(merged.manufacturer, "Siemens AG");
  EXPECT_EQ(merged.provenance.at("manufacturer"), "opcua");
  EXPECT_EQ(merged.role, "plc");
  EXPECT_EQ(merged.provenance.at("role"), "manifest");
}

TEST(MergeIdentity, ExtensibleMapMergedWithProvenanceAndPrecedence) {
  IdentityMergeConfig cfg;

  AssetIdentity merged;
  merged.extra["slot"] = "3";
  merged.extra["asset_tag"] = "OLD-TAG";
  stamp_identity_provenance(merged, "manifest");

  AssetIdentity opcua;
  opcua.extra["asset_tag"] = "NEW-TAG";      // higher authority overrides
  opcua.extra["mac"] = "00:11:22:33:44:55";  // new key fills gap
  merge_identity(merged, opcua, "opcua", cfg);

  EXPECT_EQ(merged.extra.at("slot"), "3");
  EXPECT_EQ(merged.extra.at("asset_tag"), "NEW-TAG");
  EXPECT_EQ(merged.extra.at("mac"), "00:11:22:33:44:55");
  EXPECT_EQ(merged.provenance.at("extra.slot"), "manifest");
  EXPECT_EQ(merged.provenance.at("extra.asset_tag"), "opcua");
  EXPECT_EQ(merged.provenance.at("extra.mac"), "opcua");
}

TEST(MergeIdentity, EmptyIncomingNeverOverwrites) {
  IdentityMergeConfig cfg;
  AssetIdentity merged;
  merged.serial_number = "SN-42";
  stamp_identity_provenance(merged, "opcua");

  AssetIdentity empty_incoming;  // higher authority but empty
  empty_incoming.role = "";
  merge_identity(merged, empty_incoming, "s7", cfg);

  EXPECT_EQ(merged.serial_number, "SN-42");
  EXPECT_EQ(merged.provenance.at("serial_number"), "opcua");
}

TEST(MergeIdentity, UnknownSourceFillsGapsButDoesNotOverrideKnown) {
  IdentityMergeConfig cfg;
  AssetIdentity merged;
  merged.manufacturer = "Siemens";
  stamp_identity_provenance(merged, "manifest");

  AssetIdentity unknown;
  unknown.manufacturer = "Whatever";  // unknown source must not beat a listed source
  unknown.serial_number = "SN-1";     // but fills an empty field
  merge_identity(merged, unknown, "some_unlisted_source", cfg);

  EXPECT_EQ(merged.manufacturer, "Siemens");
  EXPECT_EQ(merged.provenance.at("manufacturer"), "manifest");
  EXPECT_EQ(merged.serial_number, "SN-1");
  EXPECT_EQ(merged.provenance.at("serial_number"), "some_unlisted_source");
}

TEST(MergeIdentity, ConfigurablePrecedenceOrder) {
  // Flip the default so manifest outranks opcua.
  IdentityMergeConfig cfg;
  cfg.source_precedence = {"manifest", "opcua"};

  AssetIdentity merged;
  merged.manufacturer = "ManifestVendor";
  stamp_identity_provenance(merged, "manifest");

  AssetIdentity opcua;
  opcua.manufacturer = "OpcuaVendor";
  merge_identity(merged, opcua, "opcua", cfg);

  EXPECT_EQ(merged.manufacturer, "ManifestVendor");
  EXPECT_EQ(merged.provenance.at("manufacturer"), "manifest");
}

TEST(MergeIdentity, PreservesIncomingFieldProvenanceOnGapFill) {
  // A relaying source (low authority, e.g. a peer) carries a field it originally
  // read over "opcua". The gap-filled field must keep provenance "opcua", not be
  // re-stamped with the low-authority relay tag, so the recorded origin matches a
  // wholesale copy of the same source.
  IdentityMergeConfig cfg;

  AssetIdentity merged;
  merged.manufacturer = "Siemens";
  stamp_identity_provenance(merged, "manifest");

  AssetIdentity relayed;
  relayed.serial_number = "SN-42";
  relayed.provenance["serial_number"] = "opcua";  // origin the relay learned

  merge_identity(merged, relayed, "peer:robot_a", cfg);

  EXPECT_EQ(merged.serial_number, "SN-42");
  EXPECT_EQ(merged.provenance.at("serial_number"), "opcua");
  // The relay's low authority still protects the locally-known field.
  EXPECT_EQ(merged.manufacturer, "Siemens");
  EXPECT_EQ(merged.provenance.at("manufacturer"), "manifest");
}

TEST(MergeIdentity, FallsBackToSourceNameWhenIncomingHasNoProvenance) {
  // No provenance on the incoming source -> record the authority tag it merged under.
  IdentityMergeConfig cfg;

  AssetIdentity merged;
  AssetIdentity incoming;
  incoming.serial_number = "SN-9";  // no provenance entry
  merge_identity(merged, incoming, "opcua", cfg);

  EXPECT_EQ(merged.serial_number, "SN-9");
  EXPECT_EQ(merged.provenance.at("serial_number"), "opcua");
}

TEST(MergeIdentity, PreservesIncomingExtraProvenanceOnGapFill) {
  IdentityMergeConfig cfg;

  AssetIdentity merged;
  AssetIdentity relayed;
  relayed.extra["mac"] = "00:11:22:33:44:55";
  relayed.provenance["extra.mac"] = "ethernet_ip";

  merge_identity(merged, relayed, "peer:robot_a", cfg);

  EXPECT_EQ(merged.extra.at("mac"), "00:11:22:33:44:55");
  EXPECT_EQ(merged.provenance.at("extra.mac"), "ethernet_ip");
}

TEST(MergeIdentity, KnownSourceOverridesUnseededTarget) {
  // Documented footgun: a target whose fields were set WITHOUT stamping provenance is
  // treated as owned by an unknown (lowest authority) source, so any listed source
  // overrides it. This guards that contract.
  IdentityMergeConfig cfg;

  AssetIdentity target;
  target.manufacturer = "UnstampedGuess";  // value present, provenance NOT seeded

  AssetIdentity opcua;
  opcua.manufacturer = "Siemens AG";  // listed (known) source
  merge_identity(target, opcua, "opcua", cfg);

  EXPECT_EQ(target.manufacturer, "Siemens AG");
  EXPECT_EQ(target.provenance.at("manufacturer"), "opcua");
}
