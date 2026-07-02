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

#include "ros2_medkit_gateway/core/discovery/discovery_layer.hpp"
#include "ros2_medkit_gateway/core/discovery/manifest/asset_inventory.hpp"
#include "ros2_medkit_gateway/core/discovery/manifest/manifest_parser.hpp"
#include "ros2_medkit_gateway/core/discovery/merge_types.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"
#include "ros2_medkit_gateway/discovery/merge_pipeline.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ros2_medkit_gateway;             // NOLINT(build/namespaces)
using namespace ros2_medkit_gateway::discovery;  // NOLINT(build/namespaces)

namespace {

namespace fs = std::filesystem;

// Find a component by id in a vector (returns nullptr when absent).
const Component * find_component(const std::vector<Component> & comps, const std::string & id) {
  auto it = std::find_if(comps.begin(), comps.end(), [&](const Component & c) {
    return c.id == id;
  });
  return it == comps.end() ? nullptr : &*it;
}

bool has_tag(const Component & comp, const std::string & tag) {
  return std::find(comp.tags.begin(), comp.tags.end(), tag) != comp.tags.end();
}

// Minimal discovery layer double for exercising the real MergePipeline.
class FakeLayer : public DiscoveryLayer {
 public:
  FakeLayer(std::string name, LayerOutput output, std::unordered_map<FieldGroup, MergePolicy> policies)
    : name_(std::move(name)), output_(std::move(output)), policies_(std::move(policies)) {
  }

  std::string name() const override {
    return name_;
  }
  LayerOutput discover() override {
    return output_;
  }
  MergePolicy policy_for(FieldGroup group) const override {
    auto it = policies_.find(group);
    return it == policies_.end() ? MergePolicy::ENRICHMENT : it->second;
  }

 private:
  std::string name_;
  LayerOutput output_;
  std::unordered_map<FieldGroup, MergePolicy> policies_;
};

}  // namespace

// ── CSV parsing ─────────────────────────────────────────────────────────────

TEST(AssetCsvParseTest, BasicHeaderAndRows) {
  const std::string csv =
      "id,manufacturer,model,serial,hardware_rev,firmware,endpoint,role\n"
      "plc_1,Siemens,S7-1500,SN123,A2,2.9.1,opc.tcp://10.0.0.5:4840,controller\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  const auto & e = result.entries[0];
  EXPECT_EQ(e.id, "plc_1");
  EXPECT_EQ(e.manufacturer, "Siemens");
  EXPECT_EQ(e.model, "S7-1500");
  EXPECT_EQ(e.serial, "SN123");
  EXPECT_EQ(e.hardware_rev, "A2");
  EXPECT_EQ(e.firmware, "2.9.1");
  EXPECT_EQ(e.endpoint, "opc.tcp://10.0.0.5:4840");
  EXPECT_EQ(e.role, "controller");
  EXPECT_TRUE(result.warnings.empty());
}

TEST(AssetCsvParseTest, QuotedFieldsWithCommasAndNewlines) {
  const std::string csv =
      "id,manufacturer,model\n"
      "vfd_1,\"Acme, Inc.\",\"Model\nX\"\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].manufacturer, "Acme, Inc.");
  EXPECT_EQ(result.entries[0].model, "Model\nX");
}

TEST(AssetCsvParseTest, EscapedQuotes) {
  const std::string csv =
      "id,model\n"
      "a,\"He said \"\"hi\"\"\"\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].model, "He said \"hi\"");
}

TEST(AssetCsvParseTest, ExtraColumnsPreservedInOrder) {
  const std::string csv =
      "id,location,rack\n"
      "io_1,cell-3,R2\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  const auto & extras = result.entries[0].extras;
  ASSERT_EQ(extras.size(), 2u);
  EXPECT_EQ(extras[0].first, "location");
  EXPECT_EQ(extras[0].second, "cell-3");
  EXPECT_EQ(extras[1].first, "rack");
  EXPECT_EQ(extras[1].second, "R2");
}

TEST(AssetCsvParseTest, BlankLinesSkipped) {
  const std::string csv =
      "id,model\n"
      "\n"
      "a,X\n"
      "\n"
      "b,Y\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 2u);
  EXPECT_EQ(result.entries[0].id, "a");
  EXPECT_EQ(result.entries[1].id, "b");
}

TEST(AssetCsvParseTest, RowWithEmptyIdSkippedWithWarning) {
  const std::string csv =
      "id,model\n"
      ",Orphan\n"
      "keep,Good\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].id, "keep");
  EXPECT_EQ(result.warnings.size(), 1u);
}

TEST(AssetCsvParseTest, MissingIdColumnThrows) {
  const std::string csv = "model,serial\nX,Y\n";
  EXPECT_THROW(parse_asset_csv(csv), std::runtime_error);
}

TEST(AssetCsvParseTest, EmptyDocumentThrows) {
  EXPECT_THROW(parse_asset_csv(""), std::runtime_error);
  EXPECT_THROW(parse_asset_csv("   \n\n"), std::runtime_error);
}

TEST(AssetCsvParseTest, CaseInsensitiveHeadersAndAliases) {
  const std::string csv =
      "ID,Manufacturer,Serial_Number,Firmware_Version,HW_Rev\n"
      "x,Bosch,SER-9,1.4.0,C1\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  const auto & e = result.entries[0];
  EXPECT_EQ(e.id, "x");
  EXPECT_EQ(e.manufacturer, "Bosch");
  EXPECT_EQ(e.serial, "SER-9");
  EXPECT_EQ(e.firmware, "1.4.0");
  EXPECT_EQ(e.hardware_rev, "C1");
}

TEST(AssetCsvParseTest, CrlfLineEndingsAndTrimming) {
  const std::string csv =
      "id, model \r\n"
      "  a , X \r\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].id, "a");
  EXPECT_EQ(result.entries[0].model, "X");
}

TEST(AssetCsvParseTest, FewerColumnsThanHeaderLeavesFieldsEmpty) {
  const std::string csv =
      "id,model,serial\n"
      "a,ModelOnly\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].model, "ModelOnly");
  EXPECT_TRUE(result.entries[0].serial.empty());
}

TEST(AssetCsvParseTest, Utf8BomStrippedFromHeader) {
  // Excel "CSV UTF-8" export prepends a BOM; the first header cell must still be
  // recognized as 'id' rather than "﻿id" (which would fail id detection).
  const std::string csv =
      "\xEF\xBB\xBF"
      "id,model\n"
      "plc_1,S7-1500\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].id, "plc_1");
  EXPECT_EQ(result.entries[0].model, "S7-1500");
}

TEST(AssetCsvParseTest, LeadingSpaceBeforeQuotedFieldOpensQuote) {
  // A quote after leading unquoted whitespace must open a quoted field, not be
  // treated as a literal char (which would corrupt the value and add a column).
  const std::string csv =
      "id,model\n"
      "a, \"Quoted, X\"\n";
  auto result = parse_asset_csv(csv);
  ASSERT_EQ(result.entries.size(), 1u);
  EXPECT_EQ(result.entries[0].id, "a");
  EXPECT_EQ(result.entries[0].model, "Quoted, X");
}

// ── AssetEntry -> Component mapping ──────────────────────────────────────────

TEST(AssetEntryToComponentTest, FullMapping) {
  AssetEntry e;
  e.id = "plc_1";
  e.manufacturer = "Siemens";
  e.model = "S7-1500";
  e.serial = "SN123";
  e.hardware_rev = "A2";
  e.firmware = "2.9.1";
  e.endpoint = "opc.tcp://10.0.0.5:4840";
  e.role = "controller";
  e.extras.emplace_back("location", "cell-3");

  Component comp = asset_entry_to_component(e);
  EXPECT_EQ(comp.id, "plc_1");
  EXPECT_EQ(comp.source, "inventory");
  // A bare inventory asset declares no placement, so fqn/namespace stay empty
  // and never override the real path of a discovered node it merges with.
  EXPECT_TRUE(comp.fqn.empty());
  EXPECT_TRUE(comp.namespace_path.empty());
  EXPECT_EQ(comp.name, "Siemens S7-1500");
  // Every canonical column lands on the structured identity with per-field
  // provenance "inventory" (not folded into description / variant / tags).
  EXPECT_EQ(comp.identity.manufacturer, "Siemens");
  EXPECT_EQ(comp.identity.model, "S7-1500");
  EXPECT_EQ(comp.identity.serial_number, "SN123");
  EXPECT_EQ(comp.identity.hardware_revision, "A2");
  EXPECT_EQ(comp.identity.firmware_version, "2.9.1");
  EXPECT_EQ(comp.identity.network_endpoint, "opc.tcp://10.0.0.5:4840");
  EXPECT_EQ(comp.identity.role, "controller");
  EXPECT_EQ(comp.identity.extra.at("location"), "cell-3");
  EXPECT_EQ(comp.identity.provenance.at("manufacturer"), "inventory");
  EXPECT_EQ(comp.identity.provenance.at("serial_number"), "inventory");
  EXPECT_EQ(comp.identity.provenance.at("extra.location"), "inventory");
  EXPECT_TRUE(comp.variant.empty());
  EXPECT_TRUE(comp.tags.empty());
  EXPECT_TRUE(comp.description.empty());
}

TEST(AssetEntryToComponentTest, MinimalEntryOnlyId) {
  AssetEntry e;
  e.id = "bare";
  Component comp = asset_entry_to_component(e);
  EXPECT_EQ(comp.id, "bare");
  EXPECT_EQ(comp.source, "inventory");
  EXPECT_TRUE(comp.name.empty());
  EXPECT_TRUE(comp.description.empty());
  EXPECT_TRUE(comp.variant.empty());
  EXPECT_TRUE(comp.tags.empty());
  EXPECT_TRUE(comp.identity.empty());
}

TEST(AssetEntryToComponentTest, NameFallsBackToManufacturerWhenNoModel) {
  AssetEntry e;
  e.id = "m";
  e.manufacturer = "OnlyVendor";
  Component comp = asset_entry_to_component(e);
  EXPECT_EQ(comp.name, "OnlyVendor");
}

// ── Manifest `assets:` list ──────────────────────────────────────────────────

TEST(ManifestAssetsTest, ParsesAssetsIntoComponents) {
  const std::string yaml = R"(
manifest_version: "1.0"
assets:
  - id: robot_arm
    manufacturer: KUKA
    model: KR-6
    serial: KS-77
    hardware_rev: R3
    firmware: 8.6
    role: actuator
    location: line-B
)";
  ManifestParser parser;
  Manifest manifest = parser.parse_string(yaml);
  const Component * comp = find_component(manifest.components, "robot_arm");
  ASSERT_NE(comp, nullptr);
  EXPECT_EQ(comp->source, "inventory");
  EXPECT_EQ(comp->name, "KUKA KR-6");
  EXPECT_EQ(comp->identity.manufacturer, "KUKA");
  EXPECT_EQ(comp->identity.model, "KR-6");
  EXPECT_EQ(comp->identity.serial_number, "KS-77");
  EXPECT_EQ(comp->identity.hardware_revision, "R3");
  EXPECT_EQ(comp->identity.firmware_version, "8.6");
  EXPECT_EQ(comp->identity.role, "actuator");
  EXPECT_EQ(comp->identity.extra.at("location"), "line-B");
  EXPECT_EQ(comp->identity.provenance.at("model"), "inventory");
}

TEST(ManifestAssetsTest, AreaAndExplicitOverrides) {
  const std::string yaml = R"(
manifest_version: "1.0"
areas:
  - id: floor
    name: Floor
assets:
  - id: pump_2
    manufacturer: Grundfos
    model: CR-5
    area: floor
    name: Main Pump
    tags: [hydraulics]
)";
  ManifestParser parser;
  Manifest manifest = parser.parse_string(yaml);
  const Component * comp = find_component(manifest.components, "pump_2");
  ASSERT_NE(comp, nullptr);
  EXPECT_EQ(comp->area, "floor");
  EXPECT_EQ(comp->name, "Main Pump");  // explicit override wins over derived
  EXPECT_TRUE(has_tag(*comp, "hydraulics"));
}

TEST(ManifestAssetsTest, NamespaceAndReservedKeysApplied) {
  // namespace/variant/type/translation_id/depends_on were previously listed as
  // reserved and then silently dropped; each must now land on the Component.
  const std::string yaml = R"(
manifest_version: "1.0"
assets:
  - id: drive_1
    manufacturer: ABB
    model: ACS880
    hardware_rev: R1
    namespace: /plant
    variant: R2
    type: actuator
    translation_id: entity.drive
    depends_on: [plc_1, bus_a]
)";
  ManifestParser parser;
  Manifest manifest = parser.parse_string(yaml);
  const Component * comp = find_component(manifest.components, "drive_1");
  ASSERT_NE(comp, nullptr);
  EXPECT_EQ(comp->namespace_path, "/plant");
  EXPECT_EQ(comp->fqn, "/plant/drive_1");             // operator-declared placement
  EXPECT_EQ(comp->variant, "R2");                     // explicit variant only
  EXPECT_EQ(comp->identity.hardware_revision, "R1");  // hardware_rev lives on the identity
  EXPECT_EQ(comp->type, "actuator");
  EXPECT_EQ(comp->translation_id, "entity.drive");
  ASSERT_EQ(comp->depends_on.size(), 2u);
  EXPECT_EQ(comp->depends_on[0], "plc_1");
  EXPECT_EQ(comp->depends_on[1], "bus_a");
}

// ── ManifestManager CSV import ───────────────────────────────────────────────

class InventoryCsvManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    temp_dir_ = fs::temp_directory_path() / "asset_inventory_test";
    fs::create_directories(temp_dir_);
  }
  void TearDown() override {
    fs::remove_all(temp_dir_);
  }
  std::string write_file(const std::string & name, const std::string & content) {
    fs::path p = temp_dir_ / name;
    std::ofstream ofs(p);
    ofs << content;
    ofs.close();
    return p.string();
  }
  fs::path temp_dir_;
};

TEST_F(InventoryCsvManagerTest, LoadsAssetsFromCsvPath) {
  const std::string manifest_path = write_file("base.yaml", "manifest_version: \"1.0\"\n");
  const std::string csv_path = write_file("inventory.csv",
                                          "id,manufacturer,model,role\n"
                                          "gw_1,Selfpatch,DiagBox,gateway\n");

  ManifestManager mgr(nullptr);
  mgr.set_inventory_csv_path(csv_path);
  ASSERT_TRUE(mgr.load_manifest(manifest_path, /*strict=*/false));

  auto comps = mgr.get_components();
  const Component * comp = find_component(comps, "gw_1");
  ASSERT_NE(comp, nullptr);
  EXPECT_EQ(comp->source, "inventory");
  EXPECT_EQ(comp->name, "Selfpatch DiagBox");
  EXPECT_EQ(comp->identity.role, "gateway");
  EXPECT_EQ(mgr.get_inventory_csv_path(), csv_path);
}

TEST_F(InventoryCsvManagerTest, MalformedCsvFailsLoad) {
  const std::string manifest_path = write_file("base.yaml", "manifest_version: \"1.0\"\n");
  const std::string csv_path = write_file("bad.csv", "model,serial\nX,Y\n");  // no id column

  ManifestManager mgr(nullptr);
  mgr.set_inventory_csv_path(csv_path);
  EXPECT_FALSE(mgr.load_manifest(manifest_path, /*strict=*/false));
  EXPECT_TRUE(mgr.get_validation_result().has_errors());
}

TEST_F(InventoryCsvManagerTest, MissingCsvIsNotAnError) {
  const std::string manifest_path = write_file("base.yaml", "manifest_version: \"1.0\"\n");
  ManifestManager mgr(nullptr);
  mgr.set_inventory_csv_path((temp_dir_ / "does_not_exist.csv").string());
  EXPECT_TRUE(mgr.load_manifest(manifest_path, /*strict=*/false));
}

// ── Acceptance: CSV asset merges with protocol-discovered structure ──────────

TEST(InventoryMergeTest, AssetMergesWithRuntimeComponentById) {
  // Inventory (manifest-side) asset: identity, no live data, no placement.
  auto result = parse_asset_csv(
      "id,manufacturer,model,serial,role\n"
      "drive_1,ABB,ACS880,SN-42,drive\n");
  ASSERT_EQ(result.entries.size(), 1u);

  LayerOutput inventory_out;
  inventory_out.components.push_back(asset_entry_to_component(result.entries[0]));

  // Protocol-discovered component with the SAME id, carrying live topics and
  // its own (lower-authority) identity guesses: a conflicting manufacturer that
  // must lose to the hand-authored inventory, and a firmware version only the
  // runtime knows, which must fill the gap.
  Component runtime_comp;
  runtime_comp.id = "drive_1";
  runtime_comp.name = "drive_1";
  runtime_comp.source = "node";
  runtime_comp.namespace_path = "/plant";
  runtime_comp.fqn = "/plant/drive_1";
  runtime_comp.topics.publishes = {"/plant/drive_1/status"};
  runtime_comp.identity.manufacturer = "guessed-vendor";
  runtime_comp.identity.firmware_version = "fw-2.1.0";

  LayerOutput runtime_out;
  runtime_out.components.push_back(runtime_comp);

  // Drive the merge with the SAME policies the production layers use
  // (ManifestLayer / RuntimeLayer), so the test exercises the real hierarchy
  // precedence instead of the default ENRICHMENT.
  MergePipeline pipeline(rclcpp::get_logger("test_asset_inventory"));
  pipeline.add_layer(std::make_unique<FakeLayer>(
      "inventory", inventory_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::HIERARCHY, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::ENRICHMENT},
                                                  {FieldGroup::STATUS, MergePolicy::FALLBACK},
                                                  {FieldGroup::METADATA, MergePolicy::AUTHORITATIVE}}));
  pipeline.add_layer(std::make_unique<FakeLayer>(
      "runtime", runtime_out,
      std::unordered_map<FieldGroup, MergePolicy>{{FieldGroup::IDENTITY, MergePolicy::FALLBACK},
                                                  {FieldGroup::HIERARCHY, MergePolicy::FALLBACK},
                                                  {FieldGroup::LIVE_DATA, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::STATUS, MergePolicy::AUTHORITATIVE},
                                                  {FieldGroup::METADATA, MergePolicy::ENRICHMENT}}));

  auto merged = pipeline.execute();
  ASSERT_EQ(merged.components.size(), 1u);
  const Component & c = merged.components[0];
  EXPECT_EQ(c.id, "drive_1");
  EXPECT_EQ(c.source, "inventory");          // provenance preserved
  EXPECT_EQ(c.name, "ABB ACS880");           // inventory identity wins
  ASSERT_EQ(c.topics.publishes.size(), 1u);  // live data from the protocol layer
  EXPECT_EQ(c.topics.publishes[0], "/plant/drive_1/status");
  // The bare inventory asset carries no placement, so the discovered node's real
  // path must survive the merge - it must NOT be blanked or forced to "/drive_1".
  EXPECT_EQ(c.namespace_path, "/plant");
  EXPECT_EQ(c.fqn, "/plant/drive_1");
  // Identity merges per field with provenance: the hand-authored inventory
  // ("inventory", above "node" in the default precedence) keeps the fields it
  // declares; the runtime's conflicting manufacturer guess loses, while its
  // runtime-only firmware version fills the gap and is attributed to "node".
  EXPECT_EQ(c.identity.manufacturer, "ABB");
  EXPECT_EQ(c.identity.model, "ACS880");
  EXPECT_EQ(c.identity.serial_number, "SN-42");
  EXPECT_EQ(c.identity.role, "drive");
  EXPECT_EQ(c.identity.firmware_version, "fw-2.1.0");
  EXPECT_EQ(c.identity.provenance.at("manufacturer"), "inventory");
  EXPECT_EQ(c.identity.provenance.at("serial_number"), "inventory");
  EXPECT_EQ(c.identity.provenance.at("firmware_version"), "node");
}
