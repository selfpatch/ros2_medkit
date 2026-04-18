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

// @verifies REQ_INTEROP_MANIFEST_FRAGMENTS

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"

using ros2_medkit_gateway::discovery::ManifestManager;

namespace {

/// Minimal valid base manifest with one area and one manifest-owned component.
/// Keeps apps empty so fragments have somewhere to contribute new entities.
constexpr const char * kBaseManifest = R"(
manifest_version: "1.0"
metadata:
  name: fragments-test
  version: "0.0.1"
areas:
  - id: vehicle
    name: Test vehicle
components:
  - id: ecu-primary
    name: Primary ECU
    area: vehicle
)";

class FragmentsFixture : public ::testing::Test {
 protected:
  std::filesystem::path base_path;
  std::filesystem::path fragments_dir;

  void SetUp() override {
    auto tmp = std::filesystem::temp_directory_path();
    std::string prefix = "medkit-fragments-" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "-";
    int suffix = 0;
    do {
      fragments_dir = tmp / (prefix + std::to_string(suffix++));
    } while (std::filesystem::exists(fragments_dir));
    std::filesystem::create_directories(fragments_dir);
    base_path = fragments_dir.parent_path() / (prefix + "base.yaml");
    std::ofstream(base_path) << kBaseManifest;
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(fragments_dir, ec);
    std::filesystem::remove(base_path, ec);
  }

  void write_fragment(const std::string & name, const std::string & body) {
    std::ofstream(fragments_dir / name) << body;
  }
};

}  // namespace

TEST_F(FragmentsFixture, NoFragmentsDirLoadsBaseOnly) {
  ManifestManager mgr;
  // Leave fragments_dir unset -> behaves like before Chunk 2.
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  EXPECT_EQ(mgr.get_apps().size(), 0u);
  EXPECT_EQ(mgr.get_components().size(), 1u);
}

TEST_F(FragmentsFixture, MissingFragmentsDirIsNotAnError) {
  ManifestManager mgr;
  mgr.set_fragments_dir((fragments_dir / "does-not-exist").string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  EXPECT_EQ(mgr.get_apps().size(), 0u);
}

TEST_F(FragmentsFixture, SingleFragmentAppIsMerged) {
  write_fragment("hello.yaml", R"(
apps:
  - id: helloApp
    name: Hello World
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto apps = mgr.get_apps();
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].id, "helloApp");
}

TEST_F(FragmentsFixture, MultipleFragmentsMergeDeterministically) {
  // Names chosen so filesystem order is unstable without sorting but becomes
  // stable after the apply_fragments sort pass.
  write_fragment("z-second.yaml", R"(
apps:
  - id: secondApp
    name: Second
    is_located_on: ecu-primary
    ros_binding:
      node_name: secondApp
)");
  write_fragment("a-first.yaml", R"(
apps:
  - id: firstApp
    name: First
    is_located_on: ecu-primary
    ros_binding:
      node_name: firstApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto apps = mgr.get_apps();
  ASSERT_EQ(apps.size(), 2u);
  // Alphabetical file order -> a-first processed before z-second.
  EXPECT_EQ(apps[0].id, "firstApp");
  EXPECT_EQ(apps[1].id, "secondApp");
}

TEST_F(FragmentsFixture, NonYamlFileIsIgnored) {
  write_fragment("ignore-me.txt", "totally not yaml");
  write_fragment("pick.yaml", R"(
apps:
  - id: pickedApp
    name: Picked
    is_located_on: ecu-primary
    ros_binding:
      node_name: pickedApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  ASSERT_EQ(mgr.get_apps().size(), 1u);
}

TEST_F(FragmentsFixture, FragmentAreasAreRejected) {
  write_fragment("bad.yaml", R"(
areas:
  - id: rogue
    name: Rogue area
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error for areas";
}

TEST_F(FragmentsFixture, FragmentMetadataDescriptionIsRejected) {
  // metadata.{description,created_at} are base-manifest-owned just like
  // metadata.{name,version} - a fragment that touches them must fail.
  write_fragment("bad.yaml", R"(
metadata:
  description: rogue fragment description
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error for metadata.description";
}

TEST_F(FragmentsFixture, FragmentMetadataCreatedAtIsRejected) {
  write_fragment("bad.yaml", R"(
metadata:
  created_at: "2026-04-17T00:00:00Z"
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error for metadata.created_at";
}

TEST_F(FragmentsFixture, FragmentDiscoveryConfigIsRejected) {
  // The top-level `discovery:` key maps to ManifestConfig which has
  // non-empty defaults, so the check relies on raw-YAML key detection.
  write_fragment("bad.yaml", R"(
discovery:
  unmanifested_nodes: error
apps:
  - id: helloApp
    name: Hello
    is_located_on: ecu-primary
    ros_binding:
      node_name: helloApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  auto vr = mgr.get_validation_result();
  bool found = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD" && err.message.find("discovery") != std::string::npos) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "expected FRAGMENT_FORBIDDEN_FIELD error naming 'discovery'";
}

TEST_F(FragmentsFixture, DuplicateIdsAcrossFragmentsFailValidation) {
  write_fragment("a.yaml", R"(
apps:
  - id: clashApp
    name: A
    is_located_on: ecu-primary
    ros_binding:
      node_name: clashA
)");
  write_fragment("b.yaml", R"(
apps:
  - id: clashApp
    name: B
    is_located_on: ecu-primary
    ros_binding:
      node_name: clashB
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_FALSE(mgr.load_manifest(base_path.string(), /*strict=*/false));
}

TEST_F(FragmentsFixture, ReloadPicksUpNewFragments) {
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  ASSERT_EQ(mgr.get_apps().size(), 0u);
  // Drop a fragment after initial load and reload.
  write_fragment("late.yaml", R"(
apps:
  - id: lateApp
    name: Late
    is_located_on: ecu-primary
    ros_binding:
      node_name: lateApp
)");
  ASSERT_TRUE(mgr.reload_manifest());
  auto apps = mgr.get_apps();
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].id, "lateApp");
}

TEST_F(FragmentsFixture, ReloadPicksUpRemovedFragments) {
  write_fragment("temp.yaml", R"(
apps:
  - id: tempApp
    name: Temp
    is_located_on: ecu-primary
    ros_binding:
      node_name: tempApp
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false));
  ASSERT_EQ(mgr.get_apps().size(), 1u);

  std::filesystem::remove(fragments_dir / "temp.yaml");
  ASSERT_TRUE(mgr.reload_manifest());
  EXPECT_EQ(mgr.get_apps().size(), 0u);
}

TEST_F(FragmentsFixture, NestedKeyNamedLikeManifestVersionDoesNotBlockInjection) {
  // parse_fragment_file auto-injects `manifest_version: "1.0"` when the
  // fragment omits it. The detector that decides whether to inject scans
  // the raw YAML line-by-line; earlier revisions matched any line whose
  // first non-whitespace token was `manifest_version:`, meaning an
  // indented key of the same name anywhere in the fragment would disable
  // injection and cause a "Missing required field: manifest_version"
  // parse error - even though the fragment itself has no top-level
  // version. Fragments written by plugins that happen to use the word
  // in a nested position (e.g., inside custom metadata blocks) would
  // silently fail to merge.
  //
  // Pin the rule: only a column-0 `manifest_version:` disables injection.
  // Sub-map key `manifest_version:` inside `ros_binding:`. The schema
  // ignores unknown keys on ros_binding so the fragment is otherwise valid,
  // but the pre-fix line scanner matches the indented key and aborts the
  // synthetic version injection.
  write_fragment("nested.yaml", R"(
apps:
  - id: nestedApp
    name: Nested Key Test
    is_located_on: ecu-primary
    ros_binding:
      node_name: nestedApp
      manifest_version: "ignored-by-schema"
)");
  ManifestManager mgr;
  mgr.set_fragments_dir(fragments_dir.string());
  ASSERT_TRUE(mgr.load_manifest(base_path.string(), /*strict=*/false))
      << "injection should still happen when `manifest_version:` appears only as an indented value / list item";
  auto apps = mgr.get_apps();
  bool found = false;
  for (const auto & a : apps) {
    if (a.id == "nestedApp") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "nestedApp fragment should merge even though a component depends_on list mentions "
                        "'manifest_version'";
}

TEST_F(FragmentsFixture, SetFragmentsDirRoundTrip) {
  ManifestManager mgr;
  EXPECT_EQ(mgr.get_fragments_dir(), "");
  mgr.set_fragments_dir(fragments_dir.string());
  EXPECT_EQ(mgr.get_fragments_dir(), fragments_dir.string());
  mgr.set_fragments_dir("");
  EXPECT_EQ(mgr.get_fragments_dir(), "");
}
