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

// @verifies REQ_INTEROP_UPDATE_PROVIDER_NOTIFY

// End-to-end integration test for the plugin-driven entity-surface refresh
// introduced in plugin API v7. Drives a real GatewayNode with:
//   * HYBRID discovery + a tiny base manifest on disk
//   * a fragments_dir pointing at a tmp directory
// then walks the full lifecycle:
//   1. drop a fragment into the dir
//   2. call PluginContext::notify_entities_changed
//   3. assert the new app is visible via the ManifestManager
//   4. remove the fragment + notify again
//   5. assert the app is gone

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/plugins/entity_change_scope.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"

using namespace std::chrono_literals;

namespace {

constexpr const char * kBaseManifest = R"(
manifest_version: "1.0"
metadata:
  name: notify-integration
  version: "0.0.1"
areas:
  - id: vehicle
    name: Test vehicle
components:
  - id: ecu-primary
    name: Primary ECU
    area: vehicle
)";

class NotifyIntegrationTest : public ::testing::Test {
 protected:
  std::filesystem::path work_dir;
  std::filesystem::path manifest_path;
  std::filesystem::path fragments_dir;
  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node;

  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    auto tmp = std::filesystem::temp_directory_path();
    work_dir = tmp / ("medkit-notify-integ-" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(work_dir);
    fragments_dir = work_dir / "fragments";
    std::filesystem::create_directories(fragments_dir);
    manifest_path = work_dir / "manifest.yaml";
    std::ofstream(manifest_path) << kBaseManifest;

    // Start the GatewayNode with the base manifest + fragments_dir wired up.
    // discovery.mode=hybrid to exercise the manifest-load path. runtime
    // discovery is disabled so the only source of apps is manifest + fragments.
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"discovery.mode", "hybrid"},
        {"discovery.manifest_path", manifest_path.string()},
        {"discovery.manifest.enabled", true},
        {"discovery.manifest_strict_validation", false},
        {"discovery.manifest.fragments_dir", fragments_dir.string()},
        {"discovery.runtime.enabled", false},
        {"discovery.runtime.default_component.enabled", false},
        {"server.enabled", false},  // skip HTTP server setup
    });
    node = std::make_shared<ros2_medkit_gateway::GatewayNode>(opts);
  }

  void TearDown() override {
    node.reset();
    std::error_code ec;
    std::filesystem::remove_all(work_dir, ec);
  }

  void write_fragment(const std::string & name, const std::string & body) {
    std::ofstream(fragments_dir / name) << body;
  }

  bool manifest_has_app(const std::string & id) {
    auto * dm = node->get_discovery_manager();
    if (!dm) return false;
    auto * mm = dm->get_manifest_manager();
    if (!mm) return false;
    for (const auto & app : mm->get_apps()) {
      if (app.id == id) return true;
    }
    return false;
  }
};

}  // namespace

TEST_F(NotifyIntegrationTest, FragmentAddedAfterStartupBecomesVisibleOnNotify) {
  ASSERT_FALSE(manifest_has_app("lateApp"));

  write_fragment("late.yaml", R"(
apps:
  - id: lateApp
    name: Late
    is_located_on: ecu-primary
    ros_binding:
      node_name: lateApp
)");

  // Naive fragment write alone doesn't mutate the in-memory manifest.
  EXPECT_FALSE(manifest_has_app("lateApp")) << "fragment should not be visible before notify";

  // Any PluginContext method goes through the same make_gateway_plugin_context
  // that production plugins see.
  auto ctx =
      ros2_medkit_gateway::make_gateway_plugin_context(node.get(), node->get_fault_manager(),
                                                        nullptr);
  ctx->notify_entities_changed(
      ros2_medkit_gateway::EntityChangeScope::for_component("ecu-primary"));

  EXPECT_TRUE(manifest_has_app("lateApp"));
}

TEST_F(NotifyIntegrationTest, FragmentRemovedAfterStartupDisappearsOnNotify) {
  write_fragment("temp.yaml", R"(
apps:
  - id: tempApp
    name: Temp
    is_located_on: ecu-primary
    ros_binding:
      node_name: tempApp
)");

  auto ctx =
      ros2_medkit_gateway::make_gateway_plugin_context(node.get(), node->get_fault_manager(),
                                                        nullptr);
  ctx->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::full_refresh());
  ASSERT_TRUE(manifest_has_app("tempApp"));

  std::filesystem::remove(fragments_dir / "temp.yaml");
  ctx->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::full_refresh());
  EXPECT_FALSE(manifest_has_app("tempApp"));
}

TEST_F(NotifyIntegrationTest, NotifyWithoutAnyFragmentIsANoOp) {
  // No fragments dropped. Notify must be safe and must not erase the
  // base-manifest entities.
  auto ctx =
      ros2_medkit_gateway::make_gateway_plugin_context(node.get(), node->get_fault_manager(),
                                                        nullptr);
  EXPECT_NO_THROW(ctx->notify_entities_changed(
      ros2_medkit_gateway::EntityChangeScope::full_refresh()));

  auto * mm = node->get_discovery_manager()->get_manifest_manager();
  ASSERT_NE(mm, nullptr);
  auto comps = mm->get_components();
  auto it = std::find_if(comps.begin(), comps.end(),
                         [](const auto & c) { return c.id == "ecu-primary"; });
  EXPECT_NE(it, comps.end()) << "base manifest entity lost after notify";
}
