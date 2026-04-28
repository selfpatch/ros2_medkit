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

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_medkit_gateway/core/plugins/entity_change_scope.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using namespace std::chrono_literals;

namespace {

// Reserve a free loopback TCP port for the test-local HTTP server so parallel
// gtests do not collide on :8080 (GatewayNode unconditionally starts its REST
// server on the configured port).
int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ADD_FAILURE() << "Failed to create socket for test port reservation: " << std::strerror(errno);
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ADD_FAILURE() << "Failed to bind socket for test port reservation: " << std::strerror(errno);
    close(sock);
    return 0;
  }
  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    ADD_FAILURE() << "Failed to inspect reserved test port: " << std::strerror(errno);
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

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
    // Name the work dir from test name + pid so concurrent test-binary
    // invocations (ctest -jN, or reruns after an aborted run) cannot collide.
    // gtest's random_seed() is fixed for the entire process, so it wouldn't
    // disambiguate between test cases in this fixture.
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::string name = info ? std::string(info->name()) : std::string("unknown");
    work_dir = tmp / ("medkit-notify-integ-" + std::to_string(::getpid()) + "-" + name);
    std::error_code rm_ec;
    std::filesystem::remove_all(work_dir, rm_ec);
    std::filesystem::create_directories(work_dir);
    fragments_dir = work_dir / "fragments";
    std::filesystem::create_directories(fragments_dir);
    manifest_path = work_dir / "manifest.yaml";
    std::ofstream(manifest_path) << kBaseManifest;

    // Start the GatewayNode with the base manifest + fragments_dir wired up.
    // discovery.mode=hybrid to exercise the manifest-load path. runtime
    // discovery is disabled so the only source of apps is manifest + fragments.
    // Reserve a free loopback port per test instance - GatewayNode starts its
    // REST server unconditionally, so we cannot share :8080 with parallel
    // gtest suites. `server.enabled` is not a real parameter; override
    // `server.port` instead (matches the convention in test_discovery_handlers,
    // test_handler_context, test_gateway_node).
    const int server_port = reserve_local_port();
    ASSERT_GT(server_port, 0);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"discovery.mode", "hybrid"},
        {"discovery.manifest_path", manifest_path.string()},
        {"discovery.manifest.enabled", true},
        {"discovery.manifest_strict_validation", false},
        {"discovery.manifest.fragments_dir", fragments_dir.string()},
        {"discovery.runtime.enabled", false},
        {"discovery.runtime.default_component.enabled", false},
        {"server.host", std::string("127.0.0.1")},
        {"server.port", server_port},
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
    if (!dm) {
      return false;
    }
    auto * mm = dm->get_manifest_manager();
    if (!mm) {
      return false;
    }
    for (const auto & app : mm->get_apps()) {
      if (app.id == id) {
        return true;
      }
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
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node.get(), node->get_fault_manager(), nullptr);
  ctx->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::for_component("ecu-primary"));

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

  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node.get(), node->get_fault_manager(), nullptr);
  ctx->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::full_refresh());
  ASSERT_TRUE(manifest_has_app("tempApp"));

  std::filesystem::remove(fragments_dir / "temp.yaml");
  ctx->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::full_refresh());
  EXPECT_FALSE(manifest_has_app("tempApp"));
}

TEST_F(NotifyIntegrationTest, NestedNotifyFromRefreshPassIsSkipped) {
  // A plugin may call notify_entities_changed from inside its own
  // IntrospectionProvider::introspect() callback. `refresh_cache()` runs
  // introspect while it already holds the thread-local in-refresh flag, so
  // a nested notify on the same thread would recurse into
  // reload_manifest + refresh_cache + introspect indefinitely (stack
  // overflow). The gateway must detect this pattern and short-circuit.
  //
  // Simulate the scenario without a full plugin harness via the dedicated
  // test hook: the hook sets the same thread-local flag the real refresh
  // path uses, then invokes the notification synchronously.
  write_fragment("delta.yaml", R"(
apps:
  - id: nestedApp
    name: Would be loaded if notify ran
    is_located_on: ecu-primary
    ros_binding:
      node_name: nestedApp
)");
  ASSERT_FALSE(manifest_has_app("nestedApp")) << "precondition: fragment dropped but not yet loaded";

  node->trigger_reentrant_notification_for_testing(ros2_medkit_gateway::EntityChangeScope::full_refresh());

  EXPECT_FALSE(manifest_has_app("nestedApp"))
      << "reentrant notify must be a no-op; reloading from within an introspect callback would recurse";
}

TEST_F(NotifyIntegrationTest, InvalidFragmentOnNotifyLeavesErrorVisibleToPlugin) {
  // When a plugin drops a malformed fragment and then notifies, the gateway
  // must (a) preserve the previous valid manifest (no data loss), and
  // (b) surface the failure so the plugin can react - either through
  // ManifestManager::get_validation_result() or via the RCLCPP_WARN log
  // that handle_entity_change_notification emits when reload_manifest
  // returns false. This test pins (a)+(b) so a future refactor that silently
  // throws away the reload result gets caught.
  write_fragment("bad.yaml", R"(
areas:
  - id: rogueArea
    name: Forbidden area in fragment
apps:
  - id: wouldBeApp
    name: Never reaches manifest
    is_located_on: ecu-primary
    ros_binding:
      node_name: wouldBeApp
)");

  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node.get(), node->get_fault_manager(), nullptr);
  ctx->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::full_refresh());

  // (a) base manifest entity still there.
  auto * mm = node->get_discovery_manager()->get_manifest_manager();
  ASSERT_NE(mm, nullptr);
  auto comps = mm->get_components();
  auto it = std::find_if(comps.begin(), comps.end(), [](const auto & c) {
    return c.id == "ecu-primary";
  });
  EXPECT_NE(it, comps.end()) << "bad fragment notify must not wipe the base manifest";
  EXPECT_FALSE(manifest_has_app("wouldBeApp")) << "bad fragment must not contribute entities";

  // (b) the validation result carries a FRAGMENT_FORBIDDEN_FIELD error so
  // the plugin (or operator) can diagnose the failure after notify.
  auto vr = mm->get_validation_result();
  bool found_forbidden = false;
  for (const auto & err : vr.errors) {
    if (err.rule_id == "FRAGMENT_FORBIDDEN_FIELD") {
      found_forbidden = true;
      break;
    }
  }
  EXPECT_TRUE(found_forbidden) << "reload_manifest failure must leave a FRAGMENT_FORBIDDEN_FIELD error in "
                                  "validation_result so the notify failure is diagnosable";
}

TEST_F(NotifyIntegrationTest, NotifyWithoutAnyFragmentIsANoOp) {
  // No fragments dropped. Notify must be safe and must not erase the
  // base-manifest entities.
  auto ctx = ros2_medkit_gateway::make_gateway_plugin_context(node.get(), node->get_fault_manager(), nullptr);
  EXPECT_NO_THROW(ctx->notify_entities_changed(ros2_medkit_gateway::EntityChangeScope::full_refresh()));

  auto * mm = node->get_discovery_manager()->get_manifest_manager();
  ASSERT_NE(mm, nullptr);
  auto comps = mm->get_components();
  auto it = std::find_if(comps.begin(), comps.end(), [](const auto & c) {
    return c.id == "ecu-primary";
  });
  EXPECT_NE(it, comps.end()) << "base manifest entity lost after notify";
}
