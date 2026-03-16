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

#include <gtest/gtest.h>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include <cstdio>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>
#include <unistd.h>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/discovery_handlers.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::DiscoveryConfig;
using ros2_medkit_gateway::DiscoveryMode;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::DiscoveryHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;

namespace {

const char * kManifestYaml = R"(
manifest_version: "1.0"
metadata:
  name: "discovery-handlers-test"
  version: "1.0.0"
areas:
  - id: "vehicle"
    name: "Vehicle"
  - id: "sensors"
    name: "Sensors"
    parent_area: "vehicle"
components:
  - id: "main_ecu"
    name: "Main ECU"
    namespace: "/vehicle"
    area: "vehicle"
    depends_on: ["lidar_unit", "ghost_component"]
  - id: "lidar_unit"
    name: "Lidar Unit"
    namespace: "/sensors"
    area: "sensors"
    parent_component_id: "main_ecu"
apps:
  - id: "planner"
    name: "Planner"
    is_located_on: "main_ecu"
    description: "Path planning"
  - id: "mapper"
    name: "Mapper"
    is_located_on: "lidar_unit"
    depends_on: ["planner", "ghost_app"]
functions:
  - id: "navigation"
    name: "Navigation"
    hosts: ["planner"]
  - id: "perception"
    name: "Perception"
    hosts: ["mapper"]
)";

json parse_json(const httplib::Response & res) {
  return json::parse(res.body);
}

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;

  std::regex re(pattern);
  std::smatch matches;
  if (std::regex_match(path, matches, re)) {
    req.matches = matches;
  }

  return req;
}

std::string write_temp_manifest(const std::string & contents) {
  char path_template[] = "/tmp/ros2_medkit_discovery_handlers_XXXXXX.yaml";
  int fd = mkstemps(path_template, 5);
  EXPECT_GE(fd, 0);
  if (fd >= 0) {
    close(fd);
  }

  std::ofstream out(path_template);
  out << contents;
  out.close();
  return path_template;
}

}  // namespace

class DiscoveryHandlersValidationTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  DiscoveryHandlers handlers_{ctx_};
};

TEST_F(DiscoveryHandlersValidationTest, GetAreaMissingMatchesReturns400) {
  httplib::Request req;
  httplib::Response res;

  handlers_.handle_get_area(req, res);

  EXPECT_EQ(res.status, 400);
}

class DiscoveryHandlersFixtureTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kManifestYaml);
    node_ = std::make_shared<GatewayNode>();

    DiscoveryConfig config;
    config.mode = DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = manifest_path_;
    config.manifest_strict_validation = false;

    ASSERT_TRUE(node_->get_discovery_manager()->initialize(config));

    auto areas = node_->get_discovery_manager()->discover_areas();
    auto components = node_->get_discovery_manager()->discover_components();
    auto apps = node_->get_discovery_manager()->discover_apps();
    auto functions = node_->get_discovery_manager()->discover_functions();

    ASSERT_EQ(apps.size(), 2u);
    apps[0].is_online = true;
    apps[0].bound_fqn = "/vehicle/main_ecu/planner";
    apps[1].bound_fqn = "/sensors/lidar_unit/mapper";

    auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);

    ctx_ = std::make_unique<HandlerContext>(node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<DiscoveryHandlers>(*ctx_);
  }

  void TearDown() override {
    handlers_.reset();
    ctx_.reset();
    node_.reset();

    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> node_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<DiscoveryHandlers> handlers_;
  std::string manifest_path_;
};

TEST_F(DiscoveryHandlersFixtureTest, ListAreasReturnsSeededItems) {
  httplib::Request req;
  httplib::Response res;

  handlers_->handle_list_areas(req, res);

  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");
  auto body = parse_json(res);
  ASSERT_TRUE(body.contains("items"));
  ASSERT_EQ(body["items"].size(), 2);
}
