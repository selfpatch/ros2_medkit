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

#include <arpa/inet.h>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

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
    description: "Vehicle control unit"
    tags: ["compute", "control"]
    depends_on: ["lidar_unit", "ghost_component"]
  - id: "lidar_unit"
    name: "Lidar Unit"
    namespace: "/sensors"
    area: "sensors"
    parent_component_id: "main_ecu"
    description: "Lidar aggregation"
    tags: ["sensor"]
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

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;

  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);

  return req;
}

std::string write_temp_manifest(const std::string & contents) {
  char path_template[] = "/tmp/ros2_medkit_discovery_handlers_XXXXXX.yaml";
  int fd = mkstemps(path_template, 5);
  if (fd < 0) {
    ADD_FAILURE() << "Failed to create temp manifest file: " << std::strerror(errno);
    return {};
  }
  close(fd);

  std::ofstream out(path_template);
  if (!out) {
    ADD_FAILURE() << "Failed to open temp manifest file for writing: " << path_template;
    std::remove(path_template);
    return {};
  }
  out << contents;
  if (!out.good()) {
    ADD_FAILURE() << "Failed to write manifest contents to: " << path_template;
    out.close();
    std::remove(path_template);
    return {};
  }
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
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0);

    std::vector<std::string> args = {
        "test_discovery_handlers",   "--ros-args", "-p", "server.port:=" + std::to_string(suite_server_port_), "-p",
        "refresh_interval_ms:=60000"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }

    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    suite_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kManifestYaml);
    ASSERT_FALSE(manifest_path_.empty());
    ASSERT_NE(suite_node_, nullptr);

    DiscoveryConfig config;
    config.mode = DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = manifest_path_;
    config.manifest_strict_validation = false;

    ASSERT_TRUE(suite_node_->get_discovery_manager()->initialize(config));

    auto areas = suite_node_->get_discovery_manager()->discover_areas();
    auto components = suite_node_->get_discovery_manager()->discover_components();
    auto apps = suite_node_->get_discovery_manager()->discover_apps();
    auto functions = suite_node_->get_discovery_manager()->discover_functions();

    ASSERT_EQ(apps.size(), 2u);
    apps[0].is_online = true;
    apps[0].bound_fqn = "/vehicle/main_ecu/planner";
    apps[1].bound_fqn = "/sensors/lidar_unit/mapper";

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);

    ctx_ = std::make_unique<HandlerContext>(suite_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<DiscoveryHandlers>(*ctx_);
  }

  void TearDown() override {
    handlers_.reset();
    ctx_.reset();

    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
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

TEST_F(DiscoveryHandlersValidationTest, GetAreaInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/areas/bad@id", R"(/api/v1/areas/([^/]+))");
  httplib::Response res;

  handlers_.handle_get_area(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetAreaUnknownIdReturns404) {
  auto req = make_request_with_match("/api/v1/areas/unknown", R"(/api/v1/areas/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_area(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, GetAreaReturnsCapabilitiesAndLinks) {
  auto req = make_request_with_match("/api/v1/areas/vehicle", R"(/api/v1/areas/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_area(req, res);

  auto body = parse_json(res);
  EXPECT_EQ(body["components"], "/api/v1/areas/vehicle/components");
  EXPECT_EQ(body["contains"], "/api/v1/areas/vehicle/contains");
  EXPECT_EQ(body["_links"]["self"], "/api/v1/areas/vehicle");
  EXPECT_EQ(body["_links"]["collection"], "/api/v1/areas");
}

TEST_F(DiscoveryHandlersFixtureTest, AreaComponentsReturnsMatchingComponentsOnly) {
  auto req = make_request_with_match("/api/v1/areas/sensors/components", R"(/api/v1/areas/([^/]+)/components)");
  httplib::Response res;

  handlers_->handle_area_components(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
}

TEST_F(DiscoveryHandlersValidationTest, AreaComponentsInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/areas/bad@id/components", R"(/api/v1/areas/(.+)/components)");
  httplib::Response res;

  handlers_.handle_area_components(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, AreaComponentsUnknownAreaReturns404) {
  auto req = make_request_with_match("/api/v1/areas/unknown/components", R"(/api/v1/areas/([^/]+)/components)");
  httplib::Response res;

  handlers_->handle_area_components(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, GetSubareasReturnsChildAreas) {
  auto req = make_request_with_match("/api/v1/areas/vehicle/subareas", R"(/api/v1/areas/([^/]+)/subareas)");
  httplib::Response res;

  handlers_->handle_get_subareas(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "sensors");
  EXPECT_EQ(body["_links"]["parent"], "/api/v1/areas/vehicle");
}

TEST_F(DiscoveryHandlersValidationTest, GetSubareasInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/areas/bad@id/subareas", R"(/api/v1/areas/(.+)/subareas)");
  httplib::Response res;

  handlers_.handle_get_subareas(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetSubareasUnknownAreaReturns404) {
  auto req = make_request_with_match("/api/v1/areas/unknown/subareas", R"(/api/v1/areas/([^/]+)/subareas)");
  httplib::Response res;

  handlers_->handle_get_subareas(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, GetContainsReturnsAreaComponents) {
  auto req = make_request_with_match("/api/v1/areas/vehicle/contains", R"(/api/v1/areas/([^/]+)/contains)");
  httplib::Response res;

  handlers_->handle_get_contains(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "main_ecu");
  EXPECT_EQ(body["items"][1]["id"], "lidar_unit");
  EXPECT_EQ(body["_links"]["area"], "/api/v1/areas/vehicle");
}

TEST_F(DiscoveryHandlersValidationTest, GetContainsInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/areas/bad@id/contains", R"(/api/v1/areas/(.+)/contains)");
  httplib::Response res;

  handlers_.handle_get_contains(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetContainsUnknownAreaReturns404) {
  auto req = make_request_with_match("/api/v1/areas/unknown/contains", R"(/api/v1/areas/([^/]+)/contains)");
  httplib::Response res;

  handlers_->handle_get_contains(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, ListComponentsReturnsMetadata) {
  httplib::Request req;
  httplib::Response res;

  handlers_->handle_list_components(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "main_ecu");
  EXPECT_EQ(body["items"][0]["description"], "Vehicle control unit");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
}

TEST_F(DiscoveryHandlersValidationTest, GetComponentInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/components/bad/id", R"(/api/v1/components/(.+))");
  httplib::Response res;

  handlers_.handle_get_component(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetComponentReturnsRelationshipsAndCapabilities) {
  auto req = make_request_with_match("/api/v1/components/main_ecu", R"(/api/v1/components/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_component(req, res);

  auto body = parse_json(res);
  EXPECT_EQ(body["belongs-to"], "/api/v1/areas/vehicle");
  EXPECT_EQ(body["subcomponents"], "/api/v1/components/main_ecu/subcomponents");
  EXPECT_EQ(body["hosts"], "/api/v1/components/main_ecu/hosts");
  EXPECT_EQ(body["depends-on"], "/api/v1/components/main_ecu/depends-on");
  EXPECT_EQ(body["_links"]["area"], "/api/v1/areas/vehicle");
}

TEST_F(DiscoveryHandlersFixtureTest, GetSubcomponentsReturnsChildren) {
  auto req = make_request_with_match("/api/v1/components/main_ecu/subcomponents",
                                     R"(/api/v1/components/([^/]+)/subcomponents)");
  httplib::Response res;

  handlers_->handle_get_subcomponents(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
}

TEST_F(DiscoveryHandlersValidationTest, GetSubcomponentsInvalidIdReturns400) {
  auto req =
      make_request_with_match("/api/v1/components/bad/id/subcomponents", R"(/api/v1/components/(.+)/subcomponents)");
  httplib::Response res;

  handlers_.handle_get_subcomponents(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetSubcomponentsUnknownComponentReturns404) {
  auto req = make_request_with_match("/api/v1/components/unknown/subcomponents",
                                     R"(/api/v1/components/([^/]+)/subcomponents)");
  httplib::Response res;

  handlers_->handle_get_subcomponents(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, GetHostsReturnsHostedApps) {
  auto req = make_request_with_match("/api/v1/components/main_ecu/hosts", R"(/api/v1/components/([^/]+)/hosts)");
  httplib::Response res;

  handlers_->handle_get_hosts(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], false);
}

TEST_F(DiscoveryHandlersValidationTest, GetHostsInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/components/bad/id/hosts", R"(/api/v1/components/(.+)/hosts)");
  httplib::Response res;

  handlers_.handle_get_hosts(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetHostsUnknownComponentReturns404) {
  auto req = make_request_with_match("/api/v1/components/unknown/hosts", R"(/api/v1/components/([^/]+)/hosts)");
  httplib::Response res;

  handlers_->handle_get_hosts(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, ComponentDependsOnReturnsResolvedAndMissingDependencies) {
  auto req =
      make_request_with_match("/api/v1/components/main_ecu/depends-on", R"(/api/v1/components/([^/]+)/depends-on)");
  httplib::Response res;

  handlers_->handle_component_depends_on(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
  EXPECT_EQ(body["items"][1]["id"], "ghost_component");
  EXPECT_EQ(body["items"][1]["x-medkit"]["missing"], true);
}

TEST_F(DiscoveryHandlersValidationTest, ComponentDependsOnInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/components/bad/id/depends-on", R"(/api/v1/components/(.+)/depends-on)");
  httplib::Response res;

  handlers_.handle_component_depends_on(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, ComponentDependsOnUnknownComponentReturns404) {
  auto req =
      make_request_with_match("/api/v1/components/unknown/depends-on", R"(/api/v1/components/([^/]+)/depends-on)");
  httplib::Response res;

  handlers_->handle_component_depends_on(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersValidationTest, GetAppInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/apps/bad/id", R"(/api/v1/apps/(.+))");
  httplib::Response res;

  handlers_.handle_get_app(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, ListAppsReturnsSeededMetadata) {
  httplib::Request req;
  httplib::Response res;

  handlers_->handle_list_apps(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["component_id"], "main_ecu");
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], true);
  EXPECT_EQ(body["items"][0]["x-medkit"]["ros2"]["node"], "/vehicle/main_ecu/planner");
}

TEST_F(DiscoveryHandlersFixtureTest, GetAppUnknownIdReturns404) {
  auto req = make_request_with_match("/api/v1/apps/unknown", R"(/api/v1/apps/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_app(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, GetAppReturnsLinksAndCapabilities) {
  auto req = make_request_with_match("/api/v1/apps/mapper", R"(/api/v1/apps/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_app(req, res);

  auto body = parse_json(res);
  EXPECT_EQ(body["is-located-on"], "/api/v1/components/lidar_unit");
  EXPECT_EQ(body["depends-on"], "/api/v1/apps/mapper/depends-on");
  EXPECT_EQ(body["_links"]["self"], "/api/v1/apps/mapper");
  EXPECT_EQ(body["_links"]["is-located-on"], "/api/v1/components/lidar_unit");
  EXPECT_EQ(body["_links"]["depends-on"][0], "/api/v1/apps/planner");
  EXPECT_EQ(body["_links"]["depends-on"][1], "/api/v1/apps/ghost_app");
  EXPECT_EQ(body["x-medkit"]["source"], "manifest");
  EXPECT_EQ(body["x-medkit"]["is_online"], false);
}

TEST_F(DiscoveryHandlersFixtureTest, AppDependsOnReturnsResolvedAndMissingDependencies) {
  auto req = make_request_with_match("/api/v1/apps/mapper/depends-on", R"(/api/v1/apps/([^/]+)/depends-on)");
  httplib::Response res;

  handlers_->handle_app_depends_on(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], false);
  EXPECT_EQ(body["items"][1]["id"], "ghost_app");
  EXPECT_EQ(body["items"][1]["x-medkit"]["missing"], true);
  EXPECT_EQ(body["_links"]["app"], "/api/v1/apps/mapper");
}

TEST_F(DiscoveryHandlersValidationTest, AppDependsOnInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/apps/bad/id/depends-on", R"(/api/v1/apps/(.+)/depends-on)");
  httplib::Response res;

  handlers_.handle_app_depends_on(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, AppDependsOnUnknownAppReturns404) {
  auto req = make_request_with_match("/api/v1/apps/unknown/depends-on", R"(/api/v1/apps/([^/]+)/depends-on)");
  httplib::Response res;

  handlers_->handle_app_depends_on(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersValidationTest, GetFunctionInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/functions/bad/id", R"(/api/v1/functions/(.+))");
  httplib::Response res;

  handlers_.handle_get_function(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, ListFunctionsReturnsSeededFunctions) {
  httplib::Request req;
  httplib::Response res;

  handlers_->handle_list_functions(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "navigation");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
}

TEST_F(DiscoveryHandlersFixtureTest, GetFunctionUnknownIdReturns404) {
  auto req = make_request_with_match("/api/v1/functions/unknown", R"(/api/v1/functions/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_function(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, GetFunctionReturnsCapabilitiesAndGraphLink) {
  auto req = make_request_with_match("/api/v1/functions/navigation", R"(/api/v1/functions/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_function(req, res);

  auto body = parse_json(res);
  EXPECT_EQ(body["hosts"], "/api/v1/functions/navigation/hosts");
  EXPECT_EQ(body["x-medkit-graph"], "/api/v1/functions/navigation/x-medkit-graph");
  EXPECT_EQ(body["_links"]["self"], "/api/v1/functions/navigation");
  EXPECT_EQ(body["x-medkit"]["source"], "manifest");
}

TEST_F(DiscoveryHandlersValidationTest, FunctionHostsInvalidIdReturns400) {
  auto req = make_request_with_match("/api/v1/functions/bad/id/hosts", R"(/api/v1/functions/(.+)/hosts)");
  httplib::Response res;

  handlers_.handle_function_hosts(req, res);

  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, FunctionHostsUnknownFunctionReturns404) {
  auto req = make_request_with_match("/api/v1/functions/unknown/hosts", R"(/api/v1/functions/([^/]+)/hosts)");
  httplib::Response res;

  handlers_->handle_function_hosts(req, res);

  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, FunctionHostsReturnsHostingApps) {
  auto req = make_request_with_match("/api/v1/functions/navigation/hosts", R"(/api/v1/functions/([^/]+)/hosts)");
  httplib::Response res;

  handlers_->handle_function_hosts(req, res);

  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
  EXPECT_EQ(body["items"][0]["x-medkit"]["is_online"], false);
  EXPECT_EQ(body["_links"]["function"], "/api/v1/functions/navigation");
}
