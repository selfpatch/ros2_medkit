# DiscoveryHandlers Tests Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add unit tests for all 16 `DiscoveryHandlers` methods using a small real-node fixture instead of introducing a new mock framework.

**Architecture:** The tests will use two fixtures in a new `test_discovery_handlers.cpp` file. Validation-only tests will use `HandlerContext(nullptr, ...)`, while behavior tests will use a real `GatewayNode` with `DiscoveryManager` initialized in `MANIFEST_ONLY` mode and a matching `ThreadSafeEntityCache` seed so cache-backed and discovery-backed endpoints read the same entity set.

**Tech Stack:** C++17, GoogleTest, ROS 2 Jazzy, `colcon`, `httplib`, `nlohmann::json`, `GatewayNode`, `DiscoveryManager`, `ThreadSafeEntityCache`

---

### Task 1: Scaffold the DiscoveryHandlers test target and shared fixtures

**Files:**
- Create: `src/ros2_medkit_gateway/test/test_discovery_handlers.cpp`
- Modify: `src/ros2_medkit_gateway/CMakeLists.txt:322-324`
- Reference: `src/ros2_medkit_gateway/test/test_health_handlers.cpp`
- Reference: `src/ros2_medkit_gateway/test/test_data_handlers.cpp`

**Step 1: Write the failing scaffold**

Add the new test target in `src/ros2_medkit_gateway/CMakeLists.txt` near the existing discovery tests:

```cmake
# Add discovery handlers tests
ament_add_gtest(test_discovery_handlers test/test_discovery_handlers.cpp)
target_link_libraries(test_discovery_handlers gateway_lib)
medkit_target_dependencies(test_discovery_handlers rclcpp)
```

Create the test file with one validation test and one seeded behavior test:

```cpp
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
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override;
  void TearDown() override;

  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<DiscoveryHandlers> handlers_;
};

TEST_F(DiscoveryHandlersFixtureTest, ListAreasReturnsSeededItems) {
  httplib::Request req;
  httplib::Response res;
  handlers_->handle_list_areas(req, res);
  EXPECT_EQ(res.status, 200);
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R test_discovery_handlers --output-on-failure
```

Expected: FAIL because the new fixture setup and helper methods are still missing or the new target does not compile yet.

**Step 3: Write the minimal test infrastructure**

Expand `src/ros2_medkit_gateway/test/test_discovery_handlers.cpp` with only the helpers needed by all later tests:

```cpp
namespace {

using json = nlohmann::json;

json parse_json(const httplib::Response& res) {
  return json::parse(res.body);
}

httplib::Request make_request_with_matches(std::initializer_list<std::string> values) {
  httplib::Request req;
  for (const auto& value : values) {
    req.matches.push_back(value);
  }
  return req;
}

const char* kManifestYaml = R"(
manifest_version: "1.0"
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

}  // namespace

void DiscoveryHandlersFixtureTest::SetUp() {
  node_ = std::make_shared<ros2_medkit_gateway::GatewayNode>();

  ros2_medkit_gateway::DiscoveryConfig config;
  config.mode = ros2_medkit_gateway::DiscoveryMode::MANIFEST_ONLY;
  config.manifest_strict_validation = false;
  ASSERT_TRUE(node_->get_discovery_manager()->initialize(config));

  auto* manifest_manager = node_->get_discovery_manager()->get_manifest_manager();
  ASSERT_NE(manifest_manager, nullptr);
  ASSERT_TRUE(manifest_manager->load_manifest_from_string(kManifestYaml, false));

  auto areas = node_->get_discovery_manager()->discover_areas();
  auto components = node_->get_discovery_manager()->discover_components();
  auto apps = node_->get_discovery_manager()->discover_apps();
  auto functions = node_->get_discovery_manager()->discover_functions();

  apps[0].is_online = true;
  apps[0].bound_fqn = "/vehicle/main_ecu/planner";
  apps[1].is_online = false;
  apps[1].bound_fqn = "/sensors/lidar_unit/mapper";

  auto& cache = const_cast<ros2_medkit_gateway::ThreadSafeEntityCache&>(node_->get_thread_safe_cache());
  cache.update_all(areas, components, apps, functions);

  static CorsConfig cors{};
  static AuthConfig auth{};
  static TlsConfig tls{};
  ctx_ = std::make_unique<HandlerContext>(node_.get(), cors, auth, tls, nullptr);
  handlers_ = std::make_unique<DiscoveryHandlers>(*ctx_);
}
```

Also add the minimal include set:

```cpp
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/discovery_handlers.hpp"
```

**Step 4: Run test to verify it passes**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R test_discovery_handlers --output-on-failure
```

Expected: PASS for the scaffolded validation and list-areas tests.

**Step 5: Commit**

```bash
git add src/ros2_medkit_gateway/CMakeLists.txt src/ros2_medkit_gateway/test/test_discovery_handlers.cpp
git commit -m "test(gateway): scaffold DiscoveryHandlers tests"
```

### Task 2: Add area handler coverage

**Files:**
- Modify: `src/ros2_medkit_gateway/test/test_discovery_handlers.cpp`
- Reference: `src/ros2_medkit_gateway/src/http/handlers/discovery_handlers.cpp`

**Step 1: Write the failing area tests**

Add tests for the area endpoints:

```cpp
TEST_F(DiscoveryHandlersValidationTest, GetAreaInvalidIdReturns400) {
  auto req = make_request_with_matches({"", "bad@id"});
  httplib::Response res;
  handlers_.handle_get_area(req, res);
  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetAreaUnknownIdReturns404) {
  auto req = make_request_with_matches({"", "unknown"});
  httplib::Response res;
  handlers_->handle_get_area(req, res);
  EXPECT_EQ(res.status, 404);
}

TEST_F(DiscoveryHandlersFixtureTest, GetAreaReturnsCapabilitiesAndLinks) {
  auto req = make_request_with_matches({"", "vehicle"});
  httplib::Response res;
  handlers_->handle_get_area(req, res);
  auto body = parse_json(res);
  EXPECT_EQ(body["components"], "/api/v1/areas/vehicle/components");
  EXPECT_EQ(body["_links"]["self"], "/api/v1/areas/vehicle");
}

TEST_F(DiscoveryHandlersFixtureTest, AreaComponentsReturnsMatchingComponentsOnly) {
  auto req = make_request_with_matches({"", "sensors"});
  httplib::Response res;
  handlers_->handle_area_components(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
}

TEST_F(DiscoveryHandlersFixtureTest, GetSubareasReturnsChildAreas) {
  auto req = make_request_with_matches({"", "vehicle"});
  httplib::Response res;
  handlers_->handle_get_subareas(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "sensors");
}

TEST_F(DiscoveryHandlersFixtureTest, GetContainsReturnsAreaComponents) {
  auto req = make_request_with_matches({"", "vehicle"});
  httplib::Response res;
  handlers_->handle_get_contains(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "main_ecu");
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R test_discovery_handlers --output-on-failure
```

Expected: FAIL on at least one new area assertion until request-match helpers or fixture data are corrected.

**Step 3: Write minimal supporting test code**

Adjust only the shared helpers and assertions required to make the area tests pass:

```cpp
EXPECT_EQ(body["x-medkit"]["total_count"], 2);
EXPECT_EQ(body["x-medkit"]["parent_area_id"], "vehicle");
EXPECT_EQ(body["_links"]["parent"], "/api/v1/areas/vehicle");
EXPECT_EQ(body["items"][0]["href"], "/api/v1/components/main_ecu");
```

If `make_request_with_matches` needs to mimic `req.matches[1]`, keep it minimal:

```cpp
httplib::Request make_request_with_matches(std::initializer_list<std::string> values) {
  httplib::Request req;
  for (const auto& value : values) {
    req.matches.emplace_back(value);
  }
  return req;
}
```

**Step 4: Run test to verify it passes**

Run the same targeted command and confirm all area tests pass.

**Step 5: Commit**

```bash
git add src/ros2_medkit_gateway/test/test_discovery_handlers.cpp
git commit -m "test(gateway): cover discovery area handlers"
```

### Task 3: Add component handler coverage

**Files:**
- Modify: `src/ros2_medkit_gateway/test/test_discovery_handlers.cpp`

**Step 1: Write the failing component tests**

Add tests for component endpoints:

```cpp
TEST_F(DiscoveryHandlersFixtureTest, ListComponentsReturnsMetadata) {
  httplib::Request req;
  httplib::Response res;
  handlers_->handle_list_components(req, res);
  auto body = parse_json(res);
  EXPECT_EQ(body["items"].size(), 2);
}

TEST_F(DiscoveryHandlersValidationTest, GetComponentInvalidIdReturns400) {
  auto req = make_request_with_matches({"", "bad/id"});
  httplib::Response res;
  handlers_.handle_get_component(req, res);
  EXPECT_EQ(res.status, 400);
}

TEST_F(DiscoveryHandlersFixtureTest, GetComponentReturnsRelationshipsAndCapabilities) {
  auto req = make_request_with_matches({"", "main_ecu"});
  httplib::Response res;
  handlers_->handle_get_component(req, res);
  auto body = parse_json(res);
  EXPECT_EQ(body["belongs-to"], "/api/v1/areas/vehicle");
  EXPECT_EQ(body["hosts"], "/api/v1/components/main_ecu/hosts");
  EXPECT_EQ(body["depends-on"], "/api/v1/components/main_ecu/depends-on");
}

TEST_F(DiscoveryHandlersFixtureTest, GetSubcomponentsReturnsChildren) {
  auto req = make_request_with_matches({"", "main_ecu"});
  httplib::Response res;
  handlers_->handle_get_subcomponents(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
}

TEST_F(DiscoveryHandlersFixtureTest, GetHostsReturnsHostedApps) {
  auto req = make_request_with_matches({"", "main_ecu"});
  httplib::Response res;
  handlers_->handle_get_hosts(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "planner");
}

TEST_F(DiscoveryHandlersFixtureTest, ComponentDependsOnReturnsResolvedAndMissingDependencies) {
  auto req = make_request_with_matches({"", "main_ecu"});
  httplib::Response res;
  handlers_->handle_component_depends_on(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "lidar_unit");
  EXPECT_EQ(body["items"][1]["x-medkit"]["missing"], true);
}
```

**Step 2: Run test to verify it fails**

Run the targeted test command again.

Expected: FAIL on the new component assertions until the seeded manifest and cache include the needed metadata.

**Step 3: Write minimal supporting test data**

Extend the manifest string or post-discovery seeding only where needed:

```cpp
components[0].description = "Vehicle control unit";
components[0].tags = {"compute", "control"};
components[1].description = "Lidar aggregation";
```

Use precise assertions instead of broad snapshots:

```cpp
EXPECT_EQ(body["_links"]["area"], "/api/v1/areas/vehicle");
EXPECT_EQ(body["capabilities"][0]["href"], "/api/v1/components/main_ecu/data");
EXPECT_EQ(body["items"][0]["x-medkit"]["source"], "manifest");
```

**Step 4: Run test to verify it passes**

Run the same targeted command and confirm all component tests pass.

**Step 5: Commit**

```bash
git add src/ros2_medkit_gateway/test/test_discovery_handlers.cpp
git commit -m "test(gateway): cover discovery component handlers"
```

### Task 4: Add app and function handler coverage

**Files:**
- Modify: `src/ros2_medkit_gateway/test/test_discovery_handlers.cpp`

**Step 1: Write the failing app and function tests**

Add the remaining endpoint tests:

```cpp
TEST_F(DiscoveryHandlersFixtureTest, ListAppsReturnsRuntimeMetadata) {
  httplib::Request req;
  httplib::Response res;
  handlers_->handle_list_apps(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["x-medkit"]["component_id"], "main_ecu");
}

TEST_F(DiscoveryHandlersFixtureTest, GetAppReturnsLinksAndCapabilities) {
  auto req = make_request_with_matches({"", "mapper"});
  httplib::Response res;
  handlers_->handle_get_app(req, res);
  auto body = parse_json(res);
  EXPECT_EQ(body["is-located-on"], "/api/v1/components/lidar_unit");
  EXPECT_EQ(body["depends-on"], "/api/v1/apps/mapper/depends-on");
}

TEST_F(DiscoveryHandlersFixtureTest, AppDependsOnReturnsResolvedAndMissingDependencies) {
  auto req = make_request_with_matches({"", "mapper"});
  httplib::Response res;
  handlers_->handle_app_depends_on(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 2);
  EXPECT_EQ(body["items"][0]["id"], "planner");
  EXPECT_EQ(body["items"][1]["x-medkit"]["missing"], true);
}

TEST_F(DiscoveryHandlersFixtureTest, ListFunctionsReturnsSeededFunctions) {
  httplib::Request req;
  httplib::Response res;
  handlers_->handle_list_functions(req, res);
  auto body = parse_json(res);
  EXPECT_EQ(body["items"].size(), 2);
}

TEST_F(DiscoveryHandlersFixtureTest, GetFunctionReturnsCapabilitiesAndGraphLink) {
  auto req = make_request_with_matches({"", "navigation"});
  httplib::Response res;
  handlers_->handle_get_function(req, res);
  auto body = parse_json(res);
  EXPECT_EQ(body["hosts"], "/api/v1/functions/navigation/hosts");
  EXPECT_EQ(body["x-medkit-graph"], "/api/v1/functions/navigation/x-medkit-graph");
}

TEST_F(DiscoveryHandlersFixtureTest, FunctionHostsReturnsHostingApps) {
  auto req = make_request_with_matches({"", "navigation"});
  httplib::Response res;
  handlers_->handle_function_hosts(req, res);
  auto body = parse_json(res);
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], "planner");
}
```

**Step 2: Run test to verify it fails**

Run the same targeted command again.

Expected: FAIL on at least one app or function endpoint until the seeded manifest matches the handler expectations exactly.

**Step 3: Write minimal supporting seed data and assertions**

Set only the missing fields the tests actually inspect:

```cpp
apps[0].source = "manifest";
apps[1].source = "manifest";
functions[0].description = "Navigation function";
functions[0].tags = {"core"};
functions[1].description = "Perception function";
functions[1].tags = {"sensing"};
```

Add the missing validation tests:

```cpp
TEST_F(DiscoveryHandlersValidationTest, GetAppInvalidIdReturns400) { /* ... */ }
TEST_F(DiscoveryHandlersValidationTest, GetFunctionInvalidIdReturns400) { /* ... */ }
```

**Step 4: Run test to verify it passes**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R test_discovery_handlers --output-on-failure
```

Expected: PASS for the full `test_discovery_handlers` target.

**Step 5: Commit**

```bash
git add src/ros2_medkit_gateway/test/test_discovery_handlers.cpp
git commit -m "test(gateway): cover discovery app and function handlers"
```

### Task 5: Run full verification and clean up test-only duplication

**Files:**
- Modify: `src/ros2_medkit_gateway/test/test_discovery_handlers.cpp`
- Verify: `src/ros2_medkit_gateway/CMakeLists.txt`

**Step 1: Refactor only after green**

Tighten the test file without changing behavior:

```cpp
json expect_single_item_response(const httplib::Response& res, const std::string& expected_id) {
  auto body = parse_json(res);
  EXPECT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], expected_id);
  return body;
}
```

Keep only helpers that are used at least twice.

**Step 2: Run the targeted verification command**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R test_discovery_handlers --output-on-failure
```

Expected: PASS with no failing `DiscoveryHandlers` tests.

**Step 3: Run the package-level verification command**

Run:

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select ros2_medkit_gateway --event-handlers console_direct+
colcon test-result --verbose
```

Expected: `ros2_medkit_gateway` tests complete successfully and `test_discovery_handlers` appears in the passing results.

**Step 4: Review the diff against the original scope**

Confirm the change stays limited to:

```bash
git diff --stat HEAD~1
```

Expected: only the new test file and the `CMakeLists.txt` registration, plus any small test-only cleanup discovered during refactor.

**Step 5: Commit**

```bash
git add src/ros2_medkit_gateway/CMakeLists.txt src/ros2_medkit_gateway/test/test_discovery_handlers.cpp
git commit -m "test(gateway): add DiscoveryHandlers unit tests"
```
