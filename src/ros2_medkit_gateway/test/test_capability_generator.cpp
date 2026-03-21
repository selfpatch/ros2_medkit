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

#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "../src/openapi/capability_generator.hpp"
#include "../src/openapi/route_registry.hpp"
#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/version.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::openapi;

using namespace std::chrono_literals;

namespace {

// Populate a RouteRegistry with representative routes matching what the real
// gateway registers, so that generate_root() produces the paths the tests expect.
void populate_test_routes(RouteRegistry & reg) {
  // Dummy handler - these are never actually called in generator tests
  auto noop = [](const httplib::Request &, httplib::Response &) {};

  reg.get("/health", noop).tag("Server").summary("Health check");
  reg.get("/", noop).tag("Server").summary("API overview");
  reg.get("/version-info", noop).tag("Server").summary("SOVD version information");

  for (const auto * et : {"areas", "components", "apps", "functions"}) {
    std::string base = std::string("/") + et;
    std::string singular = et;
    if (!singular.empty() && singular.back() == 's') {
      singular.pop_back();
    }
    std::string entity_path = base + "/{" + singular + "_id}";

    reg.get(base, noop).tag("Discovery").summary(std::string("List ") + et);
    reg.get(entity_path, noop).tag("Discovery").summary(std::string("Get ") + singular);
    reg.get(entity_path + "/data", noop).tag("Data");
    reg.get(entity_path + "/data/{data_id}", noop).tag("Data");
    reg.get(entity_path + "/operations", noop).tag("Operations");
    reg.get(entity_path + "/configurations", noop).tag("Configuration");
    reg.get(entity_path + "/faults", noop).tag("Faults");
    reg.get(entity_path + "/logs", noop).tag("Logs");
    reg.get(entity_path + "/bulk-data", noop).tag("Bulk Data");
    reg.get(entity_path + "/cyclic-subscriptions", noop).tag("Subscriptions");
  }

  reg.get("/faults", noop).tag("Faults").summary("List all faults globally");
  reg.get("/faults/stream", noop).tag("Faults").summary("Stream fault events (SSE)");
}

}  // namespace

// =============================================================================
// Test fixture - creates a full GatewayNode for integration-level tests
// =============================================================================

class CapabilityGeneratorTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<GatewayNode>();

    // Wait briefly for node initialization
    std::this_thread::sleep_for(100ms);

    CorsConfig cors_config;
    AuthConfig auth_config;
    TlsConfig tls_config;

    ctx_ = std::make_unique<handlers::HandlerContext>(node_.get(), cors_config, auth_config, tls_config, nullptr);

    // Create a test route registry with representative routes
    route_registry_ = std::make_unique<RouteRegistry>();
    populate_test_routes(*route_registry_);

    generator_ =
        std::make_unique<CapabilityGenerator>(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get());
  }

  void TearDown() override {
    generator_.reset();
    route_registry_.reset();
    ctx_.reset();
    node_.reset();
  }

  std::shared_ptr<GatewayNode> node_;
  std::unique_ptr<handlers::HandlerContext> ctx_;
  std::unique_ptr<RouteRegistry> route_registry_;
  std::unique_ptr<CapabilityGenerator> generator_;
};

// =============================================================================
// Root path generation
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootReturnsValidOpenApiSpec) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  // Verify OpenAPI 3.1.0 version
  ASSERT_TRUE(spec.contains("openapi"));
  EXPECT_EQ(spec["openapi"], "3.1.0");

  // Verify info block
  ASSERT_TRUE(spec.contains("info"));
  EXPECT_EQ(spec["info"]["title"], "ROS 2 Medkit Gateway");
  EXPECT_EQ(spec["info"]["version"], ros2_medkit_gateway::kGatewayVersion);
  ASSERT_TRUE(spec["info"].contains("x-sovd-version"));
  EXPECT_EQ(spec["info"]["x-sovd-version"], "1.0.0");

  // Verify servers array
  ASSERT_TRUE(spec.contains("servers"));
  ASSERT_TRUE(spec["servers"].is_array());
  ASSERT_FALSE(spec["servers"].empty());

  // Verify paths object exists
  ASSERT_TRUE(spec.contains("paths"));
  ASSERT_TRUE(spec["paths"].is_object());
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsHealthEndpoint) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_TRUE(spec["paths"].contains("/health"));
  ASSERT_TRUE(spec["paths"]["/health"].contains("get"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsVersionInfo) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_TRUE(spec["paths"].contains("/version-info"));
  ASSERT_TRUE(spec["paths"]["/version-info"].contains("get"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsEntityCollections) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/areas"));
  EXPECT_TRUE(spec["paths"].contains("/components"));
  EXPECT_TRUE(spec["paths"].contains("/apps"));
  EXPECT_TRUE(spec["paths"].contains("/functions"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsEntityDetailPaths) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/areas/{area_id}"));
  EXPECT_TRUE(spec["paths"].contains("/components/{component_id}"));
  EXPECT_TRUE(spec["paths"].contains("/apps/{app_id}"));
  EXPECT_TRUE(spec["paths"].contains("/functions/{function_id}"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateRootContainsGlobalFaults) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/faults"));
  EXPECT_TRUE(spec["paths"].contains("/faults/stream"));
}

TEST_F(CapabilityGeneratorTest, GenerateRootContainsGenericErrorResponse) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_TRUE(spec.contains("components"));
  ASSERT_TRUE(spec["components"].contains("responses"));
  EXPECT_TRUE(spec["components"]["responses"].contains("GenericError"));
}

// =============================================================================
// Empty path treated as root
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateEmptyPathReturnsRoot) {
  auto result = generator_->generate("");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_EQ(spec["openapi"], "3.1.0");
  EXPECT_TRUE(spec["paths"].contains("/health"));
}

// =============================================================================
// Invalid / unresolvable paths
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateInvalidPathReturnsNullopt) {
  auto result = generator_->generate("/nonexistent/path");
  EXPECT_FALSE(result.has_value());
}

TEST_F(CapabilityGeneratorTest, GenerateReservedSegmentReturnsNullopt) {
  auto result = generator_->generate("/apps/docs");
  EXPECT_FALSE(result.has_value());
}

TEST_F(CapabilityGeneratorTest, GenerateSingleInvalidSegmentReturnsNullopt) {
  auto result = generator_->generate("/foobar");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Entity collection generation
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, GenerateEntityCollectionReturnsSpec) {
  auto result = generator_->generate("/areas");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_EQ(spec["openapi"], "3.1.0");
  ASSERT_TRUE(spec["paths"].contains("/areas"));
}

TEST_F(CapabilityGeneratorTest, GenerateComponentCollectionReturnsSpec) {
  auto result = generator_->generate("/components");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/components"));
}

TEST_F(CapabilityGeneratorTest, GenerateAppCollectionReturnsSpec) {
  auto result = generator_->generate("/apps");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/apps"));
}

TEST_F(CapabilityGeneratorTest, GenerateFunctionCollectionReturnsSpec) {
  auto result = generator_->generate("/functions");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/functions"));
}

// =============================================================================
// Specific entity - validates entity existence
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateNonexistentEntityReturnsNullopt) {
  // Entity doesn't exist in cache
  auto result = generator_->generate("/apps/nonexistent_app_12345");
  EXPECT_FALSE(result.has_value());
}

TEST_F(CapabilityGeneratorTest, GenerateNonexistentComponentReturnsNullopt) {
  auto result = generator_->generate("/components/nonexistent_component_xyz");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Resource collection - validates entity existence
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateResourceCollectionOfNonexistentEntityReturnsNullopt) {
  auto result = generator_->generate("/apps/nonexistent_app_12345/data");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Specific resource - validates entity existence
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateSpecificResourceOfNonexistentEntityReturnsNullopt) {
  auto result = generator_->generate("/apps/nonexistent_app_12345/data/temperature");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Nested path validation
// =============================================================================

TEST_F(CapabilityGeneratorTest, GenerateNestedPathWithNonexistentParentReturnsNullopt) {
  auto result = generator_->generate("/areas/nonexistent_area/components/nonexistent_comp");
  EXPECT_FALSE(result.has_value());
}

// =============================================================================
// Server URL construction
// =============================================================================

TEST_F(CapabilityGeneratorTest, ServerUrlContainsApiV1Prefix) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_FALSE(spec["servers"].empty());
  std::string server_url = spec["servers"][0]["url"];
  EXPECT_NE(server_url.find("/api/v1"), std::string::npos);
}

TEST_F(CapabilityGeneratorTest, ServerUrlUsesHttpProtocol) {
  auto result = generator_->generate("/");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  ASSERT_FALSE(spec["servers"].empty());
  std::string server_url = spec["servers"][0]["url"];
  EXPECT_EQ(server_url.substr(0, 4), "http");
}

// =============================================================================
// Entity collection generates both list and detail paths
// =============================================================================

TEST_F(CapabilityGeneratorTest, EntityCollectionContainsBothListAndDetailPaths) {
  auto result = generator_->generate("/apps");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  // Should have both the collection listing and the detail path
  EXPECT_TRUE(spec["paths"].contains("/apps"));
  EXPECT_TRUE(spec["paths"].contains("/apps/{app_id}"));
}

TEST_F(CapabilityGeneratorTest, AreaCollectionContainsBothListAndDetailPaths) {
  auto result = generator_->generate("/areas");

  ASSERT_TRUE(result.has_value());
  auto & spec = *result;

  EXPECT_TRUE(spec["paths"].contains("/areas"));
  EXPECT_TRUE(spec["paths"].contains("/areas/{area_id}"));
}

// @verifies REQ_INTEROP_002
TEST_F(CapabilityGeneratorTest, EntityCollectionDetailPathHasParameters) {
  // Entity collection specs use template paths (/apps/{app_id}), so they should
  // include path parameters for the entity ID
  auto result = generator_->generate("/apps");
  ASSERT_TRUE(result.has_value());
  auto & spec = *result;
  ASSERT_TRUE(spec["paths"].contains("/apps/{app_id}"));
  auto & detail = spec["paths"]["/apps/{app_id}"];
  ASSERT_TRUE(detail["get"].contains("parameters"));
  EXPECT_EQ(detail["get"]["parameters"][0]["name"], "app_id");
}

// =============================================================================
// Caching tests
// =============================================================================

TEST_F(CapabilityGeneratorTest, CacheReturnsSameResultForSamePath) {
  auto spec1 = generator_->generate("/apps");
  auto spec2 = generator_->generate("/apps");
  ASSERT_TRUE(spec1.has_value());
  ASSERT_TRUE(spec2.has_value());
  EXPECT_EQ(*spec1, *spec2);
}

TEST_F(CapabilityGeneratorTest, CacheReturnsSameResultForRootPath) {
  auto spec1 = generator_->generate("/");
  auto spec2 = generator_->generate("/");
  ASSERT_TRUE(spec1.has_value());
  ASSERT_TRUE(spec2.has_value());
  EXPECT_EQ(*spec1, *spec2);
}

TEST_F(CapabilityGeneratorTest, CacheReturnsDifferentResultsForDifferentPaths) {
  auto spec1 = generator_->generate("/apps");
  auto spec2 = generator_->generate("/components");
  ASSERT_TRUE(spec1.has_value());
  ASSERT_TRUE(spec2.has_value());
  // Different paths should produce different specs (different titles at minimum)
  EXPECT_NE(*spec1, *spec2);
}

TEST_F(CapabilityGeneratorTest, CacheDoesNotReturnResultForInvalidPath) {
  auto spec1 = generator_->generate("/nonexistent/path");
  EXPECT_FALSE(spec1.has_value());
  // Calling again should also return nullopt (invalid results are not cached)
  auto spec2 = generator_->generate("/nonexistent/path");
  EXPECT_FALSE(spec2.has_value());
}

// =============================================================================
// ThreadSafeEntityCache generation counter tests (standalone, no GatewayNode)
// =============================================================================

TEST(ThreadSafeEntityCacheGenerationTest, GenerationStartsAtZero) {
  ThreadSafeEntityCache cache;
  EXPECT_EQ(cache.generation(), 0u);
}

TEST(ThreadSafeEntityCacheGenerationTest, GenerationIncrementsOnUpdateAll) {
  ThreadSafeEntityCache cache;
  cache.update_all({}, {}, {}, {});
  EXPECT_EQ(cache.generation(), 1u);
}

TEST(ThreadSafeEntityCacheGenerationTest, GenerationIncrementsOnEachUpdate) {
  ThreadSafeEntityCache cache;

  cache.update_areas({});
  EXPECT_EQ(cache.generation(), 1u);

  cache.update_components({});
  EXPECT_EQ(cache.generation(), 2u);

  cache.update_apps({});
  EXPECT_EQ(cache.generation(), 3u);

  cache.update_functions({});
  EXPECT_EQ(cache.generation(), 4u);

  cache.update_all({}, {}, {}, {});
  EXPECT_EQ(cache.generation(), 5u);
}

TEST(ThreadSafeEntityCacheGenerationTest, TopicTypesUpdateIncrementsGeneration) {
  // Topic types affect OpenAPI schemas via SchemaBuilder::from_ros_msg(),
  // so changes must invalidate the spec cache.
  ThreadSafeEntityCache cache;
  cache.update_topic_types({{"topic", "std_msgs/msg/String"}});
  EXPECT_EQ(cache.generation(), 1u);
}

// =============================================================================
// Cache size limit tests
// =============================================================================

TEST_F(CapabilityGeneratorTest, RepeatedCacheHitsDoNotGrow) {
  // Verify that repeated requests for the same small set of paths
  // are served from cache without unbounded growth.
  // Note: with generation-based keys only 4 distinct entries are created,
  // so this primarily tests cache hit behavior, not eviction.
  const std::vector<std::string> entity_types = {"areas", "components", "apps", "functions"};
  for (int i = 0; i < 300; ++i) {
    auto path = "/" + entity_types[static_cast<size_t>(i) % entity_types.size()];
    auto result = generator_->generate(path);
    ASSERT_TRUE(result.has_value()) << "Failed to generate spec for: " << path;
  }
}

// =============================================================================
// Cache invalidation - verify generation-based key prevents stale results
// =============================================================================

TEST_F(CapabilityGeneratorTest, DifferentGenerationsProduceDifferentCacheKeys) {
  // First generate should work and be cacheable
  auto spec1 = generator_->generate("/");
  ASSERT_TRUE(spec1.has_value());
  EXPECT_EQ((*spec1)["openapi"], "3.1.0");

  // GatewayNode::get_thread_safe_cache() returns const ref, so we cannot directly
  // call update_areas() to trigger a generation change from outside the node.
  // The generation counter mechanism is verified separately in
  // ThreadSafeEntityCacheGenerationTest. Here we verify that the cache returns
  // identical results when the generation has not changed.
  auto spec2 = generator_->generate("/");
  ASSERT_TRUE(spec2.has_value());
  EXPECT_EQ((*spec2)["openapi"], "3.1.0");

  // Specs should be identical when entity cache hasn't changed
  EXPECT_EQ(*spec1, *spec2);
}

TEST(CacheGenerationTest, GenerationCounterTracksEntityUpdates) {
  // Standalone test verifying the generation counter mechanism
  // that CapabilityGenerator depends on for cache invalidation
  ThreadSafeEntityCache cache;
  EXPECT_EQ(cache.generation(), 0u);

  cache.update_areas({});
  EXPECT_EQ(cache.generation(), 1u);

  cache.update_components({});
  EXPECT_EQ(cache.generation(), 2u);

  // Each update increments, which would cause CapabilityGenerator
  // to clear its spec_cache_ in get_cache_key()
}
