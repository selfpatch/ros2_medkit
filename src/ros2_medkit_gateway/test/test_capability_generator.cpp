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
#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

using namespace ros2_medkit_gateway;
using namespace ros2_medkit_gateway::openapi;

using namespace std::chrono_literals;

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

    generator_ = std::make_unique<CapabilityGenerator>(*ctx_, *node_, node_->get_plugin_manager());
  }

  void TearDown() override {
    generator_.reset();
    ctx_.reset();
    node_.reset();
  }

  std::shared_ptr<GatewayNode> node_;
  std::unique_ptr<handlers::HandlerContext> ctx_;
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
  EXPECT_EQ(spec["info"]["version"], "0.3.0");
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

TEST(ThreadSafeEntityCacheGenerationTest, TopicTypesUpdateDoesNotIncrementGeneration) {
  ThreadSafeEntityCache cache;
  cache.update_topic_types({{"topic", "std_msgs/msg/String"}});
  EXPECT_EQ(cache.generation(), 0u);
}
