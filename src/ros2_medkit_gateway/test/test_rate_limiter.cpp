// Copyright 2026 eclipse0922
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

#include <chrono>
#include <thread>

#include "ros2_medkit_gateway/http/rate_limiter.hpp"

using namespace ros2_medkit_gateway;

// ===========================================================================
// Configuration Builder Tests
// ===========================================================================

TEST(RateLimitConfigBuilderTest, DefaultConfigIsDisabled) {
  RateLimitConfig config;
  EXPECT_FALSE(config.enabled);
}

TEST(RateLimitConfigBuilderTest, ValidConfigBuilds) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.global_requests_per_minute, 600);
  EXPECT_EQ(config.client_requests_per_minute, 60);
}

TEST(RateLimitConfigBuilderTest, DisabledConfigSkipsValidation) {
  // Invalid values should be accepted when disabled
  auto config = RateLimitConfigBuilder().with_enabled(false).with_global_rpm(0).with_client_rpm(-1).build();

  EXPECT_FALSE(config.enabled);
}

TEST(RateLimitConfigBuilderTest, ZeroGlobalRpmThrowsWhenEnabled) {
  EXPECT_THROW(RateLimitConfigBuilder().with_enabled(true).with_global_rpm(0).with_client_rpm(0).build(),
               std::invalid_argument);
}

TEST(RateLimitConfigBuilderTest, ZeroClientRpmThrowsWhenEnabled) {
  EXPECT_THROW(RateLimitConfigBuilder().with_enabled(true).with_global_rpm(100).with_client_rpm(0).build(),
               std::invalid_argument);
}

TEST(RateLimitConfigBuilderTest, ClientRpmExceedsGlobalThrows) {
  EXPECT_THROW(RateLimitConfigBuilder().with_enabled(true).with_global_rpm(10).with_client_rpm(20).build(),
               std::invalid_argument);
}

TEST(RateLimitConfigBuilderTest, InvalidCleanupIntervalThrows) {
  EXPECT_THROW(RateLimitConfigBuilder()
                   .with_enabled(true)
                   .with_global_rpm(100)
                   .with_client_rpm(10)
                   .with_cleanup_interval(0)
                   .build(),
               std::invalid_argument);
}

TEST(RateLimitConfigBuilderTest, MaxIdleLessThanCleanupThrows) {
  EXPECT_THROW(RateLimitConfigBuilder()
                   .with_enabled(true)
                   .with_global_rpm(100)
                   .with_client_rpm(10)
                   .with_cleanup_interval(300)
                   .with_max_idle(100)
                   .build(),
               std::invalid_argument);
}

TEST(RateLimitConfigBuilderTest, EndpointLimitAdded) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/*/operations/*", 10)
                    .build();

  ASSERT_EQ(config.endpoint_limits.size(), 1u);
  EXPECT_EQ(config.endpoint_limits[0].pattern, "/api/v1/*/operations/*");
  EXPECT_EQ(config.endpoint_limits[0].requests_per_minute, 10);
}

TEST(RateLimitConfigBuilderTest, InvalidEndpointRpmThrows) {
  EXPECT_THROW(RateLimitConfigBuilder()
                   .with_enabled(true)
                   .with_global_rpm(600)
                   .with_client_rpm(60)
                   .add_endpoint_limit("/api/v1/health", 0)
                   .build(),
               std::invalid_argument);
}

// ===========================================================================
// Token Bucket / Rate Limiter Tests
// ===========================================================================

class RateLimiterTest : public ::testing::Test {
 protected:
  // Create a limiter with small limits for fast testing
  RateLimitConfig make_config(int global_rpm, int client_rpm) {
    return RateLimitConfigBuilder()
        .with_enabled(true)
        .with_global_rpm(global_rpm)
        .with_client_rpm(client_rpm)
        .with_cleanup_interval(1)
        .with_max_idle(2)
        .build();
  }
};

TEST_F(RateLimiterTest, AllowsRequestsWithinLimit) {
  auto config = make_config(600, 10);
  RateLimiter limiter(config);

  // 10 requests should all be allowed for a single client
  for (int i = 0; i < 10; ++i) {
    auto result = limiter.check("192.168.1.1", "/api/v1/health");
    EXPECT_TRUE(result.allowed) << "Request " << i << " should be allowed";
  }
}

TEST_F(RateLimiterTest, RejectsRequestsOverClientLimit) {
  // Client limit: 5 requests (bucket starts full with 5 tokens)
  auto config = make_config(600, 5);
  RateLimiter limiter(config);

  // Exhaust all 5 tokens
  for (int i = 0; i < 5; ++i) {
    auto result = limiter.check("192.168.1.1", "/api/v1/health");
    EXPECT_TRUE(result.allowed) << "Request " << i << " should be allowed";
  }

  // 6th request should be rejected
  auto result = limiter.check("192.168.1.1", "/api/v1/health");
  EXPECT_FALSE(result.allowed);
  EXPECT_GT(result.retry_after_seconds, 0);
}

TEST_F(RateLimiterTest, TokensRefillOverTime) {
  // 60 RPM = 1 token per second
  auto config = make_config(600, 60);
  RateLimiter limiter(config);

  // Exhaust all tokens
  for (int i = 0; i < 60; ++i) {
    limiter.check("192.168.1.1", "/api/v1/health");
  }

  // Should be rejected now
  auto rejected = limiter.check("192.168.1.1", "/api/v1/health");
  EXPECT_FALSE(rejected.allowed);

  // Wait for tokens to refill (1 token per second, need at least 1 second)
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));

  // Should be allowed again
  auto allowed = limiter.check("192.168.1.1", "/api/v1/health");
  EXPECT_TRUE(allowed.allowed);
}

TEST_F(RateLimiterTest, DifferentClientsHaveSeparateBuckets) {
  auto config = make_config(600, 3);
  RateLimiter limiter(config);

  // Exhaust client A's tokens
  for (int i = 0; i < 3; ++i) {
    limiter.check("192.168.1.1", "/api/v1/health");
  }
  auto result_a = limiter.check("192.168.1.1", "/api/v1/health");
  EXPECT_FALSE(result_a.allowed);

  // Client B should still have tokens
  auto result_b = limiter.check("192.168.1.2", "/api/v1/health");
  EXPECT_TRUE(result_b.allowed);
}

TEST_F(RateLimiterTest, GlobalLimitAffectsAllClients) {
  // Global: 5, Client: 5 - global runs out first when multiple clients
  auto config = make_config(5, 5);
  RateLimiter limiter(config);

  // 3 requests from client A
  for (int i = 0; i < 3; ++i) {
    auto r = limiter.check("192.168.1.1", "/api/v1/health");
    EXPECT_TRUE(r.allowed);
  }

  // 2 requests from client B
  for (int i = 0; i < 2; ++i) {
    auto r = limiter.check("192.168.1.2", "/api/v1/health");
    EXPECT_TRUE(r.allowed);
  }

  // Global limit of 5 reached — next request from either client should fail
  auto result = limiter.check("192.168.1.3", "/api/v1/health");
  EXPECT_FALSE(result.allowed);
}

// ===========================================================================
// Endpoint Override Tests
// ===========================================================================

TEST_F(RateLimiterTest, EndpointOverrideAppliesLowerLimit) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/*/operations/*", 3)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();
  RateLimiter limiter(config);

  // 3 requests to operations endpoint
  for (int i = 0; i < 3; ++i) {
    auto r = limiter.check("192.168.1.1", "/api/v1/myapp/operations/calibrate");
    EXPECT_TRUE(r.allowed);
  }

  // 4th should be rejected (endpoint limit is 3)
  auto r = limiter.check("192.168.1.1", "/api/v1/myapp/operations/calibrate");
  EXPECT_FALSE(r.allowed);
}

TEST_F(RateLimiterTest, NonMatchingEndpointUsesDefaultLimit) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/*/operations/*", 3)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();
  RateLimiter limiter(config);

  // Health endpoint doesn't match pattern — uses default 60 RPM
  for (int i = 0; i < 10; ++i) {
    auto r = limiter.check("192.168.1.50", "/api/v1/health");
    EXPECT_TRUE(r.allowed);
  }
}

// ===========================================================================
// Pattern Matching Tests (via endpoint override behavior)
// ===========================================================================

TEST(PathMatchingTest, ExactPathMatch) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/health", 2)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();
  RateLimiter limiter(config);

  limiter.check("10.0.0.1", "/api/v1/health");
  limiter.check("10.0.0.1", "/api/v1/health");
  auto r = limiter.check("10.0.0.1", "/api/v1/health");
  EXPECT_FALSE(r.allowed);
}

TEST(PathMatchingTest, SingleWildcardMatch) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/*/data", 2)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();
  RateLimiter limiter(config);

  // /api/v1/mycomp/data matches /api/v1/*/data
  limiter.check("10.0.0.1", "/api/v1/mycomp/data");
  limiter.check("10.0.0.1", "/api/v1/mycomp/data");
  auto r = limiter.check("10.0.0.1", "/api/v1/mycomp/data");
  EXPECT_FALSE(r.allowed);
}

TEST(PathMatchingTest, WildcardDoesNotCrossSegments) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/*/data", 2)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();
  RateLimiter limiter(config);

  // /api/v1/a/b/data has extra segment — should NOT match /api/v1/*/data
  // So it uses the default 60 RPM and should be allowed many times
  for (int i = 0; i < 10; ++i) {
    auto r = limiter.check("10.0.0.2", "/api/v1/a/b/data");
    EXPECT_TRUE(r.allowed);
  }
}

TEST(PathMatchingTest, MultipleWildcards) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/*/operations/*/execute", 2)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();
  RateLimiter limiter(config);

  limiter.check("10.0.0.1", "/api/v1/myapp/operations/calibrate/execute");
  limiter.check("10.0.0.1", "/api/v1/myapp/operations/calibrate/execute");
  auto r = limiter.check("10.0.0.1", "/api/v1/myapp/operations/calibrate/execute");
  EXPECT_FALSE(r.allowed);
}

// ===========================================================================
// Result / Header Tests
// ===========================================================================

TEST_F(RateLimiterTest, AllowedResultHasCorrectFields) {
  auto config = make_config(600, 60);
  RateLimiter limiter(config);

  auto result = limiter.check("192.168.1.1", "/api/v1/health");
  EXPECT_TRUE(result.allowed);
  EXPECT_EQ(result.limit, 60);
  EXPECT_GE(result.remaining, 0);
  EXPECT_GT(result.reset_epoch_seconds, 0);
  EXPECT_EQ(result.retry_after_seconds, 0);
}

TEST_F(RateLimiterTest, RejectedResultHas429Fields) {
  auto config = make_config(600, 2);
  RateLimiter limiter(config);

  limiter.check("192.168.1.1", "/api/v1/health");
  limiter.check("192.168.1.1", "/api/v1/health");
  auto result = limiter.check("192.168.1.1", "/api/v1/health");

  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.remaining, 0);
  EXPECT_GT(result.retry_after_seconds, 0);
  EXPECT_GT(result.reset_epoch_seconds, 0);
}

// ===========================================================================
// Cleanup Tests
// ===========================================================================

TEST_F(RateLimiterTest, StaleClientsAreCleaned) {
  // max_idle=2s, cleanup_interval=1s
  auto config = make_config(600, 60);
  RateLimiter limiter(config);

  // Create a client entry
  limiter.check("192.168.1.100", "/api/v1/health");

  // Wait for the client to become stale (> 2 seconds)
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));

  // Trigger cleanup via another check (cleanup runs on interval)
  limiter.check("192.168.1.200", "/api/v1/health");

  // The stale client should get a fresh bucket (full tokens)
  // If cleanup worked, client 100 should have been removed and get a new bucket
  auto result = limiter.check("192.168.1.100", "/api/v1/health");
  EXPECT_TRUE(result.allowed);
  // remaining should be near max since it's a fresh bucket
  EXPECT_GE(result.remaining, 50);
}
