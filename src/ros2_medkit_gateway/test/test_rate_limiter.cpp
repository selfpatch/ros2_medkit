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

  // Wait for tokens to refill (1 token per second, need at least 1 second; 50% margin)
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

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
  // Use 2 RPM so natural refill (2/60 tokens/s) cannot recover within 2.5s
  // max_idle=2s, cleanup_interval=1s
  auto config = make_config(600, 2);
  RateLimiter limiter(config);

  // Exhaust the client's 2-token bucket
  auto first = limiter.check("192.168.1.100", "/api/v1/health");
  EXPECT_TRUE(first.allowed);
  auto second = limiter.check("192.168.1.100", "/api/v1/health");
  EXPECT_TRUE(second.allowed);
  auto rejected = limiter.check("192.168.1.100", "/api/v1/health");
  EXPECT_FALSE(rejected.allowed);

  // Wait for the client to become stale (> 2 seconds; 50% margin)
  // At 2 RPM, natural refill in 3.0s = ~0.10 tokens — not enough for a new request
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // Trigger cleanup via a different client's check (cleanup runs on interval)
  limiter.check("192.168.1.200", "/api/v1/health");

  // Client 100 should have been removed and gets a fresh full bucket
  auto result = limiter.check("192.168.1.100", "/api/v1/health");
  EXPECT_TRUE(result.allowed);
  EXPECT_GT(result.remaining, 0);
}

// ===========================================================================
// Global Token Waste Prevention Tests
// ===========================================================================

TEST_F(RateLimiterTest, PerClientRejectionDoesNotWasteGlobalTokens) {
  // Global: 6 tokens, Client: 2 tokens
  auto config = make_config(6, 2);
  RateLimiter limiter(config);

  // Exhaust client A's 2 per-client tokens
  EXPECT_TRUE(limiter.check("192.168.1.1", "/api/v1/health").allowed);
  EXPECT_TRUE(limiter.check("192.168.1.1", "/api/v1/health").allowed);

  // 4 more requests from A — rejected at per-client level, global untouched
  for (int i = 0; i < 4; ++i) {
    auto r = limiter.check("192.168.1.1", "/api/v1/health");
    EXPECT_FALSE(r.allowed) << "Request " << i << " from exhausted client A should be rejected";
  }

  // Client B should still be able to use the global pool (6 - 2 = 4 tokens remaining)
  EXPECT_TRUE(limiter.check("192.168.1.2", "/api/v1/health").allowed);
  EXPECT_TRUE(limiter.check("192.168.1.2", "/api/v1/health").allowed);
}

// ===========================================================================
// Endpoint Override with Composite Bucket Key Tests
// ===========================================================================

TEST_F(RateLimiterTest, EndpointOverrideAppliesForExistingClient) {
  auto config = RateLimitConfigBuilder()
                    .with_enabled(true)
                    .with_global_rpm(600)
                    .with_client_rpm(60)
                    .add_endpoint_limit("/api/v1/*/operations/*", 3)
                    .with_cleanup_interval(300)
                    .with_max_idle(600)
                    .build();
  RateLimiter limiter(config);

  // Client hits default endpoint first (creates bucket for default pattern)
  auto r1 = limiter.check("192.168.1.1", "/api/v1/health");
  EXPECT_TRUE(r1.allowed);

  // Same client hits operations endpoint (3 RPM) — gets a separate bucket
  for (int i = 0; i < 3; ++i) {
    auto r = limiter.check("192.168.1.1", "/api/v1/myapp/operations/calibrate");
    EXPECT_TRUE(r.allowed);
  }

  // 4th operations request should be rejected (operations bucket exhausted)
  auto r_ops = limiter.check("192.168.1.1", "/api/v1/myapp/operations/calibrate");
  EXPECT_FALSE(r_ops.allowed);

  // Default endpoint should still work (separate bucket with 59 tokens remaining)
  auto r_default = limiter.check("192.168.1.1", "/api/v1/health");
  EXPECT_TRUE(r_default.allowed);
}
