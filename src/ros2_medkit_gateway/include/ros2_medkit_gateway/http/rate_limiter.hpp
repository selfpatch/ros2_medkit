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

#pragma once

#include <httplib.h>

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief Rate limit override for a specific endpoint pattern.
 */
struct EndpointRateLimit {
  std::string pattern;         ///< Glob-like pattern (e.g. "/api/v1/*/operations/*")
  int requests_per_minute{0};  ///< Override RPM for matching paths
};

/**
 * @brief Configuration for the rate limiter.
 */
struct RateLimitConfig {
  bool enabled{false};

  /// Maximum requests per minute across all clients
  int global_requests_per_minute{600};

  /// Maximum requests per minute per client IP
  int client_requests_per_minute{60};

  /// Endpoint-specific rate limit overrides
  std::vector<EndpointRateLimit> endpoint_limits;

  /// How often to scan and remove idle client entries (seconds)
  int client_cleanup_interval_seconds{300};

  /// Remove client entries idle longer than this (seconds)
  int client_max_idle_seconds{600};
};

/**
 * @brief Builder for RateLimitConfig with validation.
 */
class RateLimitConfigBuilder {
 public:
  RateLimitConfigBuilder & with_enabled(bool enabled);
  RateLimitConfigBuilder & with_global_rpm(int requests_per_minute);
  RateLimitConfigBuilder & with_client_rpm(int requests_per_minute);
  RateLimitConfigBuilder & add_endpoint_limit(const std::string & pattern, int requests_per_minute);
  RateLimitConfigBuilder & with_cleanup_interval(int seconds);
  RateLimitConfigBuilder & with_max_idle(int seconds);

  /// Build and validate the configuration.
  /// @throws std::invalid_argument if configuration is invalid
  RateLimitConfig build();

 private:
  RateLimitConfig config_;
};

/**
 * @brief Token bucket for rate limiting.
 */
struct TokenBucket {
  double tokens{0.0};
  std::chrono::steady_clock::time_point last_refill;
  double max_tokens{0.0};
  double refill_rate{0.0};  ///< Tokens per second
};

/**
 * @brief Result of a rate limit check.
 */
struct RateLimitResult {
  bool allowed{true};
  int limit{0};                    ///< X-RateLimit-Limit
  int remaining{0};                ///< X-RateLimit-Remaining
  int64_t reset_epoch_seconds{0};  ///< X-RateLimit-Reset (Unix timestamp)
  int retry_after_seconds{0};      ///< Retry-After header (only when rejected)
};

/**
 * @brief Token-bucket-based HTTP rate limiter.
 *
 * Provides two-layer rate limiting:
 * - Global bucket: limits total request rate across all clients
 * - Per-client bucket: limits request rate per client IP
 *
 * Thread-safe for use from cpp-httplib's worker thread pool.
 */
class RateLimiter {
 public:
  explicit RateLimiter(const RateLimitConfig & config);

  /// Check if a request is allowed and return rate limit info.
  RateLimitResult check(const std::string & client_ip, const std::string & path);

  /// Set rate limit response headers on the HTTP response.
  static void apply_headers(const RateLimitResult & result, httplib::Response & res);

  /// Set 429 rejection response with SOVD-compliant error body.
  static void apply_rejection(const RateLimitResult & result, httplib::Response & res);

  /// Remove tracking entries for clients that have been idle too long.
  void cleanup_stale_clients();

  /// Check if rate limiting is enabled.
  bool is_enabled() const {
    return config_.enabled;
  }

 private:
  /// Refill tokens lazily and try to consume one.
  /// @return true if a token was consumed, false if bucket is empty
  bool try_consume(TokenBucket & bucket);

  /// Compute seconds until next token is available.
  static double seconds_until_next_token(const TokenBucket & bucket);

  /// Get the effective RPM for a given request path.
  int get_effective_rpm(const std::string & path) const;

  /// Check if a path matches a glob-like pattern (* = single segment).
  static bool path_matches_pattern(const std::string & path, const std::string & pattern);

  /// Initialize a token bucket with the given RPM.
  static TokenBucket make_bucket(int requests_per_minute);

  RateLimitConfig config_;

  TokenBucket global_bucket_;
  mutable std::mutex global_mutex_;

  struct ClientState {
    TokenBucket bucket;
    std::chrono::steady_clock::time_point last_seen;
  };
  std::unordered_map<std::string, ClientState> client_buckets_;
  mutable std::mutex clients_mutex_;

  std::chrono::steady_clock::time_point last_cleanup_;
};

}  // namespace ros2_medkit_gateway
