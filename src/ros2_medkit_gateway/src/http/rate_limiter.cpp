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

#include "ros2_medkit_gateway/http/rate_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

#include "ros2_medkit_gateway/http/error_codes.hpp"

namespace ros2_medkit_gateway {

// ---------------------------------------------------------------------------
// RateLimitConfigBuilder
// ---------------------------------------------------------------------------

RateLimitConfigBuilder & RateLimitConfigBuilder::with_enabled(bool enabled) {
  config_.enabled = enabled;
  return *this;
}

RateLimitConfigBuilder & RateLimitConfigBuilder::with_global_rpm(int requests_per_minute) {
  config_.global_requests_per_minute = requests_per_minute;
  return *this;
}

RateLimitConfigBuilder & RateLimitConfigBuilder::with_client_rpm(int requests_per_minute) {
  config_.client_requests_per_minute = requests_per_minute;
  return *this;
}

RateLimitConfigBuilder & RateLimitConfigBuilder::add_endpoint_limit(const std::string & pattern,
                                                                    int requests_per_minute) {
  config_.endpoint_limits.push_back({pattern, requests_per_minute});
  return *this;
}

RateLimitConfigBuilder & RateLimitConfigBuilder::with_cleanup_interval(int seconds) {
  config_.client_cleanup_interval_seconds = seconds;
  return *this;
}

RateLimitConfigBuilder & RateLimitConfigBuilder::with_max_idle(int seconds) {
  config_.client_max_idle_seconds = seconds;
  return *this;
}

RateLimitConfig RateLimitConfigBuilder::build() {
  if (!config_.enabled) {
    return config_;
  }

  if (config_.global_requests_per_minute <= 0) {
    throw std::invalid_argument("global_requests_per_minute must be > 0 when rate limiting is enabled");
  }
  if (config_.client_requests_per_minute <= 0) {
    throw std::invalid_argument("client_requests_per_minute must be > 0 when rate limiting is enabled");
  }
  if (config_.client_requests_per_minute > config_.global_requests_per_minute) {
    throw std::invalid_argument("client_requests_per_minute cannot exceed global_requests_per_minute");
  }
  if (config_.client_cleanup_interval_seconds <= 0) {
    throw std::invalid_argument("client_cleanup_interval_seconds must be > 0");
  }
  if (config_.client_max_idle_seconds < config_.client_cleanup_interval_seconds) {
    throw std::invalid_argument("client_max_idle_seconds must be >= client_cleanup_interval_seconds");
  }

  for (const auto & ep : config_.endpoint_limits) {
    if (ep.requests_per_minute <= 0) {
      throw std::invalid_argument("endpoint rate limit must be > 0 for pattern: " + ep.pattern);
    }
  }

  return config_;
}

// ---------------------------------------------------------------------------
// RateLimiter
// ---------------------------------------------------------------------------

TokenBucket RateLimiter::make_bucket(int requests_per_minute) {
  TokenBucket bucket;
  bucket.max_tokens = static_cast<double>(requests_per_minute);
  bucket.refill_rate = static_cast<double>(requests_per_minute) / 60.0;
  bucket.tokens = bucket.max_tokens;
  bucket.last_refill = std::chrono::steady_clock::now();
  return bucket;
}

RateLimiter::RateLimiter(const RateLimitConfig & config)
  : config_(config)
  , global_bucket_(make_bucket(config.global_requests_per_minute))
  , last_cleanup_(std::chrono::steady_clock::now()) {
}

bool RateLimiter::try_consume(TokenBucket & bucket) {
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now - bucket.last_refill).count();

  // Lazy refill
  bucket.tokens = std::min(bucket.max_tokens, bucket.tokens + elapsed * bucket.refill_rate);
  bucket.last_refill = now;

  if (bucket.tokens >= 1.0) {
    bucket.tokens -= 1.0;
    return true;
  }
  return false;
}

double RateLimiter::seconds_until_next_token(const TokenBucket & bucket) {
  if (bucket.tokens >= 1.0 || bucket.refill_rate <= 0.0) {
    return 0.0;
  }
  return (1.0 - bucket.tokens) / bucket.refill_rate;
}

int RateLimiter::get_effective_rpm(const std::string & path) const {
  for (const auto & ep : config_.endpoint_limits) {
    if (path_matches_pattern(path, ep.pattern)) {
      return ep.requests_per_minute;
    }
  }
  return config_.client_requests_per_minute;
}

bool RateLimiter::path_matches_pattern(const std::string & path, const std::string & pattern) {
  // Split both path and pattern by '/' and match segment-by-segment
  // '*' matches exactly one segment (no slash crossing)
  auto split = [](const std::string & s) -> std::vector<std::string> {
    std::vector<std::string> segments;
    std::istringstream iss(s);
    std::string segment;
    while (std::getline(iss, segment, '/')) {
      if (!segment.empty()) {
        segments.push_back(segment);
      }
    }
    return segments;
  };

  auto path_segments = split(path);
  auto pattern_segments = split(pattern);

  if (path_segments.size() != pattern_segments.size()) {
    return false;
  }

  for (size_t i = 0; i < pattern_segments.size(); ++i) {
    if (pattern_segments[i] == "*") {
      continue;
    }
    if (pattern_segments[i] != path_segments[i]) {
      return false;
    }
  }

  return true;
}

RateLimitResult RateLimiter::check(const std::string & client_ip, const std::string & path) {
  RateLimitResult result;

  // Periodic cleanup of stale client entries
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_cleanup_).count();
    if (elapsed >= static_cast<double>(config_.client_cleanup_interval_seconds)) {
      cleanup_stale_clients();
    }
  }

  // 1. Global bucket check
  {
    std::lock_guard<std::mutex> lock(global_mutex_);
    if (!try_consume(global_bucket_)) {
      result.allowed = false;
      result.limit = config_.global_requests_per_minute;
      result.remaining = 0;
      double wait = seconds_until_next_token(global_bucket_);
      result.retry_after_seconds = std::max(1, static_cast<int>(std::ceil(wait)));
      auto reset_time = std::chrono::system_clock::now() + std::chrono::seconds(result.retry_after_seconds);
      result.reset_epoch_seconds =
          std::chrono::duration_cast<std::chrono::seconds>(reset_time.time_since_epoch()).count();
      return result;
    }
  }

  // 2. Per-client bucket check
  int effective_rpm = get_effective_rpm(path);
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    auto it = client_buckets_.find(client_ip);
    if (it == client_buckets_.end()) {
      // New client — create bucket with effective RPM
      ClientState state;
      state.bucket = make_bucket(effective_rpm);
      state.last_seen = std::chrono::steady_clock::now();
      // Consume one token for this request
      state.bucket.tokens -= 1.0;
      client_buckets_[client_ip] = state;

      result.allowed = true;
      result.limit = effective_rpm;
      result.remaining = static_cast<int>(state.bucket.tokens);
      auto reset_time = std::chrono::system_clock::now() + std::chrono::seconds(60);
      result.reset_epoch_seconds =
          std::chrono::duration_cast<std::chrono::seconds>(reset_time.time_since_epoch()).count();
      return result;
    }

    auto & state = it->second;
    state.last_seen = std::chrono::steady_clock::now();

    if (!try_consume(state.bucket)) {
      result.allowed = false;
      result.limit = effective_rpm;
      result.remaining = 0;
      double wait = seconds_until_next_token(state.bucket);
      result.retry_after_seconds = std::max(1, static_cast<int>(std::ceil(wait)));
      auto reset_time = std::chrono::system_clock::now() + std::chrono::seconds(result.retry_after_seconds);
      result.reset_epoch_seconds =
          std::chrono::duration_cast<std::chrono::seconds>(reset_time.time_since_epoch()).count();
      return result;
    }

    result.allowed = true;
    result.limit = effective_rpm;
    result.remaining = std::max(0, static_cast<int>(state.bucket.tokens));
    auto reset_time = std::chrono::system_clock::now() + std::chrono::seconds(60);
    result.reset_epoch_seconds =
        std::chrono::duration_cast<std::chrono::seconds>(reset_time.time_since_epoch()).count();
  }

  return result;
}

void RateLimiter::apply_headers(const RateLimitResult & result, httplib::Response & res) {
  res.set_header("X-RateLimit-Limit", std::to_string(result.limit));
  res.set_header("X-RateLimit-Remaining", std::to_string(result.remaining));
  res.set_header("X-RateLimit-Reset", std::to_string(result.reset_epoch_seconds));
}

void RateLimiter::apply_rejection(const RateLimitResult & result, httplib::Response & res) {
  res.status = 429;
  res.set_header("Retry-After", std::to_string(result.retry_after_seconds));

  nlohmann::json error;
  error["error_code"] = ERR_RATE_LIMIT_EXCEEDED;
  error["message"] =
      "Too many requests. Please retry after " + std::to_string(result.retry_after_seconds) + " seconds.";
  error["parameters"] = {
      {"retry_after", result.retry_after_seconds}, {"limit", result.limit}, {"reset", result.reset_epoch_seconds}};

  res.set_content(error.dump(), "application/json");
}

void RateLimiter::cleanup_stale_clients() {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto now = std::chrono::steady_clock::now();
  last_cleanup_ = now;

  for (auto it = client_buckets_.begin(); it != client_buckets_.end();) {
    double idle = std::chrono::duration<double>(now - it->second.last_seen).count();
    if (idle > static_cast<double>(config_.client_max_idle_seconds)) {
      it = client_buckets_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace ros2_medkit_gateway
