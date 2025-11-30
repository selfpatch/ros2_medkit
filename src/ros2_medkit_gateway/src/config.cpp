// Copyright 2025 bburda
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

#include "ros2_medkit_gateway/config.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ros2_medkit_gateway {

CorsConfigBuilder & CorsConfigBuilder::with_origins(std::vector<std::string> origins) {
  config_.allowed_origins = std::move(origins);
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_methods(std::vector<std::string> methods) {
  config_.allowed_methods = std::move(methods);
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_headers(std::vector<std::string> headers) {
  config_.allowed_headers = std::move(headers);
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_credentials(bool credentials) {
  config_.allow_credentials = credentials;
  return *this;
}

CorsConfigBuilder & CorsConfigBuilder::with_max_age(int seconds) {
  config_.max_age_seconds = seconds;
  return *this;
}

CorsConfig CorsConfigBuilder::build() {
  // Enable CORS only if origins are configured
  config_.enabled = !config_.allowed_origins.empty();

  if (config_.enabled) {
    // Validate: credentials cannot be used with wildcard origin
    if (config_.allow_credentials) {
      auto has_wildcard = std::find(config_.allowed_origins.begin(), config_.allowed_origins.end(), "*");
      if (has_wildcard != config_.allowed_origins.end()) {
        throw std::invalid_argument("CORS: allow_credentials cannot be true when allowed_origins contains '*'");
      }
    }

    // Build cached header strings
    for (const auto & method : config_.allowed_methods) {
      if (!config_.methods_header.empty()) {
        config_.methods_header += ", ";
      }
      config_.methods_header += method;
    }

    for (const auto & header : config_.allowed_headers) {
      if (!config_.headers_header.empty()) {
        config_.headers_header += ", ";
      }
      config_.headers_header += header;
    }
  }

  return std::move(config_);
}

}  // namespace ros2_medkit_gateway
