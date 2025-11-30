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

#pragma once

#include <string>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief CORS (Cross-Origin Resource Sharing) configuration settings
 */
struct CorsConfig {
  bool enabled{false};
  std::vector<std::string> allowed_origins;
  std::vector<std::string> allowed_methods;
  std::vector<std::string> allowed_headers;
  bool allow_credentials{false};
  int max_age_seconds{86400};

  // Pre-built header values for performance
  std::string methods_header;
  std::string headers_header;
};

/**
 * @brief Builder for CorsConfig with fluent interface
 *
 * Usage:
 *   auto config = CorsConfigBuilder()
 *       .with_origins({"http://localhost:5173"})
 *       .with_methods({"GET", "PUT", "OPTIONS"})
 *       .with_headers({"Content-Type", "Accept"})
 *       .with_credentials(true)
 *       .with_max_age(3600)
 *       .build();
 */
class CorsConfigBuilder {
 public:
  CorsConfigBuilder & with_origins(std::vector<std::string> origins);
  CorsConfigBuilder & with_methods(std::vector<std::string> methods);
  CorsConfigBuilder & with_headers(std::vector<std::string> headers);
  CorsConfigBuilder & with_credentials(bool credentials);
  CorsConfigBuilder & with_max_age(int seconds);
  CorsConfig build();

 private:
  CorsConfig config_;
};

}  // namespace ros2_medkit_gateway
