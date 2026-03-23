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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/auth/auth_config.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Interface for authentication requirement policies
 *
 * Allows customizing when authentication is required for different
 * HTTP methods and paths using the Strategy pattern.
 *
 * @verifies REQ_INTEROP_086
 */
class IAuthRequirementPolicy {
 public:
  virtual ~IAuthRequirementPolicy() = default;

  /**
   * @brief Check if authentication is required for a request
   * @param method HTTP method (GET, POST, PUT, DELETE, etc.)
   * @param path Request path
   * @return true if authentication is required
   */
  virtual bool requires_authentication(const std::string & method, const std::string & path) const = 0;

  /**
   * @brief Get a human-readable description of this policy
   * @return Description string
   */
  virtual std::string description() const = 0;
};

/**
 * @brief Policy that never requires authentication
 */
class NoAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  bool requires_authentication(const std::string & method, const std::string & path) const override {
    (void)method;
    (void)path;
    return false;
  }

  std::string description() const override {
    return "NoAuth: Authentication is never required";
  }
};

/**
 * @brief Policy that always requires authentication
 *
 * Except for public endpoints (auth endpoints, health check)
 */
class AllAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  bool requires_authentication(const std::string & method, const std::string & path) const override {
    (void)method;
    // Auth endpoints are always public (to allow login)
    return path.find("/api/v1/auth/") != 0;
  }

  std::string description() const override {
    return "AllAuth: Authentication required for all endpoints except /auth/*";
  }
};

/**
 * @brief Policy that requires authentication only for write operations
 *
 * Write operations: POST, PUT, DELETE, PATCH
 * Read operations: GET, HEAD, OPTIONS (no auth required)
 *
 * Auth endpoints are always public.
 */
class WriteOnlyAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  bool requires_authentication(const std::string & method, const std::string & path) const override {
    // Auth endpoints are always public
    if (path.find("/api/v1/auth/") == 0) {
      return false;
    }

    // Write operations require auth
    return method == "POST" || method == "PUT" || method == "DELETE" || method == "PATCH";
  }

  std::string description() const override {
    return "WriteOnly: Authentication required for POST, PUT, DELETE, PATCH operations";
  }
};

/**
 * @brief Policy with configurable public paths
 *
 * Allows specifying a list of paths that don't require authentication.
 * Supports wildcards (* for single segment, ** for multiple segments).
 */
class ConfigurableAuthRequirementPolicy : public IAuthRequirementPolicy {
 public:
  /**
   * @brief Construct with list of public paths
   * @param public_paths Paths that don't require authentication (supports wildcards)
   * @param require_for_reads If true, require auth even for GET requests
   */
  explicit ConfigurableAuthRequirementPolicy(const std::vector<std::string> & public_paths,
                                             bool require_for_reads = false);

  /**
   * @brief Construct from auth requirements map
   * @param auth_requirements Map of path patterns to auth requirement levels
   */
  explicit ConfigurableAuthRequirementPolicy(
      const std::unordered_map<std::string, AuthRequirement> & auth_requirements);

  bool requires_authentication(const std::string & method, const std::string & path) const override;

  std::string description() const override {
    return "Configurable: Custom per-path authentication requirements";
  }

  /**
   * @brief Add a public path
   * @param path Path pattern (supports * and ** wildcards)
   */
  void add_public_path(const std::string & path);

 private:
  bool is_public_path(const std::string & path) const;
  AuthRequirement get_path_requirement(const std::string & path) const;
  static bool matches_path(const std::string & pattern, const std::string & path);

  std::vector<std::string> public_paths_;
  std::unordered_map<std::string, AuthRequirement> auth_requirements_;
  bool require_for_reads_;
  bool use_requirements_map_;
};

/**
 * @brief Factory to create auth requirement policies from configuration
 */
class AuthRequirementPolicyFactory {
 public:
  /**
   * @brief Create policy from AuthRequirement enum
   * @param requirement The auth requirement level
   * @return Policy implementation
   */
  static std::unique_ptr<IAuthRequirementPolicy> create(AuthRequirement requirement);

  /**
   * @brief Create policy from AuthConfig
   * @param config Full auth configuration
   * @return Policy implementation based on config.enabled and config.auth_requirements
   */
  static std::unique_ptr<IAuthRequirementPolicy> create(const AuthConfig & config);
};

}  // namespace ros2_medkit_gateway
