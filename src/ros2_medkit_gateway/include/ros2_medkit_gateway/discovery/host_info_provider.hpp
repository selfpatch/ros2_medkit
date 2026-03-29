// Copyright 2026 Selfpatch GmbH
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

#include "ros2_medkit_gateway/discovery/models/component.hpp"

#include <string>

namespace ros2_medkit_gateway {

/**
 * @brief Reads local host system info and produces a default Component entity.
 *
 * Used in runtime_only discovery mode to create a single host-level Component
 * instead of synthetic per-namespace Components.
 */
class HostInfoProvider {
 public:
  HostInfoProvider();

  /**
   * @brief Get the default Component representing the local host system.
   *
   * The Component has:
   *   - id = sanitized hostname (alphanumeric + underscore + hyphen, lowercase)
   *   - name = raw hostname
   *   - source = "runtime"
   *   - description = "OS on arch"
   *   - x-medkit.host metadata with hostname, os, arch
   *
   * @return Component entity for the local host
   */
  const Component & get_default_component() const {
    return component_;
  }

  /**
   * @brief Sanitize a string to a valid SOVD entity ID.
   *
   * Converts dots and spaces to underscores, strips other invalid characters,
   * converts to lowercase, and truncates to 256 characters.
   *
   * @param input Raw string (e.g., hostname)
   * @return Sanitized entity ID (alphanumeric + underscore + hyphen only)
   */
  static std::string sanitize_entity_id(const std::string & input);

  /// @brief Get raw hostname
  const std::string & hostname() const {
    return hostname_;
  }

  /// @brief Get OS description (from /etc/os-release PRETTY_NAME)
  const std::string & os() const {
    return os_;
  }

  /// @brief Get CPU architecture (from uname)
  const std::string & arch() const {
    return arch_;
  }

 private:
  void read_host_info();
  void build_component();

  std::string hostname_;
  std::string os_;
  std::string arch_;
  Component component_;
};

}  // namespace ros2_medkit_gateway
