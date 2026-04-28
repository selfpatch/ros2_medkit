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

#include "ros2_medkit_gateway/core/entity_validation.hpp"

#include <iomanip>
#include <sstream>

namespace ros2_medkit_gateway {

tl::expected<void, std::string> validate_entity_id(const std::string & entity_id) {
  if (entity_id.empty()) {
    return tl::unexpected("Entity ID cannot be empty");
  }

  if (entity_id.length() > 256) {
    return tl::unexpected("Entity ID too long (max 256 characters)");
  }

  // Allow: alphanumeric (a-z, A-Z, 0-9), underscore (_), hyphen (-)
  // Reject: forward slash (conflicts with URL routing), special characters, escape sequences
  // Note: Hyphens are allowed in manifest entity IDs (e.g., "engine-ecu", "front-left-door")
  for (char c : entity_id) {
    bool is_alphanumeric = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    bool is_allowed_special = (c == '_' || c == '-');

    if (!is_alphanumeric && !is_allowed_special) {
      std::string char_repr;
      if (c < 32 || c > 126) {
        std::ostringstream oss;
        oss << "0x" << std::hex << std::setfill('0') << std::setw(2)
            << static_cast<unsigned int>(static_cast<unsigned char>(c));
        char_repr = oss.str();
      } else {
        char_repr = std::string(1, c);
      }
      return tl::unexpected("Entity ID contains invalid character: '" + char_repr +
                            "'. Only alphanumeric, underscore and hyphen are allowed");
    }
  }

  return {};
}

}  // namespace ros2_medkit_gateway
