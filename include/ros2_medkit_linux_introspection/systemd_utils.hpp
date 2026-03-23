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

#pragma once

#include <cstdio>
#include <string>

namespace ros2_medkit_linux_introspection {

/// Escape a systemd unit name for use in D-Bus object paths.
/// Non-alphanumeric characters (except underscore) are hex-encoded as _XX.
inline std::string escape_unit_for_dbus(const std::string & unit) {
  std::string result;
  result.reserve(unit.size());
  for (char c : unit) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') {
      result += c;
    } else {
      char buf[8];
      snprintf(buf, sizeof(buf), "_%02x", static_cast<unsigned char>(c));
      result += buf;
    }
  }
  return result;
}

}  // namespace ros2_medkit_linux_introspection
