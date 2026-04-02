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

#include "ros2_medkit_gateway/discovery/host_info_provider.hpp"

#include <sys/utsname.h>
#include <unistd.h>

#include <cctype>
#include <fstream>
#include <string>

namespace ros2_medkit_gateway {

HostInfoProvider::HostInfoProvider() {
  read_host_info();
  build_component();
}

void HostInfoProvider::read_host_info() {
  // Hostname via gethostname()
  char buf[256];  // NOLINT(cppcoreguidelines-avoid-c-arrays)
  if (gethostname(buf, sizeof(buf)) == 0) {
    buf[sizeof(buf) - 1] = '\0';
    hostname_ = buf;
  } else {
    hostname_ = "unknown";
  }

  // OS via /etc/os-release PRETTY_NAME
  os_ = "Unknown OS";
  std::ifstream os_release("/etc/os-release");
  if (os_release.is_open()) {
    std::string line;
    while (std::getline(os_release, line)) {
      if (line.rfind("PRETTY_NAME=", 0) == 0) {
        // Strip PRETTY_NAME= prefix and surrounding quotes
        os_ = line.substr(12);
        if (os_.size() >= 2 && os_.front() == '"' && os_.back() == '"') {
          os_ = os_.substr(1, os_.size() - 2);
        }
        break;
      }
    }
  }

  // Architecture via uname()
  struct utsname uts {};
  if (uname(&uts) == 0) {
    arch_ = uts.machine;
  } else {
    arch_ = "unknown";
  }
}

std::string HostInfoProvider::sanitize_entity_id(const std::string & input) {
  std::string result;
  result.reserve(input.size());

  for (char c : input) {
    if (c == '.' || c == ' ') {
      result += '_';
    } else if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
      result += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    // Strip all other characters
  }

  // Truncate to max 256 characters
  constexpr size_t kMaxEntityIdLength = 256;
  if (result.size() > kMaxEntityIdLength) {
    result.resize(kMaxEntityIdLength);
  }

  // Fallback: if the input contained only special characters, the result
  // would be empty. Use a safe default to prevent empty entity IDs.
  if (result.empty()) {
    result = "unknown_host";
  }

  return result;
}

void HostInfoProvider::build_component() {
  component_.id = sanitize_entity_id(hostname_);
  component_.name = hostname_;
  component_.type = "Component";
  component_.source = "runtime";
  component_.description = os_ + " on " + arch_;
  component_.host_metadata = json{{"hostname", hostname_}, {"os", os_}, {"arch", arch_}};
}

}  // namespace ros2_medkit_gateway
