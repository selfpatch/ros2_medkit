// Copyright 2025 mfaferek93
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

#include <stdexcept>
#include <string>

namespace ros2_medkit_gateway {

class ROS2CLIWrapper {
 public:
  ROS2CLIWrapper() = default;

  /**
   * @brief Execute shell command and return stdout
   */
  std::string exec(const std::string & command);

  /**
   * @brief Check if command exists in PATH
   */
  bool is_command_available(const std::string & command);

  /**
   * @brief Escape shell arguments
   */
  static std::string escape_shell_arg(const std::string & arg);
};

}  // namespace ros2_medkit_gateway
