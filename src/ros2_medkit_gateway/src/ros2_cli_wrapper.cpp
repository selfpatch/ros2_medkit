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

#include "ros2_medkit_gateway/ros2_cli_wrapper.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>

namespace ros2_medkit_gateway {

// Custom deleter for FILE* from popen
struct PipeDeleter {
    void operator()(FILE* fp) const {
        if (fp) pclose(fp);
    }
};

std::string ROS2CLIWrapper::exec(const std::string& command) {
    std::array<char, 128> buffer;
    std::string result;

    std::unique_ptr<FILE, PipeDeleter> pipe(popen(command.c_str(), "r"));

    if (!pipe) {
        throw std::runtime_error("Failed to execute command: " + command);
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    // Check command exit status (release prevents double-close in deleter)
    int exit_code = pclose(pipe.release());
    if (exit_code != 0) {
        throw std::runtime_error(
            "Command failed with exit code " + std::to_string(WEXITSTATUS(exit_code)) +
            ": " + command
        );
    }

    return result;
}

bool ROS2CLIWrapper::is_command_available(const std::string& command) {
    // Search in PATH for the command using access() instead of system()
    // This avoids shell execution and potential command injection
    const char* path_env = std::getenv("PATH");
    if (!path_env) {
        return false;
    }

    std::string path(path_env);
    std::istringstream ss(path);
    std::string dir;

    while (std::getline(ss, dir, ':')) {
        if (dir.empty()) {
            continue;
        }
        std::string full_path = dir + "/" + command;
        if (access(full_path.c_str(), X_OK) == 0) {
            return true;
        }
    }

    return false;
}

// TODO(mfaferek93): Improve command injection protection
// Current: Single-quote escaping (reasonably safe, prevents shell expansion)
// Future options:
//   1. Add input validation: whitelist ROS 2 naming conventions (alphanumeric, /, _)
//   2. Reject topic names with suspicious characters
//   3. Use fork/exec or posix_spawn instead of shell execution (best solution)
std::string ROS2CLIWrapper::escape_shell_arg(const std::string& arg) {
    std::string escaped = "'";
    for (char c : arg) {
        if (c == '\'') {
            escaped += "'\\''";
        } else {
            escaped += c;
        }
    }
    escaped += "'";
    return escaped;
}

}  // namespace ros2_medkit_gateway
