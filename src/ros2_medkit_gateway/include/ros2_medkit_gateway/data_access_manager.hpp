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

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/ros2_cli_wrapper.hpp"
#include "ros2_medkit_gateway/output_parser.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

class DataAccessManager {
public:
    explicit DataAccessManager(rclcpp::Node* node);

    /**
     * @brief Get a single sample from a specific topic
     */
    json get_topic_sample(
        const std::string& topic_name,
        double timeout_sec = 3.0
    );

    /**
     * @brief Get all data from topics under a component's namespace
     */
    json get_component_data(
        const std::string& component_namespace,
        double timeout_sec = 3.0
    );

private:
    /**
     * @brief Find all topics under a given namespace
     */
    std::vector<std::string> find_component_topics(
        const std::string& component_namespace
    );

    rclcpp::Node* node_;
    std::unique_ptr<ROS2CLIWrapper> cli_wrapper_;
    std::unique_ptr<OutputParser> output_parser_;
    int max_parallel_samples_;
};

}  // namespace ros2_medkit_gateway
