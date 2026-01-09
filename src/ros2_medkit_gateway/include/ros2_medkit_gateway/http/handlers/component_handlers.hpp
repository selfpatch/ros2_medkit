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

#ifndef ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__COMPONENT_HANDLERS_HPP_
#define ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__COMPONENT_HANDLERS_HPP_

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for component-related REST API endpoints.
 *
 * Provides handlers for:
 * - GET /components - List all components
 * - GET /components/{component_id}/data - Get all topic data for a component
 * - GET /components/{component_id}/data/{topic_name} - Get specific topic data
 * - PUT /components/{component_id}/data/{topic_name} - Publish to a topic
 */
class ComponentHandlers {
 public:
  /**
   * @brief Construct component handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit ComponentHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /components - list all components.
   */
  void handle_list_components(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/data - get all topic data.
   */
  void handle_component_data(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/data/{topic_name} - get specific topic.
   */
  void handle_component_topic_data(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle PUT /components/{component_id}/data/{topic_name} - publish to topic.
   */
  void handle_component_topic_publish(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__COMPONENT_HANDLERS_HPP_
