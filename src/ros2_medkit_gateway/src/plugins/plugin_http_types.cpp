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

#include "ros2_medkit_gateway/plugins/plugin_http_types.hpp"

#include <httplib.h>

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {

// --- PluginRequest ---

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}

std::string PluginRequest::path_param(size_t index) const {
  const auto & req = *static_cast<const httplib::Request *>(impl_);
  if (index < req.matches.size()) {
    return req.matches[static_cast<std::ssub_match::difference_type>(index)].str();
  }
  return {};
}

std::string PluginRequest::header(const std::string & name) const {
  const auto & req = *static_cast<const httplib::Request *>(impl_);
  return req.get_header_value(name);
}

std::string PluginRequest::path() const {
  return static_cast<const httplib::Request *>(impl_)->path;
}

std::string PluginRequest::body() const {
  return static_cast<const httplib::Request *>(impl_)->body;
}

// --- PluginResponse ---

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}

void PluginResponse::send_json(const nlohmann::json & data) {
  handlers::HandlerContext::send_json(*static_cast<httplib::Response *>(impl_), data);
}

void PluginResponse::send_error(int status, const std::string & error_code, const std::string & message,
                                const nlohmann::json & parameters) {
  handlers::HandlerContext::send_error(*static_cast<httplib::Response *>(impl_), status, error_code, message,
                                       parameters);
}

}  // namespace ros2_medkit_gateway
