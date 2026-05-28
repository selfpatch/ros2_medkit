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

#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"

#include <algorithm>

#include <httplib.h>

#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/http/detail/primitives.hpp"

namespace ros2_medkit_gateway {

// --- PluginRequest ---

PluginRequest::PluginRequest(const void * impl) : impl_(impl) {
}

std::string PluginRequest::path_param(size_t index) const {
  const auto & req = *static_cast<const httplib::Request *>(impl_);
  if (index < req.matches.size()) {
    return req.matches[index].str();
  }
  return {};
}

std::string PluginRequest::header(const std::string & name) const {
  const auto & req = *static_cast<const httplib::Request *>(impl_);
  return req.get_header_value(name);
}

const std::string & PluginRequest::path() const {
  return static_cast<const httplib::Request *>(impl_)->path;
}

const std::string & PluginRequest::body() const {
  return static_cast<const httplib::Request *>(impl_)->body;
}

std::string PluginRequest::query_param(const std::string & name) const {
  return static_cast<const httplib::Request *>(impl_)->get_param_value(name);
}

// --- PluginResponse ---

PluginResponse::PluginResponse(void * impl) : impl_(impl) {
}

void PluginResponse::send_json(const nlohmann::json & data) {
  // Call the framework primitive directly so the plugin ABI does not depend on
  // HandlerContext's public surface (commit 30 prunes send_json from
  // HandlerContext). Pass kKeepCurrentStatus to preserve the legacy
  // PluginResponse::send_json contract that left res.status untouched (the
  // gateway's plugin route adapter pre-set it before delegation in some paths).
  http::detail::write_json_body(http::detail::FrameworkOrPluginAccess{}, *static_cast<httplib::Response *>(impl_), data,
                                http::detail::kKeepCurrentStatus);
}

void PluginResponse::send_error(int status, const std::string & error_code, const std::string & message,
                                const nlohmann::json & parameters) {
  // Pre-clamp matches the legacy behavior so the wire status stays in the
  // SOVD error range. write_generic_error clamps internally as well, but
  // keeping the explicit clamp here preserves the byte-for-byte status code
  // contract callers may have asserted against.
  ErrorInfo info;
  info.code = error_code;
  info.message = message;
  info.http_status = std::clamp(status, 400, 599);
  info.params = parameters;
  http::detail::write_generic_error(http::detail::FrameworkOrPluginAccess{}, *static_cast<httplib::Response *>(impl_),
                                    info);
}

}  // namespace ros2_medkit_gateway
