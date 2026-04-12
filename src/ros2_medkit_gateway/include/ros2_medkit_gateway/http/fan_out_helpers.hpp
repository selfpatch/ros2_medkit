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

#include <cctype>
#include <string>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/aggregation/aggregation_manager.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

namespace ros2_medkit_gateway {

inline std::string url_encode_param(const std::string & value) {
  std::string result;
  result.reserve(value.size());
  static constexpr char kHex[] = "0123456789ABCDEF";
  for (const auto ch : value) {
    auto c = static_cast<unsigned char>(ch);
    if (std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == '~') {
      result += static_cast<char>(c);
    } else {
      result += '%';
      result += kHex[static_cast<size_t>(c >> 4u)];
      result += kHex[static_cast<size_t>(c & 0x0Fu)];
    }
  }
  return result;
}

inline std::string build_fan_out_path(const httplib::Request & req) {
  if (req.params.empty()) {
    return req.path;
  }
  std::string path = req.path;
  char sep = '?';
  for (const auto & [key, value] : req.params) {
    path += sep;
    path += url_encode_param(key);
    path += '=';
    path += url_encode_param(value);
    sep = '&';
  }
  return path;
}

inline void merge_peer_items(AggregationManager * agg, const httplib::Request & req, nlohmann::json & result,
                             XMedkit & ext) {
  if (agg == nullptr) {
    return;
  }
  if (req.has_header("X-Medkit-No-Fan-Out")) {
    return;
  }
  auto fan_path = build_fan_out_path(req);
  auto fan_result = agg->fan_out_get(fan_path, req.get_header_value("Authorization"));
  if (fan_result.merged_items.is_array() && !fan_result.merged_items.empty()) {
    if (!result.contains("items") || !result["items"].is_array()) {
      result["items"] = nlohmann::json::array();
    }
    for (const auto & item : fan_result.merged_items) {
      result["items"].push_back(item);
    }
  }
  if (fan_result.is_partial) {
    ext.add("partial", true);
    ext.add("failed_peers", fan_result.failed_peers);
  }
}

}  // namespace ros2_medkit_gateway
