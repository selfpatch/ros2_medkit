// Copyright 2026 mfaferek93
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

#include "ros2_medkit_gateway/core/faults/freeze_frame_source.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Read a string field totally: json::value() throws type_error.302 when the
/// key is present but not a string, and plugin content is untrusted.
std::string string_field(const nlohmann::json & item, const char * field) {
  const auto it = item.find(field);
  return it != item.end() && it->is_string() ? it->get<std::string>() : std::string();
}

}  // namespace

nlohmann::json FreezeFrameSource::values_from_list_content(const nlohmann::json & content) {
  if (!content.contains("items") || !content["items"].is_array()) {
    return content;
  }
  nlohmann::json values = nlohmann::json::object();
  for (const auto & item : content["items"]) {
    if (!item.is_object()) {
      continue;
    }
    std::string key = string_field(item, "id");
    if (key.empty()) {
      key = string_field(item, "name");
    }
    if (key.empty()) {
      continue;
    }
    values[key] = item.contains("value") ? item["value"] : nlohmann::json{};
  }
  return values;
}

bool FreezeFrameSource::content_has_live_data(const nlohmann::json & content) {
  if (!content.is_object()) {
    return false;
  }
  // Judged on the values, not on the link flag. A bridge that serves its last
  // known values while disconnected is exactly what a loss-of-comms fault wants
  // frozen - refusing on `connected: false` denied a frame to the one fault
  // where the values before the link died are the whole story. The all-null row
  // a genuinely cold cache yields is still rejected, by values_have_data().
  return content.contains("items") && content["items"].is_array() && !content["items"].empty();
}

bool FreezeFrameSource::content_reports_disconnected(const nlohmann::json & content) {
  return content.is_object() && content.contains("connected") && content["connected"].is_boolean() &&
         !content["connected"].get<bool>();
}

bool FreezeFrameSource::values_have_data(const nlohmann::json & values) {
  if (values.is_object()) {
    for (const auto & entry : values.items()) {
      if (!entry.value().is_null()) {
        return true;
      }
    }
    return false;
  }
  return !values.is_null();
}

}  // namespace ros2_medkit_gateway
