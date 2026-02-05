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

#include "ros2_medkit_gateway/http/entity_path_utils.hpp"

#include <regex>
#include <vector>

namespace ros2_medkit_gateway {

namespace {

// Pattern definition for entity path matching
struct PatternDef {
  std::regex pattern;
  SovdEntityType type;
  bool is_nested;
};

// Get static patterns - order matters (nested patterns must come first)
const std::vector<PatternDef> & get_patterns() {
  // clang-format off
  static const std::vector<PatternDef> patterns = {
    // Nested entities (must be checked first due to more specific patterns)
    {std::regex(R"(/api/v1/areas/([^/]+)/subareas/([^/]+)(/.*)?$)"), SovdEntityType::AREA, true},
    {std::regex(R"(/api/v1/components/([^/]+)/subcomponents/([^/]+)(/.*)?$)"), SovdEntityType::COMPONENT, true},
    // Top-level entities
    {std::regex(R"(/api/v1/apps/([^/]+)(/.*)?$)"), SovdEntityType::APP, false},
    {std::regex(R"(/api/v1/components/([^/]+)(/.*)?$)"), SovdEntityType::COMPONENT, false},
    {std::regex(R"(/api/v1/areas/([^/]+)(/.*)?$)"), SovdEntityType::AREA, false},
    {std::regex(R"(/api/v1/functions/([^/]+)(/.*)?$)"), SovdEntityType::FUNCTION, false},
  };
  // clang-format on
  return patterns;
}

// Get entity type path segment
std::string get_type_segment(SovdEntityType type, bool is_nested) {
  if (is_nested) {
    switch (type) {
      case SovdEntityType::AREA:
        return "subareas";
      case SovdEntityType::COMPONENT:
        return "subcomponents";
      default:
        break;
    }
  }
  switch (type) {
    case SovdEntityType::APP:
      return "apps";
    case SovdEntityType::COMPONENT:
      return "components";
    case SovdEntityType::AREA:
      return "areas";
    case SovdEntityType::FUNCTION:
      return "functions";
    default:
      return "";
  }
}

}  // namespace

std::optional<EntityPathInfo> parse_entity_path(const std::string & request_path) {
  if (request_path.empty()) {
    return std::nullopt;
  }

  for (const auto & def : get_patterns()) {
    std::smatch match;
    if (std::regex_match(request_path, match, def.pattern)) {
      EntityPathInfo info;
      info.type = def.type;
      info.is_nested = def.is_nested;

      if (def.is_nested) {
        // For nested: match[1] = parent_id, match[2] = entity_id, match[3] = resource_path
        info.parent_id = match[1].str();
        info.entity_id = match[2].str();
        info.resource_path = match[3].matched ? match[3].str() : "";

        // Build entity path: /{parent_type}/{parent_id}/{sub_type}/{entity_id}
        std::string parent_segment = (def.type == SovdEntityType::AREA) ? "areas" : "components";
        std::string sub_segment = get_type_segment(def.type, true);
        info.entity_path.reserve(parent_segment.size() + info.parent_id.size() + sub_segment.size() +
                                 info.entity_id.size() + 4);
        info.entity_path = "/";
        info.entity_path.append(parent_segment)
            .append("/")
            .append(info.parent_id)
            .append("/")
            .append(sub_segment)
            .append("/")
            .append(info.entity_id);
      } else {
        // For top-level: match[1] = entity_id, match[2] = resource_path
        info.entity_id = match[1].str();
        info.resource_path = match[2].matched ? match[2].str() : "";

        // Build entity path: /{type}/{entity_id}
        std::string type_segment = get_type_segment(def.type, false);
        info.entity_path.reserve(type_segment.size() + info.entity_id.size() + 2);
        info.entity_path = "/";
        info.entity_path.append(type_segment).append("/").append(info.entity_id);
      }

      return info;
    }
  }

  return std::nullopt;
}

std::string extract_bulk_data_category(const std::string & path) {
  static const std::regex pattern(R"(/bulk-data/([^/]+))");
  std::smatch match;
  if (std::regex_search(path, match, pattern)) {
    return match[1].str();
  }
  return "";
}

std::string extract_bulk_data_id(const std::string & path) {
  static const std::regex pattern(R"(/bulk-data/[^/]+/([^/]+))");
  std::smatch match;
  if (std::regex_search(path, match, pattern)) {
    return match[1].str();
  }
  return "";
}

}  // namespace ros2_medkit_gateway
