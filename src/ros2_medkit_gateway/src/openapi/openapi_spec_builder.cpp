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

#include "openapi_spec_builder.hpp"

#include "schema_builder.hpp"

namespace ros2_medkit_gateway {
namespace openapi {

OpenApiSpecBuilder & OpenApiSpecBuilder::info(const std::string & title, const std::string & version) {
  title_ = title;
  version_ = version;
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::server(const std::string & url, const std::string & description) {
  servers_.push_back({url, description});
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::sovd_version(const std::string & version) {
  sovd_version_ = version;
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::add_paths(const nlohmann::json & paths) {
  for (auto & [key, val] : paths.items()) {
    paths_[key] = val;
  }
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::add_schemas(const nlohmann::json & schemas) {
  for (auto & [key, val] : schemas.items()) {
    schemas_[key] = val;
  }
  return *this;
}

OpenApiSpecBuilder & OpenApiSpecBuilder::security_scheme(const std::string & name, const nlohmann::json & scheme) {
  security_schemes_.push_back({name, scheme});
  return *this;
}

nlohmann::json OpenApiSpecBuilder::build() const {
  nlohmann::json spec;

  // 1. Always OpenAPI 3.1.0
  spec["openapi"] = "3.1.0";

  // 2. Info block
  spec["info"]["title"] = title_;
  spec["info"]["version"] = version_;
  if (!sovd_version_.empty()) {
    spec["info"]["x-sovd-version"] = sovd_version_;
  }

  // 3. Servers
  spec["servers"] = nlohmann::json::array();
  for (const auto & s : servers_) {
    nlohmann::json entry;
    entry["url"] = s.url;
    if (!s.description.empty()) {
      entry["description"] = s.description;
    }
    spec["servers"].push_back(std::move(entry));
  }

  // 4. Paths
  spec["paths"] = paths_;

  // 5. Components - schemas
  spec["components"]["schemas"] = schemas_;

  // 6. Components - GenericError response (always present)
  spec["components"]["responses"]["GenericError"]["description"] = "SOVD GenericError response";
  spec["components"]["responses"]["GenericError"]["content"]["application/json"]["schema"] =
      SchemaBuilder::generic_error();

  // 7. Security schemes (if any)
  if (!security_schemes_.empty()) {
    spec["security"] = nlohmann::json::array();
    for (const auto & ss : security_schemes_) {
      spec["components"]["securitySchemes"][ss.name] = ss.scheme;
      spec["security"].push_back({{ss.name, nlohmann::json::array()}});
    }
  }

  return spec;
}

}  // namespace openapi
}  // namespace ros2_medkit_gateway
