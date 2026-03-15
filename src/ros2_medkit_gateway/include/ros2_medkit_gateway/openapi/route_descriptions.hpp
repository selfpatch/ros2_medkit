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

#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {
namespace openapi {

class CapabilityGenerator;  // Forward declaration for friend access

// =============================================================================
// SchemaDesc - describes an OpenAPI schema object
// =============================================================================

class SchemaDesc {
 public:
  // Factory methods for primitive types
  static SchemaDesc string() {
    SchemaDesc s;
    s.json_["type"] = "string";
    return s;
  }

  static SchemaDesc number() {
    SchemaDesc s;
    s.json_["type"] = "number";
    return s;
  }

  static SchemaDesc integer() {
    SchemaDesc s;
    s.json_["type"] = "integer";
    return s;
  }

  static SchemaDesc boolean() {
    SchemaDesc s;
    s.json_["type"] = "boolean";
    return s;
  }

  // Factory method for arrays
  static SchemaDesc array(SchemaDesc items) {
    SchemaDesc s;
    s.json_["type"] = "array";
    s.json_["items"] = std::move(items.json_);
    return s;
  }

  // Factory method for objects
  static SchemaDesc object() {
    SchemaDesc s;
    s.json_["type"] = "object";
    return s;
  }

  // Factory method for $ref
  static SchemaDesc ref(const std::string & ref_name) {
    SchemaDesc s;
    s.json_["$ref"] = "#/components/schemas/" + ref_name;
    return s;
  }

  // Add a property to an object schema
  SchemaDesc & property(const std::string & name, SchemaDesc schema) {
    json_["properties"][name] = std::move(schema.json_);
    return *this;
  }

  // Set required fields for an object schema
  SchemaDesc & required(const std::vector<std::string> & fields) {
    json_["required"] = fields;
    return *this;
  }

  // Convert to JSON
  nlohmann::json to_json() const {
    return json_;
  }

 private:
  nlohmann::json json_;
};

// =============================================================================
// OperationDesc - describes a single HTTP operation (GET, POST, etc.)
// =============================================================================

class OperationDesc {
 public:
  // Set the operation description text
  OperationDesc & description(std::string desc) {
    description_ = std::move(desc);
    return *this;
  }

  // Add a path parameter
  OperationDesc & path_param(const std::string & name, const std::string & desc) {
    nlohmann::json param;
    param["name"] = name;
    param["in"] = "path";
    param["required"] = true;
    param["description"] = desc;
    param["schema"]["type"] = "string";
    parameters_.push_back(std::move(param));
    return *this;
  }

  // Add a query parameter
  OperationDesc & query_param(const std::string & name, const std::string & desc, const SchemaDesc & schema,
                              bool is_required = false) {
    nlohmann::json param;
    param["name"] = name;
    param["in"] = "query";
    param["required"] = is_required;
    param["description"] = desc;
    param["schema"] = schema.to_json();
    parameters_.push_back(std::move(param));
    return *this;
  }

  // Set the request body schema
  OperationDesc & request_body(SchemaDesc schema) {
    request_body_ = std::move(schema);
    has_request_body_ = true;
    return *this;
  }

  // Add a response for a given HTTP status code
  OperationDesc & response(int status_code, const SchemaDesc & schema, const std::string & desc = "") {
    nlohmann::json resp;
    resp["description"] = desc.empty() ? "Response" : desc;
    resp["content"]["application/json"]["schema"] = schema.to_json();
    responses_[std::to_string(status_code)] = std::move(resp);
    return *this;
  }

  // Convert to JSON
  nlohmann::json to_json() const {
    nlohmann::json j;

    if (!description_.empty()) {
      j["description"] = description_;
    }

    if (!parameters_.empty()) {
      j["parameters"] = parameters_;
    }

    if (has_request_body_) {
      j["requestBody"]["content"]["application/json"]["schema"] = request_body_.to_json();
    }

    if (!responses_.empty()) {
      j["responses"] = responses_;
    }

    return j;
  }

 private:
  std::string description_;
  std::vector<nlohmann::json> parameters_;
  SchemaDesc request_body_;
  bool has_request_body_ = false;
  std::map<std::string, nlohmann::json> responses_;
};

// =============================================================================
// PathDescBuilder - builds a path item with method operations
// =============================================================================

class PathDescBuilder {
 public:
  // Set the summary for this path
  PathDescBuilder & summary(std::string s) {
    summary_ = std::move(s);
    return *this;
  }

  // Set the GET operation
  PathDescBuilder & get(OperationDesc op) {
    operations_["get"] = std::move(op);
    return *this;
  }

  // Set the POST operation
  PathDescBuilder & post(OperationDesc op) {
    operations_["post"] = std::move(op);
    return *this;
  }

  // Set the PUT operation
  PathDescBuilder & put(OperationDesc op) {
    operations_["put"] = std::move(op);
    return *this;
  }

  // Set the DELETE operation
  PathDescBuilder & delete_op(OperationDesc op) {
    operations_["delete"] = std::move(op);
    return *this;
  }

  // Convert to JSON
  nlohmann::json to_json() const {
    nlohmann::json j;

    for (const auto & [method, op] : operations_) {
      auto op_json = op.to_json();
      if (!summary_.empty()) {
        op_json["summary"] = summary_;
      }
      j[method] = std::move(op_json);
    }

    return j;
  }

 private:
  std::string summary_;
  std::map<std::string, OperationDesc> operations_;
};

// =============================================================================
// RouteDescriptions - immutable collection of path descriptions
// =============================================================================

class RouteDescriptionsTestAccess;  // Forward declaration for test friend

class RouteDescriptions {
  friend class RouteDescriptionBuilder;
  friend class CapabilityGenerator;
  friend class RouteDescriptionsTestAccess;

 public:
  RouteDescriptions(const RouteDescriptions &) = default;
  RouteDescriptions(RouteDescriptions &&) = default;
  RouteDescriptions & operator=(const RouteDescriptions &) = default;
  RouteDescriptions & operator=(RouteDescriptions &&) = default;

 private:
  RouteDescriptions() = default;

  nlohmann::json to_json() const {
    nlohmann::json result = nlohmann::json::object();
    for (const auto & [path, builder] : paths_) {
      result[path] = builder.to_json();
    }
    return result;
  }

  bool empty() const {
    return paths_.empty();
  }

  std::map<std::string, PathDescBuilder> paths_;
};

// =============================================================================
// RouteDescriptionBuilder - fluent builder for RouteDescriptions
// =============================================================================

class RouteDescriptionBuilder {
 public:
  // Add a new path and return a reference to its PathDescBuilder for chaining
  PathDescBuilder & add(const std::string & path) {
    return paths_[path];
  }

  // Build the immutable RouteDescriptions
  RouteDescriptions build() {
    RouteDescriptions desc;
    desc.paths_ = std::move(paths_);
    return desc;
  }

 private:
  std::map<std::string, PathDescBuilder> paths_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
