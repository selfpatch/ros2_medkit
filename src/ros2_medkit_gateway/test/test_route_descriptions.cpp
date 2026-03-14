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

#include <gtest/gtest.h>

#include "ros2_medkit_gateway/openapi/route_descriptions.hpp"

using namespace ros2_medkit_gateway::openapi;

// Test helper in same namespace as RouteDescriptions to match friend declaration
namespace ros2_medkit_gateway::openapi {
class RouteDescriptionsTestAccess {
 public:
  static nlohmann::json to_json(const RouteDescriptions & desc) {
    return desc.to_json();
  }
  static bool empty(const RouteDescriptions & desc) {
    return desc.empty();
  }
};
}  // namespace ros2_medkit_gateway::openapi

using ros2_medkit_gateway::openapi::RouteDescriptionsTestAccess;

TEST(RouteDescriptionsTest, EmptyBuilderProducesEmptyDescriptions) {
  auto desc = RouteDescriptionBuilder{}.build();
  EXPECT_TRUE(RouteDescriptionsTestAccess::empty(desc));
  auto json = RouteDescriptionsTestAccess::to_json(desc);
  EXPECT_TRUE(json.is_object());
  EXPECT_TRUE(json.empty());
}

TEST(RouteDescriptionsTest, SingleGetEndpoint) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-beacon/{entity_id}")
      .summary("Get beacon data")
      .get(OperationDesc()
               .description("Returns beacon discovery data")
               .path_param("entity_id", "Entity identifier")
               .response(200, SchemaDesc::object()
                                  .property("beacons", SchemaDesc::array(SchemaDesc::string()))
                                  .required({"beacons"}))
               .response(404, SchemaDesc::ref("GenericError")));
  auto desc = b.build();
  EXPECT_FALSE(RouteDescriptionsTestAccess::empty(desc));

  auto json = RouteDescriptionsTestAccess::to_json(desc);
  ASSERT_TRUE(json.contains("/x-medkit-beacon/{entity_id}"));
  auto & path = json["/x-medkit-beacon/{entity_id}"];
  ASSERT_TRUE(path.contains("get"));
  EXPECT_EQ(path["get"]["summary"], "Get beacon data");
  EXPECT_EQ(path["get"]["responses"]["200"]["content"]["application/json"]["schema"]["type"], "object");
  EXPECT_TRUE(path["get"]["responses"].contains("404"));
}

TEST(RouteDescriptionsTest, MultipleEndpoints) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-foo").summary("Foo").get(OperationDesc().response(200, SchemaDesc::string()));
  b.add("/x-medkit-bar").summary("Bar").post(OperationDesc().response(201, SchemaDesc::string()));
  auto desc = b.build();
  auto json = RouteDescriptionsTestAccess::to_json(desc);
  EXPECT_EQ(json.size(), 2);
  EXPECT_TRUE(json.contains("/x-medkit-foo"));
  EXPECT_TRUE(json.contains("/x-medkit-bar"));
}

TEST(RouteDescriptionsTest, PostWithRequestBody) {
  RouteDescriptionBuilder b;
  b.add("/x-medkit-action")
      .summary("Execute action")
      .post(OperationDesc()
                .request_body(SchemaDesc::object().property("command", SchemaDesc::string()))
                .response(202, SchemaDesc::object().property("status", SchemaDesc::string())));
  auto desc = b.build();
  auto json = RouteDescriptionsTestAccess::to_json(desc);
  auto & post = json["/x-medkit-action"]["post"];
  EXPECT_TRUE(post.contains("requestBody"));
  EXPECT_TRUE(post["requestBody"]["content"]["application/json"].contains("schema"));
}

TEST(RouteDescriptionsTest, SchemaTypes) {
  auto str = SchemaDesc::string().to_json();
  EXPECT_EQ(str["type"], "string");
  auto num = SchemaDesc::number().to_json();
  EXPECT_EQ(num["type"], "number");
  auto integer = SchemaDesc::integer().to_json();
  EXPECT_EQ(integer["type"], "integer");
  auto boolean = SchemaDesc::boolean().to_json();
  EXPECT_EQ(boolean["type"], "boolean");
  auto arr = SchemaDesc::array(SchemaDesc::string()).to_json();
  EXPECT_EQ(arr["type"], "array");
  EXPECT_EQ(arr["items"]["type"], "string");
  auto obj = SchemaDesc::object()
                 .property("name", SchemaDesc::string())
                 .property("count", SchemaDesc::integer())
                 .required({"name"})
                 .to_json();
  EXPECT_EQ(obj["type"], "object");
  EXPECT_TRUE(obj["properties"].contains("name"));
  EXPECT_EQ(obj["required"][0], "name");
}
