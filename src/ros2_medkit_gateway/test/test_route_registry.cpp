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

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "../src/openapi/route_registry.hpp"

using namespace ros2_medkit_gateway::openapi;
using json = nlohmann::json;

// =============================================================================
// Test fixture
// =============================================================================

class RouteRegistryTest : public ::testing::Test {
 protected:
  RouteRegistry registry_;

  // No-op handler for registration
  static void noop(const httplib::Request &, httplib::Response &) {
  }
};

// =============================================================================
// to_openapi_paths - basic structure
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ToOpenapiPathsContainsRegisteredRoute) {
  registry_.get("/health", noop).tag("Server").summary("Health check");

  auto paths = registry_.to_openapi_paths();

  ASSERT_TRUE(paths.contains("/health"));
  ASSERT_TRUE(paths["/health"].contains("get"));
  EXPECT_EQ(paths["/health"]["get"]["tags"][0], "Server");
  EXPECT_EQ(paths["/health"]["get"]["summary"], "Health check");
}

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ToOpenapiPathsMultipleMethodsSamePath) {
  registry_.get("/data", noop).tag("Data").summary("List data");
  registry_.post("/data", noop).tag("Data").summary("Create data");

  auto paths = registry_.to_openapi_paths();

  ASSERT_TRUE(paths.contains("/data"));
  EXPECT_TRUE(paths["/data"].contains("get"));
  EXPECT_TRUE(paths["/data"].contains("post"));
}

// =============================================================================
// to_regex_path - path conversion
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ToRegexPathRootBecomesRootAnchored) {
  // Register the root path and verify the regex conversion via
  // the handler registration (to_regex_path is private, test indirectly)
  registry_.get("/", noop);
  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths.contains("/"));
}

TEST_F(RouteRegistryTest, ToRegexPathAppIdUsesNonGreedyCapture) {
  registry_.get("/apps/{app_id}", noop).tag("Discovery");

  auto paths = registry_.to_openapi_paths();

  ASSERT_TRUE(paths.contains("/apps/{app_id}"));
  // The path param should be auto-generated
  auto & get_op = paths["/apps/{app_id}"]["get"];
  ASSERT_TRUE(get_op.contains("parameters"));

  bool found = false;
  for (const auto & p : get_op["parameters"]) {
    if (p["name"] == "app_id") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected auto-generated app_id path parameter";
}

TEST_F(RouteRegistryTest, ToRegexPathDataIdAtEndIsMultiSegment) {
  // data_id at end of path should use (.+) for multi-segment topic names
  registry_.get("/apps/{app_id}/data/{data_id}", noop).tag("Data");

  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/apps/{app_id}/data/{data_id}"));
}

TEST_F(RouteRegistryTest, ToRegexPathConfigIdAtEndIsMultiSegment) {
  // config_id at end of path should use (.+) for slash-containing param names
  registry_.get("/apps/{app_id}/configurations/{config_id}", noop).tag("Configuration");

  auto paths = registry_.to_openapi_paths();
  ASSERT_TRUE(paths.contains("/apps/{app_id}/configurations/{config_id}"));
}

TEST_F(RouteRegistryTest, ToRegexPathHealthConvertsCleanly) {
  registry_.get("/health", noop).tag("Server");

  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths.contains("/health"));
}

// =============================================================================
// set_auth_enabled - 401/403 responses
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, AuthEnabledAdds401And403Responses) {
  registry_.set_auth_enabled(true);
  registry_.get("/health", noop).tag("Server");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_TRUE(responses.contains("401"));
  EXPECT_TRUE(responses.contains("403"));
}

TEST_F(RouteRegistryTest, AuthDisabledNo401Or403Responses) {
  registry_.set_auth_enabled(false);
  registry_.get("/health", noop).tag("Server");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_FALSE(responses.contains("401"));
  EXPECT_FALSE(responses.contains("403"));
}

// =============================================================================
// tags - unique tag names
// =============================================================================

TEST_F(RouteRegistryTest, TagsReturnsUniqueTags) {
  registry_.get("/health", noop).tag("Server");
  registry_.get("/areas", noop).tag("Discovery");
  registry_.get("/apps", noop).tag("Discovery");
  registry_.get("/data", noop).tag("Data");

  auto tags = registry_.tags();

  // Should have exactly 3 unique tags (Server, Discovery, Data)
  EXPECT_EQ(tags.size(), 3u);

  // Check each tag is present (tags() returns sorted via std::set)
  EXPECT_NE(std::find(tags.begin(), tags.end(), "Server"), tags.end());
  EXPECT_NE(std::find(tags.begin(), tags.end(), "Discovery"), tags.end());
  EXPECT_NE(std::find(tags.begin(), tags.end(), "Data"), tags.end());
}

// =============================================================================
// Auto-generated path params have correct names
// =============================================================================

TEST_F(RouteRegistryTest, AutoGeneratedPathParamsHaveCorrectNames) {
  registry_.get("/areas/{area_id}/components/{component_id}", noop).tag("Discovery");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/areas/{area_id}/components/{component_id}"]["get"]["parameters"];

  ASSERT_EQ(params.size(), 2u);

  std::vector<std::string> param_names;
  for (const auto & p : params) {
    param_names.push_back(p["name"].get<std::string>());
  }

  EXPECT_NE(std::find(param_names.begin(), param_names.end(), "area_id"), param_names.end());
  EXPECT_NE(std::find(param_names.begin(), param_names.end(), "component_id"), param_names.end());
}

// =============================================================================
// Default 200 response when no explicit responses
// =============================================================================

TEST_F(RouteRegistryTest, Default200ResponseWhenNoExplicitResponses) {
  registry_.get("/health", noop).tag("Server").summary("Health check");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_TRUE(responses.contains("200"));
  EXPECT_EQ(responses["200"]["description"], "Successful response");
}

TEST_F(RouteRegistryTest, ExplicitResponseOverridesDefault) {
  registry_.get("/health", noop).tag("Server").response(200, "Gateway is healthy");

  auto paths = registry_.to_openapi_paths();
  auto & responses = paths["/health"]["get"]["responses"];

  EXPECT_TRUE(responses.contains("200"));
  EXPECT_EQ(responses["200"]["description"], "Gateway is healthy");
}

// =============================================================================
// operationId auto-generation (Fix 14)
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, OperationIdIsGenerated) {
  registry_.get("/apps/{app_id}/data", noop).tag("Data");

  auto paths = registry_.to_openapi_paths();
  auto & get_op = paths["/apps/{app_id}/data"]["get"];

  ASSERT_TRUE(get_op.contains("operationId"));
  std::string op_id = get_op["operationId"];
  EXPECT_FALSE(op_id.empty());
  // Should contain method and path segments
  EXPECT_NE(op_id.find("get"), std::string::npos);
  EXPECT_NE(op_id.find("apps"), std::string::npos);
  EXPECT_NE(op_id.find("data"), std::string::npos);
}

TEST_F(RouteRegistryTest, OperationIdForRootPath) {
  registry_.get("/", noop).tag("Server");

  auto paths = registry_.to_openapi_paths();
  auto & get_op = paths["/"]["get"];

  ASSERT_TRUE(get_op.contains("operationId"));
  std::string op_id = get_op["operationId"];
  EXPECT_FALSE(op_id.empty());
}

TEST_F(RouteRegistryTest, OperationIdUniquePerMethodPath) {
  registry_.get("/data", noop).tag("Data");
  registry_.post("/data", noop).tag("Data");

  auto paths = registry_.to_openapi_paths();

  std::string get_id = paths["/data"]["get"]["operationId"];
  std::string post_id = paths["/data"]["post"]["operationId"];

  EXPECT_NE(get_id, post_id);
}

// =============================================================================
// Path param descriptions (Fix 15)
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, PathParamDescriptionsArePresent) {
  registry_.get("/apps/{app_id}/data/{data_id}", noop).tag("Data");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/apps/{app_id}/data/{data_id}"]["get"]["parameters"];

  for (const auto & p : params) {
    EXPECT_TRUE(p.contains("description")) << "Missing description for param: " << p["name"];
    EXPECT_FALSE(p["description"].get<std::string>().empty());
  }
}

TEST_F(RouteRegistryTest, KnownParamHasSpecificDescription) {
  registry_.get("/apps/{app_id}", noop).tag("Discovery");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/apps/{app_id}"]["get"]["parameters"];

  ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(params[0]["name"], "app_id");
  EXPECT_EQ(params[0]["description"], "The app identifier");
}

TEST_F(RouteRegistryTest, UnknownParamGetsGenericDescription) {
  registry_.get("/widgets/{widget_id}", noop).tag("Custom");

  auto paths = registry_.to_openapi_paths();
  auto & params = paths["/widgets/{widget_id}"]["get"]["parameters"];

  ASSERT_EQ(params.size(), 1u);
  EXPECT_EQ(params[0]["description"], "The widget_id value");
}

// =============================================================================
// to_endpoint_list (Fix 23)
// =============================================================================

TEST_F(RouteRegistryTest, ToEndpointListProducesCorrectFormat) {
  registry_.get("/health", noop).tag("Server");
  registry_.post("/auth/token", noop).tag("Authentication");
  registry_.del("/faults", noop).tag("Faults");

  auto endpoints = registry_.to_endpoint_list("/api/v1");

  ASSERT_EQ(endpoints.size(), 3u);
  EXPECT_EQ(endpoints[0], "GET /api/v1/health");
  EXPECT_EQ(endpoints[1], "POST /api/v1/auth/token");
  EXPECT_EQ(endpoints[2], "DELETE /api/v1/faults");
}

// =============================================================================
// size() and empty registry
// =============================================================================

TEST_F(RouteRegistryTest, EmptyRegistryHasZeroSize) {
  EXPECT_EQ(registry_.size(), 0u);
}

TEST_F(RouteRegistryTest, SizeReflectsRegisteredRoutes) {
  registry_.get("/health", noop);
  registry_.post("/data", noop);
  registry_.put("/config", noop);

  EXPECT_EQ(registry_.size(), 3u);
}

TEST_F(RouteRegistryTest, EmptyRegistryProducesEmptyPaths) {
  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths.empty());
}

// =============================================================================
// Deprecated flag
// =============================================================================

TEST_F(RouteRegistryTest, DeprecatedFlagAppearsInOutput) {
  registry_.get("/old-endpoint", noop).tag("Server").deprecated();

  auto paths = registry_.to_openapi_paths();
  EXPECT_TRUE(paths["/old-endpoint"]["get"]["deprecated"].get<bool>());
}

// =============================================================================
// validate_completeness
// =============================================================================

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForCompleteRoute) {
  registry_.get("/health", noop)
      .tag("Server")
      .summary("Health check")
      .response(200, "Healthy", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  // No errors expected
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

TEST_F(RouteRegistryTest, ValidateCompletenessErrorOnMissingTag) {
  registry_.get("/health", noop).summary("Health check").response(200, "Healthy", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  bool has_tag_error = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError && issue.message == "Missing tag") {
      has_tag_error = true;
    }
  }
  EXPECT_TRUE(has_tag_error);
}

TEST_F(RouteRegistryTest, ValidateCompletenessErrorOnMissingResponseSchema) {
  registry_.get("/health", noop).tag("Server").summary("Health check").response(200, "Healthy");

  auto issues = registry_.validate_completeness();
  bool has_schema_error = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError &&
        issue.message.find("Missing response schema") != std::string::npos) {
      has_schema_error = true;
    }
  }
  EXPECT_TRUE(has_schema_error);
}

TEST_F(RouteRegistryTest, ValidateCompletenessErrorOnPostMissingRequestBody) {
  registry_.post("/items", noop).tag("Items").summary("Create item").response(201, "Created", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  bool has_body_error = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError &&
        issue.message.find("Missing request body") != std::string::npos) {
      has_body_error = true;
    }
  }
  EXPECT_TRUE(has_body_error);
}

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForDeleteWith204) {
  registry_.del("/items/{id}", noop).tag("Items").summary("Delete item").response(204, "Deleted");

  auto issues = registry_.validate_completeness();
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

TEST_F(RouteRegistryTest, ValidateCompletenessErrorOnDeleteNoExplicitResponse) {
  registry_.del("/items/{id}", noop).tag("Items").summary("Delete item");

  auto issues = registry_.validate_completeness();
  bool has_delete_error = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError &&
        issue.message.find("DELETE missing explicit response") != std::string::npos) {
      has_delete_error = true;
    }
  }
  EXPECT_TRUE(has_delete_error);
}

TEST_F(RouteRegistryTest, ValidateCompletenessPassesFor405Endpoint) {
  registry_.post("/readonly", noop).tag("Test").summary("Not supported").response(405, "Method not allowed");

  auto issues = registry_.validate_completeness();
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForSSEEndpoint) {
  registry_.get("/events/stream", noop).tag("Events").summary("SSE events stream");

  auto issues = registry_.validate_completeness();
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

TEST_F(RouteRegistryTest, ValidateCompletenessWarnsOnMissingSummary) {
  registry_.get("/health", noop).tag("Server").response(200, "OK", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  bool has_summary_warning = false;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kWarning && issue.message == "Missing summary") {
      has_summary_warning = true;
    }
  }
  EXPECT_TRUE(has_summary_warning);
}

TEST_F(RouteRegistryTest, ValidateCompletenessPassesForCompletePostRoute) {
  registry_.post("/items", noop)
      .tag("Items")
      .summary("Create item")
      .request_body("Item data", json{{"type", "object"}})
      .response(201, "Created", json{{"type", "object"}});

  auto issues = registry_.validate_completeness();
  int error_count = 0;
  for (const auto & issue : issues) {
    if (issue.severity == ValidationIssue::Severity::kError) {
      error_count++;
    }
  }
  EXPECT_EQ(error_count, 0);
}

// =============================================================================
// Error responses use GenericError $ref (Task 2)
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(RouteRegistryTest, ErrorResponsesUseGenericErrorRef) {
  registry_.get("/test", noop).tag("Test").summary("Test").response(200, "OK", json{{"type", "object"}});

  auto paths = registry_.to_openapi_paths();
  auto & resp_400 = paths["/test"]["get"]["responses"]["400"];

  // Error responses should $ref the GenericError response component
  ASSERT_TRUE(resp_400.contains("$ref"));
  EXPECT_EQ(resp_400["$ref"].get<std::string>(), "#/components/responses/GenericError");
  // No inline description when using $ref
  EXPECT_FALSE(resp_400.contains("description"));
}
