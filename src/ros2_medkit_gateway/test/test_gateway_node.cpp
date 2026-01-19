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

#include <gtest/gtest.h>
#include <httplib.h>  // NOLINT(build/include_order)

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ros2_medkit_gateway/gateway_node.hpp"

using namespace std::chrono_literals;
using httplib::StatusCode;

// API version prefix - must match rest_server.cpp
static constexpr const char * API_BASE_PATH = "/api/v1";

class TestGatewayNode : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<ros2_medkit_gateway::GatewayNode>();

    // Get server configuration from node parameters
    server_host_ = node_->get_parameter("server.host").as_string();
    server_port_ = static_cast<int>(node_->get_parameter("server.port").as_int());

    // Wait for the server to be ready
    wait_for_server_ready();
  }

  void TearDown() override {
    node_.reset();
  }

  void wait_for_server_ready() {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5);
    httplib::Client client(server_host_, server_port_);
    const std::string health_endpoint = std::string(API_BASE_PATH) + "/health";

    while (std::chrono::steady_clock::now() - start < timeout) {
      if (auto res = client.Get(health_endpoint)) {
        if (res->status == StatusCode::OK_200) {
          return;
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    FAIL() << "HTTP server failed to start within timeout";
  }

  httplib::Client create_client() {
    return httplib::Client(server_host_, server_port_);
  }

  std::shared_ptr<ros2_medkit_gateway::GatewayNode> node_;
  std::string server_host_;
  int server_port_;
};

TEST_F(TestGatewayNode, test_health_endpoint) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/health");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_EQ(json_response["status"], "healthy");
  EXPECT_TRUE(json_response.contains("timestamp"));
  EXPECT_TRUE(json_response["timestamp"].is_number());
}

TEST_F(TestGatewayNode, test_root_endpoint) {
  // @verifies REQ_INTEROP_010
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_EQ(json_response["name"], "ROS 2 Medkit Gateway");
  EXPECT_TRUE(json_response.contains("version"));
  EXPECT_TRUE(json_response.contains("endpoints"));
  EXPECT_TRUE(json_response["endpoints"].is_array());
  EXPECT_TRUE(json_response.contains("capabilities"));
  EXPECT_TRUE(json_response["capabilities"]["discovery"]);
  EXPECT_TRUE(json_response["capabilities"]["data_access"]);
}

TEST_F(TestGatewayNode, test_version_info_endpoint) {
  // @verifies REQ_INTEROP_001
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/version-info");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.contains("version"));
  EXPECT_TRUE(json_response.contains("status"));
  EXPECT_TRUE(json_response.contains("timestamp"));
}

TEST_F(TestGatewayNode, test_list_areas_endpoint) {
  // @verifies REQ_INTEROP_003
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/areas");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
  EXPECT_TRUE(json_response["items"].is_array());
}

TEST_F(TestGatewayNode, test_list_components_endpoint) {
  // @verifies REQ_INTEROP_003
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
  EXPECT_TRUE(json_response["items"].is_array());
}

// =============================================================================
// Additional endpoint tests for improved coverage
// =============================================================================

TEST_F(TestGatewayNode, test_nonexistent_endpoint_404) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/nonexistent/path");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_invalid_area_id_bad_request) {
  auto client = create_client();

  // Test with special characters that should be rejected
  // Note: URL-encoded slash (%2F) is decoded by server, but @#$ characters are invalid
  auto res = client.Get(std::string(API_BASE_PATH) + "/areas/test@invalid");

  ASSERT_TRUE(res);
  // Should return 400 for invalid entity ID with special characters
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);
}

TEST_F(TestGatewayNode, test_nonexistent_area_404) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/areas/nonexistent_area");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_nonexistent_component_404) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_component");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_invalid_component_id_bad_request) {
  auto client = create_client();

  // Test with special characters that are invalid
  auto res = client.Get(std::string(API_BASE_PATH) + "/components/test@invalid#id");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.contains("error"));
}

TEST_F(TestGatewayNode, test_list_apps_endpoint) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/apps");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
  EXPECT_TRUE(json_response["items"].is_array());
}

TEST_F(TestGatewayNode, test_list_functions_endpoint) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/functions");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_object());
  EXPECT_TRUE(json_response.contains("items"));
}

TEST_F(TestGatewayNode, test_options_request_for_cors) {
  auto client = create_client();

  // OPTIONS request for CORS preflight
  auto res = client.Options(std::string(API_BASE_PATH) + "/components");

  ASSERT_TRUE(res);
  // Without CORS configured, server may return various success codes or 404
  // The main purpose is to verify the server handles OPTIONS without crashing
  EXPECT_TRUE(res->status == StatusCode::OK_200 || res->status == StatusCode::NoContent_204 ||
              res->status == StatusCode::NotFound_404 || res->status == StatusCode::MethodNotAllowed_405);
}

TEST_F(TestGatewayNode, test_component_data_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_component/data");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_component_operations_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_component/operations");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_component_configurations_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_component/configurations");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_component_faults_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_component/faults");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

// Note: test_global_faults_endpoint is skipped in unit tests because it requires
// the fault_manager external service which isn't available. This endpoint is
// properly tested in test_integration.test.py which starts all required nodes.

TEST_F(TestGatewayNode, test_root_endpoint_contains_api_base_path) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/");

  ASSERT_TRUE(res);
  auto json_response = nlohmann::json::parse(res->body);

  EXPECT_TRUE(json_response.contains("api_base"));
  EXPECT_EQ(json_response["api_base"], API_BASE_PATH);
}

TEST_F(TestGatewayNode, test_root_endpoint_lists_all_capabilities) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/");

  ASSERT_TRUE(res);
  auto json_response = nlohmann::json::parse(res->body);

  EXPECT_TRUE(json_response["capabilities"]["discovery"]);
  EXPECT_TRUE(json_response["capabilities"]["data_access"]);
  EXPECT_TRUE(json_response["capabilities"]["operations"]);
  EXPECT_TRUE(json_response["capabilities"]["async_actions"]);
  EXPECT_TRUE(json_response["capabilities"]["configurations"]);
  EXPECT_TRUE(json_response["capabilities"]["faults"]);
}

TEST_F(TestGatewayNode, test_area_components_nonexistent_area) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/areas/nonexistent_area/components");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_area_subareas_nonexistent_area) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/areas/nonexistent_area/subareas");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_post_to_nonexistent_operation) {
  auto client = create_client();

  auto res = client.Post(std::string(API_BASE_PATH) + "/components/nonexistent/operations/do_something", "",
                         "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

// =============================================================================
// Configuration endpoint tests
// =============================================================================

TEST_F(TestGatewayNode, test_set_configuration_invalid_json) {
  auto client = create_client();

  // POST with invalid JSON body
  auto res = client.Put(std::string(API_BASE_PATH) + "/components/gateway_node/configurations/test_param",
                        "{ invalid json }", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.contains("error"));
}

TEST_F(TestGatewayNode, test_set_configuration_missing_value_field) {
  auto client = create_client();

  // POST with valid JSON but missing 'value' field
  auto res = client.Put(std::string(API_BASE_PATH) + "/components/gateway_node/configurations/test_param",
                        R"({"name": "test_param"})", "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.contains("error"));
  // Error should mention missing 'value' field
  EXPECT_TRUE(json_response["error"].get<std::string>().find("value") != std::string::npos ||
              json_response.contains("details"));
}

TEST_F(TestGatewayNode, test_set_configuration_invalid_component_id) {
  auto client = create_client();

  auto res = client.Put(std::string(API_BASE_PATH) + "/components/invalid@id/configurations/param", R"({"value": 42})",
                        "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);
}

TEST_F(TestGatewayNode, test_get_configuration_nonexistent_component) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_comp/configurations/some_param");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_delete_configuration_nonexistent_component) {
  auto client = create_client();

  auto res = client.Delete(std::string(API_BASE_PATH) + "/components/nonexistent_comp/configurations/some_param");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

// =============================================================================
// Operation endpoint tests
// =============================================================================

TEST_F(TestGatewayNode, test_post_operation_invalid_json_body) {
  auto client = create_client();

  // First we need a valid component - use the gateway_node itself
  auto res = client.Post(std::string(API_BASE_PATH) + "/components/gateway_node/operations/some_service",
                         "{ not valid json }", "application/json");

  ASSERT_TRUE(res);
  // Should be 400 for invalid JSON or 404 if component not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_post_operation_invalid_component_id) {
  auto client = create_client();

  auto res = client.Post(std::string(API_BASE_PATH) + "/components/invalid@component/operations/test", R"({})",
                         "application/json");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);
}

TEST_F(TestGatewayNode, test_post_operation_invalid_operation_id) {
  auto client = create_client();

  auto res = client.Post(std::string(API_BASE_PATH) + "/components/gateway_node/operations/invalid@op", R"({})",
                         "application/json");

  ASSERT_TRUE(res);
  // 400 for invalid operation name or 404 if component not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_action_status_invalid_component_id) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/invalid@id/operations/test/status");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);
}

TEST_F(TestGatewayNode, test_action_cancel_missing_goal_id) {
  auto client = create_client();

  // Cancel without goal_id query parameter
  auto res = client.Delete(std::string(API_BASE_PATH) + "/components/gateway_node/operations/test/cancel");

  ASSERT_TRUE(res);
  // Should be 400 for missing goal_id or 404 if component/operation not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_action_result_missing_goal_id) {
  auto client = create_client();

  // Result without goal_id query parameter
  auto res = client.Get(std::string(API_BASE_PATH) + "/components/gateway_node/operations/test/result");

  ASSERT_TRUE(res);
  // Should be 400 for missing goal_id or 404 if not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

// =============================================================================
// Data endpoint tests
// =============================================================================

TEST_F(TestGatewayNode, test_publish_to_topic_invalid_json) {
  auto client = create_client();

  auto res = client.Post(std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic", "invalid{json}",
                         "application/json");

  ASSERT_TRUE(res);
  // 400 for invalid JSON or 404 if component not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_publish_to_topic_missing_type_field) {
  auto client = create_client();

  // Missing 'type' field in request body
  auto res = client.Post(std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic",
                         R"({"data": {"value": 1.0}})", "application/json");

  ASSERT_TRUE(res);
  // 400 for missing type or 404 if component not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_publish_to_topic_missing_data_field) {
  auto client = create_client();

  // Missing 'data' field in request body
  auto res = client.Post(std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic",
                         R"({"type": "std_msgs/msg/Float32"})", "application/json");

  ASSERT_TRUE(res);
  // 400 for missing data or 404 if component not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_publish_to_topic_invalid_message_type_format) {
  auto client = create_client();

  // Invalid message type format (should be pkg/msg/Type)
  auto res = client.Post(std::string(API_BASE_PATH) + "/components/gateway_node/data/some_topic",
                         R"({"type": "invalid_type", "data": {}})", "application/json");

  ASSERT_TRUE(res);
  // 400 for invalid type format or 404 if component not found
  EXPECT_TRUE(res->status == StatusCode::BadRequest_400 || res->status == StatusCode::NotFound_404);
}

// =============================================================================
// Subresource tests
// =============================================================================

TEST_F(TestGatewayNode, test_area_related_components_nonexistent) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/areas/nonexistent_area/related-components");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_component_subcomponents_nonexistent) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_comp/subcomponents");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_component_related_apps_nonexistent) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components/nonexistent_comp/related-apps");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_app_nonexistent) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/apps/nonexistent_app");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_function_nonexistent) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/functions/nonexistent_function");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

TEST_F(TestGatewayNode, test_function_hosts_nonexistent) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/functions/nonexistent_function/hosts");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::NotFound_404);
}

// =============================================================================
// Invalid ID pattern tests for various entity types
// =============================================================================

TEST_F(TestGatewayNode, test_invalid_app_id_bad_request) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/apps/invalid@app#id");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);
}

TEST_F(TestGatewayNode, test_invalid_function_id_bad_request) {
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/functions/invalid@func#id");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::BadRequest_400);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
