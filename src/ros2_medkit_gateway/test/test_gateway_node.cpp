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
  EXPECT_TRUE(json_response.is_array());
}

TEST_F(TestGatewayNode, test_list_components_endpoint) {
  // @verifies REQ_INTEROP_003
  auto client = create_client();

  auto res = client.Get(std::string(API_BASE_PATH) + "/components");

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, StatusCode::OK_200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_TRUE(json_response.is_array());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
