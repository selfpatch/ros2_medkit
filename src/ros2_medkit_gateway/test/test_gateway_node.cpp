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

#include <chrono>
#include <memory>
#include <thread>

#include <httplib.h>  // NOLINT(build/include_order)
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static constexpr char VERSION[] = "0.1.0";

// Simple GatewayNode class for testing
class GatewayNode : public rclcpp::Node {
public:
  GatewayNode()
      : Node("gateway_node"), http_server_(std::make_unique<httplib::Server>()),
        node_name_(this->get_name()) {
    this->declare_parameter<int>("port", 8080);
    this->declare_parameter<std::string>("host", "0.0.0.0");

    port_ = this->get_parameter("port").as_int();
    host_ = this->get_parameter("host").as_string();

    setup_endpoints();

    server_thread_ =
        std::thread([this]() { http_server_->listen(host_.c_str(), port_); });

    // Wait for the server to be ready by polling
    wait_for_server_ready();
  }

  ~GatewayNode() {
    http_server_->stop();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  int get_port() const { return port_; }
  std::string get_host() const { return host_; }

private:
  void wait_for_server_ready() {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(2);
    httplib::Client client(host_.c_str(), port_);
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start) < timeout) {
      if (auto res = client.Get("/health")) {
        if (res->status == 200) {
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    throw std::runtime_error("HTTP server failed to start within timeout");
  }

  void setup_endpoints() {
    http_server_->Get(
        "/health", [this](const httplib::Request &req, httplib::Response &res) {
          (void)req;

          nlohmann::json health_json = {{"status", "ok"},
                                        {"node", node_name_},
                                        {"timestamp", this->now().seconds()}};

          res.set_content(health_json.dump(), "application/json");
          res.status = 200;
        });

    http_server_->Get(
        "/", [this](const httplib::Request &req, httplib::Response &res) {
          (void)req;

          nlohmann::json info_json = {
              {"name", "ROS 2 Medkit Gateway"},
              {"version", VERSION},
              {"endpoints", nlohmann::json::array({
                  "GET /health",
                  "GET /version-info",
                  "GET /areas",
                  "GET /components",
                  "GET /areas/{area_id}/components",
                  "GET /components/{component_id}/data",
                  "GET /components/{component_id}/data/{topic_name}",
                  "PUT /components/{component_id}/data/{topic_name}"
              })},
              {"capabilities", {
                  {"discovery", true},
                  {"data_access", true}
              }}
          };

          res.set_content(info_json.dump(), "application/json");
          res.status = 200;
        });
  }

  std::unique_ptr<httplib::Server> http_server_;
  std::thread server_thread_;
  int port_;
  std::string host_;
  std::string node_name_;
};

class TestGatewayNode : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(TestGatewayNode, test_health_endpoint) {
  // @verifies REQ_INTEROP_001
  auto node = std::make_shared<GatewayNode>();

  // Create HTTP client
  httplib::Client client("localhost", node->get_port());

  // Call /health endpoint
  auto res = client.Get("/health");

  // Verify response
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  // Parse and verify JSON
  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_EQ(json_response["status"], "ok");
  EXPECT_EQ(json_response["node"], "gateway_node");
  EXPECT_TRUE(json_response.contains("timestamp"));
  EXPECT_TRUE(json_response["timestamp"].is_number());
}

TEST_F(TestGatewayNode, test_root_endpoint) {
  // @verifies REQ_INTEROP_001, REQ_INTEROP_010
  auto node = std::make_shared<GatewayNode>();

  // Create HTTP client
  httplib::Client client("localhost", node->get_port());

  // Call / endpoint
  auto res = client.Get("/");

  // Verify response
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  // Parse and verify JSON
  auto json_response = nlohmann::json::parse(res->body);
  EXPECT_EQ(json_response["name"], "ROS 2 Medkit Gateway");
  EXPECT_EQ(json_response["version"], "0.1.0");
  EXPECT_TRUE(json_response.contains("endpoints"));
  EXPECT_TRUE(json_response["endpoints"].is_array());
  EXPECT_EQ(json_response["endpoints"].size(), 8);
  EXPECT_TRUE(json_response.contains("capabilities"));
  EXPECT_TRUE(json_response["capabilities"]["discovery"]);
  EXPECT_TRUE(json_response["capabilities"]["data_access"]);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
