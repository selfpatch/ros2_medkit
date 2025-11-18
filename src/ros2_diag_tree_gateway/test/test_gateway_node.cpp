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

#include <httplib.h> // NOLINT(build/include_order)
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

// Simple GatewayNode class for testing
class GatewayNode : public rclcpp::Node {
public:
  GatewayNode()
      : Node("gateway_node"),
        http_server_(std::make_unique<httplib::Server>()) {
    this->declare_parameter<int>("port", 8080);
    this->declare_parameter<std::string>("host", "0.0.0.0");

    port_ = this->get_parameter("port").as_int();
    host_ = this->get_parameter("host").as_string();

    setup_endpoints();

    server_thread_ =
        std::thread([this]() { http_server_->listen(host_.c_str(), port_); });

    // Give the server a moment to start
    std::this_thread::sleep_for(100ms);
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
  void setup_endpoints() {
    http_server_->Get(
        "/health", [this](const httplib::Request &req, httplib::Response &res) {
          (void)req;

          std::string health_json = R"({
  "status": "ok",
  "node": ")" + std::string(this->get_name()) +
                                    R"(",
  "timestamp": )" + std::to_string(this->now().seconds()) +
                                    R"(
})";

          res.set_content(health_json, "application/json");
          res.status = 200;
        });
  }

  std::unique_ptr<httplib::Server> http_server_;
  std::thread server_thread_;
  int port_;
  std::string host_;
};

class TestGatewayNode : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(TestGatewayNode, test_health_endpoint) {
  auto node = std::make_shared<GatewayNode>();

  // Create HTTP client
  httplib::Client client("localhost", node->get_port());

  // Call /health endpoint
  auto res = client.Get("/health");

  // Verify response
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  EXPECT_EQ(res->get_header_value("Content-Type"), "application/json");

  // Verify JSON contains expected fields
  EXPECT_TRUE(res->body.find("\"status\": \"ok\"") != std::string::npos);
  EXPECT_TRUE(res->body.find("\"node\": \"gateway_node\"") !=
              std::string::npos);
  EXPECT_TRUE(res->body.find("\"timestamp\"") != std::string::npos);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
