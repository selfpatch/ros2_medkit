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

#include <chrono>
#include <memory>
#include <thread>

#include <httplib.h>  // NOLINT(build/include_order)
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static constexpr char VERSION[] = "0.1.0";

class GatewayNode : public rclcpp::Node {
public:
  GatewayNode()
      : Node("gateway_node"), http_server_(std::make_unique<httplib::Server>()),
        node_name_(this->get_name()) {
    // Declare parameters
    this->declare_parameter<int>("port", 8080);
    this->declare_parameter<std::string>("host", "0.0.0.0");

    port_ = this->get_parameter("port").as_int();
    host_ = this->get_parameter("host").as_string();

    // Setup HTTP endpoints
    setup_endpoints();

    RCLCPP_INFO(this->get_logger(), "Starting HTTP server on %s:%d",
                host_.c_str(), port_);

    // Start HTTP server in a separate thread
    server_thread_ =
        std::thread([this]() { http_server_->listen(host_.c_str(), port_); });

    RCLCPP_INFO(this->get_logger(), "Gateway node initialized");
  }

  ~GatewayNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down HTTP server");
    http_server_->stop();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  void setup_endpoints() {
    // Health endpoint
    http_server_->Get(
        "/health", [this](const httplib::Request &req, httplib::Response &res) {
          (void)req;  // Unused parameter

          nlohmann::json health_json = {{"status", "ok"},
                                        {"node", node_name_},
                                        {"timestamp", this->now().seconds()}};

          res.set_content(health_json.dump(), "application/json");
          res.status = 200;
        });

    // Root endpoint
    http_server_->Get(
        "/", [this](const httplib::Request &req, httplib::Response &res) {
          (void)req;  // Unused parameter

          nlohmann::json info_json = {
              {"service", "ros2_medkit_gateway"},
              {"version", VERSION},
              {"endpoints", nlohmann::json::array({"/health", "/"})}};

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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GatewayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
