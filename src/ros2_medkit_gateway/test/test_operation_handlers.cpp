// Copyright 2026 sewon
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

#include <arpa/inet.h>
#include <example_interfaces/action/fibonacci.hpp>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/operation_handlers.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::ActionGoalInfo;
using ros2_medkit_gateway::ActionGoalStatus;
using ros2_medkit_gateway::ActionInfo;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ServiceInfo;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::OperationHandlers;

namespace {

using namespace std::chrono_literals;

json parse_json(const httplib::Response & res) {
  return json::parse(res.body);
}

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ADD_FAILURE() << "Failed to create socket for test port reservation: " << std::strerror(errno);
    return 0;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;

  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ADD_FAILURE() << "Failed to bind socket for test port reservation: " << std::strerror(errno);
    close(sock);
    return 0;
  }

  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    ADD_FAILURE() << "Failed to inspect reserved test port: " << std::strerror(errno);
    close(sock);
    return 0;
  }

  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;

  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);

  return req;
}

class TestLongCalibrationActionServer : public rclcpp::Node {
 public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  TestLongCalibrationActionServer() : rclcpp::Node("test_long_calibration_action", "/powertrain/engine") {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "long_calibration", std::bind(&TestLongCalibrationActionServer::handle_goal, this, _1, _2),
        std::bind(&TestLongCalibrationActionServer::handle_cancel, this, _1),
        std::bind(&TestLongCalibrationActionServer::handle_accepted, this, _1));
  }

  void prepare_shutdown() {
    shutdown_.store(true);
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    action_server_.reset();
  }

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                          std::shared_ptr<const Fibonacci::Goal> goal) {
    if (goal->order > 50) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread(&TestLongCalibrationActionServer::execute, this, goal_handle);
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
    feedback->sequence = {0, 1};

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok() && !shutdown_.load()) {
      if (goal_handle->is_canceling()) {
        result->sequence = feedback->sequence;
        goal_handle->canceled(result);
        return;
      }

      feedback->sequence.push_back(static_cast<int32_t>(feedback->sequence.size()));
      goal_handle->publish_feedback(feedback);

      if (feedback->sequence.size() >= 6) {
        result->sequence = feedback->sequence;
        goal_handle->succeed(result);
        return;
      }

      loop_rate.sleep();
    }
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  std::thread execution_thread_;
  std::atomic<bool> shutdown_{false};
};

}  // namespace

class OperationHandlersValidationTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  OperationHandlers handlers_{ctx_};
};

TEST_F(OperationHandlersValidationTest, ListOperationsMissingMatchesReturns400) {
  httplib::Request req;
  req.path = "/api/v1/components/engine/operations";
  httplib::Response res;

  handlers_.handle_list_operations(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = parse_json(res);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

TEST_F(OperationHandlersValidationTest, ListOperationsInvalidEntityReturns400) {
  auto req =
      make_request_with_match("/api/v1/components/engine!/operations", R"(/api/v1/components/([^/]+)/operations)");
  httplib::Response res;

  handlers_.handle_list_operations(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = parse_json(res);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

class OperationHandlersFixtureTest : public ::testing::Test {
 protected:
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0);

    std::vector<std::string> args = {"test_operation_handlers",
                                     "--ros-args",
                                     "-p",
                                     "server.port:=" + std::to_string(suite_server_port_),
                                     "-p",
                                     "refresh_interval_ms:=60000",
                                     "-p",
                                     "service_call_timeout_sec:=1"};

    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }

    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    gateway_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(gateway_node_, nullptr);

    service_node_ = std::make_shared<rclcpp::Node>("test_calibrate_service", "/powertrain/engine");
    trigger_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
        "calibrate", [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = true;
          response->message = "calibration complete";
        });

    action_server_node_ = std::make_shared<TestLongCalibrationActionServer>();

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    executor_->add_node(service_node_);
    executor_->add_node(action_server_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    seed_component_cache();

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<OperationHandlers>(*ctx_);

    std::this_thread::sleep_for(200ms);
  }

  void TearDown() override {
    if (action_server_node_ != nullptr) {
      action_server_node_->prepare_shutdown();
    }

    if (executor_ != nullptr) {
      executor_->cancel();
    }

    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    handlers_.reset();
    ctx_.reset();
    trigger_service_.reset();
    action_server_node_.reset();
    service_node_.reset();
    executor_.reset();
    gateway_node_.reset();
  }

  void seed_component_cache() {
    Component component;
    component.id = "engine";
    component.name = "Engine";
    component.namespace_path = "/powertrain/engine";
    component.fqn = "/powertrain/engine";
    component.area = "powertrain";
    component.source = "manifest";
    component.services = {
        ServiceInfo{"calibrate", "/powertrain/engine/calibrate", "std_srvs/srv/Trigger", std::nullopt}};
    component.actions = {ActionInfo{"long_calibration", "/powertrain/engine/long_calibration",
                                    "example_interfaces/action/Fibonacci", std::nullopt}};

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, {component}, {}, {});
  }

  std::string create_action_execution() {
    auto req = make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions",
                                       R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions)");
    req.body = R"({"parameters":{"order":6}})";

    httplib::Response res;
    handlers_->handle_create_execution(req, res);

    EXPECT_EQ(res.status, 202);
    auto body = parse_json(res);
    EXPECT_TRUE(body.contains("id"));
    return body["id"].get<std::string>();
  }

  ActionGoalInfo get_tracked_goal_or_fail(const std::string & execution_id) {
    auto goal_info = gateway_node_->get_operation_manager()->get_tracked_goal(execution_id);
    EXPECT_TRUE(goal_info.has_value());
    return *goal_info;
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::shared_ptr<rclcpp::Node> service_node_;
  std::shared_ptr<TestLongCalibrationActionServer> action_server_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<OperationHandlers> handlers_;
};

TEST_F(OperationHandlersFixtureTest, ListOperationsReturnsServiceAndActionItems) {
  auto req =
      make_request_with_match("/api/v1/components/engine/operations", R"(/api/v1/components/([^/]+)/operations)");
  httplib::Response res;

  handlers_->handle_list_operations(req, res);

  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");
  auto body = parse_json(res);
  ASSERT_TRUE(body.contains("items"));
  ASSERT_EQ(body["items"].size(), 2);

  std::set<std::string> ids;
  for (const auto & item : body["items"]) {
    ids.insert(item["id"].get<std::string>());
  }

  EXPECT_EQ(ids, std::set<std::string>({"calibrate", "long_calibration"}));
}

TEST_F(OperationHandlersFixtureTest, ListOperationsUnknownEntityReturns404) {
  auto req =
      make_request_with_match("/api/v1/components/unknown/operations", R"(/api/v1/components/([^/]+)/operations)");
  httplib::Response res;

  handlers_->handle_list_operations(req, res);

  EXPECT_EQ(res.status, 404);
  auto body = parse_json(res);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND);
}

TEST_F(OperationHandlersFixtureTest, GetOperationReturnsActionMetadata) {
  auto req = make_request_with_match("/api/v1/components/engine/operations/long_calibration",
                                     R"(/api/v1/components/([^/]+)/operations/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_operation(req, res);

  auto body = parse_json(res);
  ASSERT_TRUE(body.contains("item"));
  EXPECT_EQ(body["item"]["id"], "long_calibration");
  EXPECT_TRUE(body["item"]["asynchronous_execution"].get<bool>());
  EXPECT_EQ(body["item"]["x-medkit"]["ros2"]["kind"], "action");
  EXPECT_EQ(body["item"]["x-medkit"]["ros2"]["action"], "/powertrain/engine/long_calibration");
}

TEST_F(OperationHandlersFixtureTest, GetOperationUnknownOperationReturns404) {
  auto req = make_request_with_match("/api/v1/components/engine/operations/does_not_exist",
                                     R"(/api/v1/components/([^/]+)/operations/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_operation(req, res);

  EXPECT_EQ(res.status, 404);
  auto body = parse_json(res);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_OPERATION_NOT_FOUND);
}

TEST_F(OperationHandlersFixtureTest, CreateExecutionOnServiceReturnsSynchronousResponse) {
  auto req = make_request_with_match("/api/v1/components/engine/operations/calibrate/executions",
                                     R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions)");
  req.body = R"({"parameters":{}})";
  httplib::Response res;

  handlers_->handle_create_execution(req, res);

  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");
  auto body = parse_json(res);
  EXPECT_TRUE(body.contains("parameters"));
  EXPECT_TRUE(body["parameters"]["success"].get<bool>());
  EXPECT_EQ(body["parameters"]["message"], "calibration complete");
}

TEST_F(OperationHandlersFixtureTest, ListExecutionsReturnsTrackedActionGoal) {
  const auto execution_id = create_action_execution();

  auto req = make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions",
                                     R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions)");
  httplib::Response res;

  handlers_->handle_list_executions(req, res);

  auto body = parse_json(res);
  ASSERT_TRUE(body.contains("items"));
  ASSERT_EQ(body["items"].size(), 1);
  EXPECT_EQ(body["items"][0]["id"], execution_id);
}

TEST_F(OperationHandlersFixtureTest, GetExecutionContainsStatusFields) {
  const auto execution_id = create_action_execution();
  gateway_node_->get_operation_manager()->update_goal_feedback(execution_id, json{{"progress", 50}});

  auto req = make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions/" + execution_id,
                                     R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
  httplib::Response res;

  handlers_->handle_get_execution(req, res);

  auto body = parse_json(res);
  EXPECT_EQ(body["status"], "running");
  EXPECT_EQ(body["capability"], "execute");
  EXPECT_EQ(body["parameters"]["progress"], 50);
  EXPECT_EQ(body["x-medkit"]["goal_id"], execution_id);
  EXPECT_EQ(body["x-medkit"]["ros2"]["action"], "/powertrain/engine/long_calibration");
}

TEST_F(OperationHandlersFixtureTest, CancelExecutionUnknownIdReturns404) {
  auto req = make_request_with_match(
      "/api/v1/components/engine/operations/long_calibration/executions/0123456789abcdef0123456789abcdef",
      R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
  httplib::Response res;

  handlers_->handle_cancel_execution(req, res);

  EXPECT_EQ(res.status, 404);
  auto body = parse_json(res);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND);
}

TEST_F(OperationHandlersFixtureTest, UpdateExecutionStopReturnsAcceptedAndLocation) {
  const auto execution_id = create_action_execution();

  auto req = make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions/" + execution_id,
                                     R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
  req.body = R"({"capability":"stop"})";
  httplib::Response res;

  handlers_->handle_update_execution(req, res);

  EXPECT_EQ(res.status, 202);
  EXPECT_EQ(res.get_header_value("Location"),
            "/api/v1/components/engine/operations/long_calibration/executions/" + execution_id);
  auto body = parse_json(res);
  EXPECT_EQ(body["id"], execution_id);
  EXPECT_EQ(body["status"], "running");

  auto goal_info = get_tracked_goal_or_fail(execution_id);
  EXPECT_EQ(goal_info.status, ActionGoalStatus::CANCELING);
}

TEST_F(OperationHandlersFixtureTest, UpdateExecutionMissingCapabilityReturns400) {
  const auto execution_id = create_action_execution();

  auto req = make_request_with_match("/api/v1/components/engine/operations/long_calibration/executions/" + execution_id,
                                     R"(/api/v1/components/([^/]+)/operations/([^/]+)/executions/([^/]+))");
  req.body = R"({"parameters":{"order":8}})";
  httplib::Response res;

  handlers_->handle_update_execution(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = parse_json(res);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}
