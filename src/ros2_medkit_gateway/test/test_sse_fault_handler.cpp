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

#include <httplib.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <chrono>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/handlers/sse_fault_handler.hpp"
#include "ros2_medkit_gateway/http/sse_client_tracker.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::SSEClientTracker;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::SSEFaultHandler;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::msg::FaultEvent;

namespace {

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }

  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;

  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(sock);
    return 0;
  }

  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    close(sock);
    return 0;
  }

  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

FaultEvent make_fault_event(const std::string & event_type, const std::string & fault_code, int32_t sec,
                            uint32_t nanosec = 0) {
  FaultEvent event;
  event.event_type = event_type;
  event.timestamp.sec = sec;
  event.timestamp.nanosec = nanosec;
  event.fault.fault_code = fault_code;
  event.fault.description = "Synthetic SSE fault";
  event.fault.severity = Fault::SEVERITY_ERROR;
  event.fault.status = Fault::STATUS_CONFIRMED;
  event.fault.occurrence_count = 3;
  event.fault.reporting_sources = {"/apps/temp_sensor"};
  event.fault.first_occurred.sec = sec - 5;
  event.fault.last_occurred.sec = sec;
  event.fault.last_occurred.nanosec = nanosec;
  return event;
}

httplib::Request make_stream_request(const std::string & remote_addr, const std::string & last_event_id = "") {
  httplib::Request req;
  req.remote_addr = remote_addr;
  if (!last_event_id.empty()) {
    req.headers.emplace("Last-Event-ID", last_event_id);
  }
  return req;
}

std::string read_stream_once(httplib::Response & res, size_t writes_before_disconnect) {
  std::string output;
  size_t write_count = 0;

  httplib::DataSink sink;
  sink.write = [&](const char * data, size_t data_len) {
    output.append(data, data_len);
    write_count++;
    return write_count < writes_before_disconnect;
  };
  sink.is_writable = []() {
    return true;
  };
  sink.done = []() {};
  sink.done_with_trailer = [](const httplib::Headers &) {};

  EXPECT_TRUE(static_cast<bool>(res.content_provider_));
  EXPECT_FALSE(res.content_provider_(0, 0, sink));

  return output;
}

json parse_sse_payload(const std::string & sse_frame) {
  auto data_pos = sse_frame.find("data: ");
  if (data_pos == std::string::npos) {
    ADD_FAILURE() << "Missing SSE data field in frame: " << sse_frame;
    return json::object();
  }
  auto data_end = sse_frame.find("\n\n", data_pos);
  if (data_end == std::string::npos) {
    ADD_FAILURE() << "Missing SSE frame terminator in frame: " << sse_frame;
    return json::object();
  }
  auto payload = sse_frame.substr(data_pos + 6, data_end - (data_pos + 6));
  return json::parse(payload);
}

void release_stream(httplib::Response & res, bool success = false) {
  if (res.content_provider_resource_releaser_) {
    res.content_provider_resource_releaser_(success);
    res.content_provider_resource_releaser_ = nullptr;
  }
}

}  // namespace

class SSEFaultHandlerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    int server_port = reserve_local_port();
    ASSERT_NE(server_port, 0);

    auto options = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(false).parameter_overrides({
        {"server.port", server_port},
    });

    node_ = std::make_shared<GatewayNode>(options);

    CorsConfig cors_config;
    AuthConfig auth_config;
    TlsConfig tls_config;

    ctx_ = std::make_unique<HandlerContext>(node_.get(), cors_config, auth_config, tls_config, nullptr);
    tracker_ = std::make_shared<SSEClientTracker>(4);
    handler_ = std::make_unique<SSEFaultHandler>(*ctx_, tracker_);

    publisher_node_ =
        std::make_shared<rclcpp::Node>("test_sse_fault_handler_publisher_" + std::to_string(test_counter_++));
    fault_events_topic_ = ros2_medkit_gateway::build_fault_manager_events_topic(node_.get());
    publisher_ = publisher_node_->create_publisher<FaultEvent>(fault_events_topic_, rclcpp::QoS(100).reliable());

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(publisher_node_);

    wait_for_subscribers();
  }

  void TearDown() override {
    executor_.reset();
    publisher_.reset();
    publisher_node_.reset();
    handler_.reset();
    ctx_.reset();
    node_.reset();
  }

  void enqueue_event(const FaultEvent & event) {
    publisher_->publish(event);

    for (int i = 0; i < 20; ++i) {
      executor_->spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  void wait_for_subscribers() {
    for (int i = 0; i < 50; ++i) {
      executor_->spin_some();
      if (publisher_->get_subscription_count() >= 1u) {
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
    FAIL() << "Timed out waiting for SSE fault handler subscription on " << fault_events_topic_;
  }

  std::shared_ptr<GatewayNode> node_;
  static inline int test_counter_ = 0;
  std::unique_ptr<HandlerContext> ctx_;
  std::shared_ptr<SSEClientTracker> tracker_;
  std::unique_ptr<SSEFaultHandler> handler_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<FaultEvent>::SharedPtr publisher_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::string fault_events_topic_;
};

TEST_F(SSEFaultHandlerTest, HandleStreamSetsSseHeadersAndChunkedProvider) {
  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;

  handler_->handle_stream(req, res);

  EXPECT_EQ(res.get_header_value("Content-Type"), "text/event-stream");
  EXPECT_EQ(res.get_header_value("Cache-Control"), "no-cache");
  EXPECT_EQ(res.get_header_value("Connection"), "keep-alive");
  EXPECT_EQ(res.get_header_value("X-Accel-Buffering"), "no");
  EXPECT_TRUE(res.is_chunked_content_provider_);
  EXPECT_EQ(handler_->connected_clients(), 1u);

  release_stream(res);
  EXPECT_EQ(handler_->connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, HandleStreamRejectsWhenClientLimitIsReached) {
  auto limited_tracker = std::make_shared<SSEClientTracker>(1);
  SSEFaultHandler limited_handler(*ctx_, limited_tracker);

  auto req_one = make_stream_request("127.0.0.1");
  auto req_two = make_stream_request("127.0.0.2");
  httplib::Response res_one;
  httplib::Response res_two;

  limited_handler.handle_stream(req_one, res_one);
  limited_handler.handle_stream(req_two, res_two);

  EXPECT_EQ(limited_handler.connected_clients(), 1u);
  EXPECT_EQ(res_two.status, 503);
  auto body = json::parse(res_two.body);
  EXPECT_EQ(body["error_code"], "service-unavailable");

  release_stream(res_one);
  EXPECT_EQ(limited_handler.connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, StreamReplaysBufferedEventsUsingSseFormat) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "TEMP_HIGH", 123, 456000000));

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_NE(output.find("id: 1\n"), std::string::npos);
  EXPECT_NE(output.find("event: fault_confirmed\n"), std::string::npos);
  EXPECT_NE(output.find("data: "), std::string::npos);

  auto payload = parse_sse_payload(output);
  EXPECT_EQ(payload["event_type"], "fault_confirmed");
  EXPECT_EQ(payload["fault"]["fault_code"], "TEMP_HIGH");
  EXPECT_EQ(payload["fault"]["severity_label"], "ERROR");
  EXPECT_DOUBLE_EQ(payload["timestamp"].get<double>(), 123.456);

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, LastEventIdReplaysOnlyMissedBufferedEvents) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_ONE", 100));
  enqueue_event(make_fault_event(FaultEvent::EVENT_UPDATED, "FAULT_TWO", 101));

  auto req = make_stream_request("127.0.0.1", "1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  EXPECT_NE(output.find("id: 2\n"), std::string::npos);
  EXPECT_EQ(payload["event_type"], "fault_updated");
  EXPECT_EQ(payload["fault"]["fault_code"], "FAULT_TWO");
  EXPECT_EQ(payload["timestamp"], 101.0);

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, InvalidLastEventIdFallsBackToFullReplay) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_X", 200));

  auto req = make_stream_request("127.0.0.1", "not-a-number");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_NE(output.find("id: 1\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output)["fault"]["fault_code"], "FAULT_X");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, BufferedEventsAreReplayedToMultipleClients) {
  enqueue_event(make_fault_event(FaultEvent::EVENT_CLEARED, "FAULT_MULTI", 55));

  auto req_one = make_stream_request("127.0.0.1");
  auto req_two = make_stream_request("127.0.0.2");
  httplib::Response res_one;
  httplib::Response res_two;

  handler_->handle_stream(req_one, res_one);
  handler_->handle_stream(req_two, res_two);

  auto output_one = read_stream_once(res_one, 1);
  auto output_two = read_stream_once(res_two, 1);

  EXPECT_NE(output_one.find("id: 1\n"), std::string::npos);
  EXPECT_NE(output_two.find("id: 1\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output_one)["fault"]["fault_code"], "FAULT_MULTI");
  EXPECT_EQ(parse_sse_payload(output_two)["fault"]["fault_code"], "FAULT_MULTI");

  release_stream(res_one);
  release_stream(res_two);
  EXPECT_EQ(handler_->connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, BufferEvictsOldestEventWhenFull) {
  for (int i = 1; i <= 101; ++i) {
    enqueue_event(make_fault_event(FaultEvent::EVENT_CONFIRMED, "FAULT_" + std::to_string(i), i));
  }

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_NE(output.find("id: 2\n"), std::string::npos);
  EXPECT_EQ(parse_sse_payload(output)["fault"]["fault_code"], "FAULT_2");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamSanitizesNewlinesInEventType) {
  enqueue_event(make_fault_event("fault_confirmed\nretry: 0\r\nevent: injected", "FAULT_SANITIZED", 300));

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  handler_->handle_stream(req, res);

  auto output = read_stream_once(res, 1);
  auto payload = parse_sse_payload(output);

  EXPECT_NE(output.find("event: fault_confirmedretry: 0event: injected\n"), std::string::npos);
  EXPECT_EQ(output.find("\nretry: 0\r\n"), std::string::npos);
  EXPECT_EQ(output.find("\nevent: injected\n"), std::string::npos);
  EXPECT_EQ(payload["event_type"], "fault_confirmedretry: 0event: injected");

  release_stream(res);
}

TEST_F(SSEFaultHandlerTest, StreamSendsKeepaliveCommentAfterTimeout) {
  auto fast_tracker = std::make_shared<SSEClientTracker>(1);
  SSEFaultHandler fast_handler(*ctx_, fast_tracker, 10ms);

  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;
  fast_handler.handle_stream(req, res);

  auto output = read_stream_once(res, 1);

  EXPECT_EQ(output, ":keepalive\n\n");

  release_stream(res);
  EXPECT_EQ(fast_handler.connected_clients(), 0u);
}

TEST_F(SSEFaultHandlerTest, NonPositiveKeepaliveOverrideLogsWarning) {
  testing::internal::CaptureStderr();
  {
    auto warn_tracker = std::make_shared<SSEClientTracker>(1);
    SSEFaultHandler warn_handler(*ctx_, warn_tracker, 0ms);
  }
  auto logs = testing::internal::GetCapturedStderr();

  EXPECT_NE(logs.find("Non-positive SSE keepalive override"), std::string::npos);
}

TEST_F(SSEFaultHandlerTest, DisconnectReleasesTrackedClientSlot) {
  auto req = make_stream_request("127.0.0.1");
  httplib::Response res;

  handler_->handle_stream(req, res);
  EXPECT_EQ(handler_->connected_clients(), 1u);

  release_stream(res, false);

  EXPECT_EQ(handler_->connected_clients(), 0u);
}
