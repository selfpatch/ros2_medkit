// Copyright 2026 mfaferek93
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

#include "ros2_medkit_gateway/http/handlers/sse_fault_handler.hpp"

#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <sstream>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/gateway_node.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

SSEFaultHandler::SSEFaultHandler(HandlerContext & ctx) : ctx_(ctx) {
  // Create subscription to fault events topic
  // Use fully qualified topic name since FaultManager publishes on ~/events
  subscription_ = ctx_.node()->create_subscription<ros2_medkit_msgs::msg::FaultEvent>(
      "/fault_manager/events", rclcpp::QoS(100).reliable(),
      [this](const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
        on_fault_event(msg);
      });

  RCLCPP_INFO(HandlerContext::logger(), "SSE fault handler initialized, subscribed to /fault_manager/events");
}

SSEFaultHandler::~SSEFaultHandler() {
  // Signal shutdown and wake up any waiting clients
  shutdown_flag_.store(true);
  queue_cv_.notify_all();
}

void SSEFaultHandler::on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
  uint64_t event_id = next_event_id_.fetch_add(1);

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Add event to queue
    event_queue_.emplace_back(event_id, *msg);

    // Trim old events if buffer is full
    while (event_queue_.size() > kMaxBufferedEvents) {
      event_queue_.pop_front();
    }
  }

  // Notify all waiting clients
  queue_cv_.notify_all();

  RCLCPP_DEBUG(HandlerContext::logger(), "Received fault event: %s for %s (id=%" PRIu64 ")", msg->event_type.c_str(),
               msg->fault.fault_code.c_str(), event_id);
}

void SSEFaultHandler::handle_stream(const httplib::Request & req, httplib::Response & res) {
  RCLCPP_INFO(HandlerContext::logger(), "SSE client connected from %s", req.remote_addr.c_str());

  // Parse Last-Event-ID header for reconnection support
  uint64_t last_event_id = 0;
  if (req.has_header("Last-Event-ID")) {
    try {
      last_event_id = std::stoull(req.get_header_value("Last-Event-ID"));
    } catch (...) {
      // Ignore invalid Last-Event-ID
    }
  }

  client_count_.fetch_add(1);

  // Set SSE headers
  res.set_header("Content-Type", "text/event-stream");
  res.set_header("Cache-Control", "no-cache");
  res.set_header("Connection", "keep-alive");
  res.set_header("X-Accel-Buffering", "no");  // Disable nginx buffering

  // Use chunked content provider for streaming
  res.set_chunked_content_provider(
      "text/event-stream",
      [this, last_event_id](size_t /*offset*/, httplib::DataSink & sink) mutable {
        // First, send any buffered events the client missed (for reconnection)
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          for (const auto & [id, event] : event_queue_) {
            if (id > last_event_id) {
              std::string sse_msg = format_sse_event(event, id);
              if (!sink.write(sse_msg.data(), sse_msg.size())) {
                return false;  // Client disconnected
              }
              last_event_id = id;
            }
          }
        }

        // Wait for new events or keepalive timeout
        auto timeout = std::chrono::seconds(kKeepaliveIntervalSec);
        std::unique_lock<std::mutex> lock(queue_mutex_);

        while (true) {
          // Check for shutdown
          if (shutdown_flag_.load()) {
            return false;  // Handler is shutting down
          }

          // Wait for new event or timeout
          auto status = queue_cv_.wait_for(lock, timeout);

          // Check for shutdown after wakeup
          if (shutdown_flag_.load()) {
            return false;  // Handler is shutting down
          }

          if (status == std::cv_status::timeout) {
            // Send keepalive comment
            const char * keepalive = ":keepalive\n\n";
            lock.unlock();
            if (!sink.write(keepalive, strlen(keepalive))) {
              return false;  // Client disconnected
            }
            lock.lock();
            continue;
          }

          // Check for new events
          bool found_new = false;
          for (const auto & [id, event] : event_queue_) {
            if (id > last_event_id) {
              std::string sse_msg = format_sse_event(event, id);
              lock.unlock();
              if (!sink.write(sse_msg.data(), sse_msg.size())) {
                return false;  // Client disconnected
              }
              lock.lock();
              last_event_id = id;
              found_new = true;
            }
          }

          if (!found_new) {
            // Spurious wakeup, continue waiting
            continue;
          }
        }

        return true;
      },
      [this, addr = req.remote_addr](bool success) {
        client_count_.fetch_sub(1);
        RCLCPP_INFO(HandlerContext::logger(), "SSE client disconnected from %s (success=%d)", addr.c_str(), success);
      });
}

size_t SSEFaultHandler::connected_clients() const {
  return client_count_.load();
}

std::string SSEFaultHandler::format_sse_event(const ros2_medkit_msgs::msg::FaultEvent & event, uint64_t event_id) {
  nlohmann::json json_event;
  json_event["event_type"] = event.event_type;
  json_event["fault"] = fault_to_json(event.fault);

  // Convert timestamp to seconds with nanosecond precision
  double timestamp_sec = static_cast<double>(event.timestamp.sec) + static_cast<double>(event.timestamp.nanosec) * 1e-9;
  json_event["timestamp"] = timestamp_sec;

  std::ostringstream sse;
  sse << "id: " << event_id << "\n";
  sse << "event: " << event.event_type << "\n";
  sse << "data: " << json_event.dump() << "\n\n";

  return sse.str();
}

nlohmann::json SSEFaultHandler::fault_to_json(const ros2_medkit_msgs::msg::Fault & fault) {
  nlohmann::json j;
  j["fault_code"] = fault.fault_code;
  j["severity"] = fault.severity;
  j["description"] = fault.description;
  j["status"] = fault.status;
  j["occurrence_count"] = fault.occurrence_count;

  // Add human-readable severity label
  static const char * severity_labels[] = {"INFO", "WARN", "ERROR", "CRITICAL"};
  if (fault.severity < std::size(severity_labels)) {
    j["severity_label"] = severity_labels[fault.severity];
  } else {
    j["severity_label"] = "UNKNOWN";
  }

  // Convert timestamps to seconds
  j["first_occurred"] =
      static_cast<double>(fault.first_occurred.sec) + static_cast<double>(fault.first_occurred.nanosec) * 1e-9;
  j["last_occurred"] =
      static_cast<double>(fault.last_occurred.sec) + static_cast<double>(fault.last_occurred.nanosec) * 1e-9;

  // Reporting sources array
  j["reporting_sources"] = fault.reporting_sources;

  return j;
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
