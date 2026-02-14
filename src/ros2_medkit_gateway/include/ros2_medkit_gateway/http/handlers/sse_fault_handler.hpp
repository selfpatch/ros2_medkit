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

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handler for Server-Sent Events (SSE) fault streaming.
 *
 * Provides real-time fault event notifications via SSE at:
 * - GET /faults/stream
 *
 * Events streamed:
 * - fault_confirmed: When a fault transitions to CONFIRMED status
 * - fault_cleared: When a fault is manually cleared
 * - fault_updated: When fault data changes (occurrence_count, sources)
 *
 * Features:
 * - Multi-client support (multiple browsers can connect simultaneously)
 * - Keepalive every 30 seconds to prevent connection timeout
 * - Automatic reconnection support via Last-Event-ID header
 * - Replay buffer of up to 100 most recent events for reconnecting clients;
 *   when the buffer is full, older events are discarded (FIFO), so clients
 *   that are disconnected for long periods may miss some events
 */
class SSEFaultHandler {
 public:
  /**
   * @brief Construct SSE fault handler with shared context.
   * @param ctx The shared handler context
   */
  explicit SSEFaultHandler(HandlerContext & ctx);

  /// Destructor - cleanup subscription
  ~SSEFaultHandler();

  // Disable copy/move
  SSEFaultHandler(const SSEFaultHandler &) = delete;
  SSEFaultHandler & operator=(const SSEFaultHandler &) = delete;
  SSEFaultHandler(SSEFaultHandler &&) = delete;
  SSEFaultHandler & operator=(SSEFaultHandler &&) = delete;

  /**
   * @brief Handle GET /faults/stream - SSE stream endpoint.
   *
   * Establishes a long-lived connection and streams fault events in SSE format:
   * @code
   * event: fault_confirmed
   * data: {"event_type":"fault_confirmed","fault":{...},"timestamp":1234567890.123}
   *
   * event: fault_cleared
   * data: {"event_type":"fault_cleared","fault":{...},"timestamp":1234567890.456}
   * @endcode
   */
  void handle_stream(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Get the number of currently connected SSE clients.
   */
  size_t connected_clients() const;

 private:
  /// Callback for fault events from ROS 2 topic
  void on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg);

  /// Format a fault event as SSE message
  static std::string format_sse_event(const ros2_medkit_msgs::msg::FaultEvent & event, uint64_t event_id);

  HandlerContext & ctx_;

  /// Subscription to fault events topic
  rclcpp::Subscription<ros2_medkit_msgs::msg::FaultEvent>::SharedPtr subscription_;

  /// Event queue for broadcasting to clients
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::pair<uint64_t, ros2_medkit_msgs::msg::FaultEvent>> event_queue_;

  /// Monotonically increasing event ID for Last-Event-ID support
  std::atomic<uint64_t> next_event_id_{1};

  /// Number of connected clients (for monitoring)
  std::atomic<size_t> client_count_{0};

  /// Maximum allowed concurrent SSE clients (from sse.max_clients parameter)
  size_t max_sse_clients_{10};

  /// Shutdown flag for clean termination
  std::atomic<bool> shutdown_flag_{false};

  /// Maximum events to buffer (for reconnecting clients)
  static constexpr size_t kMaxBufferedEvents = 100;

  /// Keepalive interval in seconds
  static constexpr int kKeepaliveIntervalSec = 30;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

