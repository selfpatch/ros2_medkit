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

#pragma once

#include <httplib.h>

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace ros2_medkit_gateway {

/**
 * @brief A single event received from a streaming connection
 *
 * Represents an SSE event with type, data payload, optional ID, and
 * the name of the peer gateway that produced it.
 */
struct StreamEvent {
  std::string event_type;  ///< SSE event name (e.g., "data", "fault")
  std::string data;        ///< Event data (JSON string)
  std::string id;          ///< Event ID (optional)
  std::string peer_name;   ///< Which peer this event came from
};

/**
 * @brief Transport-agnostic interface for proxying streaming connections
 *
 * StreamProxy abstracts the transport layer for streaming event connections
 * to peer gateways. Currently implemented with SSE, but the interface allows
 * future implementations using WebSocket or gRPC streams.
 *
 * Usage:
 *   proxy->on_event([](const StreamEvent& e) { handle(e); });
 *   proxy->open();
 *   // ... events flow via callback ...
 *   proxy->close();
 */
class StreamProxy {
 public:
  virtual ~StreamProxy() = default;

  /// Start the streaming connection (non-blocking, spawns reader thread)
  virtual void open() = 0;

  /// Stop the streaming connection and join the reader thread
  virtual void close() = 0;

  /// Check if the streaming connection is currently active
  virtual bool is_connected() const = 0;

  /// Register a callback for incoming stream events
  virtual void on_event(std::function<void(const StreamEvent &)> cb) = 0;
};

/**
 * @brief SSE (Server-Sent Events) implementation of StreamProxy
 *
 * Connects to a peer gateway's SSE endpoint using cpp-httplib and parses
 * the SSE text/event-stream format into StreamEvent objects. Runs a
 * background reader thread that invokes the registered callback for each
 * parsed event.
 *
 * Thread safety: connected_ and should_stop_ are atomic. The callback
 * is set before open() and not modified afterwards.
 */
class SSEStreamProxy : public StreamProxy {
 public:
  /**
   * @brief Construct an SSEStreamProxy
   * @param peer_url Base URL of the peer gateway (e.g., "http://localhost:8081")
   * @param path SSE endpoint path (e.g., "/api/v1/components/abc/faults/sse")
   * @param peer_name Human-readable name for the peer (used in StreamEvent::peer_name)
   */
  SSEStreamProxy(const std::string & peer_url, const std::string & path, const std::string & peer_name = "");

  ~SSEStreamProxy() override;

  void open() override;
  void close() override;
  bool is_connected() const override;
  void on_event(std::function<void(const StreamEvent &)> cb) override;

  /**
   * @brief Parse raw SSE text/event-stream data into StreamEvent objects
   *
   * SSE format: events are separated by blank lines. Each event consists
   * of field lines: "event:", "data:", "id:". Multiple "data:" lines are
   * joined with newlines.
   *
   * This is a pure function suitable for unit testing without networking.
   *
   * @param raw Raw SSE text data
   * @param peer Peer name to set on each parsed event
   * @return Vector of parsed StreamEvent objects
   */
  static std::vector<StreamEvent> parse_sse_data(const std::string & raw, const std::string & peer);

 private:
  /// Background thread loop that connects and reads SSE events
  void reader_loop();

  std::string peer_url_;
  std::string path_;
  std::string peer_name_;
  std::atomic<bool> connected_{false};
  std::atomic<bool> should_stop_{false};
  std::function<void(const StreamEvent &)> callback_;
  std::thread reader_thread_;
};

}  // namespace ros2_medkit_gateway
