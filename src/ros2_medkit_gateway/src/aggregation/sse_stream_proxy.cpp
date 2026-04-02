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

#include "ros2_medkit_gateway/aggregation/stream_proxy.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

SSEStreamProxy::SSEStreamProxy(const std::string & peer_url, const std::string & path, const std::string & peer_name)
  : peer_url_(peer_url), path_(path), peer_name_(peer_name) {
}

SSEStreamProxy::~SSEStreamProxy() {
  close();
}

void SSEStreamProxy::open() {
  // Guard against double-open: check if the reader thread is already running
  // (not just connected_, which may be false during reconnect backoff).
  if (reader_thread_.joinable()) {
    return;
  }
  should_stop_.store(false);
  reader_thread_ = std::thread(&SSEStreamProxy::reader_loop, this);
}

void SSEStreamProxy::close() {
  should_stop_.store(true);
  connected_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

bool SSEStreamProxy::is_connected() const {
  return connected_.load();
}

void SSEStreamProxy::on_event(std::function<void(const StreamEvent &)> cb) {
  callback_ = std::move(cb);
}

void SSEStreamProxy::reader_loop() {
  // Reconnect loop with exponential backoff.
  // On connection failure or stream interruption, retry with increasing delays
  // (1s, 2s, 4s, ..., max 30s) until should_stop_ is set.
  constexpr int kInitialBackoffMs = 1000;
  constexpr int kMaxBackoffMs = 30000;
  int backoff_ms = kInitialBackoffMs;

  while (!should_stop_.load()) {
    httplib::Client client(peer_url_);
    // SSE read timeout: 300s. If no data (including heartbeat comments) arrives
    // within this window, the connection is considered dead and we reconnect.
    // Peers should send periodic heartbeat comments (": keepalive\n\n") to
    // prevent timeout on idle streams.
    client.set_read_timeout(300, 0);
    client.set_connection_timeout(5, 0);

    // Track whether we received any data to set connected_ only after the
    // stream is actually delivering data (avoids race where is_connected()
    // returns true before the HTTP request even starts).
    bool received_first_data = false;
    non_sse_content_type_.store(false);

    // Use chunked content receiver to process SSE data as it arrives
    std::string buffer;
    auto result = client.Get(
        path_,
        [this](const httplib::Response & response) {
          // Header callback - check status and content type before processing body
          if (response.status != 200 || should_stop_.load()) {
            return false;
          }
          // Validate Content-Type is text/event-stream. A non-SSE 200 response
          // (e.g., application/json) would be silently misinterpreted as SSE data.
          auto ct_it = response.headers.find("Content-Type");
          if (ct_it == response.headers.end() || ct_it->second.find("text/event-stream") == std::string::npos) {
            // Not an SSE stream - abort connection. Log is not available here
            // (no rclcpp logger), so we set a flag and log in the outer scope.
            non_sse_content_type_.store(true);
            return false;
          }
          return true;
        },
        [this, &buffer, &received_first_data, &backoff_ms](const char * data, size_t data_length) {
          if (should_stop_.load()) {
            return false;  // Stop receiving
          }

          // Mark connected on first chunk of data from the stream.
          // This avoids the race condition where is_connected() returns true
          // before the HTTP connection is actually established.
          if (!received_first_data) {
            received_first_data = true;
            connected_.store(true);
            backoff_ms = 1000;  // Reset backoff on successful connection
          }

          constexpr size_t kMaxSSEBufferSize = 1 * 1024 * 1024;  // 1MB

          buffer.append(data, data_length);
          if (buffer.size() > kMaxSSEBufferSize) {
            return false;  // Disconnect - peer sending malformed stream
          }

          // Process complete events (delimited by double newline)
          size_t pos = 0;
          while (true) {
            auto boundary = buffer.find("\n\n", pos);
            if (boundary == std::string::npos) {
              break;
            }

            std::string event_block = buffer.substr(pos, boundary - pos + 1);
            pos = boundary + 2;

            auto events = parse_sse_data(event_block, peer_name_);
            if (callback_) {
              for (const auto & event : events) {
                callback_(event);
              }
            }
          }

          // Keep unprocessed data in buffer
          if (pos > 0) {
            buffer.erase(0, pos);
          }

          return true;  // Continue receiving
        });

    // Connection ended (server closed, timeout, or error) - mark disconnected
    connected_.store(false);

    // Log if the peer returned a non-SSE Content-Type (e.g. application/json)
    if (non_sse_content_type_.load()) {
      fprintf(stderr,
              "[SSEStreamProxy] Warning: peer '%s' at %s%s returned non-SSE "
              "Content-Type. Expected text/event-stream. Skipping.\n",
              peer_name_.c_str(), peer_url_.c_str(), path_.c_str());
      non_sse_content_type_.store(false);
    }

    // If stop was requested, exit without retrying
    if (should_stop_.load()) {
      break;
    }

    // Exponential backoff before reconnecting
    // Sleep in small increments so we can check should_stop_ promptly
    int slept_ms = 0;
    while (slept_ms < backoff_ms && !should_stop_.load()) {
      constexpr int kSleepStepMs = 100;
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepStepMs));
      slept_ms += kSleepStepMs;
    }

    backoff_ms = std::min(backoff_ms * 2, kMaxBackoffMs);
  }
}

std::vector<StreamEvent> SSEStreamProxy::parse_sse_data(const std::string & raw, const std::string & peer) {
  std::vector<StreamEvent> events;

  if (raw.empty()) {
    return events;
  }

  // Current event being built
  std::string current_event_type;
  std::string current_data;
  std::string current_id;
  bool has_data = false;

  std::istringstream stream(raw);
  std::string line;

  while (std::getline(stream, line)) {
    // Remove trailing \r if present (handles \r\n line endings)
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    // Blank line = event boundary
    if (line.empty()) {
      if (has_data) {
        StreamEvent event;
        event.event_type = current_event_type.empty() ? "message" : current_event_type;
        event.data = current_data;
        event.id = current_id;
        event.peer_name = peer;
        events.push_back(std::move(event));
      }
      // Reset for next event
      current_event_type.clear();
      current_data.clear();
      current_id.clear();
      has_data = false;
      continue;
    }

    // Skip comment lines (starting with ':')
    if (line[0] == ':') {
      continue;
    }

    // Parse field: value
    auto colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
      // Field with no value - ignored per SSE spec
      continue;
    }

    std::string field = line.substr(0, colon_pos);
    std::string value;
    if (colon_pos + 1 < line.size()) {
      // Skip optional single space after colon
      size_t value_start = colon_pos + 1;
      if (value_start < line.size() && line[value_start] == ' ') {
        value_start++;
      }
      value = line.substr(value_start);
    }

    if (field == "event") {
      current_event_type = value;
    } else if (field == "data") {
      if (has_data) {
        current_data += "\n";
      }
      current_data += value;
      has_data = true;
    } else if (field == "id") {
      current_id = value;
    }
    // "retry" and unknown fields are ignored
  }

  // Handle trailing event without final blank line
  if (has_data) {
    StreamEvent event;
    event.event_type = current_event_type.empty() ? "message" : current_event_type;
    event.data = current_data;
    event.id = current_id;
    event.peer_name = peer;
    events.push_back(std::move(event));
  }

  return events;
}

}  // namespace ros2_medkit_gateway
