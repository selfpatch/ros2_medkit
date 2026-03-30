// Copyright 2026 Selfpatch GmbH
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

#include <sstream>
#include <string>
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
  if (connected_.load()) {
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
  httplib::Client client(peer_url_);
  // Set reasonable timeouts - SSE connections are long-lived.
  // Use a long read timeout instead of 0 (which causes immediate timeout in
  // cpp-httplib when SO_RCVTIMEO is set to zero). 24 hours allows SSE streams
  // to stay open indefinitely while still having a finite timeout for cleanup.
  client.set_read_timeout(86400, 0);
  client.set_connection_timeout(5, 0);

  connected_.store(true);

  // Use chunked content receiver to process SSE data as it arrives
  std::string buffer;
  auto result = client.Get(
      path_,
      [this](const httplib::Response & response) {
        // Header callback - check that we got the right content type
        return response.status == 200 && !should_stop_.load();
      },
      [this, &buffer](const char * data, size_t data_length) {
        if (should_stop_.load()) {
          return false;  // Stop receiving
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

  connected_.store(false);
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
