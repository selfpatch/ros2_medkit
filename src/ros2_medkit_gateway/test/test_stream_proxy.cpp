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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "ros2_medkit_gateway/aggregation/stream_proxy.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// Construction and lifecycle tests
// =============================================================================

TEST(StreamProxy, sse_proxy_can_be_constructed) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_a");
  // Should not throw or crash
}

TEST(StreamProxy, initially_not_connected) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_a");
  EXPECT_FALSE(proxy.is_connected());
}

TEST(StreamProxy, close_is_idempotent) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_a");
  // Calling close() multiple times without open() should not crash
  proxy.close();
  proxy.close();
  proxy.close();
  EXPECT_FALSE(proxy.is_connected());
}

// =============================================================================
// SSE parsing tests (pure function, no networking required)
// =============================================================================

TEST(StreamProxy, parse_sse_single_event) {
  std::string raw = "data: {\"temperature\": 42}\n\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_a");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "{\"temperature\": 42}");
  EXPECT_EQ(events[0].event_type, "message");  // Default SSE type
  EXPECT_EQ(events[0].peer_name, "peer_a");
  EXPECT_TRUE(events[0].id.empty());
}

TEST(StreamProxy, parse_sse_multiple_events) {
  std::string raw =
      "data: {\"temp\": 42}\n"
      "\n"
      "data: {\"temp\": 43}\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_b");

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0].data, "{\"temp\": 42}");
  EXPECT_EQ(events[0].peer_name, "peer_b");
  EXPECT_EQ(events[1].data, "{\"temp\": 43}");
  EXPECT_EQ(events[1].peer_name, "peer_b");
}

TEST(StreamProxy, parse_sse_multiline_data) {
  std::string raw =
      "data: line one\n"
      "data: line two\n"
      "data: line three\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_c");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "line one\nline two\nline three");
  EXPECT_EQ(events[0].event_type, "message");
}

TEST(StreamProxy, parse_sse_with_event_type) {
  std::string raw =
      "event: data_update\n"
      "data: {\"sensor\": \"imu\"}\n"
      "id: 123\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_d");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].event_type, "data_update");
  EXPECT_EQ(events[0].data, "{\"sensor\": \"imu\"}");
  EXPECT_EQ(events[0].id, "123");
  EXPECT_EQ(events[0].peer_name, "peer_d");
}

TEST(StreamProxy, parse_sse_empty_input) {
  auto events = SSEStreamProxy::parse_sse_data("", "peer_e");
  EXPECT_TRUE(events.empty());
}

// =============================================================================
// Additional SSE parsing edge cases
// =============================================================================

TEST(StreamProxy, parse_sse_with_comments) {
  std::string raw =
      ": this is a comment\n"
      "data: hello\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_f");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "hello");
}

TEST(StreamProxy, parse_sse_event_without_trailing_blank_line) {
  // An event at end of stream without a trailing blank line should still parse
  std::string raw = "event: fault_added\ndata: {\"fault_id\": \"F001\"}\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_g");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].event_type, "fault_added");
  EXPECT_EQ(events[0].data, "{\"fault_id\": \"F001\"}");
}

TEST(StreamProxy, parse_sse_mixed_events_with_and_without_types) {
  std::string raw =
      "event: fault_added\n"
      "data: {\"fault_id\": \"F001\"}\n"
      "\n"
      "data: {\"heartbeat\": true}\n"
      "\n"
      "event: fault_cleared\n"
      "data: {\"fault_id\": \"F001\"}\n"
      "id: 42\n"
      "\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_h");

  ASSERT_EQ(events.size(), 3u);

  EXPECT_EQ(events[0].event_type, "fault_added");
  EXPECT_EQ(events[0].data, "{\"fault_id\": \"F001\"}");
  EXPECT_TRUE(events[0].id.empty());

  EXPECT_EQ(events[1].event_type, "message");
  EXPECT_EQ(events[1].data, "{\"heartbeat\": true}");

  EXPECT_EQ(events[2].event_type, "fault_cleared");
  EXPECT_EQ(events[2].data, "{\"fault_id\": \"F001\"}");
  EXPECT_EQ(events[2].id, "42");
}

TEST(StreamProxy, parse_sse_with_crlf_line_endings) {
  std::string raw = "event: update\r\ndata: payload\r\n\r\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_i");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].event_type, "update");
  EXPECT_EQ(events[0].data, "payload");
}

TEST(StreamProxy, parse_sse_data_with_colons_in_value) {
  // The SSE spec says only the first colon splits field from value
  std::string raw = "data: http://example.com:8080/path\n\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_j");

  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].data, "http://example.com:8080/path");
}

TEST(StreamProxy, parse_sse_blank_lines_only) {
  // Only blank lines with no data should produce no events
  std::string raw = "\n\n\n\n";

  auto events = SSEStreamProxy::parse_sse_data(raw, "peer_k");
  EXPECT_TRUE(events.empty());
}

TEST(StreamProxy, on_event_can_set_callback) {
  SSEStreamProxy proxy("http://localhost:8081", "/api/v1/faults/sse", "peer_l");
  bool called = false;
  proxy.on_event([&called](const StreamEvent &) {
    called = true;
  });
  // Callback is set but not invoked without open()
  EXPECT_FALSE(called);
}
