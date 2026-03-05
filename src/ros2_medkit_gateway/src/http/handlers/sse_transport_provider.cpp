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

#include "ros2_medkit_gateway/http/handlers/sse_transport_provider.hpp"

#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {

SseTransportProvider::SseTransportProvider(SubscriptionManager & sub_mgr,
                                           std::shared_ptr<SSEClientTracker> client_tracker)
  : sub_mgr_(sub_mgr), client_tracker_(std::move(client_tracker)) {
}

tl::expected<std::string, std::string> SseTransportProvider::start(const CyclicSubscriptionInfo & info,
                                                                   ResourceSamplerFn json_sampler, GatewayNode *) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    streams_[info.id] = StreamState{std::move(json_sampler)};
  }

  // SSE event_source is the events endpoint URL
  return std::string(API_BASE_PATH) + "/" + info.entity_type + "/" + info.entity_id + "/cyclic-subscriptions/" +
         info.id + "/events";
}

void SseTransportProvider::notify_update(const std::string & sub_id) {
  // SSE transport re-reads from sub_mgr_ each iteration, so no-op here.
  (void)sub_id;
}

void SseTransportProvider::stop(const std::string & sub_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  streams_.erase(sub_id);
  // The SSE loop detects inactive via sub_mgr_.is_active() and exits.
}

bool SseTransportProvider::handle_client_connect(const std::string & sub_id, const httplib::Request & req,
                                                 httplib::Response & res) {
  ResourceSamplerFn sampler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = streams_.find(sub_id);
    if (it == streams_.end()) {
      return false;
    }
    sampler = it->second.sampler;
  }

  // Check combined SSE client limit
  if (!client_tracker_->try_connect()) {
    RCLCPP_WARN(handlers::HandlerContext::logger(),
                "SSE client limit reached (%zu), rejecting cyclic subscription stream from %s",
                client_tracker_->max_clients(), req.remote_addr.c_str());
    handlers::HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE,
                                         "Maximum number of SSE clients reached. Please try again later.");
    return true;  // We handled it (with an error)
  }

  RCLCPP_INFO(handlers::HandlerContext::logger(), "SSE cyclic subscription client connected from %s (%zu/%zu)",
              req.remote_addr.c_str(), client_tracker_->connected_clients(), client_tracker_->max_clients());

  // Set SSE headers
  res.set_header("Cache-Control", "no-cache");
  res.set_header("Connection", "keep-alive");
  res.set_header("X-Accel-Buffering", "no");

  auto captured_sub_id = sub_id;
  auto captured_sampler = std::move(sampler);

  res.set_chunked_content_provider(
      "text/event-stream",
      [this, captured_sub_id, captured_sampler](size_t, httplib::DataSink & sink) {
        auto keepalive_timeout = std::chrono::seconds(kKeepaliveIntervalSec);
        auto last_write = std::chrono::steady_clock::now();

        while (true) {
          // Check if subscription is still active
          if (!sub_mgr_.is_active(captured_sub_id)) {
            return false;
          }

          // Send keepalive if no data was written recently
          auto since_last_write = std::chrono::steady_clock::now() - last_write;
          if (since_last_write >= keepalive_timeout) {
            const char * keepalive = ":keepalive\n\n";
            if (!sink.write(keepalive, std::strlen(keepalive))) {
              return false;
            }
            last_write = std::chrono::steady_clock::now();
          }

          // Get current subscription info (may have been updated via PUT)
          auto current_sub = sub_mgr_.get(captured_sub_id);
          if (!current_sub) {
            return false;
          }

          auto sample_interval = interval_to_duration(current_sub->interval);

          // Sample the resource via the registered sampler
          json envelope;

          // UTC timestamp in ISO 8601 format
          auto now = std::chrono::system_clock::now();
          auto time_t_val = std::chrono::system_clock::to_time_t(now);
          auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
          std::ostringstream ts;
          struct tm tm_buf;
          gmtime_r(&time_t_val, &tm_buf);
          ts << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
          ts << "." << std::setw(3) << std::setfill('0') << ms << "Z";
          envelope["timestamp"] = ts.str();

          try {
            auto sample_result = captured_sampler(current_sub->entity_id, current_sub->resource_path);
            if (sample_result.has_value()) {
              envelope["payload"] = *sample_result;
            } else {
              json error;
              error["error_code"] = ERR_VENDOR_ERROR;
              error["message"] = sample_result.error();
              envelope["error"] = error;
            }
          } catch (const std::exception & e) {
            json error;
            error["error_code"] = ERR_INTERNAL_ERROR;
            error["message"] = std::string("Sampler exception: ") + e.what();
            envelope["error"] = error;
          }

          // Format SSE data frame
          std::string sse_msg = "data: " + envelope.dump() + "\n\n";
          if (!sink.write(sse_msg.data(), sse_msg.size())) {
            return false;
          }
          last_write = std::chrono::steady_clock::now();

          // Wait for interval or notification
          sub_mgr_.wait_for_update(captured_sub_id, sample_interval);
        }

        return true;
      },
      [this, captured_sub_id](bool) {
        client_tracker_->disconnect();
        RCLCPP_DEBUG(rclcpp::get_logger("rest_server"), "SSE cyclic subscription stream disconnected: %s",
                     captured_sub_id.c_str());
      });

  return true;
}

}  // namespace ros2_medkit_gateway
