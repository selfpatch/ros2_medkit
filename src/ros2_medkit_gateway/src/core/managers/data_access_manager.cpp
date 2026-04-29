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

#include "ros2_medkit_gateway/core/managers/data_access_manager.hpp"

#include <chrono>
#include <utility>

namespace ros2_medkit_gateway {

DataAccessManager::DataAccessManager(std::shared_ptr<TopicTransport> transport, double topic_sample_timeout_sec)
  : transport_(std::move(transport)), topic_sample_timeout_sec_(topic_sample_timeout_sec) {
  // Validate topic_sample_timeout_sec_ against the supported range [0.1, 30.0].
  // Out-of-range values fall back to a safe default rather than aborting; the
  // value is operator-supplied so we prefer graceful degradation here.
  if (topic_sample_timeout_sec_ < 0.1 || topic_sample_timeout_sec_ > 30.0) {
    topic_sample_timeout_sec_ = 1.0;
  }
}

json DataAccessManager::publish_to_topic(const std::string & topic_path, const std::string & msg_type,
                                         const json & data, double timeout_sec) {
  return transport_->publish(topic_path, msg_type, data, std::chrono::duration<double>(timeout_sec));
}

json DataAccessManager::get_topic_sample_with_fallback(const std::string & topic_name, double timeout_sec) {
  // Use configured parameter when callers pass a sentinel negative timeout.
  const double effective_timeout = (timeout_sec < 0.0) ? topic_sample_timeout_sec_ : timeout_sec;
  return get_topic_sample_native(topic_name, effective_timeout);
}

json DataAccessManager::get_topic_sample_native(const std::string & topic_name, double timeout_sec) {
  // Fast path: when no publishers are present, return metadata-only without
  // engaging the transport. This mirrors the legacy behaviour of returning
  // a metadata frame for idle topics rather than throwing.
  auto [pubs, subs] = transport_->count_publishers_subscribers(topic_name);
  if (pubs == 0) {
    json out;
    out["topic"] = topic_name;
    out["status"] = "metadata_only";
    out["publisher_count"] = pubs;
    out["subscriber_count"] = subs;
    return out;
  }

  TopicSample sample = transport_->sample(topic_name, std::chrono::duration<double>(timeout_sec));

  json out;
  out["topic"] = sample.topic.empty() ? topic_name : sample.topic;
  out["status"] = sample.status;
  out["publisher_count"] = sample.publisher_count;
  out["subscriber_count"] = sample.subscriber_count;
  if (!sample.type.empty()) {
    out["type"] = sample.type;
    if (!sample.type_info.empty() && !sample.type_info.is_null()) {
      out["type_info"] = sample.type_info;
    }
  }
  if (sample.status == "data" && !sample.data.is_null()) {
    out["data"] = sample.data;
    out["timestamp"] =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
  }
  if (!sample.success && !sample.error_message.empty()) {
    out["error"] = sample.error_message;
  }
  return out;
}

}  // namespace ros2_medkit_gateway
