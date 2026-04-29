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

#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/// RAII handle for a single trigger-side topic subscription. Destruction
/// unsubscribes; manager holds these in its tracking map.
class TopicSubscriptionHandle {
 public:
  TopicSubscriptionHandle() = default;
  TopicSubscriptionHandle(const TopicSubscriptionHandle &) = delete;
  TopicSubscriptionHandle & operator=(const TopicSubscriptionHandle &) = delete;
  TopicSubscriptionHandle(TopicSubscriptionHandle &&) = delete;
  TopicSubscriptionHandle & operator=(TopicSubscriptionHandle &&) = delete;
  virtual ~TopicSubscriptionHandle() = default;
};

/// Port for the trigger subsystem to subscribe to a topic and receive each
/// sample as JSON. Replaces the direct `TriggerTopicSubscriber *` pointer
/// the trigger manager currently holds; the adapter wraps the existing
/// subscriber to preserve its destructor pattern.
class TopicSubscriptionTransport {
 public:
  using SampleCallback = std::function<void(const json & sample)>;

  TopicSubscriptionTransport() = default;
  TopicSubscriptionTransport(const TopicSubscriptionTransport &) = delete;
  TopicSubscriptionTransport & operator=(const TopicSubscriptionTransport &) = delete;
  TopicSubscriptionTransport(TopicSubscriptionTransport &&) = delete;
  TopicSubscriptionTransport & operator=(TopicSubscriptionTransport &&) = delete;
  virtual ~TopicSubscriptionTransport() = default;

  /// Subscribe and return a handle. Returns nullptr on transport error
  /// (topic not advertised, type mismatch, etc.). The handle's destructor
  /// guarantees no callback fires after destruction completes.
  virtual std::unique_ptr<TopicSubscriptionHandle> subscribe(const std::string & topic_path,
                                                             const std::string & msg_type, SampleCallback callback) = 0;
};

}  // namespace ros2_medkit_gateway
