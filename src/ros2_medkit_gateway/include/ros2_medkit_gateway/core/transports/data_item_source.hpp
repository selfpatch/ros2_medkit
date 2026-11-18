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

#include <memory>
#include <string>

#include "ros2_medkit_gateway/core/transports/topic_subscription_transport.hpp"

namespace ros2_medkit_gateway {

/// Delivers the value of one data item of an entity to a callback, repeatedly.
///
/// For a host whose data is served by a plugin and not by topics. The
/// `TriggerManager` uses it for data triggers that have no topic.
class DataItemSource {
 public:
  using SampleCallback = TopicSubscriptionTransport::SampleCallback;

  DataItemSource() = default;
  DataItemSource(const DataItemSource &) = delete;
  DataItemSource & operator=(const DataItemSource &) = delete;
  DataItemSource(DataItemSource &&) = delete;
  DataItemSource & operator=(DataItemSource &&) = delete;
  virtual ~DataItemSource() = default;

  /// Start delivering `item_name` of `entity_id` to `callback`. Returns nullptr when the source cannot watch it.
  /// Destroying the handle stops the delivery; no callback runs once the destructor has returned.
  virtual std::unique_ptr<TopicSubscriptionHandle> watch(const std::string & entity_id, const std::string & item_name,
                                                         SampleCallback callback) = 0;
};

}  // namespace ros2_medkit_gateway
