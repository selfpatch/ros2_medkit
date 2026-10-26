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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/transports/data_item_source.hpp"

namespace ros2_medkit_gateway {

class PluginManager;

/// A `DataItemSource` that reads each watched item from one thread, once per interval.
///
/// A new watch is read at once. A read that fails delivers nothing and is tried again at the next interval.
class PollingDataItemSource : public DataItemSource {
 public:
  using ReadFn = std::function<tl::expected<nlohmann::json, std::string>(const std::string & entity_id,
                                                                         const std::string & item_name)>;

  PollingDataItemSource(ReadFn read, std::chrono::milliseconds interval);
  ~PollingDataItemSource() override;

  std::unique_ptr<TopicSubscriptionHandle> watch(const std::string & entity_id, const std::string & item_name,
                                                 SampleCallback callback) override;

 private:
  struct Shared;
  class Handle;

  std::shared_ptr<Shared> shared_;
};

/// A polling source over `read_plugin_data_item`. The sample is the `data` field of what
/// `GET /{entity}/data/{item}` answers for an entity a plugin serves, which is what a topic trigger sees: the message.
/// A body with no `data` field is the sample as it is. `plugins` must outlive the source.
std::unique_ptr<DataItemSource> make_plugin_data_item_source(PluginManager * plugins,
                                                             std::chrono::milliseconds interval);

}  // namespace ros2_medkit_gateway
