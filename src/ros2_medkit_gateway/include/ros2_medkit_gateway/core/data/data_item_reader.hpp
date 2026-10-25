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

#include <string>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/data/data_types.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/dto/data.hpp"

namespace ros2_medkit_gateway {

class DataAccessManager;
class PluginManager;

/// An `x-medkit-plugin-error` with the status clamped to [400, 599] and the message cut to 512 characters.
ErrorInfo make_plugin_error(int http_status, const std::string & message, nlohmann::json extra_params = {});

/// The plugin error for a failed `DataProvider` call.
ErrorInfo make_provider_error(const DataProviderErrorInfo & info, const std::string & entity_id);

/// A data item is addressed by its full topic path. The leading slash is optional in a route segment.
std::string to_full_topic_path(const std::string & topic_name);

/// Reads one data item of a plugin-owned entity: through its `DataProvider`, else through its
/// `x-plc-data` route. `GET /{entity}/data/{item}` and the cyclic-subscription sampler both use it.
tl::expected<dto::DataValue, ErrorInfo> read_plugin_data_item(PluginManager * plugins, const std::string & entity_id,
                                                              const std::string & topic_name);

/// Samples one topic through the `TopicDataProvider` of `data`. Answers 503 when none is configured.
/// `GET /{entity}/data/{item}` and the cyclic-subscription sampler both use it.
tl::expected<TopicSampleResult, ErrorInfo> sample_data_topic(DataAccessManager & data,
                                                             const std::string & full_topic_path);

}  // namespace ros2_medkit_gateway
