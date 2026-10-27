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

#include "ros2_medkit_gateway/core/data/data_item_reader.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <utility>

#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/logging.hpp"
#include "ros2_medkit_gateway/core/managers/data_access_manager.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_support.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;
using handlers::make_error;

ErrorInfo make_plugin_error(int http_status, const std::string & message, json extra_params) {
  static constexpr size_t kMaxMessageLength = 512;
  int status = http_status < 400 ? 400 : (http_status > 599 ? 599 : http_status);
  std::string msg = message.size() > kMaxMessageLength ? message.substr(0, kMaxMessageLength) + "..." : message;
  return make_error(status, ERR_PLUGIN_ERROR, std::move(msg), std::move(extra_params));
}

ErrorInfo make_provider_error(const DataProviderErrorInfo & info, const std::string & entity_id) {
  return make_plugin_error(info.http_status, info.message, json{{"entity_id", entity_id}});
}

std::string to_full_topic_path(const std::string & topic_name) {
  if (topic_name.empty() || topic_name.front() == '/') {
    return topic_name;
  }
  return "/" + topic_name;
}

tl::expected<dto::DataValue, ErrorInfo> read_plugin_data_item(PluginManager * plugins, const std::string & entity_id,
                                                              const std::string & topic_name) {
  const log::Logger logger("rest_server");
  auto * data_prov = plugins ? plugins->get_data_provider_for_entity(entity_id) : nullptr;
  if (data_prov == nullptr) {
    // Same x-plc-data route fallback as the data list, single-item form.
    if (auto via_route = plugins ? plugins->fetch_entity_data_via_route(entity_id, topic_name) : std::nullopt) {
      return dto::DataValue{std::move(*via_route)};
    }
    return tl::make_unexpected(
        make_error(404, ERR_RESOURCE_NOT_FOUND, "No data provider for plugin entity '" + entity_id + "'"));
  }
  try {
    auto result = data_prov->read_data(entity_id, topic_name);
    if (!result) {
      return tl::make_unexpected(make_provider_error(result.error(), entity_id));
    }
    return std::move(*result);
  } catch (const std::exception & e) {
    MEDKIT_LOG_ERROR(logger, "Plugin DataProvider threw for entity '%s': %s", entity_id.c_str(), e.what());
    return tl::make_unexpected(make_plugin_error(500, "Plugin threw exception", json{{"entity_id", entity_id}}));
  } catch (...) {
    MEDKIT_LOG_ERROR(logger, "Plugin DataProvider threw unknown exception for entity '%s'", entity_id.c_str());
    return tl::make_unexpected(
        make_plugin_error(500, "Plugin threw unknown exception", json{{"entity_id", entity_id}}));
  }
}

PluginDataReaders plugin_data_readers(PluginManager * plugins) {
  PluginDataReaders readers;
  readers.provider = [plugins](const std::string & entity_id) -> DataProvider * {
    return plugins ? plugins->get_data_provider_for_entity(entity_id) : nullptr;
  };
  readers.route = [plugins](const std::string & entity_id) -> std::optional<json> {
    return plugins ? plugins->fetch_entity_data_via_route(entity_id) : std::nullopt;
  };
  return readers;
}

tl::expected<TopicSampleResult, ErrorInfo> sample_data_topic(DataAccessManager & data,
                                                             const std::string & full_topic_path) {
  // Sampling goes through the pool-backed TopicDataProvider.
  auto * provider = data.get_topic_data_provider();
  if (provider == nullptr) {
    return tl::make_unexpected(make_error(503, ERR_SERVICE_UNAVAILABLE, "Topic sampling is not configured"));
  }
  const auto timeout_ms =
      std::chrono::milliseconds{static_cast<std::int64_t>(std::max(data.get_topic_sample_timeout(), 0.0) * 1000.0)};
  return provider->sample(full_topic_path, timeout_ms);
}

}  // namespace ros2_medkit_gateway
