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

#include <nlohmann/json.hpp>
#include <string>
#include <tuple>
#include <utility>

#include "ros2_medkit_gateway/dto/config.hpp"
#include "ros2_medkit_gateway/dto/contract.hpp"
#include "ros2_medkit_gateway/dto/data.hpp"
#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/dto/errors.hpp"
#include "ros2_medkit_gateway/dto/faults.hpp"
#include "ros2_medkit_gateway/dto/locks.hpp"
#include "ros2_medkit_gateway/dto/operations.hpp"
#include "ros2_medkit_gateway/dto/schema_writer.hpp"
#include "ros2_medkit_gateway/dto/triggers.hpp"
#include "ros2_medkit_gateway/dto/x_medkit.hpp"

namespace ros2_medkit_gateway {
namespace dto {

/// The single compile-time list of every named DTO. Each domain header
/// (Phase 2/3) appends its types here. Order is irrelevant.
using AllDtos =
    std::tuple<GenericError, XMedkitRos2, XMedkitArea, XMedkitComponent, XMedkitApp, XMedkitFunction, XMedkitCollection,
               AreaListItem, AreaDetail, ComponentListItem, ComponentDetail, AppListItem, AppDetail, FunctionListItem,
               FunctionDetail, Collection<AreaListItem>, Collection<ComponentListItem>, Collection<AppListItem>,
               Collection<FunctionListItem>, FaultListItem, Collection<FaultListItem>, FaultListXMedkit,
               FaultListAggXMedkit, FaultStatus, FaultItem, FaultEnvironmentData, FaultXMedkit, FaultDetail,
               XMedkitOperationItem, XMedkitOperationExecution, OperationItem, Collection<OperationItem>,
               OperationDetail, OperationExecution, ExecutionUpdateRequest,
               // Configuration domain DTOs
               ConfigXMedkitItem, ConfigurationMetaData, Collection<ConfigurationMetaData>, ConfigListXMedkit,
               ConfigValueXMedkit, ConfigurationReadValue, ConfigurationWriteRequest, ConfigurationDeleteResultItem,
               ConfigurationDeleteMultiStatus,
               // Data domain DTOs
               XMedkitDataItem, DataItem, Collection<DataItem>, DataListXMedkit, DataWriteRequest,
               // Lock domain DTOs
               Lock, Collection<Lock>, AcquireLockRequest, ExtendLockRequest,
               // Trigger domain DTOs
               Trigger, Collection<Trigger>, TriggerCreateRequest, TriggerUpdateRequest>;

namespace detail {
template <class Tuple, std::size_t... I>
nlohmann::json collect_impl(std::index_sequence<I...> /*seq*/) {
  nlohmann::json schemas = nlohmann::json::object();
  ((schemas[std::string(dto_name<std::tuple_element_t<I, Tuple>>)] =
        SchemaWriter<std::tuple_element_t<I, Tuple>>::schema()),
   ...);
  return schemas;
}
}  // namespace detail

/// Build the components/schemas object from every DTO in AllDtos.
inline nlohmann::json collect_component_schemas() {
  return detail::collect_impl<AllDtos>(std::make_index_sequence<std::tuple_size_v<AllDtos>>{});
}

}  // namespace dto
}  // namespace ros2_medkit_gateway
