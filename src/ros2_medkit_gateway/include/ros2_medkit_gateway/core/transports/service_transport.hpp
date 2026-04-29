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
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/operations/operation_types.hpp"

namespace ros2_medkit_gateway {

class ServiceTransport {
 public:
  ServiceTransport() = default;
  ServiceTransport(const ServiceTransport &) = delete;
  ServiceTransport & operator=(const ServiceTransport &) = delete;
  ServiceTransport(ServiceTransport &&) = delete;
  ServiceTransport & operator=(ServiceTransport &&) = delete;
  virtual ~ServiceTransport() = default;

  virtual ServiceCallResult call(const std::string & service_path, const std::string & service_type,
                                 const json & request, std::chrono::duration<double> timeout) = 0;
};

}  // namespace ros2_medkit_gateway
