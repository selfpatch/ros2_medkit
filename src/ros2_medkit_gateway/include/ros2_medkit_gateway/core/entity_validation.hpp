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

#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Validate an entity ID against naming conventions.
/// Allow: alphanumeric (a-z, A-Z, 0-9), underscore (_), hyphen (-).
/// Max 256 characters.
/// @return void on success, error message string on failure.
tl::expected<void, std::string> validate_entity_id(const std::string & entity_id);

}  // namespace ros2_medkit_gateway
